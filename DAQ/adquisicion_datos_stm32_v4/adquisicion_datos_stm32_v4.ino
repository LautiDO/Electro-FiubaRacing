//====================================================
//                    LIBRERÍAS
//====================================================
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <TinyGPS.h>



#include <libmaple/spi.h>
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
//#include <libmaple/afio.h>


//====================================================
//           ENVÍO DE DATOS (SPI SLAVE)
//
//  Estructura del paquete (33 bytes):
//  [0]      → 0xAA  (sync byte)
//  [1..4]   → angulo         (float, 4 bytes)
//  [5..8]   → acel_lateral   (float, 4 bytes)
//  [9..12]  → dx1            (float, 4 bytes)
//  [13..16] → dx2            (float, 4 bytes)
//  [17..20] → dx3            (float, 4 bytes)
//  [21..24] → dx4            (float, 4 bytes)
//  [25..28] → dx5            (float, 4 bytes)
//  [29..32] → tiempo_us      (uint32_t, 4 bytes)
//====================================================
#define MAX_DATOS 34   // 1 sync + 7×4 float + 1×4 uint32

volatile uint8_t bufferActivo    = 0;
         uint8_t bufferEscritura = 1;
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;

int pin_cambios = PA2;


//====================================================
//                      GPS
//====================================================
TinyGPS gps;
volatile bool newData = false;


//====================================================
//               EXTENSÓMETROS
//====================================================
const float DMIN    =  0.0f;
const float DMAX    = 75.0f;
const float DFIJO   =  0.0f;
const float K_ELAST = 35.0f;

const int PIN_EXT1 = PA0;
const int PIN_EXT2 = PA1;
const int PIN_EXT3 = PA2;
const int PIN_EXT4 = PA3;
const int PIN_EXT5 = PB0;


//====================================================
//                      IMU
//====================================================
Adafruit_MPU6050 mpu;

bool     imu_online        = false;
uint32_t imu_ultimo_intento = 0;
const uint32_t IMU_REINTENTO_MS = 2000;

const float OFFSET_LATERAL = 0.06f;


//====================================================
//          ENCODER VOLANTE
//====================================================
const int PIN_ENCODER_A = PB12;
const int PIN_ENCODER_B = PB13;

volatile long pulsos         = 0;
         long pulsos_anterior = 0;

float angulo = 0.0f;
const float FACTOR_CONVERSION = 0.6f;


//====================================================
//                      SETUP
//====================================================
void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, FALLING);

  Wire.begin();
  //Wire.setSCLTimeout(100);
  intentarConectarIMU();

  configurarSPISlave();
  Serial.println("[OK] SPI Slave configurado.");

  memset(misDatos, 0, sizeof(misDatos));
  Serial.println("[OK] Setup completo. Iniciando adquisicion...");
}


//====================================================
//                      LOOP
//====================================================
void loop() {

  //--- Reconexión IMU (no bloquea) ------------------
  if (!imu_online) {
    uint32_t ahora = millis();
    if (ahora - imu_ultimo_intento >= IMU_REINTENTO_MS) {
      imu_ultimo_intento = ahora;
      intentarConectarIMU();
    }
  }

  //--- Extensómetros --------------------------------
  float dx1 = leerExtensometro(PIN_EXT1);
  /*
  float dx2 = leerExtensometro(PIN_EXT2);
  float dx3 = leerExtensometro(PIN_EXT3);
  float dx4 = leerExtensometro(PIN_EXT4);
  float dx5 = leerExtensometro(PIN_EXT5);
  */

  float dx2 = 1;
  float dx3 = 2;
  float dx4 = 3;
  float dx5 = 4;

  //int cambio = leer_cambio(pin_cambios);
  

  //--- IMU ------------------------------------------
  float acel_lateral_g = 0.0f;
  if (imu_online) {
    if (imuResponde()) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      acel_lateral_g = calcularAceleracionLateral(
        a.acceleration.x, a.acceleration.y, a.acceleration.z
      );
    } else {
      imu_online = false;
      Serial.println("[IMU] Desconexion detectada. Reintentando en 2 s...");
    }
  }

  //--- Encoder --------------------------------------
  noInterrupts();
  long copia_pulsos = pulsos;
  interrupts();
  if (copia_pulsos != pulsos_anterior) {
    angulo          = copia_pulsos * FACTOR_CONVERSION;
    pulsos_anterior = copia_pulsos;
  }

  //--- GPS ------------------------------------------
  while (Serial3.available()) {
    char c = Serial3.read();
    if (gps.encode(c)) newData = true;
  }
  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("[GPS] LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0f : flat, 6);
    Serial.print("  LON=");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0f : flon, 6);
    newData = false;
  }

  //--- Armar y publicar paquete SPI -----------------
  uint32_t tiempo_actual = micros();
  armarPaquete(angulo, acel_lateral_g, dx1, dx2, dx3, dx4, dx5, tiempo_actual);
  bufferActivo    = bufferEscritura;
  bufferEscritura = 1 - bufferActivo;

  //--- Debug serial ---------------------------------
  Serial.print(imu_online ? "[IMU OK] " : "[IMU --] ");
  Serial.print("acel="); Serial.print(acel_lateral_g, 3);
  Serial.print(" g  ang="); Serial.print(angulo, 2);
  Serial.print("°  dx1="); Serial.println(dx1, 1);
 // Serial.print("°  cambio=");Serial.print(cambio);
}


//====================================================
//         CONFIGURACIÓN SPI SLAVE
//====================================================
/*
void configurarSPISlave() {
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);

  gpio_set_mode(GPIOA, 4, GPIO_INPUT_FLOATING);  // NSS
  gpio_set_mode(GPIOA, 5, GPIO_INPUT_FLOATING);  // SCK
  gpio_set_mode(GPIOA, 7, GPIO_INPUT_FLOATING);  // MOSI
  gpio_set_mode(GPIOA, 6, GPIO_AF_OUTPUT_PP);    // MISO

  SPI1->regs->CR1  = 0;
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE;
  SPI1->regs->CR1 |= SPI_CR1_SPE;

  SPI1->regs->DR = misDatos[bufferActivo][0];
  nvic_irq_enable(NVIC_SPI1);
}
*/

void configurarSPISlave() {
  // 1. Habilitar relojes
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);
  rcc_clk_enable(RCC_GPIOB);
  rcc_clk_enable(RCC_AFIO); 

  // 2. Liberar pines JTAG (PA15, PB3, PB4) para que funcionen como SPI
  // Esto es CRÍTICO para que el micro no ignore estos pines
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // 3. Activar el Remap usando la constante correcta de libmaple
  afio_remap(AFIO_REMAP_SPI1); 

  // 4. Configurar los NUEVOS pines según el Remap de SPI1
  // NSS: PA15 | SCK: PB3 | MISO: PB4 | MOSI: PB5
  gpio_set_mode(GPIOA, 15, GPIO_INPUT_FLOATING); 
  gpio_set_mode(GPIOB, 3, GPIO_INPUT_FLOATING);  
  gpio_set_mode(GPIOB, 5, GPIO_INPUT_FLOATING);  
  gpio_set_mode(GPIOB, 4, GPIO_AF_OUTPUT_PP);    

  // 5. Configuración de registros SPI (igual que antes)
  SPI1->regs->CR1  = 0;
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE;
  SPI1->regs->CR1 |= SPI_CR1_SPE;

  // Cargar primer byte
  SPI1->regs->DR = misDatos[bufferActivo][0];
  nvic_irq_enable(NVIC_SPI1);
}


//====================================================
//         ISR — SPI1
//====================================================
extern "C" void __irq_spi1() {
  if (SPI1->regs->SR & SPI_SR_RXNE) {
    (void)SPI1->regs->DR;
    i_dato++;
    if (i_dato >= MAX_DATOS) i_dato = 0;
    SPI1->regs->DR = misDatos[bufferActivo][i_dato];
  }
}


//====================================================
//              FUNCIONES AUXILIARES
//====================================================

void armarPaquete(float ang, float acel,
                  float dx1, float dx2, float dx3, float dx4, float dx5,
                  uint32_t t) {
  uint8_t* buf = misDatos[bufferEscritura];
  int pos = 0;

  buf[pos++] = 0xAA;
  memcpy(&buf[pos], &ang,  4); pos += 4;
  memcpy(&buf[pos], &acel, 4); pos += 4;
  memcpy(&buf[pos], &dx1,  4); pos += 4;
  memcpy(&buf[pos], &dx2,  4); pos += 4;
  memcpy(&buf[pos], &dx3,  4); pos += 4;
  memcpy(&buf[pos], &dx4,  4); pos += 4;
  memcpy(&buf[pos], &dx5,  4); pos += 4;
  memcpy(&buf[pos], &t,    4); // pos = 33 — fin del paquete, sin checksum
}

void intentarConectarIMU() {
  imu_ultimo_intento = millis();
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    imu_online = true;
    Serial.println("[IMU] Conectada.");
  } else {
    imu_online = false;
    Serial.println("[IMU] No responde. Reintento en 2 s...");
  }
}

bool imuResponde() {
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(0x68, 1);
  if (!Wire.available()) return false;
  return (Wire.read() == 0x68);
}

float leerExtensometro(int pin) {
  int raw    = analogRead(pin);
  float dmed = DMIN + (DMAX - DMIN) * (float)(4095 - raw) / 4095.0f;
  return dmed - DFIJO;
}

float calcularAceleracionLateral(float Ax, float Ay, float Az) {
  float pitch  = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll   = atan2f(Ay, Az);
  float gy     = 9.81f * sinf(roll) * cosf(pitch);
  float ay_lin = Ay - gy;
  return (ay_lin / 9.81f) - OFFSET_LATERAL;
}

void encoderISR() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) pulsos++;
  else                                    pulsos--;
}

int leer_cambio(int pin_cambios) {
    // Leemos el ADC (suponiendo resolución de 12 bits: 0-4095)
    int val = analogRead(pin_cambios);
    Serial.print(val); 
    
}
