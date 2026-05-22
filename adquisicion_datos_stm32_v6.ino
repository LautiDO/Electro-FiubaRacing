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


//====================================================
//           ENVÍO DE DATOS (SPI SLAVE)
//
//  Estructura del paquete (45 bytes):
//  [0]      → 0xAA       (sync byte)
//  [1..4]   → angulo         (float, 4 bytes)  [°]
//  [5..8]   → acel_lateral   (float, 4 bytes)  [g]
//  [9..12]  → dx1            (float, 4 bytes)  [mm]
//  [13..16] → dx2            (float, 4 bytes)  [mm]
//  [17..20] → dx3            (float, 4 bytes)  [mm]
//  [21..24] → dx4            (float, 4 bytes)  [mm]
//  [25..28] → dx5            (float, 4 bytes)  [mm]
//  [29..32] → presion1       (float, 4 bytes)
//  [33..36] → presion2       (float, 4 bytes)
//  [37..40] → cambio         (uint32_t, 4 bytes)
//  [41..44] → tiempo_us      (uint32_t, 4 bytes)
//====================================================
#define MAX_DATOS 45   // 1 sync + 9×4 float + 2×4 uint32

volatile uint8_t bufferActivo    = 0;
         uint8_t bufferEscritura = 1;
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;


//====================================================
//                      CAMBIOS
//====================================================
int t1        = 329;
int t2        = 495;
int t3        = 577;
int t4        = 614;
int t5        = 625;
int t6        = 637;
int neutro    = 670;
int pin_cambios = PA2;
int tolerancia  = 5;
int cambio      = 0;


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
const int PIN_EXT3 = PA3;
const int PIN_EXT4 = PA4;
const int PIN_EXT5 = PB0;


//====================================================
//                      PRESIÓN
//====================================================
const int PIN_PRESION1 = PA5;
const int PIN_PRESION2 = PA6;


//====================================================
//                      IMU
//====================================================
Adafruit_MPU6050 mpu;

bool     imu_online         = false;
uint32_t imu_ultimo_intento = 0;
const uint32_t IMU_REINTENTO_MS = 2000;

const float OFFSET_LATERAL = 0.06f;


//====================================================
//          ENCODER VOLANTE
//====================================================
const int PIN_ENCODER_A = PB12;
const int PIN_ENCODER_B = PB13;

volatile long pulsos          = 0;
         long pulsos_anterior = 0;

float angulo_enc = 0.0f;
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
  float dx2 = leerExtensometro(PIN_EXT2);
  float dx3 = leerExtensometro(PIN_EXT3);
  float dx4 = leerExtensometro(PIN_EXT4);
  float dx5 = leerExtensometro(PIN_EXT5);
  dx1 = 1;  
  dx2 = 2;
  dx3 = 3;
  dx4 = 4;
  dx5 = 5;
  
  
  //--- Presión --------------------------------------
  float presion1 = leer_presion(PIN_PRESION1);
  float presion2 = leer_presion(PIN_PRESION2);
  presion1 = 23;
  presion2 = 26;

  //--- Cambio de marcha -----------------------------
  int cambio_actual = leer_cambio(pin_cambios);
  cambio_actual = 1;

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
    angulo_enc      = copia_pulsos * FACTOR_CONVERSION;
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
  armarPaquete(angulo_enc, acel_lateral_g,
               dx1, dx2, dx3, dx4, dx5,
               presion1, presion2,
               (uint32_t)cambio_actual,
               tiempo_actual);
  bufferActivo    = bufferEscritura;
  bufferEscritura = 1 - bufferActivo;

  //--- Debug serial ---------------------------------
  Serial.print(imu_online ? "[IMU OK] " : "[IMU --] ");
  Serial.print("acel=");     Serial.print(acel_lateral_g, 3);
  Serial.print("g  ang=");   Serial.print(angulo_enc, 2);
  Serial.print("°  dx1=");   Serial.print(dx1, 1);
  Serial.print("mm  p1=");   Serial.print(presion1, 2);
  Serial.print("  p2=");     Serial.print(presion2, 2);
  Serial.print("  cambio="); Serial.println(cambio_actual);

  delay(20);
}


//====================================================
//         CONFIGURACIÓN SPI SLAVE
//====================================================
void configurarSPISlave() {
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);
  rcc_clk_enable(RCC_GPIOB);
  rcc_clk_enable(RCC_AFIO);

  // Liberar pines JTAG para usarlos como SPI
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

  // Remap SPI1: NSS=PA15 | SCK=PB3 | MISO=PB4 | MOSI=PB5
  afio_remap(AFIO_REMAP_SPI1);

  gpio_set_mode(GPIOA, 15, GPIO_INPUT_FLOATING);  // NSS
  gpio_set_mode(GPIOB,  3, GPIO_INPUT_FLOATING);  // SCK
  gpio_set_mode(GPIOB,  5, GPIO_INPUT_FLOATING);  // MOSI
  gpio_set_mode(GPIOB,  4, GPIO_AF_OUTPUT_PP);    // MISO

  SPI1->regs->CR1  = 0;
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE;
  SPI1->regs->CR1 |= SPI_CR1_SPE;

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

void armarPaquete(float ang,     float acel,
                  float dx1,     float dx2,  float dx3,
                  float dx4,     float dx5,
                  float pres1,   float pres2,
                  uint32_t cam,  uint32_t t) {

  uint8_t* buf = misDatos[bufferEscritura];
  int pos = 0;

  buf[pos++] = 0xAA;               // sync
  memcpy(&buf[pos], &ang,   4); pos += 4;
  memcpy(&buf[pos], &acel,  4); pos += 4;
  memcpy(&buf[pos], &dx1,   4); pos += 4;
  memcpy(&buf[pos], &dx2,   4); pos += 4;
  memcpy(&buf[pos], &dx3,   4); pos += 4;
  memcpy(&buf[pos], &dx4,   4); pos += 4;
  memcpy(&buf[pos], &dx5,   4); pos += 4;
  memcpy(&buf[pos], &pres1, 4); pos += 4;
  memcpy(&buf[pos], &pres2, 4); pos += 4;
  memcpy(&buf[pos], &cam,   4); pos += 4;
  memcpy(&buf[pos], &t,     4);        // pos = 44, total = 45 bytes
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

float leer_presion(int pin) {
  float raw = analogRead(pin);
  // Ajustá esta conversión según tu sensor de presión
  return raw;
}

void encoderISR() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) pulsos++;
  else                                    pulsos--;
}

int leer_cambio(int pin) {
  int valor = analogRead(pin);

  if      (valor >= t1 - tolerancia && valor <= t1 + tolerancia) return 1;
  else if (valor >= t2 - tolerancia && valor <= t2 + tolerancia) return 2;
  else if (valor >= t3 - tolerancia && valor <= t3 + tolerancia) return 3;
  else if (valor >= t4 - tolerancia && valor <= t4 + tolerancia) return 4;
  else if (valor >= t5 - tolerancia && valor <= t5 + tolerancia) return 5;
  else if (valor >= t6 - tolerancia && valor <= t6 + tolerancia) return 6;
  else if (valor >= neutro - tolerancia && valor <= neutro + tolerancia) return 0;
  else return -1;  // valor fuera de rango
}
