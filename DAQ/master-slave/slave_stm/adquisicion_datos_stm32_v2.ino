    //====================================================
//                    LIBRERÍAS
//====================================================
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include <VL53L0X.h>
#include <Wire.h>
#include <TinyGPS.h>

// Acceso a registros STM32 (SPI slave)
#include <libmaple/spi.h>
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>


//====================================================
//           ENVÍO DE DATOS (SPI SLAVE)
//
//  Estructura del paquete (34 bytes):
//  [0]      → 0xAA  (sync byte)
//  [1..4]   → angulo         (float, 4 bytes)
//  [5..8]   → acel_lateral   (float, 4 bytes)
//  [9..12]  → dx1            (float, 4 bytes)
//  [13..16] → dx2            (float, 4 bytes)
//  [17..20] → dx3            (float, 4 bytes)
//  [21..24] → dx4            (float, 4 bytes)
//  [25..28] → dx5            (float, 4 bytes)
//  [29..32] → tiempo_us      (uint32_t, 4 bytes)
//  [33]     → checksum XOR   (uint8_t, 1 byte)
//====================================================
#define MAX_DATOS 34

// Doble buffer para evitar condición de carrera entre loop() e ISR
volatile uint8_t bufferActivo    = 0;  // ISR lee de este buffer
         uint8_t bufferEscritura = 1;  // loop() escribe en este buffer
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;


//====================================================
//                      GPS
//====================================================
TinyGPS gps;
volatile bool newData = false;

// Serial3 → GPS  (TX = PB10, RX = PB11)


//====================================================
//        SENSOR LÁSER (VL53L0X) — TODO: integrar lectura
//====================================================
// VL53L0X sensor;
// TwoWire Wire2(PB9, PB8);
// Al inicializar: Wire2.begin(); sensor.init(); sensor.startContinuous();
// Para leer:      uint16_t distancia_mm = sensor.readRangeContinuousMillimeters();


//====================================================
//               EXTENSÓMETROS
//  Pines reasignados: PA4–PA7 son del SPI1.
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
//
//  Estrategia de resiliencia:
//    - Si el MPU6050 no responde al arrancar, el setup
//      continúa igual (no bloquea).
//    - En cada ciclo del loop se intenta reconectar si
//      el sensor está offline, pero solo cada
//      IMU_REINTENTO_MS milisegundos para no trabar el loop.
//    - Mientras está offline, acel_lateral_g = 0.0f y
//      se manda igual el paquete SPI con ese valor.
//    - La Raspberry puede detectar falla por la columna
//      imu_ok del CSV (si se agrega) o por valores = 0.
//====================================================
Adafruit_MPU6050 mpu;

bool    imu_online         = false;   // true cuando el MPU responde
uint32_t imu_ultimo_intento = 0;      // timestamp del último intento de reconexión
const uint32_t IMU_REINTENTO_MS = 2000;  // reintentar cada 2 segundos

const float OFFSET_LATERAL = 0.0f;   // calibrar en banco


//====================================================
//          ENCODER VOLANTE
//====================================================
const int PIN_ENCODER_A = PB12;
const int PIN_ENCODER_B = PB13;

volatile long pulsos      = 0;
         long pulsos_anterior = 0;

float angulo = 0.0f;
const float FACTOR_CONVERSION = 0.6f;  // 360° / 600 pulsos


//====================================================
//                      SETUP
//====================================================
void setup() {

  Serial.begin(115200);
  Serial3.begin(9600);   // GPS

  // ---------- Encoder ---------------
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, FALLING);

  // ---------- IMU -------------------
  // intentarIMU() hace un solo intento sin bloquear.
  // Si falla, imu_online queda false y el loop reintenta solo.
  intentarConectarIMU();

  // ---------- SPI Slave -------------
  configurarSPISlave();
  Serial.println("[OK] SPI Slave configurado.");

  // ---------- Buffers ---------------
  memset(misDatos, 0, sizeof(misDatos));

  Serial.println("[OK] Setup completo. Iniciando adquisicion...");
}


//====================================================
//                      LOOP
//====================================================
void loop() {

  //================================================
  //   RECONEXIÓN IMU (no bloquea, solo cada 2 s)
  //================================================
  if (!imu_online) {
    uint32_t ahora = millis();
    if (ahora - imu_ultimo_intento >= IMU_REINTENTO_MS) {
      imu_ultimo_intento = ahora;
      intentarConectarIMU();
    }
  }


  //================================================
  //            EXTENSÓMETROS
  //================================================
  float dx1 = leerExtensometro(PIN_EXT1);
  float dx2 = leerExtensometro(PIN_EXT2);
  float dx3 = leerExtensometro(PIN_EXT3);
  float dx4 = leerExtensometro(PIN_EXT4);
  float dx5 = leerExtensometro(PIN_EXT5);


  //================================================
  //   IMU — solo se lee si está online
  //   Si no, acel_lateral_g = 0.0f y se sigue
  //================================================
  float acel_lateral_g = 0.0f;

  if (imu_online) {
    sensors_event_t a, g, temp;

    // getEvent puede fallar silenciosamente si el sensor
    // se desconectó después del begin(). Lo envolvemos en
    // una lectura de sanidad por I2C para detectarlo.
    if (imuResponde()) {
      mpu.getEvent(&a, &g, &temp);
      acel_lateral_g = calcularAceleracionLateral(
        a.acceleration.x,
        a.acceleration.y,
        a.acceleration.z
      );
    } else {
      // Perdió conexión en caliente → marcar offline
      imu_online = false;
      Serial.println("[IMU] Desconexion detectada. Reintentando en 2 s...");
    }
  }


  //================================================
  //              POSICIÓN VOLANTE
  //================================================
  noInterrupts();
  long copia_pulsos = pulsos;
  interrupts();

  if (copia_pulsos != pulsos_anterior) {
    angulo          = copia_pulsos * FACTOR_CONVERSION;
    pulsos_anterior = copia_pulsos;
  }


  //================================================
  //                  GPS
  //================================================
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
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0f : flon, 6);
    Serial.print("  SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print("  HDOP=");
    Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());

    newData = false;
  }


  //================================================
  //           ARMAR Y PUBLICAR PAQUETE SPI
  //================================================
  uint32_t tiempo_actual = micros();

  armarPaquete(angulo, acel_lateral_g, dx1, dx2, dx3, dx4, dx5, tiempo_actual);

  bufferActivo    = bufferEscritura;
  bufferEscritura = 1 - bufferActivo;


  //================================================
  //           DEBUG SERIAL
  //   Comentar en pista si afecta el timing
  //================================================
  Serial.print(imu_online ? "[IMU OK] " : "[IMU --] ");
  Serial.print("Acel lat="); Serial.print(acel_lateral_g, 3);
  Serial.print(" g  |  Ang="); Serial.print(angulo, 2);
  Serial.print("°  |  dx1="); Serial.print(dx1, 1);
  Serial.print(" dx2=");      Serial.print(dx2, 1);
  Serial.print(" dx3=");      Serial.print(dx3, 1);
  Serial.print(" dx4=");      Serial.print(dx4, 1);
  Serial.print(" dx5=");      Serial.println(dx5, 1);
}


//====================================================
//              FUNCIONES IMU
//====================================================

// -------- Intento de conexión/reconexión -------------
//  Un solo intento, sin delay ni while(1).
//  Actualiza imu_online y configura el sensor si tiene éxito.
void intentarConectarIMU() {
  imu_ultimo_intento = millis();

  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    imu_online = true;
    Serial.println("[IMU] Conectada correctamente.");
  } else {
    imu_online = false;
    Serial.println("[IMU] No responde. Proximo intento en 2 s...");
  }
}


// -------- Verificación rápida por I2C ----------------
//  Hace un ping al registro WHO_AM_I del MPU6050 (0x75).
//  Devuelve true si el sensor responde con el valor esperado (0x68).
//  Tarda ~100 µs — seguro para correr en cada ciclo del loop.
bool imuResponde() {
  Wire.beginTransmission(0x68);   // dirección I2C del MPU6050
  Wire.write(0x75);               // registro WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(0x68, 1);
  if (!Wire.available()) return false;

  uint8_t whoami = Wire.read();
  return (whoami == 0x68);        // valor esperado según datasheet
}


//====================================================
//         CONFIGURACIÓN SPI SLAVE (STM32)
//
//  Pines SPI1:
//    PA4 → NSS  (chip select, entrada)
//    PA5 → SCK  (clock, entrada)
//    PA6 → MISO (datos hacia Raspberry, salida)
//    PA7 → MOSI (datos desde Raspberry, entrada)
//====================================================
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

// -------- ARMAR PAQUETE SPI -----------------------
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
  memcpy(&buf[pos], &t,    4); pos += 4;

  uint8_t cs = 0;
  for (int k = 0; k < pos; k++) cs ^= buf[k];
  buf[pos] = cs;
}


// -------- EXTENSÓMETROS ---------------------------
float leerExtensometro(int pin) {
  int raw   = analogRead(pin);
  float dmed = DMIN + (DMAX - DMIN) * (float)(4095 - raw) / 4095.0f;
  return dmed - DFIJO;
}


// -------- IMU: ACELERACIÓN LATERAL ----------------
float calcularAceleracionLateral(float Ax, float Ay, float Az) {
  float pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll  = atan2f(Ay, Az);

  float gy    = 9.81f * sinf(roll) * cosf(pitch);
  float ay_lin = Ay - gy;

  return (ay_lin / 9.81f) - OFFSET_LATERAL;
}


// -------- ENCODER ISR -----------------------------
void encoderISR() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) {
    pulsos++;
  } else {
    pulsos--;
  }
}


//====================================================
//           FUNCIONES PENDIENTES
//====================================================
// void leerFrenoLaser() {
//   uint16_t dist_mm = sensor.readRangeContinuousMillimeters();
// }
// void leerPresionFreno() {}
// void leerPresionCombustible() {}
