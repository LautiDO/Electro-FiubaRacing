//====================================================
//                    LIBRERÍAS
//====================================================
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
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

volatile int i_dato = 0;  // índice del byte actual a enviar por SPI


//====================================================
//                      GPS
//====================================================
TinyGPS gps;
volatile bool newData = false;  // flag: llegó sentencia NMEA válida

// Serial3 → GPS  (TX = PB10, RX = PB11)


//====================================================
//        SENSOR LÁSER (VL53L0X) — TODO: integrar lectura
//====================================================
// VL53L0X sensor;
// TwoWire Wire2(PB9, PB8);   // I2C2
// Al inicializar: Wire2.begin(); sensor.init(); sensor.startContinuous();
// Para leer:      uint16_t distancia_mm = sensor.readRangeContinuousMillimeters();


//====================================================
//               EXTENSÓMETROS
//  ⚠️  Pines reasignados: PA4–PA7 son del SPI1 y no
//      pueden usarse como ADC al mismo tiempo.
//====================================================
const float DMIN   =  0.0f;   // desplazamiento mínimo [mm]
const float DMAX   = 75.0f;   // desplazamiento máximo [mm]
const float DFIJO  =  0.0f;   // offset mecánico en reposo [mm]
const float K_ELAST = 35.0f;  // constante elástica [N/mm]

const int PIN_EXT1 = PA0;
const int PIN_EXT2 = PA1;
const int PIN_EXT3 = PA2;
const int PIN_EXT4 = PA3;
const int PIN_EXT5 = PB0;


//====================================================
//                      IMU
//====================================================
Adafruit_MPU6050 mpu;
const float OFFSET_LATERAL = 0.0f;  // calibrar en banco, restar bias


//====================================================
//          ENCODER VOLANTE (POSICIÓN)
//  Resolución: 600 pulsos/vuelta → 360°/600 = 0.6°/pulso
//====================================================
const int PIN_ENCODER_A = PB12;
const int PIN_ENCODER_B = PB13;

volatile long pulsos    = 0;
         long pulsos_anterior = 0;

float angulo = 0.0f;
const float FACTOR_CONVERSION = 0.6f;  // grados por pulso


//====================================================
//                      SETUP
//====================================================
void setup() {

  // ---------- Serial debug ----------
  Serial.begin(115200);
  Serial3.begin(9600);   // GPS

  // ---------- Encoder ---------------
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, FALLING);

  // ---------- IMU -------------------
  if (!mpu.begin()) {
    while (1) {
      delay(500); 
      Serial.println("[ERROR] MPU6050 no encontrado. Verificar cableado.");
      }
  }
  Serial.println("[OK] MPU6050 inicializado.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // ---------- SPI Slave -------------
  configurarSPISlave();
  Serial.println("[OK] SPI Slave configurado.");

  // ---------- Buffers ---------------
  memset(misDatos, 0, sizeof(misDatos));

  delay(500);
  Serial.println("[OK] Setup completo. Iniciando adquisicion...");
}


//====================================================
//                      LOOP
//====================================================
void loop() {

  //================================================
  //            EXTENSÓMETROS
  //================================================
  float dx1 = leerExtensometro(PIN_EXT1);
  float dx2 = leerExtensometro(PIN_EXT2);
  float dx3 = leerExtensometro(PIN_EXT3);
  float dx4 = leerExtensometro(PIN_EXT4);
  float dx5 = leerExtensometro(PIN_EXT5);


  //================================================
  //                  IMU
  //================================================
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float acel_lateral_g = calcularAceleracionLateral(
    a.acceleration.x,
    a.acceleration.y,
    a.acceleration.z
  );


  //================================================
  //              POSICIÓN VOLANTE
  //================================================
  noInterrupts();
  long copia_pulsos = pulsos;
  interrupts();

  if (copia_pulsos != pulsos_anterior) {
    angulo = copia_pulsos * FACTOR_CONVERSION;
    pulsos_anterior = copia_pulsos;
  }


  //================================================
  //                  GPS
  //================================================
  while (Serial3.available()) {
    char c = Serial3.read();
    if (gps.encode(c)) {
      newData = true;
    }
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

  // Swap atómico: la ISR empieza a usar el nuevo buffer
  bufferActivo    = bufferEscritura;
  bufferEscritura = 1 - bufferActivo;

  // Reset índice para el próximo ciclo
  // (la ISR lo maneja, pero si hubo silencio reseteamos por seguridad)
  // No tocar i_dato desde acá — es responsabilidad de la ISR


  //================================================
  //           DEBUG SERIAL (opcional, comentar
  //           si afecta el timing en pista)
  //================================================
  Serial.print("[IMU] Acel lateral (g): ");
  Serial.println(acel_lateral_g, 3);
  Serial.print("[ENC] Angulo volante (°): ");
  Serial.println(angulo, 2);
  Serial.print("[EXT] dx1="); Serial.print(dx1, 1);
  Serial.print("  dx2=");     Serial.print(dx2, 1);
  Serial.print("  dx3=");     Serial.print(dx3, 1);
  Serial.print("  dx4=");     Serial.print(dx4, 1);
  Serial.print("  dx5=");     Serial.println(dx5, 1);
}


//====================================================
//         CONFIGURACIÓN SPI SLAVE (STM32)
//
//  Pines SPI1:
//    PA4 → NSS  (chip select, entrada)
//    PA5 → SCK  (clock, entrada)
//    PA6 → MISO (datos hacia Raspberry, salida)
//    PA7 → MOSI (datos desde Raspberry, entrada)
//
//  La ISR se dispara cada vez que el master (Raspberry)
//  envía un byte. El STM32 responde con el siguiente byte
//  del paquete pre-cargado en el registro DR.
//====================================================
void configurarSPISlave() {
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);

  gpio_set_mode(GPIOA, 4, GPIO_INPUT_FLOATING);  // NSS
  gpio_set_mode(GPIOA, 5, GPIO_INPUT_FLOATING);  // SCK
  gpio_set_mode(GPIOA, 7, GPIO_INPUT_FLOATING);  // MOSI
  gpio_set_mode(GPIOA, 6, GPIO_AF_OUTPUT_PP);    // MISO

  SPI1->regs->CR1  = 0;               // modo slave, MSB first, CPOL=0, CPHA=0
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE; // interrupción al recibir byte
  SPI1->regs->CR1 |= SPI_CR1_SPE;    // habilitar SPI

  // Pre-cargar el primer byte del paquete
  SPI1->regs->DR = misDatos[bufferActivo][0];

  nvic_irq_enable(NVIC_SPI1);
}


//====================================================
//         ISR — SPI1 (dispara por cada byte recibido)
//
//  Cada vez que la Raspberry hace un ciclo de clock,
//  el STM32 envía misDatos[bufferActivo][i_dato] y
//  pre-carga el siguiente en DR para la próxima transferencia.
//====================================================
extern "C" void __irq_spi1() {
  if (SPI1->regs->SR & SPI_SR_RXNE) {
    (void)SPI1->regs->DR;  // leer DR limpia el flag RXNE (dato recibido se descarta)

    i_dato++;
    if (i_dato >= MAX_DATOS) i_dato = 0;

    SPI1->regs->DR = misDatos[bufferActivo][i_dato];
  }
}


//====================================================
//              FUNCIONES AUXILIARES
//====================================================

// -------- ARMAR PAQUETE SPI ----------------------
//  Escribe en bufferEscritura (inactivo) y al terminar
//  el loop() hace el swap. La ISR nunca toca este buffer
//  mientras se arma.
void armarPaquete(float ang, float acel,
                  float dx1, float dx2, float dx3, float dx4, float dx5,
                  uint32_t t) {
  uint8_t* buf = misDatos[bufferEscritura];
  int pos = 0;

  buf[pos++] = 0xAA;                         // sync byte

  memcpy(&buf[pos], &ang,  4); pos += 4;     // ángulo volante
  memcpy(&buf[pos], &acel, 4); pos += 4;     // aceleración lateral
  memcpy(&buf[pos], &dx1,  4); pos += 4;     // suspensión 1
  memcpy(&buf[pos], &dx2,  4); pos += 4;     // suspensión 2
  memcpy(&buf[pos], &dx3,  4); pos += 4;     // suspensión 3
  memcpy(&buf[pos], &dx4,  4); pos += 4;     // suspensión 4
  memcpy(&buf[pos], &dx5,  4); pos += 4;     // suspensión 5
  memcpy(&buf[pos], &t,    4); pos += 4;     // timestamp [µs]

  // Checksum XOR de todos los bytes anteriores
  uint8_t cs = 0;
  for (int k = 0; k < pos; k++) cs ^= buf[k];
  buf[pos] = cs;  // byte 33 → checksum
}


// -------- EXTENSÓMETROS ---------------------------
//  Convierte la lectura ADC (0–4095) a desplazamiento [mm]
//  usando interpolación lineal float (evita truncamiento de map()).
float leerExtensometro(int pin) {
  int raw = analogRead(pin);

  // ADC 4095 → dmin,  ADC 0 → dmax  (sensor invertido)
  float dmed = DMIN + (DMAX - DMIN) * (float)(4095 - raw) / 4095.0f;
  float dx   = dmed - DFIJO;

  return dx;  // [mm]  — para fuerza: return K_ELAST * dx;
}


// -------- IMU: ACELERACIÓN LATERAL ----------------
//  Elimina la componente gravitacional proyectada sobre cada eje
//  antes de devolver la aceleración lateral en [g].
float calcularAceleracionLateral(float Ax, float Ay, float Az) {
  float pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll  = atan2f(Ay, Az);

  // Componente gravitacional estimada en cada eje
  float gx =  9.81f * -sinf(pitch);
  float gy =  9.81f *  sinf(roll) * cosf(pitch);
  // float gz = 9.81f * cosf(roll) * cosf(pitch);  // no se usa

  // Aceleración lineal (sin gravedad)
  float ay_lin = Ay - gy;

  return (ay_lin / 9.81f) - OFFSET_LATERAL;  // [g]
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
//           FUNCIONES PENDIENTES DE IMPLEMENTAR
//====================================================
// void leerFrenoLaser() {
//   uint16_t dist_mm = sensor.readRangeContinuousMillimeters();
//   // empaquetar en misDatos si se agrega al protocolo
// }

// void leerPresionFreno() {}
// void leerPresionCombustible() {}
