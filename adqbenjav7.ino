//====================================================
//                    LIBRERÍAS
//====================================================
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <libmaple/iwdg.h>

#include <libmaple/spi.h>
#include <libmaple/gpio.h>
#include <libmaple/rcc.h>
#include <libmaple/nvic.h>
//#include <libmaple/afio.h>

//====================================================
//           ENVÍO DE DATOS (SPI SLAVE)
//====================================================
// Trama de 62 bytes:
//  [0]      0xAA
//  [1..2]   contador de muestra (uint16)
//  [3..54]  13 floats
//  [55..58] cambio (int32)
//  [59..60] contador de muestra repetido
//  [61]     checksum XOR de los bytes 1..60
#define MAX_DATOS 62
#define SPI_SYNC  0xAA

// Bits de registros SPI (por si tu versión de libmaple no los define)
#ifndef SPI_CR2_TXEIE
#define SPI_CR2_TXEIE (1U << 7)
#endif
#ifndef SPI_SR_TXE
#define SPI_SR_TXE    (1U << 1)
#endif

// ESQUEMA DE TRANSMISIÓN (anti byte repetido):
//  - RX (RXNE):  solo recibe los bytes de comando de la Raspi y cuenta bytes (i_dato).
//  - TX (TXE):   apenas el byte N empieza a salir, TXE avisa y ahí se carga el byte N+1.
//                Así el siguiente byte tiene un byte entero (~32 us @ 250 kHz) para estar
//                listo en DR, en vez de los pocos us de hueco entre bytes del master.
//  - El byte 0 (0xAA) se precarga en DR cuando CS sube, y es el mismo en los dos buffers.
volatile uint8_t bufferActivo = 0;
volatile uint8_t bufferEscritura = 1;
volatile bool datoNuevoListo = false;
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;          // bytes RECIBIDOS en la transacción en curso
volatile int tx_idx = 1;          // próximo byte a CARGAR en DR (el 0 ya está precargado)
volatile uint32_t cntBytes = 0;   // bytes recibidos por SPI (lo usa el timeout del loop)

//====================================================
//     COMANDOS RASPI -> STM32 (MOSI, bytes 0..6)
//====================================================
// 0x55, código, parámetro int32 (4 bytes), checksum XOR (código + parámetro)
// Si la Raspi manda ceros, no pasa nada.
#define CMD_LEN 7
volatile uint8_t rxCmd[CMD_LEN];
volatile bool    comandoNuevo = false;
volatile uint8_t cmdCodigo    = 0;
volatile int32_t cmdParam     = 0;

//====================================================
//                      CAMBIOS
//====================================================
#define TOLERANCIA 40
#define NUM_CONFIRMACIONES 2   

int t1      = 3420;   // Primera
int t2      = 2980;   // Segunda
int t3      = 1976;   // Tercera
int t4      = 3959;   // Cuarta
int t5      = 3860;   // Quinta
int t6      = 3700;   // Sexta
int neutro  = 4095;   // Neutro
int pin_cambios = PA4;
int cambio = 0;

int estado_actual = -1;         // último cambio CONFIRMADO
int candidato = -2;             // valor que se está evaluando
int contador_confirmacion = 0;  // cuántas veces seguidas se repitió el candidato 
  
//====================================================
//                      GPS
//====================================================
TinyGPS gps;
volatile bool newData = false;

//====================================================
//                EXTENSÓMETROS
//====================================================
#define NUM_EXTENSOMETROS 5

const float DMIN    =   0.0f;
float       DMAX    =  110.0f;   // ya no es const: se puede cambiar por comando
const float DFIJO   =   0.0f;
const float K_ELAST =  35.0f;

const int PIN_EXT1 = PA6;
const int PIN_EXT2 = PA5;
const int PIN_EXT3 = PB1;
const int PIN_EXT4 = PA3;
const int PIN_EXT5 = PB0;

//====================================================
//                      IMU
//====================================================
Adafruit_MPU6050 mpu;

bool     imu_online        = false;
uint32_t imu_ultimo_intento = 0;
const uint32_t IMU_REINTENTO_MS = 2000; // cada cuánto reintenta mpu.begin() si está offline

const float OFFSET_LATERAL = 0.06f;
float OFFSET_LONGITUDINAL  = 0;      // movido acá para que el loop pueda cambiarlo por comando

float acel_lateral_g = 0.0f;
float acel_long_g    = 0.0f;
float magnitud_g     = 0.0f;
float angulo_vector  = 0.0f;

float ax = 0.0f, ay = 0.0f, az = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f;

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
//                      PRESIONES
//====================================================
int pin_presion1 = PA3;
int pin_presion2 = PA2;

//====================================================
//                      SETUP
//====================================================
void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);

  // Verificación de reinicio por Watchdog en libmaple
  if (RCC_BASE->CSR & RCC_CSR_IWDGRSTF) {
    Serial.println("[ALERTA] El STM32 se reinicio por WATCHDOG (IWDG)!");
    RCC_BASE->CSR |= RCC_CSR_RMVF; // Limpiar banderas de reset
  }

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoderISR, FALLING);

  Wire.begin();
  intentarConectarIMU();

  configurarSPISlave();
  Serial.println("[OK] SPI Slave configurado.");

  memset(misDatos, 0, sizeof(misDatos));

  // Inicialización del IWDG (~2.0 segundos de timeout)
  iwdg_init(IWDG_PRE_64, 1250);
  IWDG_BASE->KR = 0xCCCC;
  Serial.println("[OK] Setup completo. Iniciando adquisicion...");
}

//====================================================
//                      LOOP
//====================================================
const uint32_t PERIODO_MUESTREO_US = 20000; // 20.000 us = 50 Hz
uint32_t tiempo_previo_us = 0;

void loop() {
  
  //--- Red de seguridad: trama cortada y el flanco de CS no llegó ----
  // Si hay una trama a medias y pasaron más de 2 ms sin bytes nuevos, resincroniza.
  static uint32_t cntPrev  = 0;
  static uint32_t tCntPrev = 0;
  uint32_t cntNow  = cntBytes;
  uint32_t ahoraUs = micros();
  if (cntNow != cntPrev) {
    cntPrev  = cntNow;
    tCntPrev = ahoraUs;
  } else if (i_dato != 0 && (ahoraUs - tCntPrev) > 2000) {
    noInterrupts();
    if (i_dato != 0 && cntBytes == cntPrev) resincronizarSPI();
    interrupts();
  }

  //--- Ejecutar comando recibido de la Raspi (fuera de la ISR) -------
  if (comandoNuevo) {
    noInterrupts();
    uint8_t c = cmdCodigo;
    int32_t p = cmdParam;
    comandoNuevo = false;
    interrupts();

    switch (c) {
      case 1: OFFSET_LONGITUDINAL = p / 1000.0f; break;                // offset longitudinal (milésimas de g)
      case 2: pulsos = 0; pulsos_anterior = 0; angulo = 0; break;      // volante en cero
      case 3: DMAX = p / 10.0f; break;                                 // DMAX en décimas de mm
      default: break;
    }
  }

  uint32_t tiempo_actual = micros();

  // Ejecuta la adquisición estrictamente cada 20 ms (50 Hz)
  if (tiempo_actual - tiempo_previo_us >= PERIODO_MUESTREO_US) {
    tiempo_previo_us += PERIODO_MUESTREO_US;

    iwdg_feed();

    //--- Reconexión IMU (no bloquea el resto del loop) ------------------
    if (!imu_online) {
      uint32_t ahora = millis();
      if (ahora - imu_ultimo_intento >= IMU_REINTENTO_MS) {
        imu_ultimo_intento = ahora;
        intentarConectarIMU();
      }
    }
    
    //--- Extensómetros --------------------------------
    float dx1 = leerExtensometro(PIN_EXT1, 0);
    float dx2 = leerExtensometro(PIN_EXT2, 1);
    float dx3 = leerExtensometro(PIN_EXT3, 2);

    //--- Cambios ---------------------------------------
    uint16_t cambio_adc = analogRead(pin_cambios);
    int cambio_confirmado = leer_cambio_confirmado(cambio_adc);

    //--- Presión -----------------------------------------
    float presion1 = leer_presion(pin_presion1);
    float presion2 = leer_presion(pin_presion2);

    //--- IMU -----------------------------------------------------------
    if (imu_online) {
      if (imuResponde()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        calcularFuerzasG(a.acceleration.x, a.acceleration.y, a.acceleration.z, acel_lateral_g, acel_long_g);

        magnitud_g = sqrtf(acel_lateral_g * acel_lateral_g + acel_long_g * acel_long_g);

        ax = a.acceleration.x;
        ay = a.acceleration.y;
        az = a.acceleration.z;
        gx = g.gyro.x;
        gy = g.gyro.y;
        gz = g.gyro.z;
        /*
        Serial.print("Magnitud:"); Serial.print(magnitud_g); Serial.print(" ");
        Serial.print("ax="); Serial.print(ax); Serial.print(", ");
        Serial.print("ay="); Serial.print(ay); Serial.print(", ");
        Serial.print("az="); Serial.print(az); Serial.print(", ");
        Serial.print("gx="); Serial.print(gx); Serial.print(", ");
        Serial.print("gy="); Serial.print(gy); Serial.print(", ");
        Serial.print("gz="); Serial.print(gz);
        */
      } else {
        imu_online = false;
        Serial.println("[IMU] Desconexion detectada. Reintentando en 2 s...");
      } 
    }

    //--- Encoder --------------------------------------
    long copia_pulsos = pulsos;
    if (copia_pulsos != pulsos_anterior) {
      angulo          = copia_pulsos * FACTOR_CONVERSION;
      pulsos_anterior = copia_pulsos;
    }

    //--- Armar paquete en el buffer secundario (inactivo) ---
    armarPaquete(angulo, acel_lateral_g, ax, ay, az, gx, gy, gz,
                 dx1, dx2, dx3, cambio_confirmado, presion1, presion2);

    //--- Sincronización Ping-Pong segura ---
    noInterrupts();
    if (i_dato == 0 && digitalRead(PA15) == HIGH) {
      // SPI en reposo: intercambiamos buffers inmediatamente.
      // DR ya tiene precargado 0xAA (igual en ambos buffers), no hace falta tocarlo.
      bufferActivo    = bufferEscritura;
      bufferEscritura = 1 - bufferActivo;
      datoNuevoListo  = false;
    } else {
      // SPI transmitiendo: no tocamos i_dato ni bufferActivo, avisamos a la ISR
      datoNuevoListo = true;
    }
    interrupts();

    //--- Debug serial ---------------------------------
    //Serial.print(" Angulo:"); Serial.print(angulo, 1); Serial.print(" | ");
        /*
    int pin1 = digitalRead(PIN_ENCODER_A);
    int pin2 = digitalRead(PIN_ENCODER_B);
    Serial.print(pin1); Serial.print(" | pin2: ");
    Serial.print(pin2); 

    Serial.print("dx1:"); Serial.print(dx1, 1); Serial.print(" | ");
    Serial.print("dx2:"); Serial.print(dx2, 1); Serial.print(" | ");
    Serial.print("dx3:"); Serial.print(dx3, 1); Serial.print(" | ");
    Serial.print("cambio="); Serial.print(cambio_confirmado);
    Serial.print(" | pin1: ");

    */
    // (comentado: este println vacío se ejecutaba cada 20 ms y mantiene activo el USB)
    //Serial.println();
  }
}

//====================================================
//         CONFIGURACIÓN SPI SLAVE
//====================================================
void configurarSPISlave() {
  rcc_clk_enable(RCC_SPI1);
  rcc_clk_enable(RCC_GPIOA);
  rcc_clk_enable(RCC_GPIOB);
  rcc_clk_enable(RCC_AFIO); 

  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);
  afio_remap(AFIO_REMAP_SPI1); 

  pinMode(PA15, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PA15), nss_rising_isr, RISING);

  gpio_set_mode(GPIOA, 15, GPIO_INPUT_FLOATING); 
  gpio_set_mode(GPIOB, 3, GPIO_INPUT_FLOATING);  
  gpio_set_mode(GPIOB, 5, GPIO_INPUT_FLOATING);  
  gpio_set_mode(GPIOB, 4, GPIO_AF_OUTPUT_PP);    

  SPI1->regs->CR1  = 0;
  SPI1->regs->CR2 |= SPI_CR2_RXNEIE;
  SPI1->regs->CR1 |= SPI_CR1_SPE;

  precargarTx();                      // DR = 0xAA, tx_idx = 1, habilita TXEIE
  nvic_irq_enable(NVIC_SPI1);
}

// Deja listo el slave para la PRÓXIMA transacción: byte 0 en DR y habilita TXE.
// Con DR cargado, TXE=0, así que la interrupción TXE no dispara hasta que empiece a salir el byte 0.
void precargarTx() {
  tx_idx = 1;
  SPI1->regs->DR = SPI_SYNC;
  SPI1->regs->CR2 |= SPI_CR2_TXEIE;
}
//====================================================
//         ISR — CHIP SELECT (PA15 / NSS)
//====================================================
// Valida el comando recibido en los primeros 7 bytes de la trama (si hubo uno)
void procesarRx() {
  if (i_dato < CMD_LEN) return;          // trama demasiado corta
  if (rxCmd[0] != 0x55) return;          // sin comando (la Raspi mandó ceros)

  uint8_t chk = 0;
  for (int j = 1; j <= 5; j++) chk ^= rxCmd[j];
  if (chk != rxCmd[6]) return;           // checksum malo: se descarta

  cmdCodigo = rxCmd[1];
  int32_t p;
  memcpy(&p, (const void*)&rxCmd[2], 4);
  cmdParam = p;
  comandoNuevo = true;
  rxCmd[0] = 0;                          // evita ejecutarlo dos veces
}

// Resincronización de emergencia, la llama el loop (con interrupciones deshabilitadas)
void resincronizarSPI() {
  procesarRx();
  i_dato = 0;
  if (datoNuevoListo) {
    bufferActivo    = bufferEscritura;
    bufferEscritura = 1 - bufferActivo;
    datoNuevoListo  = false;
  }
  precargarTx();
}

void nss_rising_isr() {
  // Cuando la Raspi levanta CS a 3.3V, la transferencia terminó.
  procesarRx();
  i_dato = 0;

  // Si el loop preparó datos frescos durante la transmisión, intercambiamos buffer acá
  if (datoNuevoListo) {
    bufferActivo    = bufferEscritura;
    bufferEscritura = 1 - bufferActivo;
    datoNuevoListo  = false;
  }

  // Precargamos inmediatamente el primer byte (0xAA) para la PRÓXIMA lectura
  precargarTx();
}
//====================================================
//         ISR — SPI1
//====================================================
extern "C" void __irq_spi1() {
  uint32_t sr = SPI1->regs->SR;

  // Si hubo desborde por alguna demora, se limpia leyendo DR y luego SR,
  // y se vuelven a leer los flags (el DR ya se consumió acá, no hay que leerlo de nuevo)
  if (sr & SPI_SR_OVR) {
    (void)SPI1->regs->DR;
    (void)SPI1->regs->SR;
    sr = SPI1->regs->SR;
  }

  // --- RX: byte que manda la Raspi (comando o ceros). Ya NO carga nada en DR. ---
  if (sr & SPI_SR_RXNE) {
    uint8_t rx = SPI1->regs->DR;
    if (i_dato < CMD_LEN) rxCmd[i_dato] = rx;
    cntBytes++;
    i_dato++;
  }

  // --- TX: el byte anterior acaba de pasar al shift register → cargar el siguiente ---
  if ((sr & SPI_SR_TXE) && (SPI1->regs->CR2 & SPI_CR2_TXEIE)) {
    if (tx_idx < MAX_DATOS) {
      SPI1->regs->DR = misDatos[bufferActivo][tx_idx];
      tx_idx++;
    }
    if (tx_idx >= MAX_DATOS) {
      // Ya se cargó el último byte (checksum): se apaga TXE para que no dispare en bucle.
      // Se vuelve a habilitar en precargarTx() cuando CS sube.
      SPI1->regs->CR2 &= ~SPI_CR2_TXEIE;
    }
  }
}

//====================================================
//              FUNCIONES AUXILIARES
//====================================================
void armarPaquete(float ang, float acel,
                  float ax, float ay, float az,
                  float gx, float gy, float gz,
                  float dx1, float dx2, float dx3, int cambio,
                  float presion1, float presion2) {
  uint8_t* buf = misDatos[bufferEscritura];
  int pos = 0;

  static uint16_t nMuestra = 0;
  uint16_t cnt = ++nMuestra;

  buf[pos++] = 0xAA;
  memcpy(&buf[pos], &cnt,  2); pos += 2;   // contador de muestra (inicio)
  memcpy(&buf[pos], &ang,  4); pos += 4;
  memcpy(&buf[pos], &acel, 4); pos += 4;
  memcpy(&buf[pos], &ax,   4); pos += 4;
  memcpy(&buf[pos], &ay,   4); pos += 4;
  memcpy(&buf[pos], &az,   4); pos += 4;
  memcpy(&buf[pos], &gx,   4); pos += 4;
  memcpy(&buf[pos], &gy,   4); pos += 4;
  memcpy(&buf[pos], &gz,   4); pos += 4;
  memcpy(&buf[pos], &dx1,  4); pos += 4;
  memcpy(&buf[pos], &dx2,  4); pos += 4;
  memcpy(&buf[pos], &dx3,  4); pos += 4;
  memcpy(&buf[pos], &presion1,  4); pos += 4;
  memcpy(&buf[pos], &presion2,  4); pos += 4;
  memcpy(&buf[pos], &cambio,  4); pos += 4;
  memcpy(&buf[pos], &cnt,  2); pos += 2;   // contador de muestra (fin)

  uint8_t chk = 0;
  for (int j = 1; j < pos; j++) chk ^= buf[j];
  buf[pos] = chk;
}

//====================================================
//         RECUPERACIÓN DE BUS I2C
//====================================================
#define I2C_RECOVERY_SCL PB6
#define I2C_RECOVERY_SDA PB7

void recuperarBusI2C() {
  pinMode(I2C_RECOVERY_SDA, INPUT_PULLUP);
  pinMode(I2C_RECOVERY_SCL, OUTPUT);

  if (digitalRead(I2C_RECOVERY_SDA) == HIGH) {
    Wire.begin();
    return;
  }

  Serial.println("[I2C] Bus trabado (SDA en bajo). Recuperando...");

  for (int i = 0; i < 9 && digitalRead(I2C_RECOVERY_SDA) == LOW; i++) {
    digitalWrite(I2C_RECOVERY_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_RECOVERY_SCL, HIGH);
    delayMicroseconds(5);
  }

  pinMode(I2C_RECOVERY_SDA, OUTPUT);
  digitalWrite(I2C_RECOVERY_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_RECOVERY_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_RECOVERY_SDA, HIGH);
  delayMicroseconds(5);

  if (digitalRead(I2C_RECOVERY_SDA) == LOW) {
    Serial.println("[I2C] No se pudo liberar SDA. Revisar cableado/pull-ups.");
  } else {
    Serial.println("[I2C] Bus liberado.");
  }

  Wire.begin();
}

void intentarConectarIMU() {
  imu_ultimo_intento = millis();
  recuperarBusI2C();
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

void calcularFuerzasG(float Ax, float Ay, float Az, float &lat, float &lon) {
  float pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll  = atan2f(Ay, Az);

  float gy = 9.81f * sinf(roll) * cosf(pitch);
  float gx = 9.81f * sinf(pitch); 

  float ay_lin = Ay - gy;
  float ax_lin = Ax - gx;

  lat = (ay_lin / 9.81f) - OFFSET_LATERAL;
  lon = (ax_lin / 9.81f) - OFFSET_LONGITUDINAL; 
}

float leerExtensometro(int pin, int id) {
    const float ALPHA = 0.5;
    static float filtrado[NUM_EXTENSOMETROS] = {0};
    static int inicializado[NUM_EXTENSOMETROS] = {0};

    int raw = analogRead(pin);

    if (!inicializado[id]) {
        filtrado[id] = (float)raw;
        inicializado[id] = 1;
    } else {
        filtrado[id] += ALPHA * ((float)raw - filtrado[id]);
    }

    float dmed = DMIN + (DMAX - DMIN) * (4095.0f - raw) / 4095.0f;
    return dmed;
}

float calcularAceleracionLateral(float Ax, float Ay, float Az) {
  float pitch  = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll   = atan2f(Ay, Az);
  float gy     = 9.81f * sinf(roll) * cosf(pitch);
  float ay_lin = Ay - gy;
  return (ay_lin / 9.81f) - OFFSET_LATERAL;
}

void encoderISR() {
  if (digitalRead(PIN_ENCODER_B) == HIGH) pulsos = pulsos + 1;
  else                                    pulsos = pulsos - 1;
}

bool esTransicionValida(int desde, int hacia) {
  if (desde == -1) return true;
  if (desde == hacia) return true;
  switch (desde) {
    case 0: return (hacia == 1 || hacia == 2);
    case 1: return (hacia == 0 || hacia == 2);
    case 2: return (hacia == 0 || hacia == 1 || hacia == 3);
    case 3: return (hacia == 2 || hacia == 4);
    case 4: return (hacia == 3 || hacia == 5);
    case 5: return (hacia == 4 || hacia == 6);
    case 6: return (hacia == 5);
    default: return false;
  }
}

int clasificar_lectura(int valor) {
  if (valor >= (t1 - TOLERANCIA) && valor <= (t1 + TOLERANCIA)) return 1;
  if (valor >= (t2 - TOLERANCIA) && valor <= (t2 + TOLERANCIA)) return 2;
  if (valor >= (t3 - TOLERANCIA) && valor <= (t3 + TOLERANCIA)) return 3;
  if (valor >= (t4 - TOLERANCIA) && valor <= (t4 + TOLERANCIA)) return 4;
  if (valor >= (t5 - TOLERANCIA) && valor <= (t5 + TOLERANCIA)) return 5;
  if (valor >= (t6 - TOLERANCIA) && valor <= (t6 + TOLERANCIA)) return 6;
  if (valor >= (neutro - TOLERANCIA) && valor <= (neutro + TOLERANCIA)) return 0;
  return -1;
}

int leer_cambio_confirmado(int valor_adc) {
  int lectura = clasificar_lectura(valor_adc);

  if (lectura == -1) {
    contador_confirmacion = 0;
    candidato = -2;
    return estado_actual;
  }

  if (lectura == candidato) {
    contador_confirmacion++;
  } else {
    candidato = lectura;
    contador_confirmacion = 1;
  }

  if (contador_confirmacion >= NUM_CONFIRMACIONES) {
    estado_actual = candidato;
  }

  return estado_actual;
}

float leer_presion(int pin_presion) {
    // 1. Lectura del ADC de 12 bits (0 a 4095)
    int adc_raw = analogRead(pin_presion);

    // 2. Tensión medida en el pin del STM32 (0 a 3.3V)
    float v_adc = (adc_raw * 3.3f) / 4095.0f;

    // 3. Reconstrucción de la tensión original del sensor antes del divisor
    // Factor de atenuación: (R1 + R2) / R2 = (1.8k + 3.3k) / 3.3k = 5.1 / 3.3
    float v_sensor = v_adc * (5.1f / 3.3f);

    // 4. Conversión lineal a PSI: P = 400 * (V_sensor - 0.5)
    float presion = 400.0f * (v_sensor - 0.5f);

    // Evitar valores negativos por ruido electromagnético en reposo
    if (presion < 0.0f) {
        presion = 0.0f;
    }

    return presion;
}
