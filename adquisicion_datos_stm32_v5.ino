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
#define MAX_DATOS 45   // 1 sync + 7×4 float + 1×4 uint32

volatile uint8_t bufferActivo = 0;
uint8_t bufferEscritura = 1;
uint8_t misDatos[2][MAX_DATOS];

volatile int i_dato = 0;


//====================================================
//                      CAMBIOS
//====================================================

#define TOLERANCIA 70
#define NUM_CONFIRMACIONES 1   

int t1      = 3420;   // Primera
int t2      = 2980;   // Segunda
int t3      = 1976;   // Tercera
int t4      = 3959;   // Cuarta
int t5      = 3860;   // Quinta
int t6      = 3700;   // Sexta
int neutro  = 4095;   // Neutro
int pin_cambios = PA4;
int tolerancia = 5;
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
//               EXTENSÓMETROS
//====================================================

#define NUM_EXTENSOMETROS 5

const float DMIN    =  0.0f;
const float DMAX    = 75.0f;
const float DFIJO   =  0.0f;
const float K_ELAST = 35.0f;

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
//                      IMU
//====================================================

int pin_presion1 = PA1;
int pin_presion2 = PA2;




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
const uint32_t PERIODO_MUESTREO_US = 20000; // 20.000 us = 50 Hz
uint32_t tiempo_previo_us = 0;


void loop() {
  uint32_t tiempo_actual = micros();

  // Ejecuta la adquisición estrictamente cada 20 ms (50 Hz)
  if (tiempo_actual - tiempo_previo_us >= PERIODO_MUESTREO_US) {
    tiempo_previo_us += PERIODO_MUESTREO_US;

    //--- Reconexión IMU (no bloquea) ------------------
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
    float dx3 = leerExtensometro(PIN_EXT3 , 2);
    /*
    float dx4 = leerExtensometro(PIN_EXT4);
    float dx5 = leerExtensometro(PIN_EXT5);
    */
  
   // float dx2 = 1;
    //float dx3 = 2;
    float dx4 = 3;
    float dx5 = 4;
    //--- PRESION ------------------------------------------
  
    uint16_t cambio_adc = analogRead(pin_cambios);
    
  
    int cambio_confirmado = leer_cambio_confirmado(cambio_adc);
  
    
  
      
    //int cambio = leer_cambio(pin_cambios);
  
    //Serial.print(valor);
    
    //int cambio = 4;
    //--- PRESION ------------------------------------------
  
    float presion1 = leer_presion(pin_presion1);
    float presion2 = leer_presion(pin_presion2);
  
  
  
  
  //--- IMU ------------------------------------------

    float acel_lateral_g = 0.0f;
    float acel_long_g = 0.0f;
    float magnitud_g = 0.0f;
    float angulo_vector = 0.0f;
    if (imu_online) {
      if (imuResponde()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // La función actualiza directamente acel_lateral_g y acel_long_g
        calcularFuerzasG(a.acceleration.x, a.acceleration.y, a.acceleration.z, acel_lateral_g, acel_long_g);
        
        // Magnitud total (radio del vector)
        magnitud_g = sqrtf(acel_lateral_g * acel_lateral_g + acel_long_g * acel_long_g);
        
        // Dirección del vector en grados (de -180 a 180)
        angulo_vector = atan2f(acel_lateral_g, acel_long_g) * 180.0f / PI;

        // Impresión para el monitor/graficador
        Serial.print("Magnitud:"); Serial.print(magnitud_g); Serial.print(" ");
        Serial.print("Angulo:"); Serial.print(angulo_vector);
 

      } else {
        imu_online = false;
        Serial.println("[IMU] Desconexion detectada. Reintentando en 2 s...");
      }
    }


    /*  
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
    }*/

    //--- Encoder --------------------------------------
    noInterrupts();
    long copia_pulsos = pulsos;
    interrupts();
    if (copia_pulsos != pulsos_anterior) {
      angulo          = copia_pulsos * FACTOR_CONVERSION;
      pulsos_anterior = copia_pulsos;
    }
  
  
    //--- Armar y publicar paquete SPI -----------------
    uint32_t tiempo_actual = micros();
    armarPaquete(angulo, acel_lateral_g, dx1, dx2, dx3, dx4, dx5, cambio_confirmado, tiempo_actual, presion1, presion2);
    bufferActivo    = bufferEscritura;
    bufferEscritura = 1 - bufferActivo;
  
    //--- Debug serial ---------------------------------
    
    Serial.print("Angulo:"); Serial.print(angulo, 1); Serial.print(" ");
    Serial.print("dx1:");    Serial.print(dx1, 1);    Serial.print(" ");
    Serial.print("dx2:");    Serial.print(dx2, 1);    Serial.print(" ");
    Serial.print("dx3:");    Serial.print(dx3, 1);    Serial.print(" "); // <-- Espacio agregado aquí
    Serial.print("FuerzaG:"); Serial.print(acel_lateral_g);              // <-- Etiqueta unida y con ':'
    Serial.print("°  cambio=");Serial.print(cambio_confirmado);
    Serial.print("°  cambio adc=");Serial.print(cambio_adc);
    Serial.println();
   // Serial.print("presion1="); Serial.println(presion1);
    //Serial.print("presion2="); Serial.println(presion2, 1);
  }
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
                  float dx1, float dx2, float dx3, float dx4, float dx5, int cambio,
                  uint32_t t, float presion1, float presion2) {
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
  memcpy(&buf[pos], &presion1,  4); pos += 4;
  memcpy(&buf[pos], &presion2,  4); pos += 4;
  memcpy(&buf[pos], &cambio,  4); pos += 4;
  memcpy(&buf[pos], &t,    4); // pos = 33 
  
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
/*
float leerExtensometro(int pin) {
  int raw    = analogRead(pin);
  float dmed = DMIN + (DMAX - DMIN) * (float)(4095 - raw) / 4095.0f;
  return dmed - DFIJO;
}
*/

float OFFSET_LONGITUDINAL = 0;




void calcularFuerzasG(float Ax, float Ay, float Az, float &lat, float &lon) {
  // 1. Calcular inclinación (pitch y roll) usando tu lógica
  float pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  float roll  = atan2f(Ay, Az);
  
  // 2. Calcular cuánta gravedad afecta a cada eje por la inclinación
  float gy = 9.81f * sinf(roll) * cosf(pitch);
  float gx = 9.81f * sinf(pitch); 
  
  // 3. Restar la gravedad para obtener la aceleración lineal pura
  float ay_lin = Ay - gy;
  float ax_lin = Ax - gx;
  
  // 4. Convertir a Gs y restar tu offset
  lat = (ay_lin / 9.81f) - OFFSET_LATERAL;

  lon = (ax_lin / 9.81f) - OFFSET_LONGITUDINAL; 
}







float leerExtensometro(int pin, int id) {
    const float ALPHA = 0.30f;
    static float filtrado[NUM_EXTENSOMETROS] = {0};
    static int inicializado[NUM_EXTENSOMETROS] = {0};

    int raw = analogRead(pin);

    // Primera lectura: inicializa sin rampa de subida
    if (!inicializado[id]) {
        filtrado[id] = (float)raw;
        inicializado[id] = 1;
    } else {
        filtrado[id] += ALPHA * ((float)raw - filtrado[id]);
    }

    float dmed = DMIN + (DMAX - DMIN) * (4095.0f - filtrado[id]) / 4095.0f;
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


int clasificar_lectura(int valor) {
  if (valor >= (t1 - TOLERANCIA) && valor <= (t1 + TOLERANCIA)) return 1;
  if (valor >= (t2 - TOLERANCIA) && valor <= (t2 + TOLERANCIA)) return 2;
  if (valor >= (t3 - TOLERANCIA) && valor <= (t3 + TOLERANCIA)) return 3;
  if (valor >= (t4 - TOLERANCIA) && valor <= (t4 + TOLERANCIA)) return 4;
  if (valor >= (t5 - TOLERANCIA) && valor <= (t5 + TOLERANCIA)) return 5;
  if (valor >= (t6 - TOLERANCIA) && valor <= (t6 + TOLERANCIA)) return 6;
  if (valor >= (neutro - TOLERANCIA) && valor <= (neutro + TOLERANCIA)) return 0;
  return -1; // en tránsito, no matchea ninguna
}


int leer_cambio_confirmado(int valor_adc) {
  int lectura = clasificar_lectura(valor_adc);

  if (lectura == -1) {
    // en tránsito: se resetea el conteo, no se confirma nada
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
    estado_actual = candidato; // se confirma (aunque sea el mismo que ya tenía)
  }

  return estado_actual;
}



float leer_presion(int pin_presion){
  float presion = analogRead(pin_presion);
  return presion;
  
  
  }
