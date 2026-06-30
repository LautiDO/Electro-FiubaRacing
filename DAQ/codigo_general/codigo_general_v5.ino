#include <SPI.h>		
#include <SD.h>		
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <VL53L0X.h>
#include <Wire.h>

#include <TinyGPS.h>

//COSAS DEL ENVIO DE DATOS
#include <libmaple/spi.h>//control spi
#include <libmaple/gpio.h> //control registros
#include <libmaple/rcc.h> //control del clock

#define max_datos 4
uint8_t misDatos[max_datos];
int i_dato = 0;  
//--------GPS-------------------------------

TinyGPS gps;
float newData = 0; //flag de que llego mas info

// En STM32, Serial1 usa pines: TX = PA9, RX = PA10
// Conecta GPS TX → PA10 (RX Blue Pill)
// GPS RX → PA9 (TX Blue Pill) (si quieres enviarle comandos, opcional)





//------------SENSOR LASER FRENO------------
VL53L0X sensor;
TwoWire Wire2(PB9, PB8);

//---------------- RPM --------------------
//const int pinOut = PA8;     // salida de señal cuadrada
const int PIN_RPM = PB6;    // entrada con interrupción EXTI5

const unsigned long pulseWidth = 7000;  // 7000 us HIGH + 7000 us LOW ≈ 71 Hz

volatile unsigned long previousmicros = 0;
volatile unsigned long periodo = 0;
float rpm_med = 0;

//-----------------------------------------


unsigned long tiempoMuestra=0;
const int periodoMuestreo = 20; 

//----------------------------------------- extensometros
float dmin = 0;
float dmax = 75;
float dfijo = 0;
float K = 35; //preguntar a hans
int pin_extensometros = PB1; //ver que otros pines adc usar

//------------imu---------------------

Adafruit_MPU6050 mpu;
float angulo_fuerzag = 0;
float magnitud_fuerzag= 0;
float offset = 0;


//------------Posicion Volante---------------------
const int pinA = PB12;
const int pinB = PB13;

volatile long pulsos = 0; // Cambiado a long para mayor rango
long anteriorPulsos = 0;
float angulo = 0;

// Constante de conversión: 360 grados / 600 pulsos
const float factorConversion = 0.6; 


void setup() {
  
  pinMode(pinA, INPUT_PULLUP); // ACTIVO LAS R DE PULL UP PARA EL ENCODER
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, FALLING);
  
  Serial.begin(115200);
  // 2. GPS en Serial3 (PB10 y PB11)
  Serial3.begin(9600);

  //--------CONFIGURACION SLAVE----------------------
  rcc_clk_enable(RCC_SPI1); //habilito el clock del spi
  rcc_clk_enable(RCC_GPIOA); //activo el clock del puerto a, osea de pa4 a pa7

  gpio_set_mode(GPIOA, 4, GPIO_INPUT_FLOATING); // NSS
  gpio_set_mode(GPIOA, 5, GPIO_INPUT_FLOATING); // SCK
  gpio_set_mode(GPIOA, 7, GPIO_INPUT_FLOATING); // MOSI
  gpio_set_mode(GPIOA, 6, GPIO_AF_OUTPUT_PP);    // MISO

  SPI1->regs->CR1 = 0; //spi slave, 8 bits, msb
  SPI1->regs->CR1 |= SPI_CR1_SPE; // Habilitar SPI
  //dr = data register
  //sr = status register
  SPI1->regs->DR = misDatos[0]; //seteo en el primer valor para que no tenga basura


  // ----------RPM-----------------------
  //pinMode(pinOut, OUTPUT);
  pinMode(PIN_RPM, INPUT_PULLUP);

  // PB5 usa EXTI5 → ok
 //attachInterrupt(digitalPinToInterrupt(PIN_RPM), RPM, FALLING);
  //--------------------------------------

  // Try to initialize!
  if (!mpu.begin(0x68, &Wire2)) { //fijarse si esto anda
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      Serial.println("Failed to find MPU6050 chip");
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G  probar con diversas alternativas
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s  probar con diversas alternativas
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 5-10-21-44-94-184-260 Hz   probar con diversas alternativas
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  
  delay(1000);

 
}


void loop() {
  // ------------------ RPM -----------------------
  noInterrupts();
  unsigned long p = periodo;
  interrupts();

 // unsigned long tiempoActual = millis();
 // if((tiempoActual - tiempoMuestra >= periodoMuestreo)){ 

    if (p > 0) {
      rpm_med = 60000000.0 / p;
    }
   // Serial.print("   RPM: ");
   // Serial.println(rpm_med);
  
    float dx = func_extensometros(pin_extensometros);
    
    //Serial.print("dx:");
    //Serial.println(dx);

    //inicializaciones para poder llamar a la funcion
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
      
    float acel_lateral_g;
    calcularAceleracionLateral(a.acceleration.x, a.acceleration.y, a.acceleration.z, &acel_lateral_g);
      
    Serial.print("Aceleracion lateral (g): ");
    Serial.println(acel_lateral_g, 3);

//---------------pos volante---------------------------------------------
    //se fija si hay una modificacion y lo muestra en la terminal
    if (pulsos != anteriorPulsos) {
        // Protegemos la lectura de la variable volátil
        noInterrupts();
        long copiaPulsos = pulsos;
        interrupts();
    
        // Calculamos el ángulo real
        angulo = copiaPulsos * factorConversion;
        anteriorPulsos = copiaPulsos;
      }
      
     Serial.print("Angulo: ");
     Serial.println(angulo, 2); // 2 decimales de precisión

//-------------------GPS-----------------------------------------
  //ESTO DESPUES LO PONEMOS AFUERA
  unsigned long chars;
  unsigned short sentences, failed;

  // Parsear datos durante 1 segundo
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial3.available()) {
      //Serial.print(Serial1.available());
      //Serial.println("Test1");

      char c = Serial3.read();
      if (gps.encode(c)){  // ¿Llego una nueva sentencia válida?
        newData = 2;
      }
        //Serial.println(newData);
        //Serial.println("AGHAS");
    }
  }

  if (newData ==2) {
    //tengo valores nuevos
           // Serial.println("Ahbtffv2");

    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);

    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.println();
    newData = 0;
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print("CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");


//--------PARTE ENVIO DATOS A RASPBERRY----------------
    misDatos[0] = angulo;
    misDatos[1] = acel_lateral_g;
    misDatos[2] = dx;
    
    if (SPI1->regs->SR & SPI_SR_RXNE) {

      uint8_t recibido = SPI1->regs->DR;  //leo el data register  y limpias el rxne para que no entre si no tiene data nueva
  
      // 2. Preparar el SIGUIENTE dato para la Raspi
      i_dato++;
      if (i_dato >= 8) i_dato = 0; // Reiniciar array si llegamos al final
      
      SPI1->regs->DR = misDatos[i_dato];
  
      // Opcional: ver qué mandó la Raspi
      Serial.print("Recibido: ");
      Serial.println(recibido);
    }



//   tiempoMuestra = tiempoActual;
  }
//}







// --------------------------- RPM ---------------------------
void RPM() {
  unsigned long ahora = micros();
  if (previousmicros > 0) {
  float dt = ahora - previousmicros;
    if(dt>20){
      periodo = dt;
     }
  }
  previousmicros = ahora;
}

//-----------------------------extesometros-------------------------------

float func_extensometros(int pin_ext1){
  int x = analogRead(pin_ext1); //leo el valor actual de resistencia
  //Serial.println(r1);
  
  float dmed = map(x, 4095 , 0 , dmin , dmax); //lo mapeo a valores de distancia
  //Serial.print("dmed:");
  //Serial.println(dmed);
  
  float dx = dmed - dfijo; //variacion de distancia
 // Serial.print("dx:");
 // Serial.println(dx);
  
  
  float F_elastica = K * dx;
  
  return dx; //despues cambiarlo a f_elastica
}

//-----------------------IMU-------------------------
void calcularAceleracionLateral(float Ax, float Ay, float Az, float* acel_lateral_g) {
  // Calcular inclinación
  float pitch = atan2(-Ax, sqrt(Ay * Ay + Az * Az));
  float roll  = atan2(Ay, Az);

  // Gravedad proyectada en cada eje
  float gx = 9.81 * -sin(pitch);
  float gy = 9.81 *  sin(roll) * cos(pitch);
  float gz = 9.81 *  cos(roll) * cos(pitch);

  // Quitar gravedad
  float ax_lin = Ax - gx;
  float ay_lin = Ay - gy;
  float az_lin = Az - gz;

  // Aceleración lateral en g (en este ejemplo uso eje Y como lateral)
  *acel_lateral_g = ay_lin / 9.81 - offset;
}

//-------------------ENCODER----------------
void encoderISR() {
  // Leemos B para determinar sentido
  if (digitalRead(pinB) == HIGH) {
    pulsos++;
  } else {
    pulsos--;
  }
}

//-para implementar---
void presion_frenos(){}
void presion_combustible(){}
