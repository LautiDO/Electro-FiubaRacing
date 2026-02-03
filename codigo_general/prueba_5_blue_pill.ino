#include <SPI.h>		
#include <SD.h>		
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;


#define SSpin PA4		// Slave Select en pin digital PA4

File archivo;			// objeto archivo del tipo File
char nombre_archivo[] = "aceang.txt"; 
int dato = 0;
bool write = 1; // elegir entre leer o escribir



//---------------- RPM --------------------
const int pinOut = PA8;     // salida de señal cuadrada
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

//------------imu---------------------


float angulo_fuerzag = 0;
float magnitud_fuerzag= 0;
float offset = 0;


//------------Posicion Volante---------------------
const int pinA = PA0;
const int pinB = PA1;

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

  Serial.println("Inicializando tarjeta ...");
  if (!SD.begin(SSpin)) {			
    Serial.println("fallo en inicializacion !");
    return;					
  }
  Serial.println("inicializacion correcta");	

  // lectura del archivo
 if(!write){
  archivo = SD.open(nombre_archivo);
    if (archivo) {		
      while (archivo.available()) {		// mientras exista contenido en el archivo
        Serial.write(archivo.read());  		// lectura de a un caracter por vez
      }
      archivo.close();				
    } else {
      Serial.println("error en la apertura de prueba.txt");
    }
  }

  // ----------RPM-----------------------
  pinMode(pinOut, OUTPUT);
  pinMode(PIN_RPM, INPUT_PULLUP);

  // PB5 usa EXTI5 → ok
 // attachInterrupt(digitalPinToInterrupt(PIN_RPM), RPM, FALLING);
  //--------------------------------------



  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
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
  
    float dx = func_extensometros(PB1);
    
    Serial.print("dx:");
    Serial.println(dx);

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
    
        Serial.print("Angulo: ");
        Serial.println(angulo, 2); // 2 decimales de precisión
    
        anteriorPulsos = copiaPulsos;
      }
    
    
    //escritura del archivo
    if(write){
      archivo = SD.open(nombre_archivo, FILE_WRITE);	
  
      if (archivo) {
       // Serial.println("Escribiendo");
        archivo.print("RPM ");
        archivo.println(rpm_med);
        archivo.print("dist ");
        archivo.println(dx);
        archivo.close(); 
      } else {
        Serial.println("error en apertura del archivo");
      }
      dato = dato+1;
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
