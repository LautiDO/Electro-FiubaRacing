// ejemplo configuracion de la IMU
//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <SPI.h>    // libreria interfaz SPI
#include <SD.h>     // libreria para tarjetas SD

//-------------------------Variables guardado memoria---------------------------------


#define SSpin 38    // Slave Select en pin digital 10
bool test;
File archivo;


unsigned long tiempoMuestra=0;
const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos


//-------------------------Variables IMU---------------------------------

Adafruit_MPU6050 mpu;
float angulo_fuerzag = 0;
float magnitud_fuerzag= 0;


//-------------------------Variables RPM---------------------------------

volatile unsigned long currentmicros = 0;
volatile unsigned long previousmicros = 0;
volatile unsigned long periodo = 1;
volatile unsigned long rpm = 0;
int dientes = 36;

//int rpm = 0;                                //Variable donde se guardarán las rpm
//unsigned long previousmicros = 0;           //Variable donde guardaremos el tiempo anterior para contabilizar la frecuencia
//unsigned long currentmicros = 0;            //Variable donde guardaremos el tiempo actual para contabilizar la frecuencia
//unsigned long periodo = 30000000;           //Variable donde guardaremos la duracion entre interrupciones. Iniciamos en 30000000 para que el primer calculo de rpm no sea infinito
float aceleracion_x = 0;
//-------------------------Variables TPS---------------------------------

const int pinPot = 0;
float TPS = 0;

//-------------------------Variables temperatura motor---------------------------------

int pin_temp_motor = PA7;

float temperatura_motor = 0;



//-------------------------Variables temperatura del aire---------------------------------


int pin_aire =PA8;
float temperatura_aire = 0;

//-------------------------Variables IMU---------------------------------

int pin_tps = PA9; //fijarse cual era el asignado
int min_tps=0;
int max_tps =1023;//esto hay que cambiarlo una vez puesto en el motor


//-------------------------Variables lambda---------------------------------
float AFR = 0;

int pin_lambda = PA10;



//--------------------------------------------------------------------

void setup() {

Serial.begin(115200);



pinMode(PA12, INPUT);  // Configura PA12 como entrada
attachInterrupt(digitalPinToInterrupt(PA12), RPM, FALLING);  // Interrupción por flanco descendente


Serial.println("Inicializando tarjeta ...");  // texto en ventana de monitor
test = SD.begin(SSpin);



  Serial.println((int)test);
  if (!test) {      // inicializacion de tarjeta SD
    Serial.println("fallo en inicializacion !");// si falla se muestra texto correspondiente y
    return;         // se sale del setup() para finalizar el programa
  }


  Serial.println("inicializacion correcta");  // texto de inicializacion correcta
  archivo = SD.open("prueba.txt", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt



/*

delay(1000);
  //------------------------Inicializacion de la IMU--------------------------------
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
*/
 

}

void loop() {
 

    unsigned long tiempoActual = millis();
    

  

    if((tiempoActual - tiempoMuestra >= periodoMuestreo))
    {     
 //-----------------------Llamar funciones-------------------------

     if (periodo > 0) {
     rpm = (60* 1000000)/(periodo * dientes);
     } 
     Serial.print("RPM: ");
     Serial.println(rpm);    
      
     temperatura_motor = medir_temp_motor(pin_temp_motor , 100 , 50000);    
     Serial.print("temperatura motor: ");
     Serial.println(temperatura_motor); 


     AFR = medir_lambda(pin_lambda);
     Serial.print("AFR: ");
     Serial.println(AFR); 

     
     temperatura_aire = medir_temp_aire(pin_temp_motor , 100 , 50000);    
     Serial.print("temperatura aire: ");
     Serial.println(temperatura_aire); 
        
     TPS = medir_tps(pin_tps , min_tps , max_tps);
     Serial.print("TPS: ");
     Serial.println(temperatura_aire , '%');  

     

     guardar_datos(temperatura_motor , rpm);
 /*...............0   
 
 //-----------------------Codigo imu-------------------------     
      //inicializaciones para poder llamar a la funcion
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      calcularFuerzaG(a.acceleration.x, a.acceleration.y, &angulo_fuerzag, &magnitud_fuerzag);

      aceleracion_x = a.acceleration.x * -1;//aceleracion real, por eso multiplico por -1, va en el otro sentido
      
 //-----------------------Enviar datos-------------------------
      matlab_send(aceleracion_x, 1,1);

    /*/
      tiempoMuestra = tiempoActual;
    }
  
}



//-------------------------FUNCIONES---------------------------------


//-----------------------Envio de datos a MATLAB-------------------------
void matlab_send(float dato1, float dato2, float dato3){
  //Serial.write("abcd");
  Serial.write(97);
  Serial.write(98);
  Serial.write(99);
  Serial.write(100);
  
  byte * b0 = (byte *) &dato1;
  Serial.write(b0,4);
  
  byte * b1 = (byte *) &dato2;
  Serial.write(b1,4);
//  
byte * b2 = (byte *) &dato3;
Serial.write(b2,4);
}

//-----------------------IMU-------------------------
void calcularFuerzaG(float Ax, float Ay, float* angulo, float* magnitud){
  //transformo en fuerza g 
  float Ax_G = Ax / 9.81; 
  float Ay_G = Ay / 9.81;

  *angulo = atan2(Ay_G, Ax_G) * 180.0 / PI;
  *magnitud = sqrt(Ax_G * Ax_G + Ay_G * Ay_G);
}



//-------------------------Funcion temperatura motor---------------------------------

float medir_temp_motor(int pin_motor, float Rmin , float Rmax){
    const float Vcc = 3.3;
    const float R_fija = 3300.0; // ohms

    // Leer el ADC (0 a 4095) y convertir a voltaje
    uint16_t adc_val = analogRead(pin_motor);
    
    float vt = (adc_val * Vcc) / 4095.0;

     Serial.print("tension leida: ");
     Serial.println(vt);    
    
    // Calcular resistencia del termistor
    float Rt = (vt * R_fija) / (Vcc - vt);
    //map(potValor, potMin, potMax, 0, 180);
    float temp_motor = map(Rt , Rmin , Rmax, 0 , 100);

    return temp_motor; // resistencia en ohms
}

//-------------------------Funcion TPS---------------------------------


float medir_tps(int pin_tps, int min_val, int max_val) {
    //min_val y max_val son los limites del potenciometro
    //min_val = con el acelerador en 0
    //max_val = con el acelerador en 100
    
    //leo lo que mide el potenciometro
    int lectura = analogRead(pin_tps);

    //lo mapeo entre 0 y 100 asi tengo el porcentaje de acelerador apretado
    return map(lectura, min_val, max_val, 0, 100);
}


//-------------------------funcion temperatura del aire--------------------------------



float medir_temp_aire(int pin_aire, float Rmin, float Rmax) {
    const float Vcc = 3.3;
    const float R_fija = 3300.0; // ohms

    uint16_t adc_val = analogRead(pin_aire);
    float vt = (adc_val * Vcc) / 4095.0;

    float Rt = (vt * R_fija) / (Vcc - vt);

    float temp_aire = map(Rt, Rmin, Rmax, 0, 100);

    return temp_aire;
}


//-------------------------funcion lambda---------------------------------


float medir_lambda(int pin_afr) {
  
    const float Vref = 3.3;
    uint16_t adc = analogRead(pin_afr);

    float volt = (adc * Vref) / 4095.0; //veo que tension devuelve 

    // Conversión directa sin mapFloat
    float afr = (volt - 0.2) * (25.0 - 8.7) / (4.8 - 0.2) + 8.7;

    return afr;
}








//-------------------------funcion RPM---------------------------------
void RPM()                                  //Funcion a realizar en cada interrupcion externa del pin 2
{
  currentmicros = micros();                 //Almacenamos los us transcurridos en la variable currentmicros
  periodo = currentmicros - previousmicros; //El periodo de la señal serán los us actuales menos los us anteriores
  previousmicros = currentmicros;           //Igualamos us actuales a us anteriores para calcular el proximo periodo
} 


//-------------------------funcion guardar datos---------------------------------

void guardar_datos(float dato1 , float dato2){
  File archivo = SD.open("prueba.txt", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt

  if (archivo) {
    archivo.println("temperatura");
    archivo.println(dato1);  // escritura de una linea de texto en archivo
    archivo.println("RPM");
    archivo.println(dato1);  // escritura de una linea de texto en archivo
    
    Serial.println("Escribiendo en archivo prueba.txt..."); // texto en monitor serie
    archivo.close();        // cierre del archivo
    Serial.println("escritura correcta"); // texto de escritura correcta en monitor serie
  } else {
    Serial.println("error en apertura de prueba.txt");  // texto de falla en apertura de archivo
  }

}
