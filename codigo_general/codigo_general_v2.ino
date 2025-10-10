// ejemplo configuracion de la IMU
//-------------------------Librerias---------------------------------
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <SPI.h>    // libreria interfaz SPI
#include <SD.h>     // libreria para tarjetas SD
#include <TinyGPS.h> 

//-------------------------Variables guardado memoria---------------------------------


#define SSpin PA4     // Slave Select en pin digital 10
bool test;
File archivo;      // objeto archivo del tipo File
char nombre_archivo[] = "aceang.txt"; 
int dato = 0;
bool write_on = 1; // elegir entre leer o escribir


unsigned long tiempoMuestra=0;
const int periodoMuestreo = 20;   //cada cuanto tiempo se toman muestras en milisegundos


// Contador para hacer flush() cada N escrituras
int contador_memoria = 0;
bool flag_interrupcion = false;

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

const int pinPot = PA6;
float TPS = 0;

//-------------------------Variables temperatura motor---------------------------------

int pin_temp_motor = PA11;

float temperatura_motor = 0;



//-------------------------Variables temperatura del aire---------------------------------


int pin_aire =PB0;
float temperatura_aire = 0;

//-------------------------Variables IMU---------------------------------

int pin_tps = PA11; //fijarse cual era el asignado
int min_tps=0;
int max_tps =1023;//esto hay que cambiarlo una vez puesto en el motor


//-------------------------Variables lambda---------------------------------
float AFR = 0;

int pin_lambda = PB1; 



//-------------------------Variables GPS---------------------------------

unsigned long chars;
unsigned short sentences, failed;
TinyGPS gps;
float newData = 0;
float flat, flon;
unsigned long age;
float contadorgps = 0;


//--------------------------------------- Logica antirebote ----------- 
bool grabando_datos = false;
volatile unsigned long tiempo_ultimo_rebote = 0;
const long demora_rebote = 250; // Tiempo en ms para ignorar rebotes
//--------------------------------------------------------------------

void setup() {

Serial.begin(115200);
Serial2.begin(9600);        // UART1 para GPS (Neo-7M normalmente usa 9600 baud)}
  

Serial.print("setup");

pinMode(PA12, INPUT);  // Configura PA12 como entrada

attachInterrupt(digitalPinToInterrupt(PA12), RPM, FALLING);  // Interrupción por flanco descendente

pinMode(PB13, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(PB13) ,isr_boton , FALLING);


Serial.println("Inicializando tarjeta ...");
  if (!SD.begin(SSpin)) {      
    Serial.println("fallo en inicializacion !");
    return;         
  }
  Serial.println("inicializacion correcta");  

  // lectura del archivo
 
 if(!write_on){
  archivo = SD.open(nombre_archivo);
    if (archivo) {    
      while (archivo.available()) {   // mientras exista contenido en el archivo
        Serial.write(archivo.read());     // lectura de a un caracter por vez
      }
      archivo.close();        
    } else {
      Serial.println("error en la apertura de prueba.txt");
    }
  }



    
/*
Serial.println("Inicializando tarjeta ...");  // texto en ventana de monitor
test = SD.begin(SSpin);



  Serial.println((int)test);
  if (!test) {      // inicializacion de tarjeta SD
    Serial.println("fallo en inicializacion !");// si falla se muestra texto correspondiente y
    return;         // se sale del setup() para finalizar el programa
  }


  Serial.println("inicializacion correcta");  // texto de inicializacion correcta
  archivo = SD.open("prueba.txt", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt


*/




delay(1000);
  //------------------------Inicializacion de la IMU--------------------------------
 /*
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
    

  

    if((tiempoActual - tiempoMuestra >= periodoMuestreo)){ 



      
 //-----------------------Llamar funciones-------------------------

    guardar_datos(AFR ,1);
    contador_memoria = contador_memoria + 1;
    
    Serial.println(contador_memoria);
    if((contador_memoria > 50) && (grabando_datos) && write_on == 1){ 
      //esperamos 50 iteraciones para guardar 
      Serial.println("guardando cada 50 iteraciones");
      archivo.flush();
      contador_memoria = 0;
    }
    
    //aca es solo para abrir el archivo

    if (flag_interrupcion) {
    // Se detectó una pulsación, ahora hacemos el anti-rebote
    if (millis() - tiempo_ultimo_rebote > demora_rebote) {
      
      // Es una pulsación válida, cambiamos el estado
      grabando_datos = !grabando_datos;
      
      if (grabando_datos) {
        
        // --- LÓGICA PARA INICIAR GRABACIÓN ---
        Serial.println("Iniciando grabación...");
        
        // Abrimos el archivo y lo guardamos en la variable GLOBAL
        archivo = SD.open(nombre_archivo, FILE_WRITE);
        if (archivo) {
          // Escribimos la cabecera del archivo
          archivo.println("AFR,Dato2");
          
          Serial.println("Archivo abierto correctamente.");
        } else {
          Serial.println("Error al abrir el archivo.");
          grabando_datos = false; // No podemos grabar, cancelamos
        }
        
      } else {
        // --- LÓGICA PARA DETENER GRABACIÓN ---
        Serial.println("...Grabación detenida.");
        
        if (archivo) {
          archivo.close(); // Cerramos el archivo
          Serial.println("Archivo cerrado y guardado.");
        }
      }
      tiempo_ultimo_rebote = millis(); // Actualizamos el tiempo
    }
    flag_interrupcion = false; // Reseteamos la bandera
  }


    
/*
     if (periodo > 0) {
     rpm = (60* 1000000)/(periodo * dientes);
     } 
     Serial.print("RPM: ");
     Serial.println(rpm);  

       
      
     temperatura_motor = medir_temp_motor(pin_temp_motor , 100 , 50000);    
     Serial.print("temperatura motor: ");
     Serial.println(temperatura_motor); 

*/

    // AFR = medir_lambda(pin_lambda);
     //Serial.print("AFR: ");
    // Serial.println(AFR); 

 /*    
     temperatura_aire = medir_temp_aire(pin_temp_motor , 100 , 50000);    
     Serial.print("temperatura aire: ");
     Serial.println(temperatura_aire); 
        
     TPS = medir_tps(pin_tps , min_tps , max_tps);
     Serial.print("TPS: ");
     Serial.println(temperatura_aire , '%');  


     guardar_datos(temperatura_motor , rpm);
 */

 /*
 //-----------------------Codigo imu-------------------------     
      //inicializaciones para poder llamar a la funcion
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      float acel_lateral_g;
      calcularAceleracionLateral(a.acceleration.x, a.acceleration.y, a.acceleration.z, &acel_lateral_g);
      
      Serial.print("Aceleracion lateral (g): ");
      Serial.println(acel_lateral_g, 3);

    */
    
//-----------------------Codigo GPS------------------------- 
  
    
    char c = Serial2.read();
    if (gps.encode(c)){  // ¿Llego una nueva sentencia válida?
      newData = 2;
       // v v  Serial.println(newData);
    }
    if (newData ==2) {
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


      
 //-----------------------Enviar datos-------------------------
     // matlab_send(aceleracion_x, 1,1);
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
  *acel_lateral_g = ay_lin / 9.81;
}



//-------------------------Funcion temperatura motor---------------------------------

float medir_temp_motor(int pin_motor, float Rmin , float Rmax){
    const float Vcc = 3.3;
    const float R_fija = 100000.0; // ohms

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
    
    //float afr map()
    float afr = 6.161 * volt + 9.127;

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




//-------------------------funcion guardar datos---------------------------------

/*
void guardar_datos(float dato1 , float dato2 ,File archivo){
    contador_escritura = 0;
    if(write){
      //File archivo = SD.open("aceang.txt", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt
      //archivo = SD.open(nombre_archivo, FILE_WRITE);  
      if (archivo){
        archivo.println("rpm");
        archivo.println(dato1);  // escritura de una linea de texto en archivo
        archivo.println("temp");
        archivo.println(dato2);  // escritura de una linea de texto en archivo
        Serial.println("Escribiendo en archivo prueba.txt..."); // texto en monitor serie
        //archivo.close();        // cierre del archivo
        Serial.println("escritura correcta"); // texto de escritura correcta en monitor serie
      } else {
        Serial.println("error en apertura de prueba.txt");  // texto de falla en apertura de archivo
      }
  
    }
  
}

*/
/*
 * =======================================================================
 * ==         ISR CON LÓGICA ANTI-REBOTE (DEBOUNCE) INTEGRADA           ==
 * =======================================================================
 * Esta función se ejecuta en cada flanco de bajada del pin del botón.
 * 
 * 
 */

void guardar_datos(float dato1, float dato2) {
  if (archivo) { // Solo si el archivo es válido
    archivo.print(dato1);
    archivo.print(",");
    archivo.println(dato2);
  }
}

void isr_boton() {
  flag_interrupcion = true; // Solo levanta la bandera
}

/*
void isr_boton_debounce() {
  // Leemos el tiempo actual
  unsigned long tiempo_actual = millis();
  grabando_datos != grabando_datos;
  // Comparamos con el tiempo de la última pulsación válida
  if (tiempo_actual - tiempo_ultimo_rebote > demora_rebote) {
    // Si ha pasado suficiente tiempo, es una pulsación válida, no un rebote.
    Serial.print("Boton apretado");
    if(grabando_datos == true){
    Serial.print("Archivo abierto");
    File archivo = SD.open("aceang.txt", FILE_WRITE);  // apertura para lectura/escritura de archivo prueba.txt
    archivo = SD.open(nombre_archivo, FILE_WRITE);  
    }
    else
    {
      //se cierra el archivo
      archivo.close(); 
      Serial.print("Archivo cerrado");
    }
    // 2. Actualizamos el tiempo de la última pulsación válida
    tiempo_ultimo_rebote = tiempo_actual;
  }
  // Si no ha pasado suficiente tiempo, la función simplemente termina,
  // ignorando eficazmente el rebote.
}
*/
