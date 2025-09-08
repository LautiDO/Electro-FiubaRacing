int rpm = 1200;      // Variable global
float tps = 0.5;     // Valor inicial en V

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Temperaturas fijas
  Serial.print("Temperatura motor = ");
  Serial.println(80);

  Serial.print("Temperatura aire = ");
  Serial.println(20);

  // RPM simulada
  Serial.print("RPM: ");
  Serial.println(rpm);
  rpm += 100;
  if (rpm > 5500) rpm = 1200; // Reinicia ciclo de RPM

  // Lambda simulada
  float lambdaValue = random(100, 901) / 1000.0;
  Serial.print("AFR = ");
  Serial.println(lambdaValue, 3);

  // TPS simulada
  Serial.print("TPS: ");
  Serial.print(tps, 2);
  Serial.println(" V");
  tps += 0.1;
  if (tps > 4.5) tps = 0.5; // Reinicia ciclo del TPS

  delay(200); // 5 lecturas por segundo

  sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      
      float acel_lateral_g;
      calcularAceleracionLateral(a.acceleration.x, a.acceleration.y, a.acceleration.z, &acel_lateral_g);
      
      Serial.print("Aceleracion lateral (g): ");
      Serial.println(acel_lateral_g, 3);
     // guardar_datos(acel_lateral_g);

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

}
