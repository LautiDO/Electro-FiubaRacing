#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

unsigned long t_prev = 0;
float vx = 0;   // velocidad acumulada en X


void setup() {
    Serial.begin(115200);

    Wire.begin();
    
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

}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float acel_lateral_g;
    float ang;
    float modulo;
    float velocidad_x;
    
    //calcularAceleracionLateral(a.acceleration.x, a.acceleration.y, a.acceleration.z, &ang , &modulo, &velocidad_x);

     calcularAceleracionLateral(a.acceleration.x,
                            a.acceleration.y,
                            a.acceleration.z,
                            g.gyro.x,
                            g.gyro.y,
                            g.gyro.z,
                            &ang,
                            &modulo,
                            &velocidad_x);
    //Serial.print("Aceleracion lateral (g): ");
   // Serial.println(acel_lateral_g, 3);
   // Serial.print("modulo");
    //Serial.println(modulo);
    //Serial.print("fase");
   // Serial.println(ang);
    Serial.print("velocidad");
    Serial.println(velocidad_x);
    
}




void calcularAceleracionLateral(float Ax, float Ay, float Az,float Gx, float Gy, float Gz, float* ang, float* modulo, float* vel_x) {

  static unsigned long t_prev = 0;
  static float vx = 0;
  static float pitch = 0;
  static float roll = 0;

  const float alpha = 0.98;   // filtro complementario

  // ---- Tiempo ----
  unsigned long t_now = millis();
  float dt = (t_now - t_prev) / 1000.0;
  if (dt <= 0) return;
  t_prev = t_now;

  // ---- Ángulos por acelerómetro ----
  float pitch_acc = atan2(-Ax, sqrt(Ay * Ay + Az * Az));
  float roll_acc  = atan2(Ay, Az);

  // ---- Integración del giroscopio ----
  pitch += Gy * dt;   // ojo: depende de orientación del sensor
  roll  += Gx * dt;

  // ---- Filtro complementario ----
  pitch = alpha * pitch + (1 - alpha) * pitch_acc;
  roll  = alpha * roll  + (1 - alpha) * roll_acc;

  // ---- Gravedad estimada ----
  float gx = 9.81 * -sin(pitch);
  float gy = 9.81 *  sin(roll) * cos(pitch);
  float gz = 9.81 *  cos(roll) * cos(pitch);

  // ---- Aceleración lineal ----
  float ax_lin = Ax - gx;
  float ay_lin = Ay - gy;
  float az_lin = Az - gz;

  // ---- Módulo y ángulo horizontal ----
  float a_mod = sqrt(ax_lin * ax_lin + ay_lin * ay_lin);
  *modulo = a_mod / 9.81;

  float ang_rad = atan2(ay_lin, ax_lin);
  *ang = ang_rad * 180.0 / PI;

  // ---- Integración para velocidad ----
  vx += ax_lin * dt;
  *vel_x = vx;
}


/*
//-----------------------IMU-------------------------
void calcularAceleracionLateral(float Ax, float Ay, float Az, float* ang , float* modulo, float* velocidad_x) {
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
 // *acel_lateral_g = ay_lin / 9.81;

  float a_mod = sqrt(ax_lin*ax_lin + ay_lin*ay_lin);
  *modulo = a_mod / 9.81;

  float ang_rad = atan2(ay_lin, ax_lin);        // radianes
  *ang = ang_rad * 180.0 / PI;        // grados

  unsigned long t_now = millis();
  float dt = (t_now - t_prev) / 1000.0;
  t_prev = t_now;

   // --- Integración para velocidad en X ---
  vx += ax_lin * dt;     // m/s
  *velocidad_x = vx;  
}
*/
