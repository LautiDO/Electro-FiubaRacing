
  float dmin = 0;
  float dmax = 75;
  float dfijo = 0;
  float K = 10; //

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);


}

void loop() {


int r1 = analogRead(PB1); //leo el valor actual de resistencia

Serial.println(r1);

float dmed = map(r1, 4095 , 0 , dmin , dmax); //lo mapeo a valores de distancia
Serial.print("dmed:");
Serial.println(dmed);

float dx = dmed - dfijo; //variacion de distancia
Serial.print("dx:");
Serial.println(dx);


float F_elastica = K * dx;
Serial.print("F_elastica:");
Serial.println(F_elastica);
}
