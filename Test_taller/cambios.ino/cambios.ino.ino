int t1 = 329;
int t2 = 495;
int t3 = 577;
int t4 = 614;
int t5 = 625;
int t6 = 637;
int neutro = 670;

int tolerancia = 5;

int pinAnalogico = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {

  int valor = analogRead(pinAnalogico);

  Serial.print("Valor: ");
  Serial.print(valor);
  Serial.print(" -> ");

  if (valor >= (t1 - tolerancia) && valor <= (t1 + tolerancia)) {
    Serial.println("Primera");
  }
  else if (valor >= (t2 - tolerancia) && valor <= (t2 + tolerancia)) {
    Serial.println("Segunda");
  }
  else if (valor >= (t3 - tolerancia) && valor <= (t3 + tolerancia)) {
    Serial.println("Tercera");
  }
  else if (valor >= (t4 - tolerancia) && valor <= (t4 + tolerancia)) {
    Serial.println("Cuarta");
  }
  else if (valor >= (t5 - tolerancia) && valor <= (t5 + tolerancia)) {
    Serial.println("Quinta");
  }
  else if (valor >= (t6 - tolerancia) && valor <= (t6 + tolerancia)) {
    Serial.println("Sexta");
  }
  else if (valor >= (neutro - tolerancia) && valor <= (neutro + tolerancia)) {
    Serial.println("Neutro");
  }

  delay(200);
}
