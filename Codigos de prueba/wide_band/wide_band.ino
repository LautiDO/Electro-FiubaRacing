void setup() {
  Serial.begin(9600);  // Iniciar comunicación serie
}

void loop() {
  float valor = analogRead(PA0);               // Leer ADC (0–4095)
  float voltaje = (valor / 4095.0) * 3.3;      // Convertir a voltaje
  Serial.println(voltaje);                    // Imprimir voltaje para graficar
  delay(100);                                  // Tiempo entre lecturas
}
