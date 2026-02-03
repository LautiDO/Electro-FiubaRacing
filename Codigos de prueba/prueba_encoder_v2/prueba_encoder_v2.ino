// Pines para STM32 (basado en tus variables PA0, PA1)
const int pinA = PA0;
const int pinB = PA1;

volatile long pulsos = 0; // Cambiado a long para mayor rango
long anteriorPulsos = 0;
float angulo = 0;

// Constante de conversión: 360 grados / 600 pulsos
const float factorConversion = 0.6; 

void setup() {
  pinMode(pinA, INPUT_PULLUP); // Recomendado usar Pullup interno
  pinMode(pinB, INPUT_PULLUP);

  Serial.begin(115200); // Sube la velocidad para no perder tiempo en el loop

  // Interrupción en modo CHANGE para mayor fluidez o FALLING para mantener los 600
  attachInterrupt(digitalPinToInterrupt(pinA), encoderISR, FALLING);
  
  Serial.println("Sensor de Volante Inicializado - Calibrado en 0");
}

void loop() {
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
}

void encoderISR() {
  // Leemos B para determinar sentido
  if (digitalRead(pinB) == HIGH) {
    pulsos++;
  } else {
    pulsos--;
  }
}
