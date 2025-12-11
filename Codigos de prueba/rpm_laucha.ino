// Pines STM32 (Arduino_Core_STM32)
const int pinOut = PA8;     // salida de señal cuadrada
const int PIN_RPM = PB6;    // entrada con interrupción EXTI5

const unsigned long pulseWidth = 7000;  // 7000 us HIGH + 7000 us LOW ≈ 71 Hz

volatile unsigned long previousmicros = 0;
volatile unsigned long periodo = 0;

float rpm_med = 0;

void setup() {
  Serial.begin(115200);

  pinMode(pinOut, OUTPUT);
  pinMode(PIN_RPM, INPUT_PULLUP);

  // PB5 usa EXTI5 → ok
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), RPM, FALLING);
}


void loop() {

  // Genero señal cuadrada
  digitalWrite(pinOut, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pinOut, LOW);
  delayMicroseconds(pulseWidth);

  // Copio periodo de forma segura
  noInterrupts();
  unsigned long p = periodo;
  interrupts();

  

  if (p > 0) {
    rpm_med = 60000000.0 / p;
  }

  Serial.print("Periodo (us): ");
  Serial.print(p);
  Serial.print("   RPM: ");
  Serial.println(rpm_med);
}

// --------------------------- ISR ---------------------------
void RPM() {
  unsigned long ahora = micros();
  if (previousmicros > 0) {
    periodo = ahora - previousmicros;
  }
  previousmicros = ahora;
}
