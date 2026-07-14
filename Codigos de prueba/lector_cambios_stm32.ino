#define PIN_ADC PA4
#define RESOLUCION_ADC 12
#define TOLERANCIA 70
#define NUM_CONFIRMACIONES 4   // lecturas seguidas iguales para confirmar

int t1      = 2060;   // Primera
int t2      = 2995;   // Segunda
int t3      = 3400;   // Tercera
int t4      = 3720;   // Cuarta
int t5      = 3850;   // Quinta
int t6      = 3950;   // Sexta
int neutro  = 4095;   // Neutro

// ---- variables de estado para el debounce ----
int estado_actual = -1;         // último cambio CONFIRMADO
int candidato = -2;             // valor que se está evaluando
int contador_confirmacion = 0;  // cuántas veces seguidas se repitió el candidato

int clasificar_lectura(int valor) {
  if (valor >= (t1 - TOLERANCIA) && valor <= (t1 + TOLERANCIA)) return 1;
  if (valor >= (t2 - TOLERANCIA) && valor <= (t2 + TOLERANCIA)) return 2;
  if (valor >= (t3 - TOLERANCIA) && valor <= (t3 + TOLERANCIA)) return 3;
  if (valor >= (t4 - TOLERANCIA) && valor <= (t4 + TOLERANCIA)) return 4;
  if (valor >= (t5 - TOLERANCIA) && valor <= (t5 + TOLERANCIA)) return 5;
  if (valor >= (t6 - TOLERANCIA) && valor <= (t6 + TOLERANCIA)) return 6;
  if (valor >= (neutro - TOLERANCIA) && valor <= (neutro + TOLERANCIA)) return 0;
  return -1; // en tránsito, no matchea ninguna
}

// Envuelve clasificar_lectura() con el filtro de confirmaciones consecutivas
int leer_cambio_confirmado(int valor_adc) {
  int lectura = clasificar_lectura(valor_adc);

  if (lectura == -1) {
    // en tránsito: se resetea el conteo, no se confirma nada
    contador_confirmacion = 0;
    candidato = -2;
    return estado_actual;
  }

  if (lectura == candidato) {
    contador_confirmacion++;
  } else {
    candidato = lectura;
    contador_confirmacion = 1;
  }

  if (contador_confirmacion >= NUM_CONFIRMACIONES) {
    estado_actual = candidato; // se confirma (aunque sea el mismo que ya tenía)
  }

  return estado_actual;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Inicio ADC");
}

void loop() {
  uint16_t adc = analogRead(PIN_ADC);
  float voltaje = adc * 3.3 / 4095.0;

  int cambio_confirmado = leer_cambio_confirmado(adc);

  Serial.print("crudo=");
  Serial.print(clasificar_lectura(adc));
  Serial.print("  confirmado=");
  Serial.print(cambio_confirmado);
  Serial.print("  ADC=");
  Serial.print(adc);
  Serial.print("  V=");
  Serial.println(voltaje, 3);

  delay(200);
}
