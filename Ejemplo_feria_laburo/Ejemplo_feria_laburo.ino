#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define BOTON 6
#define BUZZER 4
void setup() {
  lcd.init();
  lcd.backlight();
  
  pinMode(BOTON, INPUT_PULLUP);
  
  pinMode(BUZZER, OUTPUT);

  
}

void loop() {

  lcd.clear();
  
  // 1. Barra de carga
 for (int i =0; i<=4 ;i++){
  for (int j = 0; j<4; j++){
    lcd.print(char(255));}
  delay(1000);  
 }
    
  
  
  delay(random(2000, 5000));
  
  // 3. Señal de inicio
  lcd.clear();
  lcd.setCursor(6, 0);
  lcd.print("YA!");
  digitalWrite(BUZZER, HIGH);
  unsigned long tiempoInicio = millis();
  
  // 4. Esperar pulsación
  while (digitalRead(BOTON) == HIGH) {
    // Espera activa
  }
  digitalWrite(BUZZER, LOW);
  // 5. Calcular y mostrar
  unsigned long tiempoReaccion = millis() - tiempoInicio;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tu tiempo:");
  lcd.setCursor(0, 1);
  lcd.print(tiempoReaccion);
  lcd.print(" ms");
  
  delay(3000);
}
