#include <TinyGPS.h>   // O TinyGPSPlus si prefieres la versión más nueva

TinyGPS gps;

// En STM32, Serial1 usa pines: TX = PA9, RX = PA10
// Conecta GPS TX → PA10 (RX Blue Pill)
// GPS RX → PA9 (TX Blue Pill) (si quieres enviarle comandos, opcional)

void setup() {
  Serial.begin(115200);       // Puerto USB para debug
  Serial2.begin(9600);        // UART1 para GPS (Neo-7M normalmente usa 9600 baud)}
  
  Serial.println("Inicializando GPS...");
}

void loop() {
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Parsear datos durante 1 segundo
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (Serial2.available()) {
      //Serial.print(Serial1.available());
      //Serial.println("Test1");

      char c = Serial2.read();
      if (gps.encode(c))  // ¿Llego una nueva sentencia válida?
        newData = true;
        //Serial.println(newData);
        //Serial.println("AGHAS");
    }
  }

  if (newData==0) {
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
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.print("CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}

