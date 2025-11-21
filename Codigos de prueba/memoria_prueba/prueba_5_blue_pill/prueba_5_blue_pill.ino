#include <SPI.h>		
#include <SD.h>		

#define SSpin PA4		// Slave Select en pin digital PA4

File archivo;			// objeto archivo del tipo File
char nombre_archivo[] = "aceang.txt"; 
int dato = 0;
bool write = true; // elegir entre leer o escribir

void setup() {

  Serial.begin(9600);

  Serial.println("Inicializando tarjeta ...");
  //if (!SD.begin(SSpin)) {			
    //Serial.println("fallo en inicializacion !");
    //return;					
  //}
  //Serial.println("inicializacion correcta");	
  
// Inicialización
  if (!SD.begin(SSpin)) {
    Serial.println("Fallo: No se detecta la tarjeta o el formato es incorrecto.");
    // Bucle infinito para detener la ejecución si no hay SD
    while (1); 
  }
  Serial.println("SD Iniciada correctamente.");
  // lectura del archivo
 if(!write){
  archivo = SD.open(nombre_archivo);
    if (archivo) {		
      while (archivo.available()) {		// mientras exista contenido en el archivo
        Serial.write(archivo.read());  		// lectura de a un caracter por vez
      }
      archivo.close();				
    } else {
      Serial.println("error en la apertura de prueba.txt");
    }
  }
}

void loop() {

  //escritura del archivo
  if(write){
    archivo = SD.open(nombre_archivo, FILE_WRITE);	

    if (archivo) {
      Serial.println("Escribiendo");
      archivo.print("RPM ");
      archivo.println(dato);
      archivo.close(); 
    } else {
      Serial.println("error en apertura del archivo");
      Serial.println(archivo);
    }
    dato = dato+1;
  }
}