# Simulador de Datos y Emulador de Puerto Serie

Buenas! Bienvenido al simulador. La idea de esta guía es que puedas ponerlo a andar sin volverte loco, aunque no sepas nada de programación.

## Qué es esto y para qué sirve?

Este proyecto tiene dos partes principales que te ayudan a probar la telemetría sin necesidad de tener el auto andando:

**Generador de Datos** `simulador_datos.py`: Es un programa que inventa datos de sensores del auto (RPM, temperatura, etc.) y los guarda en archivos. Sirve para tener archivos de prueba y subirlos al programa de telemetría.

**Emulador de Arduino** `emulador_arduino.py`: Este es un poco más avanzado. Simula que tenés un Arduino conectado por USB a la compu, mandando datos en tiempo real. Sirve para probar la aplicación de telemetría como si estuviera recibiendo información en vivo desde el auto.

## Requisitos
Antes de arrancar, una vez que tengas el repositorio [clonado](https://docs.github.com/es/repositories/creating-and-managing-repositories/cloning-a-repository), necesitás tener un par de cosas instaladas en tu compu.


### Para el simulador de datos:
**Python 3**: Versión 3.10 o más nueva.
Para chequear si ya tenés Python y Pip, abrí una terminal (o commnand prompt, el equivalente a una terminal en Windows) y escribí:
```
#Linux
python3 --version

#Windows
py --version

#Ambos
pip3 --version
```

**Librería Pyserial**: Se instala con pip. (Únicamente necesario para Windows)
```
pip install pyserial
```
### Para el emulador de puertos
_Si solo te interesa ver los datos simulados por pantalla y no te querés meter con puertos virtuales para recibirlos en otra aplicación,
te podes saltear este paso._

- **Linux:** Vas a necesitar socat. Aunque probablemente venga instalado por defecto, te podés asegurar corriendo en la terminal
```
sudo apt install socat
```
- **Windows:** Se necesita una herramienta llamada [com0com](https://sourceforge.net/projects/com0com/). Se tiene que usar para
crear los puertos virtuales que luego se usan en el emulador (por defecto están puestos COM3 y COM4, pero pueden ser distintos).

## Cómo lo uso?
1. **Generar datos** `simulador_datos.py`:

Tiene dos modos de uso.
- `python3 Simulador/simulador_datos.py` (terminal de Linux) o `py .\Simulador\simulador_datos.py` (command prompt de Windows, estando parado en la carpeta de Electro-FiubaRacing) 

Imprime datos constantemente por pantalla
- `python3 Simulador/simulador_datos.py [formato] [tiempo] [nombre]` (Linux) o `py .\Simulador\simulador_datos.py [formato] [tiempo] [nombre]` (Windows)
      
  - **formato**: puede ser `csv` (comma separated values) o `arduino` (un formato más simple de leer, que sigue el formato usado en
        codigo_general.ino)

  - **tiempo**: la duración, en segundos, de los datos que querés generar (por defecto, 300s). Ojo que no va a tardar ese tiempo en ejecutarse,
      simplemente genera al momento un tiempo determinado de datos.
  - **Nombre**: El nombre que querés para tu archivo, sin la extensión (por defecto, datos_prueba).
  Se va a crear en una subcarpeta llamada _pruebas_
   
   **Ejemplos**:

   ```
       #Genera un CSV de 60 segundos llamado 'test_rapido.csv'
       python3 Simulador/simulador_datos.py csv 60 test_rapido

       #Genera un TXT de 10 minutos (600s) con formato Arduino llamado 'prueba_larga.txt'
       python3 Simulador/simulador_datos.py arduino 600 prueba_larga
  ```


2. **Emular un Arduino en tiempo real** `emulador_arduino.py`:

Esto crea un puerto serie virtual y empieza a "transmitir" datos en tiempo real, como si fuera el auto.
Para correrlo, simplemente corré el programa en tu IDE, o ejecutá el archivo desde la terminal:

`python3 Simulador/emulador_arduino.py` (Linux) o `py .\Simulador\emulador_arduino.py` (Windows)

Al arrancar, te va a decir qué puertos creó. Por ejemplo, en Linux:
```
Puertos virtuales creados:
   Arduino simulado: /tmp/ttyArduino
   Lector: /tmp/ttyReader
```
La aplicación de telemetría debería "escuchar" del puerto lector (_/tmp/ttyReader_ en este caso). El simulador se queda corriendo y mandando datos hasta que lo frenes.

## Cómo se si está funcionando?
Podés "espiar" lo que el emulador está mandando.

- **En Linux**: Abrí otra terminal y usa el comando _cat_:

`cat < /tmp/ttyReader`

- **En Windows**: Abrí un Command Prompt,
y usa el comando _copy_:

`copy COM4: CON:`

(_Reemplazá /tmp/ttyReader o COM4 por el puerto "Lector" que te indicó el emulador_)

Deberías ver líneas de datos apareciendo constantemente.

Con esto ya deberías tener todo lo necesario para usar el simulador. Si tenés alguna duda o sugerencia,
no dudes en contactar al equipo de telemetría!

