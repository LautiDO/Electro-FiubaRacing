import spidev
import time


HEADER1 = 0xAA
HEADER2 = 0x55
FRAME_SIZE = 8

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 100000

ultimo_frame_id = None

def checksum(data):
    cs = 0
    for b in data:
        cs ^= b
    return cs

while True:

    # leer una trama
    data = spi.xfer2([0]*FRAME_SIZE)

    # -------- HEADER --------
    if data[0] != HEADER1 or data[1] != HEADER2:
        print("Header incorrecto:", data)
        continue

    # -------- CHECKSUM --------
    cs_calculado = checksum(data[0:7])
    cs_recibido = data[7]

    if cs_calculado != cs_recibido:
        print("Error checksum:", data)
        continue

    # -------- DECODIFICAR --------
    frame_id = data[2]

    rpm = data[3] | (data[4] << 8)
    temp = data[5]
    pres = data[6]

    # -------- DETECTAR PERDIDA --------
    if ultimo_frame_id is not None:

        esperado = (ultimo_frame_id + 1) & 0xFF

        if frame_id != esperado:
            print("Trama perdida! Esperado:", esperado, "Recibido:", frame_id)

    ultimo_frame_id = frame_id

    # -------- PRINT --------
    print("Frame:", frame_id,
          "RPM:", rpm,
          "Temp:", temp,
          "Pres:", pres)
    
    time.sleep(0.01)