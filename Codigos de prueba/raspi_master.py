import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 100000 # Velocidad baja para pruebas (0.1 MHz)

# Mandamos 5 bytes "basura" para que el reloj genere 5 pulsos 
# y el STM32 nos devuelva sus 5 datos.
respuesta = spi.xfer2([0, 0, 0, 0, 0])

print("Los 5 datos del STM32 son:", respuesta)

spi.close()