import can
import time

# --- ID's ---
ID_MOTOR = 0x640
ID_TEMP  = 0x649

# --- MÁSCARAS ---
MASK_16BIT = 0xFFFF
MASK_8BIT  = 0xFF

# --- OFFSETS DE DATOS---
OFF_RPM      = 0
OFF_TPS      = 6
OFF_COOLANT  = 0
OFF_OIL      = 1
OFF_FUEL     = 2

# --- FACTORES DE ESCALAMIENTO Formato: (Multiplier, Divisor, Adder)
# Motor (16-bit)
RPM_CONV  = (1, 6, 0)
TPS_CONV  = (1, 1, 0)

# Temperaturas (8-bit)
TEMP_CONV = (10, 1, -400)

# --- CONFIGURACIÓN DE BUS ---
CAN_CHANNEL  = 'can0'
BITRATE      = 500000 
SEND_DELAY   = 0.5 # Frecuencia, esto probablemente se va tener que adecuar a la pantalla 

def preparar_payload(current_payload, valor, offset, length, mask, multiplier, divisor, adder):
    # Escalamiento
    raw = int((valor - adder) * divisor / multiplier)
    
    # Mascara de bits
    raw &= mask

    # Bytes (Estoy asumiendo Big Endian si no funciona cambiar a Litle Endian)
    if length == 1:
        current_payload[offset] = raw & 0xFF
    elif length == 2:
        current_payload[offset] = (raw >> 8) & 0xFF     # Byte mas significativo
        current_payload[offset + 1] = raw & 0xFF       # Byte menos significativo
        
    return current_payload

#--------------- SIMULACION -------------------------------
# Configuracion del BUS
try:
    bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
except OSError:
    print(f"ERROR: {CAN_CHANNEL} no encontrado. Ejecuta: sudo ip link set {CAN_CHANNEL} up type can bitrate {BITRATE}")
    exit()

try:
    print(f">>> Iniciando Simulación de Telemetría [ID {hex(ID_MOTOR)} y {hex(ID_TEMP)}]...")
    
    while True:
        # --- Parametros del MOTOR---
        payload_640 = [0] * 8        
        payload_640 = preparar_payload(payload_640, 4500, OFF_RPM, 2, MASK_16BIT, *RPM_CONV) # RPM
        payload_640 = preparar_payload(payload_640, 85, OFF_TPS, 2, MASK_16BIT, *TPS_CONV)# TPS

        msg1 = can.Message(arbitration_id=ID_MOTOR, data=payload_640, is_extended_id=False)
        bus.send(msg1)

        # --- TEMPERATURAS  ---
        payload_649 = [0] * 8 
        payload_649 = preparar_payload(payload_649, 95, OFF_COOLANT, 1, MASK_8BIT, *TEMP_CONV) # Coolant
        payload_649 = preparar_payload(payload_649, 110, OFF_OIL, 1, MASK_8BIT, *TEMP_CONV) # Oil
        payload_649 = preparar_payload(payload_649, 30, OFF_FUEL, 1, MASK_8BIT, *TEMP_CONV) # Fuel
        
        msg2 = can.Message(arbitration_id=ID_TEMP, data=payload_649, is_extended_id=False)
        bus.send(msg2)

        print(f"TX {hex(ID_MOTOR)}: {payload_640} | TX {hex(ID_TEMP)}: {payload_649}")
        
        time.sleep(SEND_DELAY) 

except KeyboardInterrupt:
    print("\nSimulación finalizada por el usuario.")
    bus.shutdown()