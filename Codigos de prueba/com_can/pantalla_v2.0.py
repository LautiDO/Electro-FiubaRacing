
import can
import time

# --- MACROS DE FORMATO Y ESCALAMIENTO ---
ID_MOTOR      = 0x640
ID_FUEL_PRESS = 0x641
ID_OIL_PRESS  = 0x644
ID_TEMP       = 0x649
ID_GEAR       = 0x64D

LEN_16BIT  = 2 # Longitud en bytes
LEN_8BIT   = 1

MASK_16BIT = 0xFFFF
MASK_8BIT  = 0xFF
MASK_4BIT  = 0x0F # Máscara para el cambio de (4 bits)

MULT_BASE = 1
MULT_TEMP = 1
DIV_BASE  = 1
DIV_RPM  = 2
ADD_BASE  = 0
ADD_TEMP  =40

#Datos a cambiar
data_rpm = 1500	
data_tps = 56
data_coolant = 57 
data_fuel = 62
data_oil = 23
data_oil_press = 20
data_fuel_press = 30
data_gear = 0
# --- CONFIGURACIÓN DE BUS ---
CAN_CHANNEL  = 'can0'
BITRATE      = 1000000 
SEND_DELAY   = 0.5 # Frecuencia, esto probablemente se va tener que adecuar a la pantalla 

# --- Diccionario con los parametros {'off': offset, 'len': length, 'mask': mask, 'mult': multiplier, 'div': divisor, 'add': adder} ----------------
SIG_RPM        = {'off': 0, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': MULT_BASE, 'div': 2,  'add': ADD_BASE}
SIG_TPS        = {'off': 6, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': MULT_BASE, 'div': 1000, 'add': ADD_BASE}
SIG_FUEL_PRESS = {'off': 4, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': 1000, 'div': 1, 'add': ADD_BASE}
SIG_OIL_PRESS  = {'off': 6, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': 1000, 'div': 1, 'add': ADD_BASE}
SIG_COOLANT    = {'off': 0, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_OIL        = {'off': 1, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_FUEL       = {'off': 2, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_GEAR       = {'off': 6, 'len': LEN_8BIT,  'mask': MASK_4BIT,  'mult': MULT_BASE, 'div': DIV_BASE, 'add': ADD_BASE}

def preparar_payload(payload, valor, sig): # En valor le pasamo el dator recolectado de la ECU
    raw = int((valor + sig['add']) * sig['div'] * sig['mult'])
    raw &= sig['mask']

    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']] = (raw >> 8) & 0xFF 
        payload[sig['off'] + 1] = raw & 0xFF 
        
    return payload

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
        # --- Parametros del MOTOR (ID 0x640) ---
        payload_640 = [0] * 8 
        payload_640 = preparar_payload(payload_640, data_rpm, SIG_RPM) 
        payload_640 = preparar_payload(payload_640, data_tps, SIG_TPS) 
        data_rpm = data_rpm +1000    
             
        msg1 = can.Message(arbitration_id=ID_MOTOR, data=payload_640, is_extended_id=False)
        bus.send(msg1)

        # --- TEMPERATURAS (0x649) ---
        payload_649 = [0] * 8 
        payload_649 = preparar_payload(payload_649, data_coolant, SIG_COOLANT) 
        payload_649 = preparar_payload(payload_649, data_oil, SIG_OIL) 
        payload_649 = preparar_payload(payload_649, data_fuel, SIG_FUEL) 

        msg2 = can.Message(arbitration_id=ID_TEMP, data=payload_649, is_extended_id=False)
        bus.send(msg2)
	# --- ENVIO DE LAS PRESIONES ---
        payload_644 = [0] * 8 
        payload_641 = [0] * 8 

        payload_641 = preparar_payload(payload_641, data_fuel_press, SIG_FUEL_PRESS) 
        payload_644 = preparar_payload(payload_641, data_oil_press, SIG_OIL_PRESS) 

        msg3 = can.Message(arbitration_id=ID_FUEL_PRESS, data=payload_641, is_extended_id=False)
        bus.send(msg3)
        msg4 = can.Message(arbitration_id=ID_OIL_PRESS, data=payload_644, is_extended_id=False)
        bus.send(msg4)
 
        payload_64D = [0] * 8
        payload_64D = preparar_payload(payload_64D, data_gear, SIG_GEAR) 

        msg5 = can.Message(arbitration_id=ID_GEAR, data=payload_64D, is_extended_id=False)
        bus.send(msg5)

        print(f"rpm: {data_rpm} TX {hex(ID_MOTOR)}: {payload_640} | TX {hex(ID_TEMP)}: {payload_649}")
        time.sleep(SEND_DELAY) 
		
except KeyboardInterrupt:
    print("\nSimulación finalizada por el usuario.")
    bus.shutdown()
