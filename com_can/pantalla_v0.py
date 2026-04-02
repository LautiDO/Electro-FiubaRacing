import can
import time

# --- FUNCIÓN UNIVERSAL DE EMPAQUETAMIENTO ---
def preparar_payload(current_payload, valor, offset, length, mask, multiplier, divisor, adder):
    """
    Inserta un dato en el payload de 8 bytes siguiendo las regla de la tabla.
    """
    # Escalamiento 
    raw = int((valor - adder) * divisor / multiplier)
    
    # Mascara de bits
    raw &= mask

    # Bytes (Big Endian)
    if length == 1:
        current_payload[offset] = raw & 0xFF
    elif length == 2:
        current_payload[offset] = (raw >> 8) & 0xFF     # Byte mas significativo
        current_payload[offset + 1] = raw & 0xFF       # Byte menos significativo
        
    return current_payload

# --- CONFIGURACIÓN DEL BUS ---
try:
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
except OSError:
    print("ERROR: can0 no encontrado. Ejecuta: sudo ip link set can0 up type can bitrate 500000")
    exit()

# --- EJECUCIÓN DE PRUEBA ---
try:
    print(">>> Iniciando Simulación de Telemetría [ID 0x640 y 0x649]...")
    print(">>> Abre otra terminal y corre 'candump can0' para verificar.")
    
    while True:
        # --- MOTOR (ID 0x640) ---
        payload_640 = [0] * 8
        # RPM
        payload_640 = preparar_payload(payload_640, 4500, 0, 2, 0xFFFF, 1, 6, 0)
        # TPS 
        payload_640 = preparar_payload(payload_640, 85, 6, 2, 0xFFFF, 1, 1, 0)
        
        msg1 = can.Message(arbitration_id=0x640, data=payload_640, is_extended_id=False)
        bus.send(msg1)

        # --- TEMPERATURAS (ID 0x649) ---
        payload_649 = [0] * 8
        # Coolant
        payload_649 = preparar_payload(payload_649, 95, 0, 1, 0xFF, 10, 1, -400)
        # Oil
        payload_649 = preparar_payload(payload_649, 110, 1, 1, 0xFF, 10, 1, -400)
        # Fuel
        payload_649 = preparar_payload(payload_649, 30, 2, 1, 0xFF, 10, 1, -400)
        
        msg2 = can.Message(arbitration_id=0x649, data=payload_649, is_extended_id=False)
        bus.send(msg2)

        # Monitor de consola
        print(f"TX 640 (RPM/Thr): {payload_640} | TX 649 (Temps): {payload_649}")
        
        time.sleep(0.5) # Enviamos cada 500ms para que sea fácil de leer

except KeyboardInterrupt:
    print("\nSimulación finalizada por el usuario.")
    bus.shutdown()
