import serial
import time
import csv
import os
import struct
import sys
import can
from datetime import datetime

# --- CONFIGURACIÓN ECU ---
COM_PORT_MANUAL = "/dev/ttyUSB0"
BAUDRATE = 115200

# --- MACROS Y Diccinario de parametros ---
ID_MOTOR      = 0x640
ID_TEMP       = 0x649

LEN_16BIT  = 2
LEN_8BIT   = 1
MASK_16BIT = 0xFFFF
MASK_8BIT  = 0xFF

MULT_BASE = 1
MULT_TEMP = 10
DIV_BASE  = 1
DIV_RPM   = 6
ADD_BASE  = 0
ADD_TEMP  = -400

CAN_CHANNEL  = 'can0'

# Aqui agregamos los valores que necesitamos enviar para la pantalla
SIG_RPM     = {'off': 0, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': MULT_BASE, 'div': DIV_RPM,  'add': ADD_BASE}
SIG_TPS     = {'off': 6, 'len': LEN_16BIT, 'mask': MASK_16BIT, 'mult': MULT_BASE, 'div': DIV_BASE, 'add': ADD_BASE}
SIG_COOLANT = {'off': 0, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_OIL     = {'off': 1, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}
SIG_FUEL    = {'off': 2, 'len': LEN_8BIT,  'mask': MASK_8BIT,  'mult': MULT_TEMP, 'div': DIV_BASE, 'add': ADD_TEMP}

def calcular_checksum(trama_sin_checksum):
    total = sum(trama_sin_checksum) & 0xFFFFFFFF
    return struct.pack('<I', total)

def armar_trama(cmd_bytes, payload=b''):
    header = b'\xFF\x7F'
    footer = b'\x7F\xFF'
    n_bytes_val = 10 + len(payload)

    componentes_suma = (
        header + bytes([cmd_bytes[0]]) + bytes([cmd_bytes[1]]) +
        bytes([cmd_bytes[2]]) + bytes([n_bytes_val]) + payload + footer
    )
    chk_bytes = calcular_checksum(componentes_suma)

    trama = bytearray()
    trama += header + bytes([cmd_bytes[0]]) + bytes([cmd_bytes[1]])
    trama += chk_bytes
    trama += bytes([cmd_bytes[2]]) + bytes([n_bytes_val]) + payload + footer
    return trama

def procesar_datos(payload, timestamp_float, csv_writer):
    try:
        ecu_time_ms = struct.unpack_from('>I', payload, 0)[0]
        rpm = struct.unpack_from('>h', payload, 8)[0]
        afr = struct.unpack_from('B', payload, 10)[0] / 10.0
        tps = struct.unpack_from('>h', payload, 14)[0] / 10.0
        temp_motor = struct.unpack_from('>h', payload, 18)[0] / 10.0
        vbat = struct.unpack_from('>h', payload, 22)[0] / 100.0

        csv_writer.writerow([timestamp_float.strftime('%H:%M:%S.%f'), ecu_time_ms, rpm, afr, tps, vbat, temp_motor])
        return ecu_time_ms, rpm, afr, tps, vbat, temp_motor
    except struct.error:
        pass
    except Exception:
        pass
    return None

#------------- CAN -------------------------
def preparar_payload(payload, valor, sig):
    raw = int((valor - sig['add']) * sig['div'] / sig['mult'])
    raw &= sig['mask']

    if sig['len'] == 1:
        payload[sig['off']] = raw & 0xFF
    elif sig['len'] == 2:
        payload[sig['off']] = (raw >> 8) & 0xFF 
        payload[sig['off'] + 1] = raw & 0xFF 
    return payload

if __name__ == "__main__":
    home_dir = os.path.expanduser("~")
    nombre_archivo = os.path.join(home_dir, f"datalog_r1000_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    # ----------------Init config Bus CAN---------------------------------------

    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, bustype='socketcan')
        sys.stdout.write(f"Bus CAN iniciado\n")
    except OSError:
        sys.stdout.write(f"Bus CAN no encontrado\n")
        exit()

    # -----------------End config Bus CAN-------------------------------------

    try:
        ser = serial.Serial(COM_PORT_MANUAL, BAUDRATE, timeout=0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        sys.stdout.write(f"Abriendo {COM_PORT_MANUAL}...\n")
        
        with open(nombre_archivo, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['TIME', 'TIME_ECU_MS', 'RPM', 'AFR', 'TPS', 'BATERIA', 'TEMP_MOTOR'])

            sys.stdout.write("Enviando Handshake...\n")
            ser.write(armar_trama(bytes([0, 0, 0])))
            time.sleep(0.05)
            ser.reset_input_buffer()

            sys.stdout.write("Conexión OK. Puente ECU -> CAN activo. (Ctrl+C para detener)\n")
            sys.stdout.write("TIEMPO RPI   | ECU_ms   | RPM   | AFR  | TPS    | BAT    | TEMP\n")
            sys.stdout.write("-" * 75 + "\n")
            sys.stdout.flush()

            packet_online = armar_trama(bytes([6, 0, 0]))
            
            buffer_rx = bytearray()
            contador_display = 0
            
            last_request_time = 0
            period = 0.04  # 25 Hz El periodo de recpcion de los datos de la ECU
            timestamp_read = datetime.now()

            while True:
                now = time.perf_counter()

                if now - last_request_time >= period:
                    ser.write(packet_online)
                    last_request_time = now

                bytes_to_read = ser.in_waiting
                if bytes_to_read:
                    timestamp_read = datetime.now()
                    data = ser.read(bytes_to_read)
                    buffer_rx.extend(data)

                while b'\xFF\x7F' in buffer_rx and b'\x7F\xFF' in buffer_rx:
                    start_idx = buffer_rx.find(b'\xFF\x7F')
                    end_idx = buffer_rx.find(b'\x7F\xFF', start_idx)

                    if start_idx != -1 and end_idx != -1:
                        trama_completa = buffer_rx[start_idx : end_idx + 2]
                        buffer_rx = buffer_rx[end_idx + 2:]

                        if len(trama_completa) > 12 and trama_completa[2] == 180:
                            payload_ecu = trama_completa[10:-2]
                            datos = procesar_datos(payload_ecu, timestamp_read, writer)

                            # Evio mediante el bus CAN
                            if datos:
                                ecu_ms, rpm, afr, tps, vbat, temp_motor = datos
                                
                                # --- Parametros del MOTOR  ---
                                payload_640 = [0] * 8 
                                payload_640 = preparar_payload(payload_640, rpm, SIG_RPM) 
                                payload_640 = preparar_payload(payload_640, tps, SIG_TPS) 
                                bus.send(can.Message(arbitration_id=ID_MOTOR, data=payload_640, is_extended_id=False))

                                # --- Parametros de temperaturas ---
                                payload_649 = [0] * 8 
                                payload_649 = preparar_payload(payload_649, temp_motor, SIG_COOLANT) 
                                payload_649 = preparar_payload(payload_649, 0, SIG_OIL) 
                                payload_649 = preparar_payload(payload_649, 0, SIG_FUEL) # Al enviar 0 lo que se esta enviando es el 0x28 por el escalamiento
                                bus.send(can.Message(arbitration_id=ID_TEMP, data=payload_649, is_extended_id=False))

                                # Impresion de los datos recibidos de la ECU
                                contador_display += 1
                                if contador_display >= 5:
                                    hora_str = timestamp_read.strftime('%H:%M:%S.%f')[:-3]
                                    out_str = f"{hora_str} | {ecu_ms:<8} | {rpm:<5} | {afr:<4.1f} | {tps:>5.1f}% | {vbat:>5.2f}V | {temp_motor:>4.1f}°C\n"
                                    sys.stdout.write(out_str)
                                    contador_display = 0
                    else:
                        break
                
                time.sleep(0.001)

    except KeyboardInterrupt:
        sys.stdout.write(f"\nDetenido. Datos guardados intactos en {nombre_archivo}\n")
    except Exception as e:
        sys.stdout.write(f"\nError general: {e}\n")
    finally:
        if 'bus' in locals():
            bus.shutdown()
        if 'ser' in locals() and ser.is_open:
            ser.close()