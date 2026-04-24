import serial
import time
import csv
import os
import struct
import sys
from datetime import datetime

# ==========================================
# CONFIGURACIÓN
# ==========================================
COM_PORT_MANUAL = "/dev/ttyUSB0"
BAUDRATE = 115200

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

# ==========================================
# DECODIFICACIÓN Y GUARDADO
# ==========================================
def procesar_datos(payload, timestamp_float, csv_writer):
    try:
        # ✅ CORREGIDO: BIG ENDIAN (>)
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

# ==========================================
# EJECUCIÓN PRINCIPAL
# ==========================================
if __name__ == "__main__":
    home_dir = os.path.expanduser("~")
    nombre_archivo = os.path.join(home_dir, f"datalog_r1000_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

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

            sys.stdout.write("Conexión OK. Grabando datos en tiempo real... (Ctrl+C para detener)\n")
            sys.stdout.write("TIEMPO RPI   | ECU_ms   | RPM   | AFR  | TPS    | BAT    | TEMP\n")
            sys.stdout.write("-" * 75 + "\n")
            sys.stdout.flush()

            packet_online = armar_trama(bytes([6, 0, 0]))
            
            buffer_rx = bytearray()
            contador_display = 0
            
            last_request_time = 0
            period = 0.04  # 25 Hz
            timestamp_read = datetime.now()

            while True:
                now = time.perf_counter()

                # 1. Petición controlada por tiempo
                if now - last_request_time >= period:
                    ser.write(packet_online)
                    last_request_time = now

                # 2. Lectura no bloqueante
                bytes_to_read = ser.in_waiting
                if bytes_to_read:
                    timestamp_read = datetime.now()
                    data = ser.read(bytes_to_read)
                    buffer_rx.extend(data)

                # 3. Procesamiento de buffer
                while b'\xFF\x7F' in buffer_rx and b'\x7F\xFF' in buffer_rx:
                    start_idx = buffer_rx.find(b'\xFF\x7F')
                    end_idx = buffer_rx.find(b'\x7F\xFF', start_idx)

                    if start_idx != -1 and end_idx != -1:
                        trama_completa = buffer_rx[start_idx : end_idx + 2]
                        buffer_rx = buffer_rx[end_idx + 2:]

                        if len(trama_completa) > 12 and trama_completa[2] == 180:
                            payload = trama_completa[10:-2]
                            datos = procesar_datos(payload, timestamp_read, writer)

                            if datos:
                                contador_display += 1
                                if contador_display >= 5:
                                    ecu_ms, r, a, t, b, tm = datos
                                    hora_str = timestamp_read.strftime('%H:%M:%S.%f')[:-3]
                                    out_str = f"{hora_str} | {ecu_ms:<8} | {r:<5} | {a:<4.1f} | {t:>5.1f}% | {b:>5.2f}V | {tm:>4.1f}°C\n"
                                    sys.stdout.write(out_str)
                                    contador_display = 0
                    else:
                        break
                
                # 4. Evitar CPU al 100%
                time.sleep(0.001)

    except KeyboardInterrupt:
        sys.stdout.write(f"\nDetenido. Datos guardados intactos en {nombre_archivo}\n")
    except Exception as e:
        sys.stdout.write(f"\nError general: {e}\n")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()