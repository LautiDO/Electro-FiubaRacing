import serial
import serial.tools.list_ports
import struct
import time
import csv
from datetime import datetime

# ==========================================
# CONFIGURACIÓN
# ==========================================
COM_PORT_MANUAL = "COM6"  # Tu puerto
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
def procesar_datos(payload, csv_writer):
    try:
        # --- LECTURA BIG ENDIAN (>) ---
        
        # Byte 8: RPM (Int16)
        rpm = struct.unpack_from('>h', payload, 8)[0]
        
        # Byte 10: AFR / LAMBDA (UInt8 - 1 Byte) -> / 10.0
        afr = struct.unpack_from('B', payload, 10)[0] / 10.0
        
        # Byte 14: TPS (Int16) -> / 10.0
        tps = struct.unpack_from('>h', payload, 14)[0] / 10.0
        
        # Byte 18: Temp Motor (Int16) -> / 10.0
        temp_motor = struct.unpack_from('>h', payload, 18)[0] / 10.0
        
        # Byte 22: Batería (Int16) -> / 100.0
        vbat = struct.unpack_from('>h', payload, 22)[0] / 100.0
        
        # Timestamp actual
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 1. MOSTRAR EN PANTALLA (Sin MAP)
        print(f"{timestamp} | RPM: {rpm:<5} | AFR: {afr:<4.1f} | TPS: {tps:>5.1f}% | Bat: {vbat:>5.2f} V | T.Mot: {temp_motor:>4.1f}°C")
        
        # 2. GUARDAR EN CSV
        csv_writer.writerow([timestamp, rpm, afr, tps, vbat, temp_motor])
        
    except Exception as e:
        print(f"Error procesando: {e}")

# ==========================================
# EJECUCIÓN
# ==========================================
if __name__ == "__main__":
    # Generar nombre de archivo único con fecha y hora
    nombre_archivo = f"datalog_r1000_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    try:
        # Abrir puerto Serial
        ser = serial.Serial(COM_PORT_MANUAL, BAUDRATE, timeout=1.0)
        print(f"Abriendo {COM_PORT_MANUAL}...")
        
        # Abrir archivo CSV para escritura
        print(f"Creando archivo: {nombre_archivo}")
        with open(nombre_archivo, mode='w', newline='') as f:
            writer = csv.writer(f)
            
            # Escribir encabezados del CSV
            writer.writerow(['TIEMPO', 'RPM', 'AFR', 'TPS', 'BATERIA', 'TEMP_MOTOR'])
            
            # --- 1. HANDSHAKE ---
            print("Enviando Handshake...")
            ser.write(armar_trama(bytes([0, 0, 0])))
            time.sleep(0.5)
            ser.read(ser.in_waiting) 
            
            print("Conexión OK. Grabando datos... (Presiona Ctrl+C para detener)\n")
            print("TIEMPO          | RPM   | AFR  | TPS    | BATERIA  | TEMP")
            print("-" * 75)

            # --- 2. BUCLE DE DATOS ---
            packet_online = armar_trama(bytes([6, 0, 0]))
            
            while True:
                ser.write(packet_online)
                
                # Buffer de lectura
                buffer = ser.read(150) 
                
                if buffer and len(buffer) > 12:
                    start_idx = buffer.find(b'\xFF\x7F')
                    if start_idx != -1 and (start_idx + 12) < len(buffer):
                        if buffer[start_idx + 2] == 180:
                            payload_start = start_idx + 10
                            payload = buffer[payload_start:-2]
                            
                            # Pasamos el objeto writer a la función
                            procesar_datos(payload, writer)
                
                time.sleep(0.05) # ~20Hz

    except KeyboardInterrupt:
        print(f"\nDetenido. Datos guardados en {nombre_archivo}")
    except Exception as e:
        print(f"\nError general: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()