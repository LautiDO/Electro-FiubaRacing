#!/usr/bin/env python3
"""
=============================================================
  RECEPTOR SPI — Raspberry Pi  ←→  STM32
=============================================================
  Solicita 34 bytes cada 20 ms al STM32 via SPI,
  valida el paquete (sync byte + checksum XOR),
  muestra los datos en consola y los guarda en CSV.

  Estructura del paquete (34 bytes):
    [0]      → 0xAA  sync byte
    [1..4]   → angulo         float  [°]
    [5..8]   → acel_lateral   float  [g]
    [9..12]  → dx1            float  [mm]
    [13..16] → dx2            float  [mm]
    [17..20] → dx3            float  [mm]
    [21..24] → dx4            float  [mm]
    [25..28] → dx5            float  [mm]
    [29..32] → tiempo_us      uint32 [µs]
    [33]     → checksum XOR

  Dependencias:
    pip install spidev

  Habilitar SPI en la Raspberry:
    sudo raspi-config → Interface Options → SPI → Enable
=============================================================
"""

import spidev
import struct
import csv
import time
import os
import sys
from datetime import datetime


# ============================================================
#                     CONFIGURACIÓN
# ============================================================

SPI_BUS        = 0          # /dev/spidev0.x
SPI_DEVICE     = 0          # CE0 (GPIO8) → conectar a NSS del STM32
SPI_SPEED_HZ   = 1_000_000  # 1 MHz — ajustar si hay errores de comunicación
SPI_MODE       = 0b00       # CPOL=0, CPHA=0 — debe coincidir con el STM32

PERIODO_MS     = 20         # período de muestreo [ms]
PERIODO_S      = PERIODO_MS / 1000.0

SYNC_BYTE      = 0xAA
PACKET_SIZE    = 34

# Archivo CSV — nombre con timestamp para no sobreescribir sesiones anteriores
TIMESTAMP_SESION = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_DIR        = os.path.join(os.path.dirname(__file__), "sesiones")
CSV_FILENAME   = os.path.join(CSV_DIR, f"sesion_{TIMESTAMP_SESION}.csv")

CSV_HEADER = [
    "tiempo_pc_ms",   # tiempo del sistema Raspberry [ms desde inicio]
    "tiempo_stm_us",  # timestamp del STM32 [µs]
    "angulo_deg",
    "acel_lateral_g",
    "dx1_mm",
    "dx2_mm",
    "dx3_mm",
    "dx4_mm",
    "dx5_mm",
]


# ============================================================
#                   PARSEO DEL PAQUETE
# ============================================================

def parsear_paquete(raw: bytes) -> dict | None:
    """
    Valida y parsea los 34 bytes recibidos del STM32.
    Retorna un dict con los campos, o None si el paquete es inválido.
    """

    if len(raw) != PACKET_SIZE:
        return None

    # 1. Sync byte
    if raw[0] != SYNC_BYTE:
        return None

    # 2. Checksum XOR (todos los bytes excepto el último)
    cs_calculado = 0
    for b in raw[:-1]:
        cs_calculado ^= b

    if cs_calculado != raw[-1]:
        return None

    # 3. Desempaquetar — little-endian (ARM es LE por defecto)
    #    Formato: < = little-endian
    #    7f = siete floats (angulo + acel + dx1..dx5) × 4 bytes = 28 bytes
    #    I  = uint32 (tiempo_us)                               ×  4 bytes =  4 bytes
    #    Total: 1 (sync) + 28 + 4 + 1 (checksum) = 34 bytes ✓
    angulo, acel, dx1, dx2, dx3, dx4, dx5, tiempo_us = struct.unpack_from(
        "<7fI",
        raw,
        1   # offset: saltar el sync byte
    )

    return {
        "angulo_deg":     angulo,
        "acel_lateral_g": acel,
        "dx1_mm":         dx1,
        "dx2_mm":         dx2,
        "dx3_mm":         dx3,
        "dx4_mm":         dx4,
        "dx5_mm":         dx5,
        "tiempo_stm_us":  tiempo_us,
    }


# ============================================================
#                   DISPLAY EN CONSOLA
# ============================================================

# Códigos ANSI para colores
VERDE    = "\033[92m"
AMARILLO = "\033[93m"
ROJO     = "\033[91m"
CYAN     = "\033[96m"
RESET    = "\033[0m"
BOLD     = "\033[1m"
CLEAR    = "\033[2J\033[H"  # limpiar pantalla + cursor al inicio


def mostrar_consola(datos: dict, t_pc_ms: float, n_muestra: int,
                    errores: int, total: int):
    """Imprime los datos del paquete en consola con formato legible."""

    tasa_error = (errores / total * 100) if total > 0 else 0.0

    print(CLEAR, end="")
    print(f"{BOLD}{CYAN}{'='*52}{RESET}")
    print(f"{BOLD}{CYAN}   STM32 → Raspberry  |  SPI Data Logger{RESET}")
    print(f"{CYAN}{'='*52}{RESET}")

    print(f"\n  {BOLD}Muestra #{n_muestra:>6}{RESET}   "
          f"t_PC = {t_pc_ms:>10.1f} ms   "
          f"t_STM = {datos['tiempo_stm_us']:>10} µs")

    print(f"\n  {'─'*48}")
    print(f"  {'SENSOR':<22}  {'VALOR':>10}  {'UNIDAD'}")
    print(f"  {'─'*48}")
    print(f"  {'Ángulo volante':<22}  {datos['angulo_deg']:>10.2f}  °")
    print(f"  {'Acel. lateral':<22}  {datos['acel_lateral_g']:>10.3f}  g")
    print(f"  {'─'*48}")
    print(f"  {'Suspensión 1 (dx1)':<22}  {datos['dx1_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 2 (dx2)':<22}  {datos['dx2_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 3 (dx3)':<22}  {datos['dx3_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 4 (dx4)':<22}  {datos['dx4_mm']:>10.1f}  mm")
    print(f"  {'Suspensión 5 (dx5)':<22}  {datos['dx5_mm']:>10.1f}  mm")
    print(f"  {'─'*48}")

    color_error = VERDE if tasa_error < 1.0 else (AMARILLO if tasa_error < 5.0 else ROJO)
    print(f"\n  Paquetes OK: {VERDE}{total - errores}{RESET}   "
          f"Errores: {color_error}{errores} ({tasa_error:.1f}%){RESET}")
    print(f"  CSV → {CSV_FILENAME}")
    print(f"\n  {CYAN}Ctrl+C para detener{RESET}\n")


# ============================================================
#                      MAIN
# ============================================================

def main():

    # ── Crear directorio de sesiones si no existe ──────────
    os.makedirs(CSV_DIR, exist_ok=True)

    # ── Inicializar SPI ────────────────────────────────────
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode         = SPI_MODE
    spi.bits_per_word = 8

    print(f"[OK] SPI abierto: bus={SPI_BUS}, device={SPI_DEVICE}, "
          f"velocidad={SPI_SPEED_HZ/1e6:.1f} MHz, modo={SPI_MODE}")
    print(f"[OK] Guardando en: {CSV_FILENAME}")
    print(f"[..] Iniciando adquisición a {1000/PERIODO_MS:.0f} Hz...\n")
    time.sleep(1.0)

    # ── Abrir CSV ──────────────────────────────────────────
    csv_file   = open(CSV_FILENAME, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_HEADER)
    csv_writer.writeheader()

    n_muestra   = 0
    n_errores   = 0
    t_inicio    = time.monotonic()

    try:
        while True:
            t_ciclo_inicio = time.monotonic()

            # ── Solicitar paquete al STM32 ─────────────────
            # xfer2 mantiene CS activo durante toda la transferencia.
            # Enviamos 34 bytes dummy (0x00) para generar el clock;
            # el STM32 responde con sus datos en MISO.
            raw = bytes(spi.xfer2([0x00] * PACKET_SIZE))

            t_pc_ms = (time.monotonic() - t_inicio) * 1000.0
            n_muestra += 1

            # ── Parsear y validar ──────────────────────────
            datos = parsear_paquete(raw)

            if datos is None:
                n_errores += 1
                # Mostrar advertencia sin limpiar pantalla
                print(f"\r  {AMARILLO}[!] Paquete inválido #{n_muestra} "
                      f"(sync={raw[0]:#04x} cs_rx={raw[-1]:#04x}){RESET}",
                      end="", flush=True)
            else:
                # ── Guardar en CSV ─────────────────────────
                fila = {
                    "tiempo_pc_ms":   round(t_pc_ms, 3),
                    "tiempo_stm_us":  datos["tiempo_stm_us"],
                    "angulo_deg":     round(datos["angulo_deg"],     3),
                    "acel_lateral_g": round(datos["acel_lateral_g"], 4),
                    "dx1_mm":         round(datos["dx1_mm"],         2),
                    "dx2_mm":         round(datos["dx2_mm"],         2),
                    "dx3_mm":         round(datos["dx3_mm"],         2),
                    "dx4_mm":         round(datos["dx4_mm"],         2),
                    "dx5_mm":         round(datos["dx5_mm"],         2),
                }
                csv_writer.writerow(fila)
                csv_file.flush()  # asegura escritura en disco en tiempo real

                # ── Mostrar en consola ─────────────────────
                mostrar_consola(datos, t_pc_ms, n_muestra,
                                n_errores, n_muestra)

            # ── Esperar hasta completar el período de 20 ms ──
            t_transcurrido = time.monotonic() - t_ciclo_inicio
            t_espera = PERIODO_S - t_transcurrido
            if t_espera > 0:
                time.sleep(t_espera)

    except KeyboardInterrupt:
        print(f"\n\n{VERDE}[OK] Adquisición detenida por el usuario.{RESET}")

    finally:
        spi.close()
        csv_file.close()

        duracion_s = time.monotonic() - t_inicio
        tasa_ok    = ((n_muestra - n_errores) / n_muestra * 100) if n_muestra > 0 else 0

        print(f"\n{'─'*52}")
        print(f"  Resumen de sesión")
        print(f"{'─'*52}")
        print(f"  Duración:        {duracion_s:.1f} s")
        print(f"  Muestras totales:{n_muestra}")
        print(f"  Paquetes OK:     {n_muestra - n_errores}  ({tasa_ok:.1f}%)")
        print(f"  Errores:         {n_errores}")
        print(f"  CSV guardado en: {CSV_FILENAME}")
        print(f"{'─'*52}\n")


if __name__ == "__main__":
    main()
