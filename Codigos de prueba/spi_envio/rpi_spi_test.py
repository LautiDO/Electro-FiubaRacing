#!/usr/bin/env python3
# ====================================================
#  Raspberry Pi — SPI MASTER TEST
#  Lee paquetes del STM32 y verifica integridad.
#
#  Instalar dependencias:
#    pip3 install spidev
#
#  Permisos SPI:
#    sudo raspi-config → Interfaz → SPI → Enable
#    (o: sudo modprobe spi-bcm2835)
#
#  Conexión física (con remap STM32):
#    RPi MOSI (GPIO10) → STM32 MOSI (PB5)
#    RPi MISO (GPIO9)  → STM32 MISO (PB4)
#    RPi SCK  (GPIO11) → STM32 SCK  (PB3)
#    RPi CE0  (GPIO8)  → STM32 NSS  (PA15)
#    GND ←→ GND
# ====================================================

import spidev
import time
import struct

# ── Configuración ──────────────────────────────────────────────────────────────
SPI_BUS       = 1
SPI_DEVICE    = 0
SPI_SPEED_HZ  = 1_000_000   # 1 MHz — bajá si hay ruido (500_000, 250_000)
SPI_MODE      = 0

MAX_DATOS     = 40
SYNC_BYTE     = 0xAA
TEXTO_ESPERADO = b"HELLO_STM32_SPI_TEST_PACKET_OK!!"

# ── Contadores globales ────────────────────────────────────────────────────────
stats = {
    "total":           0,
    "ok":              0,
    "sin_sync":        0,
    "checksum_fail":   0,
    "texto_corrupto":  0,
    "secuencia_salto": 0,
}

ultimo_contador = None


def abrir_spi():
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode = SPI_MODE
    spi.bits_per_word = 8
    return spi


def leer_paquete(spi):
    """Lee MAX_DATOS bytes del SPI (manda 0x00 como dummy)."""
    datos = spi.xfer2([0x00] * MAX_DATOS)
    return bytes(datos)


def verificar_paquete(raw: bytes) -> dict:
    """
    Verifica el paquete y devuelve un dict con el resultado.
    Campos:
        ok          → bool, True si todo pasó
        sync_ok     → bool
        contador    → int o None
        texto_ok    → bool
        checksum_ok → bool
        errores     → list[str] con descripciones legibles
        raw_hex     → representación hex del paquete
    """
    resultado = {
        "ok": False,
        "sync_ok": False,
        "contador": None,
        "texto_ok": False,
        "checksum_ok": False,
        "errores": [],
        "raw_hex": raw.hex(" ").upper(),
    }

    if len(raw) < MAX_DATOS:
        resultado["errores"].append(f"Paquete corto: {len(raw)} bytes (esperado {MAX_DATOS})")
        return resultado

    # ── Sync ──────────────────────────────────────────────────────────────────
    if raw[0] == SYNC_BYTE:
        resultado["sync_ok"] = True
    else:
        resultado["errores"].append(
            f"Sync incorrecto: 0x{raw[0]:02X} (esperado 0xAA)"
        )

    # ── Contador ──────────────────────────────────────────────────────────────
    contador = struct.unpack_from("<I", raw, 1)[0]  # uint32 little-endian
    resultado["contador"] = contador

    # ── Texto ─────────────────────────────────────────────────────────────────
    texto_recibido = raw[5:37]
    if texto_recibido == TEXTO_ESPERADO:
        resultado["texto_ok"] = True
    else:
        # Mostrar exactamente qué bytes difieren
        diffs = []
        for i, (e, r) in enumerate(zip(TEXTO_ESPERADO, texto_recibido)):
            if e != r:
                diffs.append(f"  pos {5+i}: esperado 0x{e:02X}('{chr(e) if 32<=e<127 else '?'}') "
                             f"recibido 0x{r:02X}('{chr(r) if 32<=r<127 else '?'}')")
        resultado["errores"].append(
            f"Texto corrupto ({len(diffs)} byte/s diferente/s):\n" + "\n".join(diffs[:10])
        )

    # ── Checksum ──────────────────────────────────────────────────────────────
    chk_calc = 0
    for b in raw[1:37]:
        chk_calc ^= b
    chk_recibido  = raw[37]
    chk_comp      = raw[38]
    chk_ctrl      = raw[39]

    chk_ok = (
        chk_recibido == chk_calc and
        chk_comp == (~chk_calc & 0xFF) and
        chk_ctrl == (chk_calc ^ 0x55)
    )
    resultado["checksum_ok"] = chk_ok
    if not chk_ok:
        resultado["errores"].append(
            f"Checksum fallido: calculado=0x{chk_calc:02X}  "
            f"recibido=[0x{chk_recibido:02X}, 0x{chk_comp:02X}, 0x{chk_ctrl:02X}]"
        )

    resultado["ok"] = (
        resultado["sync_ok"] and
        resultado["texto_ok"] and
        resultado["checksum_ok"]
    )
    return resultado


def imprimir_resultado(res: dict, numero_lectura: int):
    global ultimo_contador, stats

    stats["total"] += 1

    # ── Verificar salto de secuencia ──────────────────────────────────────────
    salto = 0
    if res["contador"] is not None and ultimo_contador is not None:
        esperado = (ultimo_contador + 1) & 0xFFFFFFFF
        if res["contador"] != esperado:
            salto = res["contador"] - ultimo_contador
            stats["secuencia_salto"] += 1
    if res["contador"] is not None:
        ultimo_contador = res["contador"]

    # ── Actualizar contadores ─────────────────────────────────────────────────
    if res["ok"]:
        stats["ok"] += 1
    if not res["sync_ok"]:
        stats["sin_sync"] += 1
    if not res["checksum_ok"]:
        stats["checksum_fail"] += 1
    if not res["texto_ok"]:
        stats["texto_corrupto"] += 1

    # ── Imprimir ──────────────────────────────────────────────────────────────
    estado = "✓ OK" if res["ok"] else "✗ ERROR"
    cnt_str = f"cnt={res['contador']}" if res["contador"] is not None else "cnt=?"

    print(f"[{numero_lectura:>6}] {estado}  {cnt_str}", end="")

    if salto and salto != 0:
        print(f"  ⚠ SALTO de secuencia: +{salto}", end="")

    print()  # newline

    if not res["ok"]:
        for err in res["errores"]:
            print(f"         → {err}")
        # Mostrar hex completo del paquete cuando hay error
        print(f"         → RAW: {res['raw_hex']}")
        print()


def imprimir_resumen():
    t = stats["total"]
    ok = stats["ok"]
    tasa = (ok / t * 100) if t > 0 else 0.0

    print()
    print("=" * 60)
    print("  RESUMEN DE LA PRUEBA SPI")
    print("=" * 60)
    print(f"  Total de paquetes leídos : {t}")
    print(f"  Correctos                : {ok}  ({tasa:.1f}%)")
    print(f"  Sin sync byte            : {stats['sin_sync']}")
    print(f"  Checksum fallido         : {stats['checksum_fail']}")
    print(f"  Texto corrupto           : {stats['texto_corrupto']}")
    print(f"  Saltos de secuencia      : {stats['secuencia_salto']}")
    print("=" * 60)


def main():
    print("=" * 60)
    print("  STM32 ↔ Raspberry Pi — Test de integridad SPI")
    print(f"  Bus SPI{SPI_BUS}.{SPI_DEVICE}  @  {SPI_SPEED_HZ // 1000} kHz")
    print("  Presioná Ctrl+C para ver el resumen y salir.")
    print("=" * 60)
    print()

    spi = abrir_spi()
    numero = 0

    try:
        while True:
            raw = leer_paquete(spi)
            res = verificar_paquete(raw)
            imprimir_resultado(res, numero)
            numero += 1

            # Mostrar resumen parcial cada 500 paquetes
            if numero % 500 == 0:
                imprimir_resumen()

            time.sleep(0.01)  # 100 lecturas/seg — debe coincidir con el delay del STM32

    except KeyboardInterrupt:
        pass
    finally:
        spi.close()
        imprimir_resumen()


if __name__ == "__main__":
    main()
