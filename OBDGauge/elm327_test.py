#!/usr/bin/env python3
"""ELM327 Bluetooth test - posílá AT/OBD příkazy přes RFCOMM (bez rfcomm a pyserial).

Použití:
  python3 elm327_test.py                      # výchozí test (PIDy pro OBDGauge)
  python3 elm327_test.py 010C 010D ATRV       # vlastní příkazy
  python3 elm327_test.py --mac XX:XX:... 0105 # jiný dongle

Dongle: A2:2A:19:04:00:00 (bez jména, kanál 1, bez párování), klon "ELM327 v2.1".
Neumí ATMA/ATAL/ATSH/ATCRA/ATCF/ATCM/ATST -> sniffovat nejde, na to REZIM_SNIFF v gauge.

Subaru XV EE20 (2026-11-17): protokol 6 (CAN 11bit 500k), ECU 7E8.
Podporované mode 01: 01 04 05 0B 0C 0D 0F 10 11 1C 1F 20 21 23 30 31 33 40 42 46 49 4A
Neumí 0x87 (MAP) ani 0x5C (olej).
"""
import socket, sys, time

MAC = "A2:2A:19:04:00:00"
KANAL = 1

INIT = ["ATZ", "ATE0", "ATL0", "ATS1", "ATH1", "ATSP6", "ATDP", "ATRV"]
VYCHOZI = ["0100", "0120", "0140",            # podporované PIDy
           "0105", "010B", "0133", "0187",    # coolant, MAP, baro, MAP2
           "015C", "0142", "010C", "010D"]    # olej, napětí ECU, otáčky, rychlost


def dekoduj(odp):
    """Lidsky čitelná hodnota pro pár PIDů, co OBDGauge používá."""
    for radek in odp.split("\n"):
        b = radek.split()
        if len(b) < 4 or b[2] != "41":
            continue
        pid, d = b[3], [int(x, 16) for x in b[4:]]
        if pid == "05": return f"coolant {d[0] - 40} °C"
        if pid == "0B": return f"MAP {d[0]} kPa"
        if pid == "33": return f"baro {d[0]} kPa"
        if pid == "87": return f"MAP {((d[1] << 8) | d[2]) / 32:.1f} kPa"
        if pid == "5C": return f"olej {d[0] - 40} °C"
        if pid == "42": return f"ECU {((d[0] << 8) | d[1]) / 1000:.2f} V"
        if pid == "0C": return f"otáčky {((d[0] << 8) | d[1]) / 4:.0f} rpm"
        if pid == "0D": return f"rychlost {d[0]} km/h"
        if int(pid, 16) % 0x20 == 0 and len(d) >= 4:
            maska = (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]
            base = int(pid, 16)
            umi = [f"{base + i + 1:02X}" for i in range(32) if maska & (1 << (31 - i))]
            return "umí: " + " ".join(umi)
    return ""


def main():
    args = sys.argv[1:]
    mac = MAC
    if args[:1] == ["--mac"]:
        mac, args = args[1], args[2:]
    prikazy = INIT + (args or VYCHOZI)

    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.settimeout(8)
    s.connect((mac, KANAL))
    print("připojeno", mac)

    for c in prikazy:
        cekej = 10.0 if c[:2].isdigit() else 4.0
        s.sendall((c + "\r").encode())
        buf, konec = b"", time.time() + cekej
        while time.time() < konec and b">" not in buf:
            try:
                d = s.recv(1024)
            except socket.timeout:
                break
            if not d:
                break
            buf += d
        odp = buf.decode(errors="replace").replace("\r", "\n").strip(" >\n")
        info = dekoduj(odp)
        print(f">> {c}\n{odp}" + (f"\n   = {info}" if info else "") + "\n")
    s.close()


if __name__ == "__main__":
    main()
