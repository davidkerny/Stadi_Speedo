# OBDGauge – Subaru XV 2.0D (EE20, 2017)

HW: Stadi_Speedo PCB V2 (ATmega328PB 16 MHz) + MCP2515 modul na AUX (A0=CS, A1=SCK, D4=MISO, D5=MOSI), SH1106 1.3" OLED.
OBD konektor: pin 6 = CAN-H, pin 14 = CAN-L, pin 16 = +12 V trvale, pin 4/5 = GND.

## Co budík ukazuje (V0.2)

| Hodnota | Zdroj (OBD mode 01) | Poznámka |
|---|---|---|
| BOOST | MAP (PID 0x87, jinak 0x0B) − barometr (PID 0x33) | 0x0B má strop 255 kPa (≈ +1.5 bar). Když na něj narazí, svítí `BOOST MAX`. Proto preferujeme 0x87, pokud ho ECU nabídne. |
| COOLANT | PID 0x05 | |
| OIL | PID 0x5C, jinak volitelně mode 22 | viz níž |
| BATT | PID 0x42 | napětí na ECU ≈ palubní síť |

Po zapnutí se firmware ECU zeptá, které PIDy umí (`...` vpravo nahoře). Co neumí, ukazuje `N/A` a víc se na to neptá.

## Olejová teplota

Standardní PID 0x5C umí jen některá ECU. Pokud na EE20 ukáže `N/A`, zbývá výrobcův **mode 22** (v Torque „custom PID“, hlavička 7E0).
Pro EE20 není veřejně ověřený PID olejové teploty. Postup:

1. V Torque zkus PIDy z projektu SubaruDieselCrew / ParsePID (forum subaruforester.org, vlákno „EE20 DPF data logging over OBD2“).
   Tam jsou ověřené mode 22 PIDy pro EE20 (DPF, ředění oleje `22114E`, …); olejová teplota tam může a nemusí být.
2. Kandidát, který v Torque dává rozumné číslo (studený motor = venkovní teplota, zahřátý ~90–110 °C, roste pomaleji než voda), zadej do `OBDGauge.ino`:
   ```c
   #define OLEJ_MODE22 1
   const uint16_t OLEJ_M22_PID = 0x....;   // Torque "22xxxx" -> 0xxxxx
   const int      OLEJ_M22_OFFSET = -40;   // Torque vzorec "A-40"
   ```
   Firmware umí vzorec `A + offset` (1 bajt). Jiný vzorec (např. `(A*256+B)/10`) = úprava v `zpracujData()`.
3. Když ECU PID neumí, odpoví negativně a slot se vyřadí (`N/A`).

Záložní varianta bez ECU: čidlo do motoru (třeba místo výpustného šroubu / sendvičová deska pod filtr) + NTC na A2/A3 – ThermoGauge to umí, ale AUX piny jsou obsazené MCP2515, takže by šlo o úpravu desky.

## Ztlumení displeje při rozsvícených světlech

Dvě cesty, obě ve firmwaru připravené (`DIM_ZDROJ`):

### 1) Drát – doporučeno, funguje hned
`DIM_ZDROJ 1`: D3 (vstup ENG RPM na desce, má odpor 200R/1W, diodu a dělič – snese 12 V) připojit na **osvětlení / obrysová světla**
(drát podsvícení spínačů, napájení přídavné zásuvky ilumin., nebo vodič k obrysovkám).
Po zapnutí světel se vpravo nahoře rozsvítí tečka. Vstup ENG RPM jde přes optočlen Q1, který D3 stahuje k zemi:
napětí na vstupu = LOW, proto `DIM_AKTIVNI_UROVEN LOW` a interní pull-up na D3 (ověřeno na desce).

### 2) Z CAN sběrnice – bez drátu, ale musí se najít
Subaru posílá stav světel do přístrojovky po CANu. Pro GP/XV ID není veřejně zdokumentované a není jisté, že je vidět na OBD konektoru
(novější Subaru filtrují přes gateway; GP generace OBD pustí hodně provozu, takže šance je dobrá).

1. `#define REZIM_SNIFF 1`, nahrát, USB do notebooku, serial monitor 115200.
   MCP je v listen-only módu – do auta nic nevysílá.
2. Zapalování ON, motor může běžet. Prvních 10 s na **nic nesahat** (učí se šum: čítače, otáčky…).
3. Pak několikrát zapni/vypni obrysovky/potkávačky. Hledej řádek, který se opakuje přesně s přepnutím, např.
   `48211 6D1 b3 0x00->0x04 (bit 2)` → ID `0x6D1`, bajt 3, maska `0x04`.
4. Zadej:
   ```c
   #define REZIM_SNIFF 0
   #define DIM_ZDROJ 2
   const uint16_t DIM_CAN_ID = 0x6D1;
   const byte DIM_CAN_BAJT = 3;
   const byte DIM_CAN_MASKA = 0x04;
   ```
   Rámec světel dostane vlastní HW filtr v MCP2515 (RXB1), OBD jede dál přes RXB0.

Pozn.: Totéž sniffováním jde i pro olej, kdyby ho motor posílal rovnou do přístrojovky – postup je stejný, jen hledáš bajt, který pomalu roste s teplotou.

### Jas
`JAS_DEN` / `JAS_NOC` = kontrast OLED (noc 0x00, Laskakit SH1106 pořád svítí).
`HLUBOKY_DIM 1` v noci navíc posílá pre-charge/VCOMH – na Laskakit SH1106 to dimuje sotva znatelně a zasekává I2C, proto je vypnuté.
