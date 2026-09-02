#include <Arduino.h>
#include <EEPROM.h>

// ============================================================
//  JEDNORÁZOVÁ UTILITKA: nastavení počátečního stavu ODO
// ============================================================
//
//  POUŽITÍ:
//  1) Uprav hodnotu ODO_KM níže na to, co chceš nastavit.
//  2) Nahraj TENTO soubor do desky (místo hlavního programu
//     tachometru).
//  3) Otevři Serial Monitor (9600 baud) - potvrdí se zápis.
//  4) Nahraj zpátky hlavní program tachometru.
//
//  Musí sedět se stejnými konstantami (adresy, rozsah), jaké
//  používá hlavní program tachometr.ino - jinak by si hlavní
//  program přečetl hodnotu ze špatného místa.
// ============================================================

// <<< SEM NAPIŠ POŽADOVANÝ NÁJEZD >>>
const unsigned long ODO_KM = 1042;

// --- Musí odpovídat hlavnímu programu tachometr.ino ---
const int ODO_ADR_START = 0;
const int ODO_ADR_KONEC = 252;   // 64 pozic po 4 bajtech


void setup()
{
  Serial.begin(9600);
  while (!Serial) { /* počkej na otevření Serial Monitoru (jen u desek, kde je potřeba) */ }

  unsigned long odoMetry = ODO_KM * 1000UL;

  Serial.println(F("Mazu stare hodnoty ODO v EEPROM..."));

  // Nejdřív vymažeme (nastavíme na "prázdno") všechny pozice,
  // kam hlavní program ukládá ODO. Hlavní program totiž při
  // startu bere NEJVYŠŠÍ nalezenou hodnotu ze všech pozic -
  // kdybychom nevymazali staré, mohla by tam zůstat vyšší
  // hodnota, než tu novou, kterou chceme nastavit.
  for (int adr = ODO_ADR_START; adr <= ODO_ADR_KONEC; adr += 4)
  {
    unsigned long prazdno = 0xFFFFFFFFUL;
    EEPROM.put(adr, prazdno);
  }

  Serial.print(F("Zapisuji novou hodnotu ODO: "));
  Serial.print(ODO_KM);
  Serial.print(F(" km ("));
  Serial.print(odoMetry);
  Serial.println(F(" m)"));

  // Zapíšeme na první pozici - hlavní program si ji odtud
  // při startu normálně přečte a bude v ní dál pokračovat
  // (round-robin ukládání poběží dál od téhle adresy).
  EEPROM.put(ODO_ADR_START, odoMetry);

  // Kontrolní zpětné čtení, ať víme jistě, že se zápis povedl.
  unsigned long kontrola = 0;
  EEPROM.get(ODO_ADR_START, kontrola);

  if (kontrola == odoMetry)
  {
    Serial.println(F("HOTOVO. Zapis probehl uspesne."));
  }
  else
  {
    Serial.println(F("CHYBA! Precteno neco jineho, nez bylo zapsano."));
  }

  Serial.println(F("Ted muzes nahrat zpatky hlavni program tachometru."));
}

void loop()
{
  // Nic - je to jednorázová utilitka.
}
