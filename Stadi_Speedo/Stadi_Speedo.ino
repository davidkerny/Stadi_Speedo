#include <Arduino.h>
#include <Wire.h>
#include "U8g2lib.h"
#include <EEPROM.h>
#include <avr/wdt.h>
#include "stadionlogo.h"

// ============================================================
//  STADI SPEEDO  V6.0
//  Čte otáčky kola pomocí snímače (jazýčkový/Hall) a počítá
//  rychlost, celkový nájezd (ODO v EEPROM) a denní nájezd
//  (TRIP jen v RAM, po restartu vždy od nuly).
// ============================================================
//
//  1) NASTAVENÍ           - konstanty, které si můžeš přeladit
//  2) STAV PROGRAMU        - proměnné, které se mění za běhu
//  3) MĚŘENÍ NAPÁJENÍ      - hlídání napětí Vcc kvůli EEPROM
//  4) EEPROM               - ukládání a čtení najetých metrů
//  5) PŘERUŠENÍ            - co se stane při každém pulzu ze snímače
//  6) FILTR RYCHLOSTI      - vyhlazení + ochrana proti rušení
//  7) ÚVODNÍ LOGO
//  8) HLAVNÍ SMYČKA        - setup() a loop(), kde se to spojí
//  9) DISPLEJ              - vykreslení dashboardu
//
// ============================================================


// ============================================================
// 1) NASTAVENÍ
// ============================================================

// --- Info o firmwaru, zobrazí se při startu ---
const char FW_VERZE[] = "Stadi Speedo 6.0";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const byte SENSOR_PIN = 2;

// Obvod kola v metrech. Jeden pulz snímače = ujetí této
// vzdálenosti.
const float OBVOD_KOLA_M = 1.77;

// --- Ochrana proti rušení (jiskření od zapalování apod.) ---

// Moped reálně nepřekročí ~120 km/h. Pulz rychlejší než toto
// (tedy s kratší mezerou než 45 ms) bereme jako rušení.
//   1.77 m / 0.045 s = 39.3 m/s = 141.6 km/h
// (bezpečná rezerva nad 120 km/h)
const unsigned long MIN_MEZERA_PULZU_US = 45000UL;

// Absolutní strop rychlosti - cokoliv nad je nesmysl.
const float MAX_ROZUMNA_RYCHLOST_KMH = 120.0;

// Když se nová (syrová) rychlost od té vyhlazené liší o víc
// než tohle, je to podezřelé - buď rušení, nebo prudká změna.
const float MAX_SKOK_KMH = 25.0;

// Když je podezřelý pulz, ale hned další pulz s ním souhlasí
// (rozdíl menší než tohle), věříme, že jde o reálné zrychlení,
// ne o rušení. Musí to být menší číslo než MAX_SKOK_KMH,
// protože porovnáváme dva pulzy těsně po sobě, ne pulz
// s dlouhodobě vyhlazenou hodnotou.
const float MAX_ROZDIL_SOUSEDNICH_PULZU_KMH = 8.0;

// Pokud 2 sekundy nepřijde žádný pulz, stojíme.
const unsigned long CAS_DO_ZASTAVENI_MS = 2000UL;

// Kolik milisekund se ukazuje logo při startu.
const unsigned long LOGO_DOBA_ZOBRAZENI_MS = 2000UL;

// Kolikrát blikne LED13 během zobrazení loga.
const byte LOGO_POCET_BLIKNUTI = 4;

// --- Kalmanův filtr (vyhlazení rychlosti) ---
const float KALMAN_Q = 0.5;  // jak moc věříme nové změně
const float KALMAN_R = 8.0;  // jak moc věříme měření snímače

// --- EEPROM adresy pro ODO ---
// ODO se ukládá "round robin" přes víc adres, aby se
// EEPROM neopotřebovala zápisem stále na stejné místo
// (EEPROM vydrží cca 100 000 zápisů na buňku).
const int ODO_ADR_START = 0;
const int ODO_ADR_KONEC = 252;      // 64 pozic po 4 bajtech
const unsigned long KROK_ULOZENI_M = 100UL;  // ukládej po 100 m

// --- Sanitace při čtení ODO z EEPROM ---
// Ochrana proti tomu, aby poškozená buňka (např. po výpadku
// napájení uprostřed zápisu) nastavila nesmyslný nájezd.
const unsigned long ODO_FYZICKY_STROP_M = 100000000UL;   // 100 000 km
const unsigned long ODO_MAX_SKOK_MEZI_BUNKAMI_M = 1000000UL; // 1 000 km

// --- Ochrana zápisu do EEPROM při nízkém napětí ---
// Pokud napětí klesne pod tuhle hranici (např. při startování
// motoru), zápis do EEPROM raději přeskočíme - zápis při
// nestabilním napájení může poškodit celou buňku, ne jen
// nedopsat hodnotu.
//
// Tohle je schválně #define, ne proměnná: rozhoduje totiž,
// jestli se vůbec přeloží kód, co čte registr ADC. Když ho
// vypneš (nastav na 0), nactiNapetiVcc() se z buildu úplně
// vynechá a EEPROM se zapisuje vždy - hodí se pro testování
// na jiné desce/čipu, kde ti na týhle ochraně nezáleží.
#define KONTROLA_NAPETI_ZAPNUTA 1
const long MIN_VCC_PRO_ZAPIS_MV = 4000L; // 4.0 V

// TRIP se NEukládá do EEPROM - při každém restartu začíná
// od nuly (typické chování "denního" počítadla).


// ============================================================
// 2) STAV PROGRAMU
// ============================================================

// -- Ujetá vzdálenost --
unsigned long odoMetry = 0;   // celkový nájezd, ukládá se do EEPROM
unsigned long tripMetry = 0;  // dílčí nájezd, jen v RAM, po restartu 0

// -- Aktuální rychlost --
unsigned int zobrazenaRychlostKmh = 0;

// -- Kalmanův filtr --
float kalman_x = 0;   // odhad rychlosti
float kalman_p = 1;   // nejistota odhadu

// -- Pomoc s rozhodováním "je to rušení, nebo zrychlení?" --
float posledniSurovaRychlostKmh = 0.0;
bool mamePredchoziPulz = false;

// Jsme právě v pohybu? Na rozdíl od "kalman_x > nějaká
// rychlost" tohle NENÍ vázané na konkrétní číslo km/h -
// jakmile se moped jednou rozjede, ochrana proti rušení
// zůstává aktivní úplně do zastavení, ať jedeš 5, nebo
// 80 km/h. Kdyby byla vázaná na absolutní rychlost, tak
// při jízdě těsně kolem té hranice (třeba 5-10 km/h) by se
// ochrana neustále zapínala a vypínala.
bool jsmeVPohybu = false;

// -- EEPROM housekeeping --
int odoAdresaAktualni = ODO_ADR_START;
unsigned long odoNaposledyUlozeno = 0;

// -- Časování displeje --
unsigned long displejNaposledyCas = 0;
const unsigned long DISPLEJ_INTERVAL_MS = 200;

// -- Sdíleno s přerušením (ISR) --
// "volatile" je nutné, protože se to mění uvnitř přerušení.
volatile unsigned long isr_casPoslednihoPulzuUs = 0;
volatile unsigned long isr_delkaPoslednihoIntervaluUs = 0;
volatile bool isr_mameNovyPulz = false;


// ============================================================
// 3) MĚŘENÍ NAPÁJENÍ (Vcc)
// ============================================================
//
// Trik "interní voltmetr": změříme interní 1.1V bandgap
// referenci vůči Vcc jako referenčnímu napětí. Čím nižší
// naměřená hodnota, tím vyšší je skutečné Vcc (a naopak).
// Nepotřebuje žádný externí pin ani dělič napětí.
//
// Celý blok je schovaný za KONTROLA_NAPETI_ZAPNUTA (viz
// NASTAVENÍ výše) - když je vypnutý, tahle funkce se vůbec
// nepřeloží, takže sketch jde nahrát i na desku/core, který
// nemá kompletní definice registrů pro tenhle čip.
#if KONTROLA_NAPETI_ZAPNUTA

long nactiNapetiVcc()
{
  // Bit 5 v registru ADCSRB (na 328PB rozšiřuje ADC mux o
  // extra kanály). Zapisujeme přímo číslo bitu místo symbolu
  // "MUX5" - některé verze avr-libc/board core ho i na 328PB
  // nedefinují, takže symbol nemusí jít přeložit. Zápis 0 na
  // tenhle bit je bezpečný i na klasickém 328P, kde je
  // rezervovaný/nevyužitý.
  ADCSRB &= ~(1 << 5);
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delayMicroseconds(500);           // čas na ustálení reference
  ADCSRA |= _BV(ADSC);              // spuštění převodu
  while (bit_is_set(ADCSRA, ADSC)); // čekání na dokončení

  // "ADC" je 16bitový alias, který za nás správně přečte
  // nejdřív ADCL a pak ADCH (v tomhle pořadí to čip vyžaduje).
  uint16_t vysledekAdc = ADC;

  // 1125300 = 1.1 V (referenční napětí) * 1023 (rozsah ADC) * 1000
  // (převod na milivolty). Odvození vzorce viz AVR aplikační
  // poznámka "Reading Vcc via internal bandgap reference".
  const long VCC_KALIBRACNI_KONSTANTA = 1125300L;

  return VCC_KALIBRACNI_KONSTANTA / vysledekAdc;  // mV, např. 4950 = 4.95 V
}

#endif // KONTROLA_NAPETI_ZAPNUTA


// ============================================================
// 4) EEPROM - ukládání ujeté vzdálenosti
// ============================================================

// Po startu najdeme v EEPROM nejvyšší uloženou hodnotu ODO -
// to je náš poslední známý stav (round-robin zápis znamená,
// že nevíme dopředu, na které adrese skončil poslední zápis,
// tak to musíme najít).
//
// Cestou zahazujeme zjevně poškozené buňky (viz sanitace
// v NASTAVENÍ) - ale je fér říct, že tahle ochrana není
// dokonalá: úplně první nalezená platná buňka se stane
// nekontrolovanou základnou pro porovnávání dalších. Pokud
// by byla poškozená, ale zůstala pod fyzickým stropem, mohla
// by se chybně stát vítězem. Pro běžné použití je to ale
// dostatečná ochrana proti typickému poškození při výpadku
// napájení uprostřed zápisu.
void nactiOdoZEEPROM()
{
  unsigned long nejvyssiPlatnaHodnota = 0;
  int adresaSNejvyssiHodnotou = ODO_ADR_START;

  for (int adr = ODO_ADR_START; adr <= ODO_ADR_KONEC; adr += 4)
  {
    unsigned long hodnota = 0;
    EEPROM.get(adr, hodnota);

    // Nezapsaná buňka.
    if (hodnota == 0xFFFFFFFFUL) continue;

    // Fyzikálně nesmyslná hodnota (nad 100 000 km).
    if (hodnota > ODO_FYZICKY_STROP_M) continue;

    // Podezřelý skok oproti dosud nejvyšší nalezené hodnotě -
    // pravděpodobně poškozená buňka z výpadku napájení.
    bool jePodezreleVysoka =
      nejvyssiPlatnaHodnota > 0 &&
      hodnota > nejvyssiPlatnaHodnota + ODO_MAX_SKOK_MEZI_BUNKAMI_M;

    if (jePodezreleVysoka) continue;

    if (hodnota >= nejvyssiPlatnaHodnota)
    {
      nejvyssiPlatnaHodnota = hodnota;
      adresaSNejvyssiHodnotou = adr;
    }
  }

  odoMetry = nejvyssiPlatnaHodnota;
  odoAdresaAktualni = adresaSNejvyssiHodnotou;
}

// Zapíše ODO do další pozice v round-robin cyklu.
//
// Vrací true, pokud se zápis skutečně provedl, false pokud
// byl přeskočen kvůli nízkému napětí. DŮLEŽITÉ: volající musí
// tuhle návratovou hodnotu kontrolovat - jinak by si program
// myslel, že má hodnotu uloženou, i když ve skutečnosti
// zápis vůbec neproběhl.
bool ulozOdoDoEEPROM(unsigned long hodnotaMetru)
{
#if KONTROLA_NAPETI_ZAPNUTA
  if (nactiNapetiVcc() < MIN_VCC_PRO_ZAPIS_MV)
  {
    return false;
  }
#endif

  odoAdresaAktualni += 4;
  if (odoAdresaAktualni > ODO_ADR_KONEC)
  {
    odoAdresaAktualni = ODO_ADR_START;
  }

  EEPROM.put(odoAdresaAktualni, hodnotaMetru);
  return true;
}


// ============================================================
// 5) PŘERUŠENÍ (ISR) - volá se při každé sestupné hraně
// ============================================================
//
// Zásada pro ISR: dělej tu jen to nejnutnější a nejrychlejší.
// Veškerou "chytrou" logiku (Kalman, filtrování) necháváme
// na loop().

void snimacPreruseni()
{
  unsigned long ted = micros();

  // První pulz po startu nemáme s čím porovnat.
  if (isr_casPoslednihoPulzuUs == 0)
  {
    isr_casPoslednihoPulzuUs = ted;
    return;
  }

  unsigned long interval = ted - isr_casPoslednihoPulzuUs;

  // Hardwarový debounce: pulzy rychlejší než limit se zahodí
  // ať se tím vůbec nezatěžuje loop().
  if (interval < MIN_MEZERA_PULZU_US)
  {
    return;
  }

  isr_delkaPoslednihoIntervaluUs = interval;
  isr_casPoslednihoPulzuUs = ted;
  isr_mameNovyPulz = true;
}


// ============================================================
// 6) FILTR RYCHLOSTI
// ============================================================

// Klasický jednorozměrný Kalmanův filtr s pevnými Q/R.
// V praxi funguje jako "chytré" exponenciální vyhlazení -
// nová hodnota má tím větší váhu, čím větší je K.
float vyhladRychlost(float noveMereniKmh)
{
  kalman_p = kalman_p + KALMAN_Q;

  float zisk = kalman_p / (kalman_p + KALMAN_R);

  kalman_x = kalman_x + zisk * (noveMereniKmh - kalman_x);
  kalman_p = (1.0 - zisk) * kalman_p;

  return kalman_x;
}

// Rozhodne, jestli je nový pulz důvěryhodný, nebo jestli
// jde nejspíš o rušení (jiskření od zapalování apod.).
//
// Vrací true = pulz přijmout, false = pulz zahodit.
//
// DŮLEŽITÉ: řeší i případ, kdy rapidně zrychlíš a starý
// vyhlazený odhad (kalman_x) je pořád nízký. Bez téhle
// opravy by první "rychlý" pulz byl zahozen jako rušení,
// kalman_x by se neposunul, a další pulz by byl zahozen
// ze stejného důvodu -> tachometr by zůstal zaseknutý.
// Řešení: pokud dva pulzy PO SOBĚ souhlasí navzájem,
// věříme jim, i když se liší od staré vyhlazené hodnoty.
bool jePulzDuveryhodny(float surovaRychlostKmh)
{
  bool vysledek = true;

  // 1) Fyzikální strop.
  if (surovaRychlostKmh > MAX_ROZUMNA_RYCHLOST_KMH)
  {
    vysledek = false;
  }

  // 2) Porovnání s dlouhodobě vyhlazenou rychlostí.
  //    Kontrolujeme jen když už jedeme - při rozjezdu z nuly
  //    (první pulz po zastavení) nemáme s čím smysluplně
  //    porovnávat, tak bychom si jinak zablokovali rozjezd.
  if (vysledek && jsmeVPohybu)
  {
    float rozdilOdVyhlazene = fabs(surovaRychlostKmh - kalman_x);

    if (rozdilOdVyhlazene > MAX_SKOK_KMH)
    {
      // Podezřelé - ALE než to zahodíme, zkusíme druhou šanci:
      // souhlasí tenhle pulz s tím bezprostředně předchozím?
      bool souhlasiSPredchozim =
        mamePredchoziPulz &&
        fabs(surovaRychlostKmh - posledniSurovaRychlostKmh)
          < MAX_ROZDIL_SOUSEDNICH_PULZU_KMH;

      if (souhlasiSPredchozim)
      {
        // Dva pulzy za sebou si odpovídají -> jde o reálnou
        // změnu rychlosti, ne o osamocené rušení.
        // "Nahodíme" filtr na aktuální rychlost, aby se
        // nemusel k realitě pomalu dohánět.
        kalman_x = surovaRychlostKmh;
        kalman_p = 1.0;
      }
      else
      {
        vysledek = false;
      }
    }
  }

  return vysledek;
}


// ============================================================
// 7) ÚVODNÍ LOGO
// ============================================================

void zobrazUvodniLogo()
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Logo dostane horních 50 px, spodní řádek zůstává
  // volný pro text s verzí firmwaru.
  const int VYSKA_OBLASTI_LOGA = 50;
  int x = (128 - STADION_WIDTH) / 2;
  int y = (VYSKA_OBLASTI_LOGA - STADION_HEIGHT) / 2;

  u8g2.clearBuffer();
  u8g2.drawXBMP(x, y, STADION_WIDTH, STADION_HEIGHT, logo_stadion);

  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.setCursor(0, 64);
  u8g2.print(FW_VERZE);

  u8g2.sendBuffer();

  // LED13 pár-krát blikne po dobu zobrazení loga.
  pinMode(13, OUTPUT);
  const unsigned long krokMs =
    LOGO_DOBA_ZOBRAZENI_MS / (LOGO_POCET_BLIKNUTI * 2);

  for (byte i = 0; i < LOGO_POCET_BLIKNUTI; i++)
  {
    digitalWrite(13, HIGH);
    delay(krokMs);
    digitalWrite(13, LOW);
    delay(krokMs);
  }
}


// ============================================================
// 8) HLAVNÍ SMYČKA
// ============================================================

void setup()
{
  // Bezpečnostní pojistka: některé bootloadery nechají
  // watchdog po resetu běžet dál. Bez tohohle řádku by se
  // MCU mohl zacyklit v nekonečném restartování hned po
  // zapnutí, ještě než se dostaneme do loop().
  wdt_disable();

  u8g2.begin();
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  zobrazUvodniLogo();

  nactiOdoZEEPROM();
  tripMetry = 0;   // TRIP po startu vždy od nuly

  odoNaposledyUlozeno = odoMetry;

  attachInterrupt(
    digitalPinToInterrupt(SENSOR_PIN),
    snimacPreruseni,
    FALLING
  );

  vykresliDashboard();

  // Watchdog zapínáme až na úplný konec setup() - pokud by
  // něco výše (např. u8g2.begin() při nezapojeném displeji)
  // viselo déle než timeout, chceme to vidět/debugovat, ne
  // aby se to potichu pořád dokola restartovalo v setupu.
  wdt_enable(WDTO_2S);
}

void zpracujNovyPulzPokudExistuje()
{
  if (!isr_mameNovyPulz)
  {
    return;
  }

  // Atomicky (s vypnutými přerušeními) přečteme sdílená data,
  // ať nám ISR něco nepřepíše uprostřed čtení.
  unsigned long intervalUs;
  noInterrupts();
  intervalUs = isr_delkaPoslednihoIntervaluUs;
  isr_mameNovyPulz = false;
  interrupts();

  // Perioda mezi pulzy -> okamžitá (syrová) rychlost.
  const float MS_PER_HODINU = 3600000.0;
  float surovaRychlostKmh = (OBVOD_KOLA_M * MS_PER_HODINU) / intervalUs;

  if (!jePulzDuveryhodny(surovaRychlostKmh))
  {
    // Zahazujeme i historii "posledního pulzu" - nechceme,
    // aby se šumový pulz použil jako referenční bod pro
    // potvrzování dalšího pulzu.
    mamePredchoziPulz = false;
    return;
  }

  mamePredchoziPulz = true;
  posledniSurovaRychlostKmh = surovaRychlostKmh;
  jsmeVPohybu = true;

  // -- Vyhlazení rychlosti --
  float vyhlazenaKmh = vyhladRychlost(surovaRychlostKmh);
  zobrazenaRychlostKmh = (unsigned int)(vyhlazenaKmh + 0.5);

  // -- Přičtení ujeté vzdálenosti --
  // Jeden accumulator pro zlomkové metry (obvod kola není
  // celé číslo metrů). Když naakumulujeme celý metr,
  // přičteme ho k odo i tripu najednou.
  static float zbytekMetruAkumulator = 0.0;
  zbytekMetruAkumulator += OBVOD_KOLA_M;

  if (zbytekMetruAkumulator >= 1.0)
  {
    unsigned long celeMetry = (unsigned long)zbytekMetruAkumulator;
    odoMetry += celeMetry;
    tripMetry += celeMetry;
    zbytekMetruAkumulator -= celeMetry;
  }

  // -- Průběžné ukládání do EEPROM --
  // Podmínku "je čas uložit" a "podařilo se uložit" musíme
  // rozlišovat - odoNaposledyUlozeno posuneme JEN pokud zápis
  // opravdu proběhl (viz komentář u ulozOdoDoEEPROM výše).
  if (odoMetry - odoNaposledyUlozeno >= KROK_ULOZENI_M)
  {
    if (ulozOdoDoEEPROM(odoMetry))
    {
      odoNaposledyUlozeno = odoMetry;
    }
    // Pokud se zápis nepovedl (nízké napětí), zkusíme to
    // znovu při dalším platném pulzu - odoMetry v RAM
    // mezitím dál roste správně, nic se neztrácí.
  }
}

void zkontrolujJestliStojime()
{
  unsigned long casPoslednihoPulzu;
  noInterrupts();
  casPoslednihoPulzu = isr_casPoslednihoPulzuUs;
  interrupts();

  bool jsmeUzNekdyJeli = (casPoslednihoPulzu != 0);
  unsigned long dobaBezPulzuUs = micros() - casPoslednihoPulzu;

  if (jsmeUzNekdyJeli && dobaBezPulzuUs > CAS_DO_ZASTAVENI_MS * 1000UL)
  {
    kalman_x = 0;
    kalman_p = 1;
    zobrazenaRychlostKmh = 0;
    mamePredchoziPulz = false;
    jsmeVPohybu = false;
  }
}

void loop()
{
  // Musí se volat pravidelně - jinak watchdog po 2 s tvrdě
  // restartuje MCU. Dej pozor, aby nikde v kódu (hlavně
  // v budoucích úpravách) nevznikl delay() nebo čekací
  // smyčka delší než tento timeout.
  wdt_reset();

  zpracujNovyPulzPokudExistuje();
  zkontrolujJestliStojime();

  unsigned long ted = millis();
  if (ted - displejNaposledyCas > DISPLEJ_INTERVAL_MS)
  {
    vykresliDashboard();
    displejNaposledyCas = ted;
  }
}


// ============================================================
// 9) DISPLEJ
// ============================================================

void vykresliDashboard()
{
  u8g2.clearBuffer();

  // -- Velké číslo rychlosti --
  u8g2.setFont(u8g2_font_logisoso58_tn);

  if (zobrazenaRychlostKmh < 10)
  {
    u8g2.setCursor(43, 57);
  }
  else if (zobrazenaRychlostKmh < 100)
  {
    u8g2.setCursor(23, 57);
  }
  else
  {
    u8g2.setCursor(0, 57);
  }
  u8g2.print(zobrazenaRychlostKmh);

  // -- Popisek "km/h" --
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(105, 33);
  u8g2.print("km/h");

  // -- ODO (celkový nájezd v km, celé číslo) --
  u8g2.setCursor(0, 64);
  u8g2.print(odoMetry / 1000);

  // -- TRIP (zarovnaný doprava, na 1 desetinné místo) --
  //
  // POZOR: na AVR (Uno/Nano/Pro Mini) neumí snprintf/sprintf
  // zpracovat %f, pokud se explicitně nezapne float podpora
  // v linkeru. Bez toho se text s %.1f prostě usekne hned
  // za posledním "normálním" znakem. dtostrf() je proto
  // standardní a bezpečná cesta, jak float na Arduinu
  // vypsat jako text.
  float tripKm = tripMetry / 1000.0;

  char cisloTripu[8];      // "12.3" apod.
  dtostrf(tripKm, 4, 1, cisloTripu);

  // Odstraníme případné mezery, které dtostrf přidá zleva
  // kvůli zarovnání na šířku pole.
  char *zacatekCisla = cisloTripu;
  while (*zacatekCisla == ' ')
  {
    zacatekCisla++;
  }

  char tripText[12];
  snprintf(tripText, sizeof(tripText), "t %s", zacatekCisla);

  int sirkaTextu = u8g2.getStrWidth(tripText);
  u8g2.setCursor(128 - sirkaTextu, 64);
  u8g2.print(tripText);

  u8g2.sendBuffer();
}
