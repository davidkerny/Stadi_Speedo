#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include "stadionlogo.h"

// --- NAČTENÍ VYHLAZENÉHO FONTU PRO RYCHLOST ---
#include <Fonts/FreeSansBold24pt7b.h>

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// --- KONFIGURACE ---
const byte SENSOR_PIN = 2;        
const float OBVOD_KOLA = 1.77;    

const int ADRESA_ODO = 0;         
const int ADRESA_TRIP = 4;        

// --- PROMĚNNÉ PRO PŘERUŠENÍ ---
volatile unsigned long posledniPulzCas = 0;
volatile unsigned long casMeziPulzy = 0;
volatile bool novyPulz = false;

const unsigned long MIN_CAS_MEZI_PULZY = 40; 

// --- GLOBÁLNÍ PROMĚNNÉ ---
unsigned int rychlost = 0;
unsigned long celkoveMetry = 0;
unsigned long odoKilometry = 0;

unsigned long tripMetry = 0;
unsigned long tripKilometry = 0;

unsigned long posledniAktualizaceCas = 0;
unsigned long posledniZapisEEPROM = 0;

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  
  EEPROM.get(ADRESA_ODO, odoKilometry);
  EEPROM.get(ADRESA_TRIP, tripKilometry);
  
  if (odoKilometry == 4294967295UL) odoKilometry = 0;
  if (tripKilometry == 4294967295UL) tripKilometry = 0;
  
  celkoveMetry = odoKilometry * 1000;
  tripMetry = tripKilometry * 1000;

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), snimacPreruseni, FALLING);
}

void loop() {
  unsigned long aktualniCas = millis();

  if (novyPulz) {
    noInterrupts();
    unsigned long interval = casMeziPulzy;
    novyPulz = false;
    interrupts();

    float rychlostFloat = (OBVOD_KOLA * 3600.0) / interval;
    
    if (rychlostFloat > 120.0) rychlost = 120; 
    else rychlost = (unsigned int)rychlostFloat;

    celkoveMetry += (OBVOD_KOLA);
    tripMetry += (OBVOD_KOLA);
    
    odoKilometry = celkoveMetry / 1000;
    tripKilometry = tripMetry / 1000;
  }

  if (aktualniCas - posledniPulzCas > 2000) {
    rychlost = 0;
  }

  if (aktualniCas - posledniZapisEEPROM > 300000) {
    unsigned long odoUlozene, tripUlozene;
    EEPROM.get(ADRESA_ODO, odoUlozene);
    EEPROM.get(ADRESA_TRIP, tripUlozene);
    
    if (odoKilometry != odoUlozene) EEPROM.put(ADRESA_ODO, odoKilometry);
    if (tripKilometry != tripUlozene) EEPROM.put(ADRESA_TRIP, tripKilometry);
    
    posledniZapisEEPROM = aktualniCas;
  }

  if (aktualniCas - posledniAktualizaceCas > 200) {
    vykresliDashboard();
    posledniAktualizaceCas = aktualniCas;
  }
}

void snimacPreruseni() {
  unsigned long cas = millis();
  if (cas - posledniPulzCas > MIN_CAS_MEZI_PULZY) {
    casMeziPulzy = cas - posledniPulzCas;
    posledniPulzCas = cas;
    novyPulz = true;
  }
}

// --- VYKRESLENÍ DASHBOARDU ---
void vykresliDashboard() {
  display.clearDisplay();
  
  // 1. Žlutá zóna: Fancy Stadioní logo nahoře
  display.drawXBitmap(0, 0, logo_stadion, STADION_WIDTH, STADION_HEIGHT, SSD1306_WHITE);
  
  // 2. Modrá zóna - OBŘÍ VYHLAZENÁ RYCHLOST
  display.setFont(&FreeSansBold24pt7b); // Aktivace anti-aliased fontu
  display.setTextColor(SSD1306_WHITE);
  
  // Pozor: Vektorové fonty kreslí od spodní linky (baseline). 
  // Y = 50 je ideální výška, aby se číslo 24pt vešlo mezi logo a spodní kilometry.
  // Používáme dynamické posunutí X podle toho, kolik má rychlost cifer (písmo nemá pevnou šířku)
  if (rychlost < 10) {
    display.setCursor(52, 52); 
  } else if (rychlost < 100) {
    display.setCursor(30, 52);
  } else {
    display.setCursor(10, 52);
  }
  display.print(rychlost);
  
  // 3. Vypnutí velkého fontu a návrat ke standardnímu pro drobné texty
  display.setFont(NULL); 
  
  // Jednotka "km/h" fixovaná v pravém rohu uprostřed modré zóny
  display.setTextSize(1);
  display.setCursor(104, 32);
  display.print("km/h");

  // 4. Spodní lišta s kilometry (úplně dole na pixelu 56)
  // ODO vlevo
  display.setCursor(0, 56);
  display.print(odoKilometry);
  display.print(" km");
  
  // TRIP vpravo
  display.setCursor(94, 56);
  display.print("T:");
  display.print(tripKilometry);

  display.display();
}
