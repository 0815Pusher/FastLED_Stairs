

#include <BH1750.h>  // Bibliothek für den BH1750 Lichtsensor
#include <Wire.h>    // Bibliothek für die I2C-Kommunikation
#include "FastLED.h" // Bibliothek für die Steuerung von LEDs

// Überprüfen der Version der FastLED-Bibliothek. Es wird mindestens Version 3.1 benötigt.
#if FASTLED_VERSION < 3001000
#error "FastLED 3.1 oder neuer wird benötigt; überprüfe github für den neuesten Code."
#endif

// Globale Variable für Debug-Modus Macro
bool debugMode = true;

#define DEBUG_PRINT(x) \
  if (debugMode)       \
  Serial.print(x)
#define DEBUG_PRINTLN(x) \
  if (debugMode)         \
  Serial.println(x)

// LED-Konfiguration
constexpr int DATA_PIN = 6;                      // Pin für das Datenkabel der WS2812b LEDs
constexpr int STAIR = 8;                         // Anzahl der Stufen
constexpr int LEDS_PER_STAIR = 15;               // Anzahl der LEDs pro Stufe
constexpr int NUM_LEDS = STAIR * LEDS_PER_STAIR; // Gesamtzahl der LEDs
constexpr int VOLTS = 5;                         // Volt der LEDs einstellen
constexpr int MAX_MA = 8000;                     // Maximalleistung der LEDs einstellen in MilliAmpere

// Variableninitialisierung
bool useLDR = true;               // flag, when true the program uses the LDR, set to "false" if you don't have a LDR sensor.
bool readPIRInputs = true;        // flag, when true, reads the PIR sensors. Disabled (false) by the program when LDR indicated that there is enough light.
int Brightness = 25;              // Globale Variable für die Helligkeit der LEDs
constexpr int LDRThreshold = 200; // Only switch on LED's at night when LDR senses low light conditions - you may have to change this value for your circumstances!
const int maxBrightness = 255;
const int minBrightness = 20;
const float minLux = 5;   // Minimaler Lux-Wert
const float maxLux = 255; // Maximaler Lux-Wert

unsigned long previousMillis = 0; // Vorheriger Zeitpunkt für Zeitmessung
unsigned long timeOut = 0;        // Zeitstempel zur Erinnerung, wann der PIR ausgelöst wurde.
int downUp = 0;                   // Variable zur Erinnerung an die Bewegungsrichtung (hoch oder runter)

// Sensor-Konfiguration
constexpr int alarmPinTop = 10;                      // PIR-Sensor oben an der Treppe
constexpr int alarmPinBottom = 11;                   // PIR-Sensor unten an der Treppe
constexpr unsigned long pirTimeout = 12000;          // Zeit in Millisekunden, nach der die LEDs ausgeschaltet werden
constexpr unsigned long delayBetweenChecks = 180000; // Intervall für LDR-Abfragen (Lichtsensor)
constexpr unsigned long totalOffTimeout = 30000;     // Zeit in Millisekunden, nach der die LEDs komplett ausgeschaltet bleiben
constexpr unsigned long debounceDelay = 3000;        // Wert für die Entprellung
// Zeitstempel für die Entprellung
unsigned long lastTriggerTimeTop = 0;
unsigned long lastTriggerTimeBottom = 0;

int alarmValueTop = LOW;    // Variable zur Speicherung des Status des oberen PIR-Sensors
int alarmValueBottom = LOW; // Variable zur Speicherung des Status des unteren PIR-Sensors
bool pirActive = true;      // Gibt an, ob der PIR-Sensor aktiv ist

BH1750 lightMeter(0x23); // Configuration des Light dependent resistor (LDR)

int LED_TAIL_LENGTH = 110; // Länge des Schweifs des Meteors
//Fire
#define COOLING 50
#define SPARKING 120

void runningLights(CRGB color = CRGB::Blue, uint16_t waveDelay = 50);

CRGB leds[NUM_LEDS]; // Array zur Speicherung der LED-Daten
// Farben-Array (aus Speicheroptimierungsgründen in PROGMEM)
const CRGB::HTMLColorCode colors[90] PROGMEM = {
    CRGB::Amethyst, CRGB::Aqua, CRGB::Blue, CRGB::BlueViolet, CRGB::Brown, CRGB::Brown, CRGB::Brown, CRGB::Chartreuse, CRGB::Chocolate, CRGB::Coral, CRGB::YellowGreen,
    CRGB::CornflowerBlue, CRGB::Crimson, CRGB::Cyan, CRGB::DarkBlue, CRGB::DarkCyan, CRGB::DarkGoldenrod, CRGB::DarkGray, CRGB::DarkGreen, CRGB::DarkMagenta, CRGB::DarkKhaki,
    CRGB::DarkOliveGreen, CRGB::DarkOrange, CRGB::DarkOrchid, CRGB::DarkRed, CRGB::DarkSalmon, CRGB::DarkSlateBlue, CRGB::DarkSlateGray, CRGB::DarkTurquoise, CRGB::DarkViolet,
    CRGB::DeepPink, CRGB::DeepSkyBlue, CRGB::DimGray, CRGB::DodgerBlue, CRGB::FireBrick, CRGB::ForestGreen, CRGB::Fuchsia, CRGB::Gold, CRGB::Goldenrod, CRGB::SandyBrown,
    CRGB::GreenYellow, CRGB::HotPink, CRGB::IndianRed, CRGB::Indigo, CRGB::Khaki, CRGB::LawnGreen, CRGB::LightSeaGreen, CRGB::LightSkyBlue, CRGB::LightSlateGray, CRGB::Lime,
    CRGB::LimeGreen, CRGB::Magenta, CRGB::Maroon, CRGB::MediumAquamarine, CRGB::MediumBlue, CRGB::MediumOrchid, CRGB::MediumPurple, CRGB::MediumSeaGreen, CRGB::MediumSlateBlue,
    CRGB::MediumSpringGreen, CRGB::MediumTurquoise, CRGB::MediumVioletRed, CRGB::MidnightBlue, CRGB::MintCream, CRGB::NavajoWhite, CRGB::Navy, CRGB::Olive, CRGB::OliveDrab,
    CRGB::Orange, CRGB::OrangeRed, CRGB::Orchid, CRGB::PaleVioletRed, CRGB::PaleGreen, CRGB::Plaid, CRGB::Purple, CRGB::Red, CRGB::RoyalBlue, CRGB::SeaGreen, CRGB::Green,
    CRGB::SlateBlue, CRGB::SkyBlue, CRGB::Snow, CRGB::SpringGreen, CRGB::SteelBlue, CRGB::Teal, CRGB::Tomato, CRGB::Turquoise, CRGB::Violet, CRGB::White, CRGB::Yellow};

void setup()
{
  Serial.begin(9600);
  DEBUG_PRINTLN("Debugging ist aktiviert.");
  DEBUG_PRINTLN("System wird initialisiert...");

  // pinMode(ledPin, OUTPUT);               // Onboard-LED (Pin 13) als Ausgang initialisieren
  pinMode(alarmPinTop, INPUT_PULLUP);    // PIR-Sensor oben an der Treppe als Eingang mit internem Pullup-Widerstand initialisieren
  pinMode(alarmPinBottom, INPUT_PULLUP); // PIR-Sensor unten an der Treppe als Eingang mit internem Pullup-Widerstand initialisieren
  delay(2000);                           // Der Sensor benötigt 2 Sekunden, um die Umgebung zu scannen, bevor er aktiviert werden kann.

  // FastLED-Bibliothek initialisieren
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(Brightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_MA); // Maximalleistung der LEDs einstellen  FastLED.clear();
  FastLED.clear();                                       // LEDs ausschalten
  FastLED.show();

  Wire.begin(); // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  if (lightMeter.begin(BH1750::ONE_TIME_LOW_RES_MODE))
  {
    DEBUG_PRINTLN("BH1750 Advanced begin");
  }
  else
  {
    DEBUG_PRINTLN("Error initializing BH1750");
  }
  if (useLDR) // Initialize all the LDR-readings to current values...
  {
    float lux = lightMeter.readLightLevel();
    DEBUG_PRINT(lux);
    DEBUG_PRINT(" LUX vom LDR gemessen, ab <= ");
    DEBUG_PRINT(LDRThreshold);
    DEBUG_PRINTLN(" LUX gehts los :-) ");
  }
  checkLDR();
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= delayBetweenChecks) // Überprüfen des Lichtsensors nur, wenn genügend Zeit vergangen ist
  {
    previousMillis = currentMillis;
    checkLDR();
  }
  if (readPIRInputs) // Handhaben der PIR-Eingänge nur, wenn der Lichtsensor dies zulässt
  {
    handlePIR();
  }
}

void checkLDR()
{
  DEBUG_PRINTLN("Intervall Checking LDR: ");

  if (lightMeter.measurementReady(true))
  {
    float lux = lightMeter.readLightLevel();
    lightMeter.configure(BH1750::ONE_TIME_LOW_RES_MODE);
    DEBUG_PRINT(lux);
    DEBUG_PRINT(" LUX vom LDR gemessen, ab <= ");
    DEBUG_PRINT(LDRThreshold);
    DEBUG_PRINTLN(" LUX werden die Led`s Aktiv :) ");

    // Helligkeit basierend auf Lux-Wert anpassen & linear zwischen minBrightness und maxBrightness skalieren
    Brightness = mapBrightness(lux, minLux, maxLux, minBrightness, maxBrightness);
    DEBUG_PRINT("Led`s angepasst auf ");
    DEBUG_PRINT(Brightness);
    DEBUG_PRINTLN(" Helligkeit !");

    if (lux > LDRThreshold && useLDR)
    {
      readPIRInputs = false;
      DEBUG_PRINTLN("Ausreichendes Umgebungslicht erkannt, Led`s Aus !");
      clearLEDs();
    }
    else
    {
      readPIRInputs = true;
      DEBUG_PRINTLN("Geringes Umgebungslicht erkannt, Led`s Aktiv !");
    }
  }
  else
  {
    DEBUG_PRINT("warte auf LDR messung...");
  }
}

// Funktion zur Skalierung der Helligkeit basierend auf Lux-Wert
int mapBrightness(float lux, float in_min, float in_max, int out_min, int out_max)
{
  // Begrenze die Lux-Werte auf den definierten Bereich
  if (lux < in_min)
    lux = in_min;
  if (lux > in_max)
    lux = in_max;
  // Lineare Skalierung der Helligkeit
  return (int)((lux - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void handlePIR()
{
  // PIR-Sensoren auslesen
  int alarmValueTop = digitalRead(alarmPinTop);
  int alarmValueBottom = digitalRead(alarmPinBottom);
  unsigned long currentMillis = millis(); // Aktuelle Zeit speichern
  if (alarmValueTop == HIGH)              // Entprellungslogik für den oberen PIR-Sensor
  {
    if (currentMillis - lastTriggerTimeTop > debounceDelay)
    {
      lastTriggerTimeTop = currentMillis; // Zeitstempel aktualisieren
      DEBUG_PRINT("   Top PIR-Sensor stabil ausgeloest Mode: ");
      timeOut = currentMillis;
      downUp = 1;
      topdown(random8(0, 42));
    }
  }
  if (alarmValueBottom == HIGH) // Entprellungslogik für den unteren PIR-Sensor
  {
    if (currentMillis - lastTriggerTimeBottom > debounceDelay)
    {
      lastTriggerTimeBottom = currentMillis; // Zeitstempel aktualisieren
      DEBUG_PRINT("   Bottom PIR-Sensor stabil ausgeloest Mode: ");
      timeOut = currentMillis;
      downUp = 2;
      topdown(random8(0, 42));
    }
  }

  // Logik für das Ausschalten der LEDs nach einer bestimmten Zeit
  if ((timeOut + pirTimeout < currentMillis) && (downUp == 1 || downUp == 2))
  {
    DEBUG_PRINT("   LEDs nach PIR-Timeout ausschalten Mode: ");
    ledoff(random8(0, 6));
    downUp = 0;
  }
  if (timeOut + totalOffTimeout < millis()) // Logik für das vollständige Ausschalten der LEDs nach einem längeren Timeout
  {
    clearLEDs();
  }
}

// Funktion zum Ausschalten der LEDs
void clearLEDs()
{
  FastLED.clear();
  FastLED.show();
}

// Funktion zur Handhabung von Lichteffekten.
void topdown(int mode)
{
  DEBUG_PRINT(mode);
  switch (mode)
  {
  case 0:
    colourWipeDownUp(colors[random8(0, 90)], 3, 1); // Farbe: zufällige Auswahl, Geschwindigkeit: 3, Richtung: unten nach oben
    DEBUG_PRINTLN(" Pattern: colourWipeDownUp");
    break;
  case 1:
    colourWipeDownUp(colors[random8(0, 90)], 3, 0); // Farbe: zufällige Auswahl, Geschwindigkeit: 3, Richtung: oben nach unten
    DEBUG_PRINTLN(" Pattern: colourWipeDownUp");
    break;
  case 2:
    DEBUG_PRINTLN(" Pattern: efekt2"); // Effekt 2 mit Farbe: zufällige Auswahl, Dauer: 50ms, Richtung: 1
    efekt2(50, colors[random8(0, 90)], 1);
    break;
  case 3:
    DEBUG_PRINTLN(" Pattern: efekt1"); // Effekt 1 mit Farbe: zufällige Auswahl, Dauer: 50ms
    efekt1(50, colors[random8(0, 90)]);
    break;
  case 4:
    DEBUG_PRINTLN(" Pattern: efekt"); // Effekt mit Farbe: zufällige Auswahl, Dauer: 50ms
    efekt(50, colors[random8(0, 90)]);
    break;
  case 5:
    DEBUG_PRINTLN(" Pattern: rainbow"); // Regenbogeneffekt
    rainbow();
    break;
  case 6:
    DEBUG_PRINTLN(" Pattern: colorCycle"); // Farbzykluseffekt mit einer Dauer von 120ms
    colorCycle(100);
    break;
  case 7:
    DEBUG_PRINTLN(" Pattern: rainbowWave"); // Regenbogenwellen-Effekt mit einer Dauer von 50ms, Schleife: true
    rainbowWave(80);
    break;
  case 8:
    DEBUG_PRINTLN(" Pattern: meteorShower"); // Meteorregen-Effekt mit Farbe: zufällige Auswahl, Geschwindigkeit: 100ms
    meteorShower(colors[random8(0, 90)], 60);
    break;
  case 9:
    DEBUG_PRINTLN(" Pattern: colorWave"); // Farbwelleneffekt mit Farbe: zufällige Auswahl, Dauer: 50ms
    colorWave(colors[random8(0, 90)], 50);
    break;
  case 10:
    DEBUG_PRINTLN(" Pattern: rainbowWave (neue Methode)"); // Regenbogenwellen-Effekt (neue Methode) mit einer Dauer von 100ms
    rainbowWave1(100);
    break;
  case 11:
    DEBUG_PRINTLN(" Pattern: meteorRain"); // Meteorschauer-Effekt mit zufälliger Farbe, Größe: 60,
    meteorRain(255, 255, 255, true, 60);
    break;
  case 12:
    DEBUG_PRINTLN(" Pattern: bottomup_colorchange"); // Farbwechsel von unten nach oben
    bottomup_colorchange();
    break;
  case 13:
    DEBUG_PRINTLN(" Pattern: breathingEffect"); // Atmungseffekt mit blauer Farbe, Dauer: 6000ms, min. Helligkeit: 10, max. Helligkeit: 100, Geschwindigkeit: 1.0
    breathingEffect(CRGB::Purple, 5000, 5, 140, 1.0);
    break;
  case 14:
    DEBUG_PRINTLN(" Pattern: cometEffect"); // Kometeneffekt mit Farbe: Violet, Größe: 60, Verfall: 64, Geschwindigkeit: 40ms
    comet(CRGB::Violet, 100, 120, 30);
    break;
  case 15:
    DEBUG_PRINTLN(" Pattern: runningLights"); // Lauflicht-Effekt
    runningLights();
    break;
  case 16:
    waveEffect(CRGB::Orchid, 50); // Parameter: Farbe (Blau), Verzögerung (100 ms) zwischen den Positionen.
    DEBUG_PRINTLN(" Pattern: waveEffect");
    break;
  case 17:
    DEBUG_PRINTLN(" Pattern: efekt2"); // Effekt 2 mit Farbe: zufällige Auswahl, Dauer: 50ms, Richtung: 0
    efekt2(50, colors[random8(0, 90)], 0);
    break;
  case 18:
    DEBUG_PRINTLN(" Pattern: wavePattern"); // Debug-Ausgabe: Name des aktuellen Musters
    wavePattern(CRGB::Red, CRGB::Blue, 80); // Wellenmuster mit rot und blau, Geschwindigkeit 80
    break;
  case 19:
    confetti(50);                        // Konfetti-Muster mit einer Verzögerung von 50 ms
    DEBUG_PRINTLN(" Pattern: confetti"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 20:
    DEBUG_PRINTLN(" Pattern: efekt1");  // Debug-Ausgabe: Name des aktuellen Musters
    efekt1(30, colors[random8(0, 90)]); // Effekt1 mit 30 ms Verzögerung und zufälliger Farbe aus dem Array 'colors'
    break;
  case 21:
    colorFlash(CRGB::Maroon, 80); // Parameter: Farbe (Rot), Blitzdauer (100 ms).
    DEBUG_PRINTLN(" Pattern: colorFlash");
    break;
  case 22:
    sparkleEffect(50, 200, 8, true);          // Funkeleffekt mit 50 ms Verzögerung, Helligkeit 200, 8 Funken, zufällige Farben
    DEBUG_PRINTLN(" Pattern: sparkleEffect"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 23:
    fadeToColor(CRGB::Purple, 30, 100);     // Farbübergang zu lila, mit 30 Schritten und Geschwindigkeit 100
    DEBUG_PRINTLN(" Pattern: fadeToColor"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 24:
    sparkleEffect(80, 200, 28, true);         // Funkeleffekt mit 50 ms Verzögerung, Helligkeit 200, 8 Funken, zufällige Farben
    DEBUG_PRINTLN(" Pattern: sparkleEffect"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 25:
    plasma();                          // Plasma-Effekt
    DEBUG_PRINTLN(" Pattern: plasma"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 26:
    twinkle(100);                       // Funkeln-Muster mit 100 ms Verzögerung
    DEBUG_PRINTLN(" Pattern: twinkle"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 27:
    colorExplosion(CRGB::DarkMagenta, 30);     // Farbeexplosionseffekt mit dunkelmagenta und Geschwindigkeit 30
    DEBUG_PRINTLN(" Pattern: colorExplosion"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 28:
    electricBolts(20, CRGB::Blue, 30);        // Elektrische Blitze mit 5 Blitzen, blau, Geschwindigkeit 30
    DEBUG_PRINTLN(" Pattern: electricBolts"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 29:
    hypnoticSpiral(20, 2);                     // Hypnotische Spirale mit Farbwechselgeschwindigkeit 10 und Geschwindigkeit 2
    DEBUG_PRINTLN(" Pattern: hypnoticSpiral"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 30:
    colorWipe(CRGB::Blue, 50);            // Farbiger Wisch-Effekt mit blau und 50 ms Verzögerung
    DEBUG_PRINTLN(" Pattern: colorWipe"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 31:
    sunrise(30, 100);                   // Sonnenaufgangseffekt mit 30 Schritten und Geschwindigkeit 100
    DEBUG_PRINTLN(" Pattern: sunrise"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 32:
    fadingCircles(CRGB::Gold, 20, 50);        // Verblassende Kreise mit Gold, 20 Schritten und 50 ms Verzögerung
    DEBUG_PRINTLN(" Pattern: fadingCircles"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 33:
    fireworks(0, 0, 255, 60); // blaue Funken
    DEBUG_PRINTLN(" Pattern: fireworks");
    break;
  case 34:
    waveEffect(CRGB::Blue, 40); // Parameter: Farbe (Blau), Verzögerung (100 ms) zwischen den Positionen.
    DEBUG_PRINTLN(" Pattern: waveEffect");
    break;
  case 35:
    policeLights(100); // Parameter: Blitzdauer (100 ms) pro Farbwechsel.
    DEBUG_PRINTLN(" Pattern: policeLights");
    break;
  case 36:
    scannerEffect(CRGB::White, 40);           // Scanner-Effekt mit weißer Farbe und einer Verzögerung von 10 ms zwischen den Positionen.
    DEBUG_PRINTLN(" Pattern: scannerEffect"); // Debug-Ausgabe: Name des aktuellen Musters
    break;
  case 37:
    fireworks(255, 0, 0, 40); // Rote Funken
    DEBUG_PRINTLN(" Pattern: fireworks");
    break;
  case 38:
    theaterChase(CRGB::Purple, 100); // Parameter: Farbe (Lila), Verzögerung (100 ms) zwischen den Blinken.
    DEBUG_PRINTLN(" Pattern: theaterChase");
    break;
  case 39:
    colorCascade(CRGB::WhiteSmoke, 20); // Parameter: Farbe (Gelb), Geschwindigkeit (50 ms) zwischen den Positionen.
    DEBUG_PRINTLN(" Pattern: colorCascade");
    break;
  case 40:
    sparkleEffectRainbow(50); // Bunter Funkeneffekt
    DEBUG_PRINTLN(" Pattern: sparkleEffect (Rainbow)");
    break;
  case 41:
    colorExplosion(CRGB::Pink, 50); // Parameter: Farbe (Pink), Blitzdauer (50 ms).
    DEBUG_PRINTLN(" Pattern: colorExplosion");
    break;
  case 42:
    randomBlink(CRGB::Cyan, 100); // Parameter: Farbe (Cyan), Verzögerung (100 ms) zwischen den Blinkaktionen.
    DEBUG_PRINTLN(" Pattern: randomBlink");
    break;
  case 43:
    theaterChase(50);
    DEBUG_PRINTLN(" Pattern: theaterChase");
    break;
  }
}

void ledoff(int mode) // Funktion zur Handhabung des Ausschaltens der LEDs
{
  DEBUG_PRINT(mode);
  switch (mode) // Auswahl der Ausschaltmethode basierend auf dem übergebenen Modus
  {
  case 0:
    DEBUG_PRINTLN(" all_off(500)");
    all_off(500); // Alle LEDs ausschalten mit einer Verzögerung von 500ms
    break;
  case 1:
    DEBUG_PRINTLN(" Pattern: efekt"); // Effekt mit schwarzer Farbe, Dauer: 50ms
    efekt(50, CRGB::Black);
    break;
  case 2:
    DEBUG_PRINTLN(" Pattern: efekt1");
    efekt1(50, CRGB::Black); // Effekt 1 mit schwarzer Farbe, Dauer: 50ms
    break;
  case 3:
    optimizedEfekt1(100, CRGB::Black, 150);     // Optimierte Version von Effekt 1 zum Ausschalten.    // Parameter: Wartezeit 50 ms, Farbe Schwarz, Fadewert 50
    DEBUG_PRINTLN(" Pattern: optimizedEfekt1"); // Effekt 2 mit schwarzer Farbe, Dauer: 50ms, Weißblitz: 0
    break;
  case 4:
    efekt2(60, CRGB::Black, 0);
    DEBUG_PRINTLN(" Pattern: efekt2"); // Effekt 2 mit schwarzer Farbe, Dauer: 50ms, Weißblitz: 0
    break;
  case 5:
    off1(50, CRGB::Black, 5); // Schaltet die LEDs mit einem Fading-Effekt aus. // Parameter: Wartezeit 50 ms, Farbe Schwarz, Fadewert 50.
    DEBUG_PRINTLN(" Pattern: off1");
    break;
  case 6:
    smoothFadeOff(CRGB::Black, 5, 50); // Reduziert die Helligkeit in 10 Schritten, jeweils mit 50 ms Wartezeit.Parameter: Farbe Schwarz, Anzahl der Schritte 10, Wartezeit pro Schritt 50 ms.
    DEBUG_PRINTLN(" OFF: smoothFadeOff");
    break;
  default:
    all_off(300); // Standardmäßiges Ausschalten, falls der Effekt nicht definiert ist
    DEBUG_PRINTLN(" all_off(300)");
    break;
  }
  DEBUG_PRINTLN("Aus, warte auf Bewegung!");
}

void all_off(int wait)
{
  if (downUp != 2) // Wenn downUp nicht 2 ist, werden die LEDs von unten nach oben ausgeschaltet.
  {
    for (int j = 0; j < STAIR; j++)
    {
      int start = NUM_LEDS / STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int i = start; i < start + LEDS_PER_STAIR; i++)
      {
        leds[i] = CRGB::Black; // Setzt alle LEDs in der aktuellen Treppenstufe auf Schwarz (aus).
      }
      FastLED.show();      // Aktualisiert die LED-Anzeige.
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zur nächsten Stufe übergegangen wird.
    }
  }
  if (downUp != 1) // Wenn downUp nicht 1 ist, werden die LEDs von oben nach unten ausgeschaltet.
  {
    for (int j = STAIR; j > 0; j--)
    {
      int start = NUM_LEDS / STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int i = start; i > start - LEDS_PER_STAIR; i--)
      {
        leds[i - 1] = CRGB::Black; // Setzt alle LEDs in der aktuellen Treppenstufe auf Schwarz (aus).
      }
      FastLED.show();      // Aktualisiert die LED-Anzeige.
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zur nächsten Stufe übergegangen wird.
    }
  }
  else
  {
    clearLEDs();
  }
}


void smoothFadeOff(CRGB color, int steps, int wait)
{
  for (int j = 0; j < steps; j++)
  {
    for (int stair = 0; stair < STAIR; stair++)
    {
      for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
      {
        leds[i] = color;
        leds[i].nscale8(255 - (255 / steps) * j); // Sanfte Reduktion der Helligkeit
      }
    }
    FastLED.show();
    FastLED.delay(wait);
  }
  FastLED.clear(); // Am Ende alle LEDs ausschalten
  FastLED.show();
}

void off1(int wait, CRGB color, int fadeAmount)
{
  for (int stair = 0; stair < STAIR; stair++)
  {
    for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
    {
      leds[i] = color;
      leds[i].fadeToBlackBy(fadeAmount); // Variabler Fadewert
    }
    FastLED.show();
    FastLED.delay(wait);
  }
}

void optimizedEfekt1(int wait, CRGB color, int fadeAmount)
{
  for (int stair = 0; stair < STAIR; stair++) // Schleife für jede Treppe
  {
    int start = stair * LEDS_PER_STAIR;
    int end = start + LEDS_PER_STAIR;
    fill_solid(leds + start, LEDS_PER_STAIR, color); // LEDs mit der angegebenen Farbe füllen
    for (int i = start; i < end; i++)                // LEDs innerhalb der Treppe verblassen
    {
      leds[i].fadeToBlackBy(fadeAmount);
    }
    FastLED.show();      // LEDs anzeigen
    FastLED.delay(wait); // Wartezeit
  }
}




void Fire()
{
 static byte heat[NUM_LEDS];

 while (millis() - timeOut < pirTimeout)
 {
   // Step 1.  Cool down every cell a little
   for (int i = 0; i < NUM_LEDS; i++)
   {
     heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
   }

   // Step 2.  Heat from each cell drifts 'up' and diffuses a little
   for (int k = NUM_LEDS - 1; k >= 2; k--)
   {
     heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
   }

   // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
   if (random8() < SPARKING)
   {
     int y = random8(7);
     heat[y] = qadd8(heat[y], random8(160, 255));
   }

   // Step 4.  Map from heat cells to LED colors
   for (int j = 0; j < NUM_LEDS; j++)
   {
     leds[j] = HeatColor(heat[j]);
   }
   FastLED.show();
   FastLED.delay(50);
 }
}


void theaterChase(int wait)
{
  while (millis() - timeOut < pirTimeout) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    for (int q = 0; q < 3; q++) // Schleife für die drei Phasen des Theater Chase Effekts
    {
      for (int i = 0; i < NUM_LEDS; i = i + 3) // Schleife für die LEDs, die in Abständen von 3 gesetzt werden
      {
        leds[i + q] = CHSV(i * 255 / NUM_LEDS, 255, 255); // Setze die LED an Position i+q auf eine Farbe, die einen Farbverlauf erzeugt
      }
      // Zeige die LEDs an
      FastLED.show();
      // Warte für die angegebene Zeit
      FastLED.delay(wait);
      for (int i = 0; i < NUM_LEDS; i = i + 3) // Lösche die Farbe der LEDs an Position i+q
      {
        leds[i + q] = CRGB::Black;
      }
    }
  }
}

void fireworks(uint8_t red, uint8_t green, uint8_t blue, uint16_t wait)
{
  while (millis() - timeOut < pirTimeout) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].fadeToBlackBy(150); // Alle LEDs leicht abdunkeln
    }
    int pos = random(NUM_LEDS);         // Zufällige Position für das Feuerwerk
    leds[pos] = CRGB(red, green, blue); // LED an der zufälligen Position auf die angegebene Farbe setzen
    FastLED.show();                     // LEDs anzeigen
    delay(wait);                        // Wartezeit
  }
}

void confetti(uint16_t wait)
{
  while (millis() - timeOut < pirTimeout) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    fadeToBlackBy(leds, NUM_LEDS, 10);      // LEDs leicht abdunkeln
    int pos = random16(NUM_LEDS);           // Zufällige Position für das Konfetti
    leds[pos] += CHSV(random8(), 200, 255); // Zufällige Farbe an der Position hinzufügen
    FastLED.show();                         // LEDs anzeigen
    delay(wait);                            // Wartezeit
  }
}

void twinkle(uint16_t wait)
{
  while (millis() - timeOut < pirTimeout) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (random8() < 25) // 25% Chance, dass die LED funkelt
      {
        leds[i] = CHSV(random8(), 255, random8(128, 255)); // Zufällige Farb- und Helligkeitswerte
      }
      else
      {
        leds[i].fadeToBlackBy(200); // LED abdunkeln
      }
    }
    FastLED.show(); // LEDs anzeigen
    delay(wait);    // Wartezeit
  }
}

void sparkleEffectRainbow(uint16_t wait)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black; // Setze alle LEDs auf Schwarz
  }
  FastLED.show(); // LEDs anzeigen

  while (millis() - timeOut < pirTimeout) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    int pos = random16(NUM_LEDS);    // Zufällige Position für das Funkeln
    uint8_t hue = random8();         // Zufälliger Farbton
    leds[pos] = CHSV(hue, 255, 255); // LED an der Position auf zufällige Regenbogenfarbe setzen
    FastLED.show();                  // LEDs anzeigen
    delay(wait);                     // Wartezeit
    leds[pos] = CRGB::Black;         // LED an der Position auf Schwarz setzen
    FastLED.show();                  // LEDs anzeigen
  }
}

void sparkleEffect(uint16_t wait, uint8_t brightness, uint8_t numSparks, bool useRandomColors)
{
  // Setze die Helligkeit auf den angegebenen Wert
  FastLED.setBrightness(brightness);
  // Setze alle LEDs auf Schwarz
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Black;
  }
  FastLED.show(); // LEDs anzeigen
  // Wiederhole den Sparkle-Effekt, solange die Zeit innerhalb des PIR-Timeouts liegt
  while (millis() - timeOut < pirTimeout)
  {
    // Setze die gewünschte Anzahl an zufälligen LEDs auf
    for (int i = 0; i < numSparks; i++)
    {
      int pos = random16(NUM_LEDS);
      if (useRandomColors)
      {
        uint8_t hue = random8(); // Zufällige Farbe
        leds[pos] = CHSV(hue, 255, 255);
      }
      else
      {
        leds[pos] = CRGB::White; // Weiße Farbe, wenn keine zufälligen Farben verwendet werden
      }
    }
    FastLED.show(); // LEDs anzeigen
    // Warte für die angegebene Zeit
    delay(wait);
    // Setze alle LEDs wieder auf Schwarz
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
    FastLED.show(); // LEDs anzeigen
  }
}

void dropEffect(CRGB color, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = color;
      FastLED.show();
      FastLED.delay(speedDelay);
      leds[i] = CRGB::Black;
    }
    for (int i = NUM_LEDS - 1; i >= 0; i--)
    {
      leds[i] = color;
      FastLED.show();
      FastLED.delay(speedDelay);
      leds[i] = CRGB::Black;
    }
  }
}
void colorFlash(CRGB color, int flashDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    {
      fill_solid(leds, NUM_LEDS, color);
      FastLED.show();
      FastLED.delay(flashDelay);
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      FastLED.delay(flashDelay);
    }
  }
}
void policeLights(int flashDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < 5; i++)
    {
      fill_solid(leds, NUM_LEDS / 2, CRGB::Blue);
      fill_solid(leds + NUM_LEDS / 2, NUM_LEDS / 2, CRGB::Red);
      FastLED.show();
      FastLED.delay(flashDelay);
      fill_solid(leds, NUM_LEDS / 2, CRGB::Red);
      fill_solid(leds + NUM_LEDS / 2, NUM_LEDS / 2, CRGB::Blue);
      FastLED.show();
      FastLED.delay(flashDelay);
    }
  }
}
void waveEffect(CRGB color, int waveDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = color;
      FastLED.show();
      FastLED.delay(waveDelay);
      leds[i] = CRGB::Purple;
    }
    for (int i = NUM_LEDS - 1; i >= 0; i--)
    {
      leds[i] = color;
      FastLED.show();
      FastLED.delay(waveDelay);
      leds[i] = CRGB::Black;
    }
  }
}

void scannerEffect(CRGB color, int speedDelay)
{
  while (millis() - timeOut < pirTimeout) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    // LEDs nacheinander von Anfang bis Ende einschalten und dann wieder ausschalten
    for (int i = 0; i < NUM_LEDS; i++) // Schleife von der ersten bis zur letzten LED
    {
      leds[i] = color;                              // Setze die LED an Position i auf die angegebene Farbe
      FastLED.show();                               // LEDs anzeigen
      FastLED.delay(speedDelay);                    // Wartezeit zwischen den Positionen
                                                    // leds[i] = CRGB::DarkBlue;     // Setze die LED an Position i auf Schwarz (ausschalten)
      leds[i] = CHSV(i * 255 / NUM_LEDS, 255, 255); // CHSV erzeugt einen Farbverlauf
                                                    // leds[i] = CRGB(128, 0, 255);  // Lila (eine Mischung aus Blau und Rot)
    }
    // LEDs nacheinander von Ende bis Anfang einschalten und dann wieder ausschalten
    for (int i = NUM_LEDS - 1; i >= 0; i--) // Schleife von der letzten bis zur ersten LED
    {
      leds[i] = color;           // Setze die LED an Position i auf die angegebene Farbe
      FastLED.show();            // LEDs anzeigen
      FastLED.delay(speedDelay); // Wartezeit zwischen den Positionen
      leds[i] = CRGB::Black;     // Setze die LED an Position i auf Schwarz (ausschalten)
    }
  }
}

void colorWipe(CRGB color, uint16_t wait)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = color;
      FastLED.show();
      delay(wait);
    }
    delay(500);
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
      FastLED.show();
      delay(wait);
    }
    delay(500);
  }
}

void colorCascade(CRGB color, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = color;
      FastLED.show();
      FastLED.delay(speedDelay);
      leds[i] = CRGB::Black;
    }
  }
}

void theaterChase(CRGB color, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {

    for (int q = 0; q < 3; q++)
    {
      for (int i = 0; i < NUM_LEDS; i = i + 3)
      {
        leds[i + q] = color;
      }
      FastLED.show();
      FastLED.delay(speedDelay);
      for (int i = 0; i < NUM_LEDS; i = i + 3)
      {
        leds[i + q] = CRGB::Black;
      }
    }
  }
}

void randomBlink(CRGB color, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
    FastLED.show();

    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[random16(NUM_LEDS)] = color;
      FastLED.show();
      FastLED.delay(speedDelay);
    }
  }
}

void hypnoticSpiral(uint8_t hueShift, uint8_t speed)
{
  static uint16_t startIndex = 0;
  startIndex += speed; // Bewegung der Spirale
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV((startIndex + i * hueShift) & 255, 255, 255);
    }
    FastLED.show();
    FastLED.delay(20);
  }
}

void colorExplosion(CRGB baseColor, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int radius = 0; radius < NUM_LEDS / 2; radius++)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        if (abs(i - NUM_LEDS / 2) <= radius)
        {
          leds[i] = baseColor;
        }
      }
      FastLED.show();
      FastLED.delay(speedDelay);
    }
  }
}

void electricBolts(int numBolts, CRGB color, int speedDelay)
{
  FastLED.clear();

  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < numBolts; i++)
    {
      int pos = random(NUM_LEDS);
      leds[pos] = color;
      FastLED.show();
      FastLED.delay(speedDelay);
      leds[pos].fadeToBlackBy(150);
    }
  }
}

void sunrise(int steps, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < steps; i++)
    {
      int red = (sin(i * PI / steps) * 255);
      int green = (sin(i * PI / (steps * 2)) * 128);
      fill_solid(leds, NUM_LEDS, CRGB(red, green, 0));
      FastLED.show();
      FastLED.delay(speedDelay);
    }
  }
}

void fadingCircles(CRGB color, int steps, int speedDelay)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < steps; i++)
    {
      fill_solid(leds, NUM_LEDS, color);
      FastLED.setBrightness(sin8(i * 256 / steps));
      FastLED.show();
      FastLED.delay(speedDelay);
    }
  }
  FastLED.setBrightness(Brightness);
}

void plasma()
{
  static uint16_t t = 0;
  static uint16_t x = 0;
  while (millis() - timeOut < pirTimeout)
  {
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
      uint8_t index = sin8(i * 16 + t) + sin8(i * 16 + t + 128);
      leds[i] = ColorFromPalette(PartyColors_p, index, 255, LINEARBLEND);
    }
    t++;
    x += 16;

    FastLED.show();
    FastLED.delay(50);
  }
}

void fadeToColor(CRGB targetColor, int steps, int delayTime)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < steps; i++)
    {
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = blend(leds[j], targetColor, 255 * i / steps);
      }
      FastLED.show();
      FastLED.delay(delayTime);
    }
  }
}

// Wave Pattern: LEDs erzeugen ein wellenartiges Muster
void wavePattern(CRGB color1, CRGB color2, uint16_t wait)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = blend(color1, color2, sin8(i * 255 / NUM_LEDS)); // Mische zwei Farben basierend auf einer Sinuswelle
    }
    FastLED.show();
    FastLED.delay(wait); // Geschwindigkeit des Effekts
  }
}

// Erweiterter Atmungseffekt: Anpassung der minimalen und maximalen Helligkeit sowie der Geschwindigkeit
void breathingEffect(uint32_t color, uint16_t duration, uint8_t minBrightness, uint8_t maxBrightness, float speedFactor)
{
  while (millis() - timeOut < pirTimeout)
  {
    // Helligkeit erhöhen
    for (uint16_t brightness = minBrightness; brightness <= maxBrightness; brightness++)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = color;
        leds[i].fadeLightBy(255 - brightness); // Helligkeit reduzieren
      }
      FastLED.show();
      FastLED.delay(duration / (512 * speedFactor)); // Geschwindigkeit anpassen
    }
    // Helligkeit verringern
    for (uint16_t brightness = maxBrightness; brightness >= minBrightness; brightness--)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = color;
        leds[i].fadeLightBy(255 - brightness); // Helligkeit reduzieren
      }
      FastLED.show();
      FastLED.delay(duration / (512 * speedFactor)); // Geschwindigkeit anpassen
    }
  }
}
// Meteorschauer: Erzeugt den Effekt eines bewegenden Kometen mit einem Schweif
void meteorShower(uint32_t color, uint16_t wait)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS + LED_TAIL_LENGTH; i++)
    {
      // Alle LEDs verblassen
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j].fadeToBlackBy(120); // Fade by a fixed amount for the trailing effect
      }
      // Erzeuge den Kometen und seinen Schweif
      for (int j = 0; j < LED_TAIL_LENGTH; j++)
      {
        if ((i - j >= 0) && (i - j < NUM_LEDS))
        {
          leds[i - j] = color; // Set current LED to the comet color
        }
      }
      FastLED.show();
      FastLED.delay(wait); // Geschwindigkeit des Effekts
    }
  }
}
// Farbwelle: Erzeugt eine wellenförmige Farbbewegung
void colorWave(uint32_t color, uint16_t wait)
{
  uint8_t waveSpeed = 20; // Geschwindigkeit der Welle
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = CHSV((i + j * waveSpeed) % 255, 255, 255); // Farbwechsel mit wellenförmiger Bewegung
      }
      FastLED.show();
      FastLED.delay(wait); // Geschwindigkeit der Welle
    }
  }
}

void rainbowWave1(uint16_t wait)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (uint16_t j = 0; j < 256; j++)
    {
      for (uint16_t i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CHSV((i + j) & 255, 255, 255); // Farbwechsel durch Verschiebung
      }
      FastLED.show();
      FastLED.delay(wait); // Geschwindigkeit des Farbwechsels
    }
  }
}

void comet(CRGB color, int cometSize, int speedDelay, int cometTrailDecay)
{
  FastLED.clear();
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS * 2; i++)
    {
      for (int j = 0; j < NUM_LEDS; j++)
      {
        if ((j < i) && (j > i - cometSize))
        {
          leds[j] = color;
        }
        else
        {
          leds[j].fadeToBlackBy(cometTrailDecay);
        }
      }
      FastLED.show();
      FastLED.delay(speedDelay);
    }
  }
}

// Farbzyklus: Wechselt alle LEDs durch den gesamten Farbraum
void colorCycle(uint16_t wait)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int hue = 0; hue < 256; hue++)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CHSV(hue, 255, 255); // Setze Farbe basierend auf dem aktuellen Farbwert
      }
      FastLED.show();
      FastLED.delay(wait); // Geschwindigkeit des Farbwechsels
    }
  }
}

// Laufende Lichter: Erzeugt eine wellenartige Bewegung in einer bestimmten Farbe
void runningLights(CRGB color, uint16_t waveDelay)
{
  int position = 0;
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS * 2; i++)
    {
      position++; // Move the light wave
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = color;
        leds[j].fadeToBlackBy((sin8(j + position * 10) * 255) / 256); // Create wave movement
      }
      FastLED.show();
      FastLED.delay(waveDelay); // Speed of the wave
    }
  }
}
// Regenbogen: Füllt die LEDs mit einem sich bewegenden Regenbogen
void rainbow()
{
  while (millis() - timeOut < pirTimeout)
  {
    fill_rainbow(leds, NUM_LEDS, 0, 7); // Regenbogenfarben füllen
    FastLED.show();
  }
}

// Regenbogenwelle: Erzeugt eine laufende Regenbogenwelle
void rainbowWave(uint16_t wait)
{
  uint16_t hue = 0;
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(hue + (i * 10), 255, 255);
    }
    FastLED.show();
    hue += 1;
    delay(wait);
  }
}

void meteorRain(CRGB color, uint8_t meteorSize, uint8_t meteorTrailDecay, boolean meteorRandomDecay, uint16_t wait)
{
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Setze alle LEDs auf Schwarz
  while (millis() - timeOut < pirTimeout)
  {
    for (int i = 0; i < NUM_LEDS + NUM_LEDS; i++)
    {
      // Bewege den Meteor vorwärts
      for (int j = 0; j < NUM_LEDS; j++)
      {
        if ((!meteorRandomDecay) || (random8() > 64))
        {
          leds[j].fadeToBlackBy(meteorTrailDecay);
        }
      }
      // Setze die LED-Position für den Meteor
      for (int j = 0; j < meteorSize; j++)
      {
        if ((i - j < NUM_LEDS) && (i - j >= 0))
        {
          leds[i - j] = color;
        }
      }
      FastLED.show();
      FastLED.delay(wait);
    }
  }
}

// Function to change colors step by step
void bottomup_colorchange()
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int stair = STAIR - 1; stair >= 0; stair--)
    {
      CRGB color = colors[random16(0, 90)];
      for (int led = 0; led < LEDS_PER_STAIR; led++)
      {
        leds[stair * LEDS_PER_STAIR + led] = color;
      }
      FastLED.show();
      delay(50);
    }
  }
}

// Function to light up the stairs from bottom to top with a single color
void bottomup_simple(CRGB color)
{
  while (millis() - timeOut < pirTimeout)
  {
    for (int stair = STAIR - 1; stair >= 0; stair--)
    {
      for (int led = 0; led < LEDS_PER_STAIR; led++)
      {
        leds[stair * LEDS_PER_STAIR + led] = color;
      }
      FastLED.show();
      delay(50);
    }
  }
}

void colourWipeDownUp(uint32_t color, int wait, int r)
{
  // Wenn downUp nicht 2 ist, werden die LEDs von unten nach oben eingefärbt.
  if (downUp != 2)
  {
    for (int j = 0; j < STAIR; j++)
    {
      if (r == 1)
      { // Falls r 1 ist, wird eine zufällige Farbe ausgewählt.
        color = colors[random8(0, 90)];
      }
      int start = NUM_LEDS / STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int k = 0; k <= 50; k++)
      { // Färbt die LEDs in der aktuellen Treppenstufe allmählich ein.
        for (int i = start; i < start + LEDS_PER_STAIR; i++)
        {
          leds[i] = blend(CRGB::Black, color, k * (255 / 50));
        }
        FastLED.show();      // Aktualisiert die LED-Anzeige.
        FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
      }
    }
  }
  // Wenn downUp nicht 1 ist, werden die LEDs von oben nach unten eingefärbt.
  if (downUp != 1)
  {
    for (int j = STAIR; j > 0; j--)
    {
      if (r == 1)
      { // Falls r 1 ist, wird eine zufällige Farbe ausgewählt.
        color = colors[random8(0, 90)];
      }
      int start = NUM_LEDS / STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int k = 0; k <= 50; k++)
      { // Färbt die LEDs in der aktuellen Treppenstufe allmählich ein.
        for (int i = start; i > start - LEDS_PER_STAIR; i--)
        {
          leds[i - 1] = blend(CRGB::Black, color, k * (150 / 50)); // Mischt die Farbe 'color' mit Schwarz, wobei die Intensität durch 'k * (150 / 50)'
                                                                   // bestimmt wird, und weist das Ergebnis der LED an Position 'i - 1' zu.
        }
        FastLED.show();      // Aktualisiert die LED-Anzeige.
        FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
      }
    }
  }
}

// Diese Funktion erzeugt einen Effekt, bei dem die LEDs einer Treppenstufe in zwei Hälften eingefärbt werden, die aufeinander zu oder voneinander weg wandern.
void efekt(uint16_t wait, uint32_t color)
{
  if (downUp != 2)
  { // Wenn downUp nicht 2 ist, wird der Effekt von unten nach oben ausgeführt.
    for (int j = 0; j < STAIR / 2; j++)
    {
      int start = LEDS_PER_STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int i = 0; i < LEDS_PER_STAIR / 2; i++)
      { // Färbt die LEDs in der ersten Hälfte der Stufe von beiden Enden zur Mitte hin.
        leds[(start * 2) + i] = color;
        leds[((start * 2) + LEDS_PER_STAIR) - i - 1] = color;
        FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
        FastLED.show();
      }
      for (int i = LEDS_PER_STAIR / 2; i > 0; i--)
      { // Färbt die LEDs in der zweiten Hälfte der Stufe von der Mitte nach außen hin.
        leds[LEDS_PER_STAIR + (start * 2) + i - 1] = color;
        leds[(LEDS_PER_STAIR + (start * 2) + LEDS_PER_STAIR) - i] = color;
        FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
        FastLED.show();      // Aktualisiert die LED-Anzeige.
      }
    }
  }

  if (downUp != 1)
  { // Wenn downUp nicht 1 ist, wird der Effekt von oben nach unten ausgeführt.
    for (int j = STAIR / 2; j > 0; j--)
    {
      int start = LEDS_PER_STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int i = 0; i < LEDS_PER_STAIR / 2; i++)
      { // Färbt die LEDs in der ersten Hälfte der Stufe von beiden Enden zur Mitte hin.
        leds[(start * 2) - i - 1] = color;
        leds[((start * 2) - LEDS_PER_STAIR) + i] = color;
        FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
        FastLED.show();      // Aktualisiert die LED-Anzeige.
      }
      for (int i = LEDS_PER_STAIR / 2; i > 0; i--)
      { // Färbt die LEDs in der zweiten Hälfte der Stufe von der Mitte nach außen hin.
        leds[start * 2 - LEDS_PER_STAIR - i] = color;
        leds[start * 2 - LEDS_PER_STAIR * 2 + i - 1] = color;
        FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
        FastLED.show();      // Aktualisiert die LED-Anzeige.
      }
    }
  }
}

// Diese Funktion erzeugt einen Effekt, bei dem die LEDs einer Treppenstufe nacheinander von den Enden zur Mitte hin eingefärbt werden.
void efekt1(uint16_t wait, uint32_t color)
{
  for (uint16_t j = 0; j < STAIR; j++)
  {                                      // Effekt von unten nach oben.
    uint16_t start = LEDS_PER_STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
    for (uint16_t i = 0; i < LEDS_PER_STAIR / 2; i++)
    { // Färbt die LEDs von beiden Enden zur Mitte hin ein.
      leds[start + i] = color;
      leds[(start + LEDS_PER_STAIR) - i - 1] = color;
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
      FastLED.show();      // Aktualisiert die LED-Anzeige.
    }
  }

  // Effekt von oben nach unten.
  for (uint16_t j = STAIR; j > 0; j--)
  { // Berechnet den Startindex für die aktuelle Treppenstufe.
    uint16_t start = LEDS_PER_STAIR * j;
    for (uint16_t i = 0; i < LEDS_PER_STAIR / 2; i++)
    { // Färbt die LEDs von beiden Enden zur Mitte hin ein.
      leds[start - i - 1] = color;
      leds[(start - LEDS_PER_STAIR) + i] = color;
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
      FastLED.show();      // Aktualisiert die LED-Anzeige.
    }
  }
}

// Diese Funktion kombiniert den efekt1-Effekt mit einem zusätzlichen Weißblitz am Ende.
void efekt2(uint16_t wait, uint32_t color, int white)
{
  for (uint16_t j = 0; j < STAIR; j++)
  {                                      // Effekt von unten nach oben.
    uint16_t start = LEDS_PER_STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
    for (uint16_t i = 0; i < LEDS_PER_STAIR / 2; i++)
    { // Färbt die LEDs von beiden Enden zur Mitte hin ein.
      leds[start + i] = color;
      leds[(start + LEDS_PER_STAIR) - i - 1] = color;
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
      FastLED.show();      // Aktualisiert die LED-Anzeige.
    }
  }

  // Effekt von oben nach unten.
  for (uint16_t j = STAIR; j > 0; j--)
  {
    uint16_t start = LEDS_PER_STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
    for (uint16_t i = 0; i < LEDS_PER_STAIR / 2; i++)
    { // Färbt die LEDs von beiden Enden zur Mitte hin ein.
      leds[start - i - 1] = color;
      leds[(start - LEDS_PER_STAIR) + i] = color;
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zum nächsten Schritt übergegangen wird.
      FastLED.show();      // Aktualisiert die LED-Anzeige.
    }
  }
  // Wenn `white` 1 ist, wird ein Weißblitz hinzugefügt.
  if (white == 1)
  {
    for (int k = 0; k <= 255; k++)
    { // Erzeugt einen allmählichen Übergang von der aktuellen Farbe zu Weiß.
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = blend(color, CRGB::White, k);
      }
      FastLED.delay(10); // Wartet eine kurze Zeit, um den Weißblitz-Effekt zu erzeugen.
    }
    FastLED.show(); // Aktualisiert die LED-Anzeige.
  }
}

