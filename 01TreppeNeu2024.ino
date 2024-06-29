#include <BH1750.h>  // Bibliothek für den BH1750 Lichtsensor
#include <Wire.h>    // Bibliothek für die I2C-Kommunikation
#include "FastLED.h" // Bibliothek für die Steuerung von LEDs

// Überprüfen der Version der FastLED-Bibliothek. Es wird mindestens Version 3.1 benötigt.
#if FASTLED_VERSION < 3001000
#error "FastLED 3.1 oder neuer wird benötigt; überprüfe github für den neuesten Code."
#endif

#define DEBUG 0 // Setze auf 1 für Debugging-Ausgaben, 0 um sie auszuschalten

#if DEBUG
#define DEBUG_PRINT(x) Serial.println(F(x))
#else
#define DEBUG_PRINT(x)
#endif

#define DATA_PIN 6                        // Pin für das Datenkabel der WS2812b LEDs
#define STAIR 8                           // Anzahl der Stufen
#define LEDS_PER_STAIR 15                 // Anzahl der LEDs pro Stufe
#define NUM_LEDS (STAIR * LEDS_PER_STAIR) // Gesamtzahl der LEDs

#define VOLTS 5     // Volt der LEDs einstellen
#define MAX_MA 6500 // Maximalleistung der LEDs einstellen in MilliAmpere

CRGB leds[NUM_LEDS]; // Array zur Speicherung der LED-Daten
void runningLights(CRGB color = CRGB::Blue, uint16_t waveDelay = 50);

// Variableninitialisierung
unsigned long timeOut = 0;  // Zeitstempel zur Erinnerung, wann der PIR ausgelöst wurde.
int downUp = 0;             // Variable zur Erinnerung an die Bewegungsrichtung (hoch oder runter)
int alarmPinTop = 10;       // PIR-Sensor oben an der Treppe
int alarmPinBottom = 11;    // PIR-Sensor unten an der Treppe
int alarmValueTop = LOW;    // Variable zur Speicherung des Status des oberen PIR-Sensors
int alarmValueBottom = LOW; // Variable zur Speicherung des Status des unteren PIR-Sensors
int ledPin = 13;            // LED-Pin auf dem Arduino-Board, blinkt, wenn der PIR aktiviert wird

// Zeitstempel für die Entprellung
unsigned long lastTriggerTimeTop = 0;
unsigned long lastTriggerTimeBottom = 0;
const unsigned long debounceDelay = 1000; // Beispielwert für die Entprellung

BH1750 lightMeter(0x23); // Configuration of the Light dependent resistor (LDR)

bool useLDR = true;        // flag, when true the program uses the LDR, set to "false" if you don't have a LDR sensor.
int LDRThreshold = 50;     // Only switch on LED's at night when LDR senses low light conditions - you may have to change this value for your circumstances!
bool readPIRInputs = true; // flag, when true, reads the PIR sensors. Disabled (false) by the program when LDR indicated that there is enough light.

// Zusätzliche globale Variablen
int brightness = 28;                      // Globale Variable für die Helligkeit der LEDs
unsigned long pirTimeout = 8000;          // Zeit in Millisekunden, nach der die LEDs ausgeschaltet werden
unsigned long delayBetweenChecks = 60000; // Intervall für LDR-Abfragen (Lichtsensor)
unsigned long previousMillis = 0;         // Vorheriger Zeitpunkt für Zeitmessung
unsigned long totalOffTimeout = 180000;   // Zeit in Millisekunden, nach der die LEDs komplett ausgeschaltet bleiben

int LED_TAIL_LENGTH = 50; // Länge des Schweifs des Meteors

CRGB defaultColor = CRGB::Blue; // Standardfarbe für die LEDs
CRGB alertColor = CRGB::Red;    // Farbe, die bei einer bestimmten Aktion angezeigt wird

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
  DEBUG_PRINT("Debugging ist aktiviert.");

  Serial.println(F("System wird initialisiert..."));

  pinMode(ledPin, OUTPUT);               // Onboard-LED (Pin 13) als Ausgang initialisieren
  pinMode(alarmPinTop, INPUT_PULLUP);    // PIR-Sensor oben an der Treppe als Eingang mit internem Pullup-Widerstand initialisieren
  pinMode(alarmPinBottom, INPUT_PULLUP); // PIR-Sensor unten an der Treppe als Eingang mit internem Pullup-Widerstand initialisieren
  delay(2000);                           // Der Sensor benötigt 2 Sekunden, um die Umgebung zu scannen, bevor er aktiviert werden kann.

  // FastLED-Bibliothek initialisieren
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(brightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_MA); // Maximalleistung der LEDs einstellen  FastLED.clear();
  FastLED.clear();                                       // LEDs ausschalten
  FastLED.show();

  Wire.begin(); // Initialize the I2C bus (BH1750 library doesn't do this automatically)

  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE))
  {
    Serial.println(F("BH1750 Advanced begin"));
  }
  else
  {
    Serial.println(F("Error initializing BH1750"));
  }
  // Initialize all the LDR-readings to current values...
  if (useLDR)
  {
    float lux = lightMeter.readLightLevel();
    Serial.print("LDR average value = ");
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" Lux");
    Serial.print("Stufen funktionieren wenn LDR messung <= ");
    Serial.println(LDRThreshold);
  }
}

void loop()
{
  unsigned long currentMillis = millis();
  // Check LDR only if enough time has passed
  if (currentMillis - previousMillis >= delayBetweenChecks)
  {
    previousMillis = currentMillis;
    checkLDR();
  }
  // Handle PIR inputs only if LDR allows
  if (readPIRInputs)
  {
    handlePIR();
  }
}

// Function to check and handle LDR readings
void checkLDR()
{
  Serial.println(F("Checking LDR..."));

  if (lightMeter.measurementReady(true))
  {
    float lux = lightMeter.readLightLevel();
    // Only configure if needed, but generally should be done in setup if configuration doesn't change
    lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);

    Serial.print(F("LDR Light level: "));
    Serial.print(lux);
    Serial.println(F(" lx"));

    if (lux > LDRThreshold && useLDR)
    {
      readPIRInputs = false;
      Serial.println(F("LDR detektiert zu Hell, Vorgabe LDRThreshold Einstellungen. Stufen Aus"));
      clearLEDs();
    }
    else
    {
      readPIRInputs = true;
      Serial.println(F("LDR Aktiv !. Stufen An"));
    }
  }
  else
  {
    Serial.println(F("Waiting for LDR measurement to be ready..."));
  }
}

void handlePIR()
{
  // PIR-Sensoren auslesen
  int alarmValueTop = digitalRead(alarmPinTop);
  int alarmValueBottom = digitalRead(alarmPinBottom);

  // Aktuelle Zeit speichern
  unsigned long currentMillis = millis();

  // Entprellungslogik für den oberen PIR-Sensor
  if (alarmValueTop == HIGH)
  {
    if (currentMillis - lastTriggerTimeTop > debounceDelay)
    {
      lastTriggerTimeTop = currentMillis; // Zeitstempel aktualisieren
      Serial.println(F("Top PIR-Sensor stabil ausgeloest"));
      timeOut = currentMillis;
      downUp = 1;
      topdown(random8(0, 18));
    }
  }

  // Entprellungslogik für den unteren PIR-Sensor
  if (alarmValueBottom == HIGH)
  {
    if (currentMillis - lastTriggerTimeBottom > debounceDelay)
    {
      lastTriggerTimeBottom = currentMillis; // Zeitstempel aktualisieren
      Serial.println(F("Bottom PIR-Sensor stabil ausgeloest"));
      timeOut = currentMillis;
      downUp = 2;
      topdown(random8(0, 18));
    }
  }

  // Logik für das Ausschalten der LEDs nach einer bestimmten Zeit
  if ((timeOut + pirTimeout < currentMillis) && (downUp == 1 || downUp == 2))
  {
    Serial.println(F("LEDs nach PIR-Timeout ausschalten"));
    ledoff(random8(0, 9));
    downUp = 0;
  }

  // Logik für das vollständige Ausschalten der LEDs nach einem längeren Timeout
  if ((timeOut + totalOffTimeout < millis()) && downUp == 0)
  {
    // Serial.println(F("LEDs komplett ausschalten nach totalem Timeout"));
    clearLEDs();
  }
}

// Function to clear and turn off LEDs
void clearLEDs()
{
  FastLED.clear();
  FastLED.show();
}

// Funktion zur Handhabung von Lichteffekten von oben nach unten
void topdown(int mode)
{
  // Ausgabe des gewählten Lichtmodus auf die serielle Konsole
  Serial.print(F("Executing topdown lighting mode: "));
  Serial.println(mode);

  // Auswahl des Lichtmusters basierend auf dem übergebenen Modus
  switch (mode)
  {
  case 0:
    // Farbe: zufällige Auswahl, Geschwindigkeit: 3, Richtung: unten nach oben
    colourWipeDownUp(colors[random8(0, 90)], 3, 1);
    Serial.println(F("Pattern: colourWipeDownUp"));
    break;
  case 1:
    // Farbe: zufällige Auswahl, Geschwindigkeit: 3, Richtung: oben nach unten
    colourWipeDownUp(colors[random8(0, 90)], 3, 0);
    Serial.println(F("Pattern: colourWipeDownUp"));
    break;
  case 2:
    // Effekt 2 mit Farbe: zufällige Auswahl, Dauer: 50ms, Richtung: 1
    Serial.println(F("Pattern: efekt2"));
    efekt2(50, colors[random8(0, 90)], 1);
    break;
  case 3:
    // Effekt 1 mit Farbe: zufällige Auswahl, Dauer: 50ms
    Serial.println(F("Pattern: efekt1"));
    efekt1(50, colors[random8(0, 90)]);
    break;
  case 4:
    // Effekt mit Farbe: zufällige Auswahl, Dauer: 50ms
    Serial.println(F("Pattern: efekt"));
    efekt(50, colors[random8(0, 90)]);
    break;
  case 5:
    // Regenbogeneffekt
    Serial.println(F("Pattern: rainbow"));
    rainbow();
    break;
  case 6:
    // Farbzykluseffekt mit einer Dauer von 120ms
    Serial.println(F("Pattern: colorCycle"));
    colorCycle(120);
    break;
  case 7:
    // Regenbogenwellen-Effekt mit einer Dauer von 50ms, Schleife: true
    Serial.println(F("Pattern: rainbowWave"));
    rainbowWave(50, true);
    break;
  case 8:
    // Meteorregen-Effekt mit Farbe: zufällige Auswahl, Geschwindigkeit: 100ms
    Serial.println(F("Pattern: meteorShower"));
    meteorShower(colors[random8(0, 90)], 100);
    break;
  case 9:
    // Farbwelleneffekt mit Farbe: zufällige Auswahl, Dauer: 50ms
    Serial.println(F("Pattern: colorWave"));
    colorWave(colors[random8(0, 90)], 50);
    break;
  case 10:
    // Regenbogenwellen-Effekt (neue Methode) mit einer Dauer von 100ms
    Serial.println(F("Pattern: rainbowWave (neue Methode)"));
    rainbowWave1(100);
    break;
  case 11:
    // Meteorschauer-Effekt mit zufälliger Farbe, Größe: 60, Verfall: 100, zufälliges Verfallen: true, Geschwindigkeit: 40ms
    Serial.println(F("Pattern: meteorRain"));
    meteorRain(randomColor(), 60, 100, true, 40);
    break;
  case 12:
    // Farbwechsel von unten nach oben
    Serial.println(F("Pattern: bottomup_colorchange"));
    bottomup_colorchange();
    break;
  case 13:
    // Atmungseffekt mit blauer Farbe, Dauer: 6000ms, min. Helligkeit: 10, max. Helligkeit: 100, Geschwindigkeit: 1.0
    Serial.println(F("Pattern: breathingEffect"));
    breathingEffect(CRGB::Blue, 6000, 10, 100, 1.0);
    break;
  case 14:
    // Kometeneffekt mit Farbe: PaleVioletRed, Größe: 60, Verfall: 64, Geschwindigkeit: 40ms
    Serial.println(F("Pattern: cometEffect"));
    cometEffect(CRGB::PaleVioletRed, 60, 64, 40);
    break;
  case 15:
    // Lauflicht-Effekt
    Serial.println(F("Pattern: runningLights"));
    runningLights();
    break;
  case 16:
    // Glitzereffekt mit weißer Farbe, Wartezeit: 100ms, Funken: 20
    Serial.println(F("Pattern: sparkleEffect"));
    sparkleEffect(CRGB::White, 100, 20);
    break;
  case 17:
    // Glitzereffekt mit variabler Intensität, Dauer: 80ms
    Serial.println(F("Pattern: variableIntensitySparkleEffect"));
    variableIntensitySparkleEffect(80);
    break;
  case 18:
    // Dynamischer Glitzereffekt, Dauer: 50ms
    Serial.println(F("Pattern: dynamicSparkleEffect"));
    dynamicSparkleEffect(50);
    break;
  }
  // Debug-LED blinkt, um den Effektwechsel anzuzeigen
  flashDebugLED();
}

// Funktion zur Handhabung des Ausschaltens der LEDs
void ledoff(int mode)
{
  // Ausgabe des gewählten Modus zum Ausschalten auf die serielle Konsole
  Serial.print(F("Turning off LEDs with mode: "));
  Serial.println(mode);

  // Auswahl der Ausschaltmethode basierend auf dem übergebenen Modus
  switch (mode)
  {
  case 0:
    // Alle LEDs ausschalten mit einer Verzögerung von 500ms
    all_off(500);
    break;
  case 1:
    // Effekt mit schwarzer Farbe, Dauer: 50ms
    efekt(50, CRGB::Black);
    break;
  case 2:
    // Effekt 1 mit schwarzer Farbe, Dauer: 50ms
    efekt1(50, CRGB::Black);
    break;
  case 3:
    // Effekt 2 mit schwarzer Farbe, Dauer: 50ms, Richtung: 0
    efekt2(50, CRGB::Black, 0);
    break;
  case 4:
    // Aus-Effekt mit schwarzer Farbe, Dauer: 50ms
    aus(50, CRGB::Black);
    break;
  case 5:
    // Aus1-Effekt mit schwarzer Farbe, Dauer: 50ms, Wiederholungen: 3
    aus1(50, CRGB::Black, 3);
    break;
  case 6:
    // Sanftes Fade-Out mit schwarzer Farbe, Dauer: 10ms, Geschwindigkeit: 50ms
    smoothFadeOff(CRGB::Black, 10, 50);
    break;
  case 7:
    // Mehrfarbiger Effekt für das Ausschalten, Dauer: 50ms
    efekt1_multicolor(50);
    break;
  case 8:
    // Weihnachtsfarben-Effekt zum Ausschalten, Dauer: 50ms
    holidayEffect(50);
    break;
  case 9:
    // Speicheroptimierter Effekt 1 mit schwarzer Farbe, Dauer: 50ms, Geschwindigkeit: 50ms
    optimizedEfekt1(50, CRGB::Black, 50);
    break;
  default:
    // Standardmäßiges Ausschalten, falls der Effekt nicht definiert ist, mit einer Verzögerung von 500ms
    all_off(500);
    break;
  }
  flashDebugLED();
}

// Function to flash onboard LED for debugging
void flashDebugLED()
{
  Serial.println(F("Flashing debug LED"));
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(10);
    digitalWrite(ledPin, LOW);
    delay(10);
  }
}

void aus(int wait, CRGB color)
{
  for (int stair = 0; stair < STAIR; stair++)
  {
    for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
    {
      leds[i] = color;           // Setze die LED auf die gewählte Farbe
      leds[i].fadeToBlackBy(50); // Verringere die Helligkeit um 50 Einheiten
    }
    FastLED.show();
    FastLED.delay(wait); // Wartezeit zwischen den Schritten
  }
}

void aus1(int wait, CRGB color, int blinkCount)
{
  for (int blink = 0; blink < blinkCount; blink++)
  {
    for (int stair = STAIR - 1; stair >= 0; stair--)
    {
      for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
      {
        leds[i] = color;            // Setze die LED auf die gewählte Farbe
        leds[i].fadeToBlackBy(128); // Reduziere die Helligkeit auf die Hälfte
      }
      FastLED.show();
      FastLED.delay(wait); // Wartezeit zwischen den Schritten
    }
    FastLED.clear();
    FastLED.show();
    FastLED.delay(2 * wait); // Verdoppelte Wartezeit für das Blinken
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

void efekt1_multicolor(int wait)
{
  for (int stair = 0; stair < STAIR; stair++)
  {
    for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
    {
      leds[i] = CHSV((stair * 255) / STAIR, 255, 255); // Farbverlauf basierend auf der Treppenstufe
      leds[i].fadeToBlackBy(50);
    }
    FastLED.show();
    FastLED.delay(wait);
  }
}

void holidayEffect(int wait)
{
  CRGB colors[] = {CRGB::Red, CRGB::Green}; // Weihnachtsfarben
  for (int stair = 0; stair < STAIR; stair++)
  {
    CRGB color = colors[stair % 2]; // Wechsel zwischen Rot und Grün
    for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
    {
      leds[i] = color;
      leds[i].fadeToBlackBy(50);
    }
    FastLED.show();
    FastLED.delay(wait);
  }
}

void optimizedEfekt1(int wait, CRGB color, int fadeAmount)
{
  for (int stair = 0; stair < STAIR; stair++)
  {
    int start = stair * LEDS_PER_STAIR;
    int end = start + LEDS_PER_STAIR;
    fill_solid(leds + start, LEDS_PER_STAIR, color); // Verwenden von fill_solid für schnelleres Setzen
    for (int i = start; i < end; i++)
    {
      leds[i].fadeToBlackBy(fadeAmount);
    }
    FastLED.show();
    FastLED.delay(wait);
  }
}

void all_off(int wait)
{
  // Wenn downUp nicht 2 ist, werden die LEDs von unten nach oben ausgeschaltet.
  if (downUp != 2)
  {
    for (int j = 0; j < STAIR; j++)
    {
      int start = NUM_LEDS / STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int i = start; i < start + LEDS_PER_STAIR; i++)
      { // Setzt alle LEDs in der aktuellen Treppenstufe auf Schwarz (aus).
        leds[i] = CRGB::Black;
      }
      FastLED.show();      // Aktualisiert die LED-Anzeige.
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zur nächsten Stufe übergegangen wird.
    }
  }
  // Wenn downUp nicht 1 ist, werden die LEDs von oben nach unten ausgeschaltet.
  if (downUp != 1)
  {
    for (int j = STAIR; j > 0; j--)
    {
      int start = NUM_LEDS / STAIR * j; // Berechnet den Startindex für die aktuelle Treppenstufe.
      for (int i = start; i > start - LEDS_PER_STAIR; i--)
      { // Setzt alle LEDs in der aktuellen Treppenstufe auf Schwarz (aus).
        leds[i - 1] = CRGB::Black;
      }
      FastLED.show();      // Aktualisiert die LED-Anzeige.
      FastLED.delay(wait); // Wartet die angegebene Zeit, bevor zur nächsten Stufe übergegangen wird.
    }
  }
}

// Funktion für den Sparkle-Effekt
void sparkleEffect(CRGB color, int wait, int numSparks)
{
  for (int cycle = 0; cycle < 10; cycle++)
  { // Wiederholt den Effekt für eine bestimmte Anzahl von Zyklen
    for (int i = 0; i < numSparks; i++)
    {                                    // Generiert eine bestimmte Anzahl von Funken
      int ledIndex = random16(NUM_LEDS); // Wählt zufällig eine LED aus
      leds[ledIndex] = color;            // Setzt die ausgewählte LED auf die angegebene Farbe
      FastLED.show();                    // Aktualisiert die LED-Anzeige
      delay(wait);                       // Wartet die angegebene Zeit
      leds[ledIndex] = CRGB::Black;      // Schaltet die LED wieder aus (zurück auf Schwarz)
    }
  }
}

void dynamicSparkleEffect(uint8_t sparkleCount)
{
  // Clear LEDs for each frame
  FastLED.clear();
  for (uint8_t i = 0; i < sparkleCount; i++)
  {
    int pos = random(NUM_LEDS);                 // Zufällige Position für den Funken
    leds[pos] = CHSV(random(0, 255), 255, 255); // Zufällige Farbe im HSV-Farbraum
  }
  FastLED.show();
}

void variableIntensitySparkleEffect(uint8_t sparkleCount)
{
  // Clear LEDs for each frame
  FastLED.clear();
  for (uint8_t i = 0; i < sparkleCount; i++)
  {
    int pos = random(NUM_LEDS);                        // Zufällige Position für den Funken
    uint8_t brightness = random(20, 255);              // Zufällige Helligkeit
    leds[pos] = CHSV(random(0, 255), 255, brightness); // Zufällige Farbe und Helligkeit im HSV-Farbraum
  }
  FastLED.show();
}

// Erweiterter Atmungseffekt: Anpassung der minimalen und maximalen Helligkeit sowie der Geschwindigkeit
void breathingEffect(uint32_t color, uint16_t duration, uint8_t minBrightness, uint8_t maxBrightness, float speedFactor)
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

// Meteorschauer: Erzeugt den Effekt eines bewegenden Kometen mit einem Schweif
void meteorShower(uint32_t color, uint16_t wait)
{
  for (int i = 0; i < NUM_LEDS + LED_TAIL_LENGTH; i++)
  {
    // Alle LEDs verblassen
    for (int j = 0; j < NUM_LEDS; j++)
    {
      leds[j].fadeToBlackBy(64); // Fade by a fixed amount for the trailing effect
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

// Farbwelle: Erzeugt eine wellenförmige Farbbewegung
void colorWave(uint32_t color, uint16_t wait)
{
  uint8_t waveSpeed = 20; // Geschwindigkeit der Welle
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

// Regenbogenwelle: Erzeugt eine laufende Regenbogenwelle
void rainbowWave1(uint16_t wait)
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

// Kometeneffekt: Erzeugt den Effekt eines Kometen mit einem Schweif
void cometEffect(uint32_t color, uint8_t cometSize, uint8_t cometSpeed, uint16_t wait)
{
  for (int pos = 0; pos < NUM_LEDS + cometSize; pos++)
  {
    // Alle LEDs verblassen
    for (int j = 0; j < NUM_LEDS; j++)
    {
      leds[j].fadeToBlackBy(cometSpeed); // Verblasse LEDs basierend auf der Geschwindigkeit
    }
    // Setze die LED-Position für den Kometen
    for (int j = 0; j < cometSize; j++)
    {
      if ((pos - j < NUM_LEDS) && (pos - j >= 0))
      {
        leds[pos - j] = color; // Set current LED to the comet color
      }
    }
    FastLED.show();
    FastLED.delay(wait); // Geschwindigkeit des Kometen
  }
}

// Farbzyklus: Wechselt alle LEDs durch den gesamten Farbraum
void colorCycle(uint16_t wait)
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

// Laufende Lichter: Erzeugt eine wellenartige Bewegung in einer bestimmten Farbe
void runningLights(CRGB color, uint16_t waveDelay)
{
  int position = 0;
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

// Regenbogen: Füllt die LEDs mit einem sich bewegenden Regenbogen
void rainbow()
{
  fill_rainbow(leds, NUM_LEDS, 0, 7); // Regenbogenfarben füllen
  FastLED.show();
}

// Regenbogenwelle: Erzeugt eine laufende Regenbogenwelle
void rainbowWave(uint16_t wait, bool)
{
  static uint8_t hue = 0;
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV((i * 256 / NUM_LEDS + hue) % 256, 255, 255); // Erzeuge eine Regenbogenwelle
  }
  FastLED.show();
  FastLED.delay(wait); // Geschwindigkeit der Welle
  hue++;               // Farbe verschieben
}

CRGB randomColor()
{
  return CRGB(random8(), random8(), random8()); // Erzeuge zufällige RGB-Werte
}

/*
CRGB randomColor()
{
  return CRGB(random8(128, 255), random8(128, 255), random8(128, 255)); // Helle Farben
}
 */
void meteorRain(CRGB color, uint8_t meteorSize, uint8_t meteorTrailDecay, boolean meteorRandomDecay, uint16_t wait)
{
  fill_solid(leds, NUM_LEDS, CRGB::Black); // Setze alle LEDs auf Schwarz
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

// Function to change colors step by step
void bottomup_colorchange()
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

// Function to light up the stairs from bottom to top with a single color
void bottomup_simple(CRGB color)
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
          leds[i - 1] = blend(CRGB::Black, color, k * (150 / 50));
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
      FastLED.delay(5); // Wartet eine kurze Zeit, um den Weißblitz-Effekt zu erzeugen.
    }
    FastLED.show(); // Aktualisiert die LED-Anzeige.
  }
}

void GreenRed(uint16_t wait)
{
  if (downUp != 2)
  {
    for (int j = 0; j < STAIR; j++)
    {
      int start = NUM_LEDS / STAIR * j;
      for (int i = start; i < start + LEDS_PER_STAIR; i++)
      {
        leds[i] = CRGB::Green;
      }
      FastLED.show();
      FastLED.delay(wait);
    }
    for (int j = 0; j < STAIR; j++)
    {
      int start = NUM_LEDS / (LEDS_PER_STAIR / 2) * j;
      for (int i = start; i < start + (LEDS_PER_STAIR / 2); i++)
      {
        leds[i + 5] = CRGB::Red;
      }
      FastLED.show();
      FastLED.delay(80);
    }
  }
  if (downUp != 1)
  {
    for (int j = STAIR; j > 0; j--)
    {
      int start = NUM_LEDS / STAIR * j;
      for (int i = start; i > start - LEDS_PER_STAIR; i--)
      {
        leds[i - 1] = CRGB::Green;
      }
      FastLED.show();
      FastLED.delay(wait);
    }
    for (int j = STAIR; j > 0; j--)
    {
      int start = NUM_LEDS / (LEDS_PER_STAIR / 2) * j;
      for (int i = start; i > start - (LEDS_PER_STAIR / 2); i--)
      {
        leds[i - 5] = CRGB::Red;
      }
      FastLED.show();
      FastLED.delay(80);
    }
  }
}
