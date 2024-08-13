
#include <BH1750.h>  // Bibliothek für den BH1750 Lichtsensor
#include <Wire.h>    // Bibliothek für die I2C-Kommunikation
#include "FastLED.h" // Bibliothek für die Steuerung von LEDs

// Überprüfen der Version der FastLED-Bibliothek. Es wird mindestens Version 3.1 benötigt.
#if FASTLED_VERSION < 3001000
#error "FastLED 3.1 oder neuer wird benötigt; überprüfe github für den neuesten Code."
#endif

// Debugging-Makros
bool debugMode = true;
#define DEBUG_PRINT(x) \
  if (debugMode)       \
  Serial.print(x)
#define DEBUG_PRINTLN(x) \
  if (debugMode)         \
  Serial.println(x)
/*
  void DEBUG_PRINTF(const char *format, ...) {
  if (debugMode) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
  }
}
  DEBUG_PRINTF("Lichtsensorwert: %d LUX, Led`s angepasst auf %d Helligkeit!\n", lux, Brightness);
  DEBUG_PRINTF("Lichtsensorwert: %d LUX, ab <= %d LUX werden die Led`s Aktiv :)\nLed`s angepasst auf %d Helligkeit!\n", lux, LDRThreshold, Brightness);
*/

// LED-Konfiguration
constexpr int DATA_PIN = 6;                      // Pin für das Datenkabel der WS2812b LEDs
constexpr int STAIR = 8;                         // Anzahl der Stufen
constexpr int LEDS_PER_STAIR = 15;               // Anzahl der LEDs pro Stufe
constexpr int NUM_LEDS = STAIR * LEDS_PER_STAIR; // Gesamtzahl der LEDs
constexpr int VOLTS = 5;                         // Volt der LEDs einstellen
constexpr int MAX_MA = 7000;                     // Maximalleistung der LEDs einstellen in MilliAmpere
// Variableninitialisierung
bool useLDR = true;        // Ob der LDR (Lichtabhängiger Widerstand) verwendet werden soll
bool readPIRInputs = true; // Ob PIR-Eingaben gelesen werden sollen

const unsigned long LDRThreshold = 380; // Only switch on LED's at night!
const int maxBrightness = 250;
const int minBrightness = 30;
const float minLux = 0.0;   // Minimaler Lux-Wert
const float maxLux = 200.0; // Maximaler Lux-Wert ab da, maping maxBrightness

// PIN-Konfiguration
constexpr int ALARM_PIN_TOP = 10;    // PIR-Sensor oben an der Treppe
constexpr int ALARM_PIN_BOTTOM = 11; // PIR-Sensor unten an der Treppe

// Zeitkonstanten
constexpr unsigned long DETECTION_TIME = 400;      // Erkennungszeit in Millisekunden
constexpr unsigned long PIR_TIMEOUT = 19000;       // Zeit in Millisekunden, nach der die LEDs ausgeschaltet werden
constexpr unsigned long TOTAL_OFF_TIMEOUT = 25000; // Zeit in Millisekunden, nach der die LEDs komplett ausgeschaltet bleiben
constexpr unsigned long shortLDRInterval = 300000; // shortIntervall für LDR-Abfragen (Lichtsensor) 300000 = 5 min
constexpr unsigned long longLDRInterval = 7200000; // 2 Stunden in Millisekunden 7200000 / 1 std 3600000

// Globale Variablen
unsigned long lastTriggerTimeTop = 0;    // Auslöse-Sperre oben
unsigned long lastTriggerTimeBottom = 0; // Auslöse-Sperre unten
unsigned long activeTimeTop = 0;         // Aktiv-Zeit oben
unsigned long activeTimeBottom = 0;      // Aktiv-Zeit unten
volatile bool isTopdownRunning = false;  // Variable zum Sperren der PIR-Auslösung
unsigned long previousMillis = 0;        // Vorheriger Zeitpunkt für Zeitmessung
unsigned long timeOut = 0;               // Zeitstempel zur Erinnerung, wann der PIR ausgelöst wurde.
int downUp = 0;                          // Variable zur Erinnerung an die Bewegungsrichtung (hoch oder runter)

// Globale Variable zur Speicherung der ursprünglichen Helligkeit
int Brightness = 0;                                  // Globale Variable für die Helligkeit der LEDs
int originalBrightness = Brightness;                 // Setze dies auf den ursprünglichen Helligkeitswert
unsigned long currentLDRInterval = shortLDRInterval; // aktuelles Abfrageintervall
bool firstLDRCheckPassed = false;

BH1750 lightMeter(0x23); // Configuration des Light dependent resistor (LDR)

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

// Funktion zum Zugriff auf eine Farbe im PROGMEM
CRGB getColorFromPROGMEM(uint8_t index)
{
  CRGB color;
  if (index < 90)
  {
    color = pgm_read_dword_near(colors + index);
  }
  else
  {
    color = CRGB::Black; // Standardfarbe, falls der Index außerhalb des Bereichs liegt
  }
  return color;
}

// Fire
static byte heat[NUM_LEDS]; // Array of temperature readings at each simulation cell
int cooldown;

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

void setup()
{
    if (debugMode)
    {
        Serial.begin(9600);
        while (!Serial)
        {
            // Warten auf serielle Verbindung
        }
        DEBUG_PRINTLN("Debugging true!");
    }
    DEBUG_PRINTLN("Initialisierung des Systems...");
    // Initialisiere die PIR-Sensoren
    initializePIRSensors();
    // Initialisiere FastLED
    initializeLEDs();
    if (useLDR)
    {
        DEBUG_PRINTLN("System erfolgreich initialisiert.");
        initializeLDR();
        checkLDR();
    }
}

void initializePIRSensors()
{
    pinMode(ALARM_PIN_TOP, INPUT_PULLUP);    // PIR-Sensor oben an der Treppe als Eingang mit internem Pullup-Widerstand initialisieren
    pinMode(ALARM_PIN_BOTTOM, INPUT_PULLUP); // PIR-Sensor unten an der Treppe als Eingang mit internem Pullup-Widerstand initialisieren
}

void initializeLEDs()
{
    FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setMaxPowerInVoltsAndMilliamps(VOLTS, MAX_MA); // Maximalleistung der LEDs einstellen
    FastLED.clear();
    delay(2000); // Verzögerung für die LED-Initialisierung
}

void initializeLDR()
{
    Wire.begin();
    if (!lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE))
    {
        while (1)
        {
            DEBUG_PRINTLN("Fehler beim Initialisieren des Lichtsensors!");
        }
    }
    delay(130); // Warten auf die erste Messung
    float lux = lightMeter.readLightLevel();
    if (lux > LDRThreshold)
    {
        readPIRInputs = false;
        clearLEDs();
    }
    else
    {
        readPIRInputs = true;
    }
}

void loop()
{
    unsigned long currentMillis = millis();
    // Überprüfen des Lichtsensors nur, wenn genügend Zeit vergangen ist
    if (currentMillis - previousMillis >= currentLDRInterval)
    {
        previousMillis = currentMillis;
        checkLDR();
    }
    // Handhaben der PIR-Eingänge nur, wenn der Lichtsensor dies zulässt
    if (readPIRInputs)
    {
        handlePIR();
    }
}

void checkLDR()
{
    static bool inLongInterval = false; // Zustand des Intervalls
    const unsigned long timeoutDuration = 500; // Timeout-Dauer in Millisekunden (z.B. 500 ms)
    unsigned long startTime = millis(); // Startzeit für den Timeout-Mechanismus

    lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE); // Start a new measurement

    // Timeout-Mechanismus: Warten auf die Messung mit einer maximalen Dauer von timeoutDuration
    while (!lightMeter.measurementReady())
    {
        if (millis() - startTime > timeoutDuration)
        {
            DEBUG_PRINTLN("Timeout beim Warten auf die Lichtmessung! Abbruch der Funktion.");
            return; // Abbruch der Funktion bei Timeout
        }
        delay(10); // Kurze Pause, um das System nicht zu überlasten
    }

    // Wenn die Messung innerhalb des Zeitrahmens abgeschlossen ist, wird sie hier verarbeitet
    float lux = lightMeter.readLightLevel();
    Brightness = mapBrightness(lux, minLux, maxLux, minBrightness, maxBrightness);
    FastLED.setBrightness(Brightness);
    printLuxInfo(lux, Brightness);

    // Prüfen, ob der Lux-Wert über dem Schwellenwert liegt
    bool luxAboveThreshold = (lux > LDRThreshold);

    // Wenn der Lux-Wert unter 1500 oder unter dem Schwellenwert liegt, Intervall auf kurz setzen
    if (lux < 1500 || lux < LDRThreshold)
    {
        currentLDRInterval = shortLDRInterval;
        inLongInterval = false; // Rücksetzen, falls vorher langes Intervall aktiv war
        firstLDRCheckPassed = false;
        DEBUG_PRINTLN("Lux-Wert unter 1500 oder unter dem Schwellenwert, kurzes Intervall aktiviert.");
    }
    else if (luxAboveThreshold)
    {
        if (!inLongInterval) // Nur umschalten, wenn nicht bereits im langen Intervall
        {
            if (firstLDRCheckPassed)
            {
                // Zweite Überprüfung: Wenn auch die zweite Messung über dem Schwellenwert liegt
                currentLDRInterval = longLDRInterval;
                inLongInterval = true;
                clearLEDs();
                DEBUG_PRINTLN("Zweite Messung ebenfalls über dem Schwellenwert, langes Intervall aktiviert!");
                firstLDRCheckPassed = false; // Zurücksetzen für zukünftige Messungen
            }
            else
            {
                // Erste Messung über dem Schwellenwert, noch keine Aktion
                firstLDRCheckPassed = true;
                currentLDRInterval = shortLDRInterval;
                DEBUG_PRINTLN("Erste Messung über dem Schwellenwert, warte auf die zweite Messung.");
            }
        }
    }

    // PIR-Eingaben deaktivieren, wenn ausreichend Licht vorhanden ist
    readPIRInputs = lux < LDRThreshold;
    if (!readPIRInputs)
    {
        DEBUG_PRINTLN("Genug Umgebungslicht erkannt, LEDs AUS !!!");
        clearLEDs();
    }
    printIntervInfo(currentLDRInterval);
}

// Funktion zur Handhabung der PIR-Sensoren (oben und unten)
void handlePIR()
{
    // Lese den aktuellen Wert des oberen PIR-Sensors aus
    int alarmValueTop = digitalRead(ALARM_PIN_TOP);
    // Lese den aktuellen Wert des unteren PIR-Sensors aus
    int alarmValueBottom = digitalRead(ALARM_PIN_BOTTOM);
    // Erhalte die aktuelle Zeit in Millisekunden
    unsigned long currentMillis = millis();

    // Verarbeite den oberen PIR-Sensor
    handlePIRSensor(alarmValueTop,
                    activeTimeTop,
                    lastTriggerTimeTop,
                    currentMillis,
                    1,
                    "Oben PIR-Sensor Bewegung oben erkannt <<<<< MODE: ");

    // Verarbeite den unteren PIR-Sensor
    handlePIRSensor(alarmValueBottom,
                    activeTimeBottom,
                    lastTriggerTimeBottom,
                    currentMillis,
                    2,
                    "Unten PIR-Sensor Bewegung unten erkannt >>>>> MODE: ");

    // LEDs nach einer bestimmten Zeit ausschalten, wenn kein Bewegungssignal mehr empfangen wird
    if ((currentMillis - timeOut > PIR_TIMEOUT) && (downUp != 0))
    {
        DEBUG_PRINT("LEDs nach PIR-Timeout ausschalten! ###### Mode: ");
        // Schalte die LEDs aus und wähle eine zufällige Farbe zwischen 0 und 8 für den Abschalteffekt
        ledoff(random8(0, 12));
        isTopdownRunning = false; // Markiere, dass die Topdown-Aktion beendet ist
        downUp = 0;               // Setze den Modus zurück
        DEBUG_PRINTLN("... warte auf weitere Bewegung!");
    }
    // LEDs komplett ausschalten, wenn eine längere Zeit (TOTAL_OFF_TIMEOUT) ohne Bewegung vergangen ist
    if ((currentMillis - timeOut > TOTAL_OFF_TIMEOUT) && (!isTopdownRunning))
    {
        clearLEDs();
        if (downUp != 0)
        {
            downUp = 0;
        }
    }
}

void handlePIRSensor(int alarmValue,                 // Der aktuelle Wert des PIR-Sensors (HIGH oder LOW)
                     unsigned long &activeTime,      // Zeit, wie lange der Sensor aktiv war (in Millisekunden)
                     unsigned long &lastTriggerTime, // Die Zeit, wann der Sensor zuletzt ausgelöst wurde (in Millisekunden)
                     unsigned long currentMillis,    // Die aktuelle Zeit (in Millisekunden)
                     int mode,                       // Modus (wird verwendet, um eine bestimmte Aktion auszuführen)
                     const char *msg)                // Nachricht, die zum Debugging ausgegeben wird
{
    // Wenn keine Topdown-Animation läuft und der PIR-Sensor ausgelöst wurde
    if (!isTopdownRunning && alarmValue == HIGH)
    {
        // Setze die Sperre sofort
        isTopdownRunning = true; // Markiere, dass eine Topdown-Aktion läuft

        // Wenn der Sensor zum ersten Mal ausgelöst wird, setze die aktive Zeit
        if (activeTime == 0)
        {
            activeTime = currentMillis;
        }
        // Wenn die Zeit, seit der Sensor ausgelöst wurde, größer als die vorgegebene DETECTION_TIME ist
        if (currentMillis - activeTime > DETECTION_TIME)
        {
            lastTriggerTime = currentMillis; // Aktualisiere die Zeit des letzten Auslösens
            timeOut = currentMillis;         // Setze das Timeout auf die aktuelle Zeit
            downUp = mode;                   // Setze den Modus (könnte eine Richtung oder Aktion sein)
            DEBUG_PRINT(msg);                // Debug-Nachricht ausgeben
            // Starte die Topdown-Aktion
            topdown(random8(0, 46)); // Wähle eine zufällige Farbe zwischen 0 und 33 und führe die Aktion aus
            // Nach Abschluss der Animation:
            // isTopdownRunning = false; // Markiere, dass die Topdown-Aktion beendet ist
            activeTime = 0; // Setze die aktive Zeit zurück, da der Sensor "verarbeitet" wurde
            return;         // Verlasse die Funktion sofort, um keine weiteren Bewegungen zu verarbeiten
        }
        else
        {
            // Wenn die Bedingung nicht erfüllt ist, hebe die Sperre auf
            isTopdownRunning = false;
        }
    }
    else
    {
        // Wenn der Sensor nicht ausgelöst ist oder gerade eine Topdown-Animation läuft, setze die aktive Zeit zurück
        activeTime = 0;
    }
}

void printLuxInfo(float lux, int brightness)
{
    DEBUG_PRINT(lux);
    DEBUG_PRINT(" Lux gemessen, ab <= ");
    DEBUG_PRINT(LDRThreshold);
    DEBUG_PRINT(" Lux werden die LEDs aktiviert! Helligkeit angepasst auf ");
    DEBUG_PRINT(brightness);
    DEBUG_PRINTLN(". ");

    // Dynamische Ausgabe, ob der Lux-Wert unter dem Schwellenwert liegt
    if (lux <= LDRThreshold)
    {
        DEBUG_PRINT("Der aktuelle Lux-Wert liegt unter dem Schwellenwert.");
    }
    else
    {
        DEBUG_PRINT("Der aktuelle Lux-Wert liegt ueber dem Schwellenwert.");
    }

    DEBUG_PRINT(" Maximal moegliche Helligkeit ist ");
    DEBUG_PRINT(maxBrightness);
    DEBUG_PRINTLN(".");
}

void printIntervInfo(unsigned long currentLDRInterval)
{
    // Berechnungen
    unsigned long seconds = currentLDRInterval / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;

    DEBUG_PRINT(" Neue Lux werte in ");
    DEBUG_PRINT(currentLDRInterval);
    DEBUG_PRINT(" Millisekunden, das entspricht ");

    if (hours > 0)
    {
        DEBUG_PRINT(hours);
        DEBUG_PRINT(" Stunden und ");
    }
    if (minutes % 60 > 0)
    {
        DEBUG_PRINT(minutes % 60);
        DEBUG_PRINT(" Minuten! ");
    }
    if (hours == 0 && minutes == 0)
    {
        DEBUG_PRINT(seconds);
        DEBUG_PRINT(" Sekunden! ");
    }
    
    if (currentLDRInterval == shortLDRInterval)
    {
        DEBUG_PRINTLN(" (Kurzes Intervall)");
        DEBUG_PRINTLN("...warte auf bewegung ! ");
    }
    else if (currentLDRInterval == longLDRInterval)
    {
        DEBUG_PRINTLN(" (Langes Intervall)");
        DEBUG_PRINTLN("longLDRInterval, Grund:   lux > LDRThreshold   ...bis nachher ! ");
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
    DEBUG_PRINTLN(": colourWipeUp");                         // (uint32_t color, int wait, int r, int downUp)
    colourWipeDownUp(colors[random8(0, 90)], 20, 1, downUp); // Farbe: zufällige Auswahl, Geschwindigkeit: 20, Richtung: oben nach unten
    break;
  case 1:
    DEBUG_PRINTLN(": efekt2");                     // Effekt 2 mit Farbe: zufällige Auswahl, Dauer: 50ms, Richtung: 0
    efekt2(80, colors[random8(0, 90)], 0, downUp); //(uint16_t wait, uint32_t color, int white, int downUp)
    break;
  case 2:
    DEBUG_PRINTLN(": efekt");                   // Effekt mit Farbe: zufällige Auswahl
    efekt(100, colors[random8(0, 90)], downUp); //(uint16_t wait, uint32_t color, int downUp)
    break;
  case 3:
    DEBUG_PRINTLN(": efekt1"); // Effekt 1 mit Farbe: zufällige Auswahl, Dauer: 50ms
    efekt1(80, colors[random8(0, 90)], downUp);
    break;
  case 4:
    DEBUG_PRINTLN(": smoothComet ");
    smoothComet(CRGB::BlueViolet, 130, 35, 25, downUp); // CRGB color, int cometSize, int speedDelay , int cometTrailDecay)
    break;
  case 5:
    DEBUG_PRINTLN(": comet");
    comet(CRGB::Blue, CRGB::Purple, 135, 32, 30, downUp); //(CRGB startColor, CRGB endColor, int cometSize, int speedDelay, int cometTrailDecay, int downUp)
    break;
  case 6:
    DEBUG_PRINTLN(": colorCycle"); // Farbzykluseffekt mit einer Dauer von 120ms
    colorCycle(30);
    break;
  case 7:
    DEBUG_PRINTLN(": glowingComet");
    glowingComet(CRGB::Violet, 140, 30, 30, downUp); // CRGB color, int cometSize, int speedDelay , int cometTrailDecay, int downUp)130, 25, 70,
    break;
  case 8:
    DEBUG_PRINTLN(": MeteorRain");                     // Meteorregen-Effekt mit Farbe: zufällige Auswahl, Geschwindigkeit: 100ms
    MeteorRain(colors[random8(0, 90)], 10, 10, false); //(int Color, int TrailDecay, int speedDelay , bool RandomDecay)
    break;
  case 9:
    DEBUG_PRINTLN(": Fire2");
    Fire2(50, 120, 12, downUp); // (Cooling, Sparks, DelayDuration, int downUp 60, 150, 10, downUp)
    break;
  case 10:
    DEBUG_PRINTLN(": rainbowWave (neue Methode)"); // Regenbogenwellen-Effekt (neue Methode) mit einer Dauer von 100ms, fadeToBlk=255
    rainbowWave1(40);
    break;
  case 11:
    DEBUG_PRINTLN(": colorWipe");          // Debug-Ausgabe: Name des aktuellen Musters
    colorWipe(colors[random8(0, 90)], 50); // Farbiger Wisch-Effekt mit blau und 50 ms Verzögerung
    break;
  case 12:
    DEBUG_PRINTLN(": bottomup_colorchange"); // Farbwechsel von unten nach oben
    bottomup_colorchange();
    break;
  case 13:
    DEBUG_PRINTLN(": BouncingBall ");
    BouncingBall(0, 3, 18, true, downUp); // Beispielaufruf mit Farbe 0 (Rot), 3 Bällen, einer Verzögerung von 20 ms und individuellen Farben für jeden Ball
    break;
  case 14:
    DEBUG_PRINTLN(": flashingBlitz ");
    flashingBlitz(CRGB::Blue, 45, downUp); //(uint32_t color, int flashDelay, int direction)
    break;
  case 15:
    DEBUG_PRINTLN(": twinkle "); // Funkeln-Muster mit 100 ms Verzögerung
    twinkle(110, 140);           //(uint16_t wait, int FadeToBlk)
    break;
  case 16:
    DEBUG_PRINTLN(": FireDiagonal");
    FireDiagonal(10, 170, 20, downUp); // (Color, Cooling, Sparks, DelayDuration, Reverse Direction)
    break;
  case 17:
    DEBUG_PRINTLN(": WaveRipple ");
    WaveRipple(colors[random8(0, 90)], 120); //(int Color, int DelayDuration)
    break;
  case 18:
    DEBUG_PRINTLN(": wavePattern");                                  // Debug-Ausgabe: Name des aktuellen Musters
    wavePattern(colors[random8(0, 90)], colors[random8(0, 90)], 40); // Wellenmuster mit rot und blau, Geschwindigkeit 80
    break;
  case 19:
    DEBUG_PRINTLN(": starrySky ");
    starrySky(colors[random8(0, 90)], 80, 40); // //(CRGB color, int flashDelay, int numberOfStars)
    break;
  case 20:
    DEBUG_PRINTLN(": FireDownwards");   // Regenbogeneffekt
    FireDownwards(10, 100, 10, downUp); // (Cooling, Sparks, DelayDuration, ReverseDirection)
    break;
  case 21:
    DEBUG_PRINTLN(": fireworks"); //(uint8_t red, uint8_t green, uint8_t blue, uint16_t wait, int FadeToBlk)
    fireworks(100, 130);          // Hier werden die Werte für wait und FadeToBlk übergeben
    break;
  case 22:
    DEBUG_PRINTLN(": sparkleEffect "); // Debug-Ausgabe: Name des aktuellen Musters
    sparkleEffect(100, 8, true);       // Funkeleffekt mit 50 ms Verzögerung, 8 Funken, zufällige Farben
    break;
  case 23:
    DEBUG_PRINTLN(": fadeToColor");               // Debug-Ausgabe: Name des aktuellen Musters
    fadeToColor(colors[random8(0, 90)], 10, 120); // Farbübergang zu lila, mit 30 Schritten und Geschwindigkeit 100
    break;
  case 24:
    DEBUG_PRINTLN(": stepColorChange ");
    stepColorChange(colors[random8(0, 90)], 100, downUp); //(uint32_t color, int wait, int downUp)
    break;
  case 25:
    DEBUG_PRINTLN(": Starburst ");
    Starburst(80, 100); //(int Color, int speedDelay , FadeToBlk)
    break;
  case 26:
    DEBUG_PRINTLN(": waveEffect ");
    waveEffect(colors[random8(0, 90)], 55); // Parameter: Farbe (Blau), Verzögerung (100 ms) zwischen den Positionen.
    break;
  case 27:
    DEBUG_PRINTLN(": colorExplosion ");
    colorExplosion(colors[random8(0, 90)], 40); // Farbeexplosionseffekt mit dunkelmagenta und Geschwindigkeit 30
    break;
  case 28:
    DEBUG_PRINTLN(": colorExplosion ");
    Heartbeat(CRGB::DarkMagenta, 1200, 30); // Farbe DarkMagenta mit 40ms Schlag und 80ms Fade-Delay
    break;
  case 29:
    DEBUG_PRINTLN(": ColorPulse");
    ColorPulse(CRGB::Red, CRGB::Blue, 30, 255); // Parameter: color1, color2, pulseSpeed, pulseRange
    break;
  case 30:
    DEBUG_PRINTLN(": Noise Storm");
    NoiseStorm(10, 255, 60); // Parameter: speed, intensity, fadeAmount
    break;
  case 31:
    DEBUG_PRINTLN(": Aurora");
    Aurora(2, 128, 50, 64, 50); // Parameter: waveSpeed, baseColor, intensity, delayDuration
    break;
  case 32:
    DEBUG_PRINTLN(": Plasma");
    Plasma(10, 50, 50); // Parameter: speed, colorShift, delayDuration
    break;
  case 33:
  {
    DEBUG_PRINTLN(": Raindrop ");
    CRGB farbe = colors[random8(0, sizeof(colors) / sizeof(colors[0]))];
    int geschwindigkeit = random(30, 70); // Zufällige Geschwindigkeit zwischen 30 und 70 ms
    int spawnWahrscheinlichkeit = 12;     // Wahrscheinlichkeit für das Erscheinen eines neuen Tropfens (0-100)
    int schwanzLaenge = random(2, 10);    // Zufällige Länge des Schweifs
    Raindrop(farbe, geschwindigkeit, spawnWahrscheinlichkeit, schwanzLaenge);
    break;
  }
  case 34:
    DEBUG_PRINTLN(": Juggle");
    Juggle(50, 1); // Parameter: waitDuration, hueIncrement
    break;
  case 35:
    DEBUG_PRINTLN(": StarryNight");
    StarryNight(20, 80, 100, 250, 15); // Beispielwerte für fadeAmount, sparkleChance, minBrightness, maxBrightness, delayDuration
    break;
  case 36:
    DEBUG_PRINTLN(": Noise Storm");
    NoiseStorm();
    break;
  case 37:
    DEBUG_PRINTLN(": Aurora2");
    Aurora2();
    break;
  case 38:
    DEBUG_PRINTLN(": Plasma2");
    Plasma2();
    break;
  case 39:
    DEBUG_PRINTLN(": Sinelon");
    Sinelon();
    break;
  case 40:
    DEBUG_PRINTLN(": LavaLamp");
    LavaLamp();
    break;
  case 41:
    DEBUG_PRINTLN(": Fireflies");
    Fireflies();
    break;
  case 42:
    DEBUG_PRINTLN(": OceanWaves");
    OceanWaves();
    break;
  case 43:
  {
    DEBUG_PRINTLN(": NoiseEffect2");
    uint8_t scale = random(10, 100);    // Zufälliger Skalierungswert
    uint8_t speed = random(1, 5);       // Zufällige Geschwindigkeit der Bewegung im Rauschraum
    uint8_t colorSpeed = random(5, 50); // Zufällige Geschwindigkeit der Farbänderung
    NoiseEffect2(scale, speed, colorSpeed);
    break;
  }
  case 44:
  {
    DEBUG_PRINTLN(": NoiseEffect");
    uint16_t scale = random16(50, 200); // Zufälliger Skalierungswert für den Noise-Effekt
    NoiseEffect(scale);
    break;
  }
  case 45:
    DEBUG_PRINTLN(": Fireflies");
    Fireflies2(Brightness, 10, 50); // Beispielwerte für brightness, density, fadeSpeed
    break;
  }
}

void ledoff(int mode) // Funktion zur Handhabung des Ausschaltens der LEDs
{
  DEBUG_PRINT(mode);
  switch (mode) // Auswahl der Ausschaltmethode basierend auf dem übergebenen Modus
  {
  case 0:
    DEBUG_PRINTLN(": spiralOffStairs");
    spiralOffStairs(100); // Wisch-Effekt zum Ausschalten
    break;
  case 1:
    DEBUG_PRINTLN(": efekt");        // Effekt mit schwarzer Farbe, Dauer: 50ms
    efekt(100, CRGB::Black, downUp); //(uint16_t wait, uint32_t color, int downUp)
    break;
  case 2:
    DEBUG_PRINTLN(": efekt1");        //(uint16_t wait, uint32_t color, int downUp)
    efekt1(100, CRGB::Black, downUp); // Effekt 1 mit schwarzer Farbe, Dauer: 50ms (uint16_t wait, uint32_t color)
    break;
  case 3:
    DEBUG_PRINTLN(": efekt2");           // Effekt 2 mit schwarzer Farbe, Dauer: 50ms, Weißblitz: 0
    efekt2(100, CRGB::Black, 1, downUp); ////(uint16_t wait, uint32_t color, int white, int downUp)
    break;
  case 4:
    DEBUG_PRINTLN(": smoothFadeOff");
    smoothFadeOff(CRGB::Blue, 10, 70); // Sanftes Ausschalten mit smoothFadeOff// (CRGB color, int steps, int wait)
    break;
  case 5:
    DEBUG_PRINTLN(": waveOffStairs");
    waveOffStairs(100); // Wellenförmiges Ausschalten
    break;

  case 6:
    DEBUG_PRINTLN(": rippleOffStairs");
    rippleOffStairs(200); // Wellenförmiges Ausschalten von der Mitte nach außen
    break;
  case 7:
    DEBUG_PRINTLN(": cascadeOffStairs");
    cascadeOffStairs(100); // Stufenweises Ausschalten von oben nach unten
    break;
  case 8:
    DEBUG_PRINTLN(": reverseSpiralOffStairs");
    reverseSpiralOffStairs(150); // Spiralförmiges Ausschalten in umgekehrter Reihenfolge
    break;
  case 9:
    DEBUG_PRINTLN(": fadeOffStairs");
    fadeOffStairs(5, 100); // Sanftes Ausblenden aller Stufen (int fadeSteps, int wait)
    break;
  case 10:
    DEBUG_PRINTLN(": randomOffStairs");
    randomOffStairs(20); // Zufälliges Ausschalten der LEDs
    break;
  case 11:
    DEBUG_PRINTLN(": collapseOffStairs");
    collapseOffStairs(200); // Ausschalten von den äußeren Rändern zur Mitte hin
    break;
  }
}

void spiralOffStairs(int wait)
{
  // Gehe jede Stufe der Treppe durch
  for (int i = 0; i < STAIR; i++)
  {
    int mid = LEDS_PER_STAIR / 2; // Berechne die mittlere Position der LEDs auf der aktuellen Stufe

    for (int j = 0; j <= mid; j++)
    {
      // Berechne die Indizes für die LEDs links und rechts der Mitte
      int index1 = i * LEDS_PER_STAIR + mid - j;
      int index2 = i * LEDS_PER_STAIR + mid + j;

      // Überprüfe, ob die Indizes gleich sind, um doppelte Ausschaltungen zu vermeiden
      if (index1 == index2)
      {
        leds[index1] = CRGB::Black;
      }
      else
      {
        // Schalte die LEDs an den berechneten Positionen aus (schwarz)
        leds[index1] = CRGB::Black;
        leds[index2] = CRGB::Black;
      }
    }

    FastLED.show();      // Aktualisiere die LEDs, um die Änderungen anzuzeigen
    FastLED.delay(wait); // Warte eine bestimmte Zeit, bevor du zur nächsten Stufe gehst
  }
}

void waveOffStairs(int wait)
{
  // Gehe jede Stufe der Treppe durch
  for (int i = 0; i < STAIR; i++)
  {
    // Gehe die Hälfte der LEDs auf der aktuellen Stufe durch
    for (int j = 0; j <= LEDS_PER_STAIR / 2; j++)
    {
      // Berechne die Indizes für die LEDs von der linken und rechten Seite
      int index1 = i * LEDS_PER_STAIR + j;
      int index2 = i * LEDS_PER_STAIR + (LEDS_PER_STAIR - 1 - j);

      // Überprüfe, ob die Indizes gleich sind, um doppelte Ausschaltungen zu vermeiden
      if (index1 == index2)
      {
        leds[index1] = CRGB::Black;
      }
      else
      {
        // Schalte die LEDs an den berechneten Positionen aus (schwarz)
        leds[index1] = CRGB::Black;
        leds[index2] = CRGB::Black;
      }
    }

    FastLED.show();      // Aktualisiere die LEDs, um die Änderungen anzuzeigen
    FastLED.delay(wait); // Warte eine bestimmte Zeit, bevor du zur nächsten Stufe gehst
  }
}

void smoothFadeOff(CRGB color, int steps, int wait)
{
  // Vorab Helligkeitswerte berechnen und in einem Array speichern
  uint8_t brightness[steps];
  for (int j = 0; j < steps; j++)
  {
    brightness[j] = 255 - (255 / steps) * j;
  }

  // Farbe initial setzen
  for (int stair = 0; stair < STAIR; stair++)
  {
    for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
    {
      leds[i] = color;
    }
  }

  // Helligkeitsschritte durchlaufen
  for (int j = 0; j < steps; j++)
  {
    uint8_t scale = brightness[j];

    // LEDs schrittweise dimmen
    for (int stair = 0; stair < STAIR; stair++)
    {
      int startIdx = stair * LEDS_PER_STAIR;
      int endIdx = (stair + 1) * LEDS_PER_STAIR;

      for (int i = startIdx; i < endIdx; i++)
      {
        leds[i].nscale8(scale); // Helligkeit für jede LED setzen
      }
    }
    FastLED.show();
    FastLED.delay(wait);
  }

  // Alle LEDs am Ende ausschalten
  for (int stair = 0; stair < STAIR; stair++)
  {
    int startIdx = stair * LEDS_PER_STAIR;
    int endIdx = (stair + 1) * LEDS_PER_STAIR;
    for (int i = startIdx; i < endIdx; i++)
    {
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();
}

void reverseSpiralOffStairs(int wait)
{
  for (int i = STAIR - 1; i >= 0; i--)
  {
    int mid = LEDS_PER_STAIR / 2;
    for (int j = 0; j <= mid; j++)
    {
      int index1 = i * LEDS_PER_STAIR + mid - j;
      int index2 = i * LEDS_PER_STAIR + mid + j;

      leds[index1] = CRGB::Black;
      leds[index2] = CRGB::Black;
    }
    FastLED.show();
    FastLED.delay(wait);
  }
}

void cascadeOffStairs(int wait)
{
  for (int stair = 0; stair < STAIR; stair++)
  {
    for (int i = stair * LEDS_PER_STAIR; i < (stair + 1) * LEDS_PER_STAIR; i++)
    {
      leds[i] = CRGB::Black;
    }
    FastLED.show();
    FastLED.delay(wait);
  }
}

void fadeOffStairs(int fadeSteps, int wait)
{
  for (int j = fadeSteps; j >= 0; j--)
  {
    uint8_t scale = (255 * j) / fadeSteps;

    for (int stair = 0; stair < STAIR; stair++)
    {
      int startIdx = stair * LEDS_PER_STAIR;
      int endIdx = (stair + 1) * LEDS_PER_STAIR;

      for (int i = startIdx; i < endIdx; i++)
      {
        leds[i].nscale8(scale);
      }
    }
    FastLED.show();
    FastLED.delay(wait);
  }

  // Sicherstellen, dass alle LEDs am Ende vollständig ausgeschaltet sind
  for (int stair = 0; stair < STAIR; stair++)
  {
    int startIdx = stair * LEDS_PER_STAIR;
    int endIdx = (stair + 1) * LEDS_PER_STAIR;
    for (int i = startIdx; i < endIdx; i++)
    {
      leds[i] = CRGB::Black;
    }
  }
  FastLED.show();
}

void randomOffStairs(int wait)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    int randomIndex = random16(NUM_LEDS);
    leds[randomIndex] = CRGB::Black;
    FastLED.show();
    FastLED.delay(wait);
  }
}

void rippleOffStairs(int wait)
{
  for (int i = 0; i < STAIR; i++)
  {
    int mid = LEDS_PER_STAIR / 2;
    for (int j = 0; j <= mid; j++)
    {
      int index1 = i * LEDS_PER_STAIR + mid - j;
      int index2 = i * LEDS_PER_STAIR + mid + j;

      leds[index1] = CRGB::Black;
      leds[index2] = CRGB::Black;

      FastLED.show();
      FastLED.delay(wait);
    }
  }
}

void collapseOffStairs(int wait)
{
  for (int i = 0; i < STAIR; i++)
  {
    int mid = LEDS_PER_STAIR / 2;
    for (int j = 0; j <= mid; j++)
    {
      int index1 = i * LEDS_PER_STAIR + j;
      int index2 = i * LEDS_PER_STAIR + (LEDS_PER_STAIR - 1 - j);

      leds[index1] = CRGB::Black;
      leds[index2] = CRGB::Black;

      FastLED.show();
      FastLED.delay(wait);
    }
  }
}

void rainbowWave1(uint16_t wait)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (uint16_t j = 0; j < 256; j++)
    {
      for (uint16_t i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CHSV((i + j) & 255, 255, 80); // Farbwechsel durch Verschiebung
      }
      FastLED.show();
      FastLED.delay(wait); // Geschwindigkeit des Farbwechsels
    }
  }
}

void MeteorRain(int Color, int TrailDecay, int speedDelay, bool RandomDecay)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if ((!RandomDecay) || (random(10) > 5))
      {
        leds[i].fadeToBlackBy(TrailDecay);
      }
    }
    int pos = random(NUM_LEDS);
    leds[pos] = CHSV(Color, 255, 150);
    FastLED.show();
    FastLED.delay(speedDelay);
  }
}

void Fire2(int Cooling, int Sparks, int DelayDuration, int downUp)
{
  int Color = random(0, 8); // Zufällige Farbe zwischen 0 und 7 auswählen

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++) // 1) Slight cool down for each cell
    {
      cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);
      if (cooldown > heat[i])
      {
        heat[i] = 0;
      }
      else
      {
        heat[i] = heat[i] - cooldown;
      }
    }
    for (int k = (NUM_LEDS - 1); k >= 2; k--) // 2) Heat from each cell drifts up and diffuses slightly
    {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }
    if (random(255) < Sparks) // 3) Randomly ignite new Sparks near bottom of the flame
    {
      int y = random(7);
      heat[y] = heat[y] + random(160, 255);
    }
    for (int n = 0; n < NUM_LEDS; n++) // 4) Convert heat cells to LED colors
    {
      byte temperature = heat[n];                     // Temperature ranges from 0 (black/cold) to 255 (white/hot)
      byte t192 = round((temperature / 255.0) * 191); // Rescale heat from 0-255 to 0-191

      byte heatramp = t192 & 0x3F; // 0...63       // Calculate ramp up from
      heatramp <<= 2;              // scale up to 0...252
      int BottomColor[8][3] = {
          // Array of Bottom flame colors (for spark)
          {heatramp / 2, heatramp / 2, 255}, // 0 - blue sparks on white flame
          {255, 255, heatramp},              // 1 - white/yellow sparks on red flame
          {255, heatramp, heatramp},         // 2 - white/red sparks on yellow flame
          {heatramp, heatramp, 255},         // 3 - white/blue sparks on green flame
          {heatramp, heatramp, 255},         // 4 - white/blue sparks on cyan flame
          {255, 255, heatramp},              // 5 - white/yellow sparks on blue flame
          {255, heatramp, heatramp},         // 6 - white/red sparks on purple flame
          {255, heatramp, heatramp},         // 7 - white/red sparks on pink flame
      };
      int MiddleColor[8][3] = {
          // Array of Middle flame colors
          {heatramp / 2, heatramp / 2, heatramp},     // 0 - white/blue
          {255, heatramp, 0},                         // 1 - red/yellow
          {heatramp, heatramp, 0},                    // 2 - yellow
          {0, 255, heatramp / 2},                     // 3 - green/blue
          {0, heatramp, heatramp},                    // 4 - cyan
          {0, heatramp, 255},                         // 5 - blue/green
          {heatramp / 3, 0, heatramp / 2},            // 6 - purple
          {heatramp, heatramp / 4, heatramp * 2 / 3}, // 7 - pink
      };
      int TopColor[8][3] = {
          // Array of Top flame colors
          {heatramp, heatramp, heatramp},             // 0 - white
          {heatramp, 0, 0},                           // 1 - red
          {heatramp, heatramp, 0},                    // 2 - yellow
          {0, heatramp, 0},                           // 3 - green
          {0, heatramp, heatramp},                    // 4 - cyan
          {0, 0, heatramp},                           // 5 - blue
          {heatramp / 3, 0, heatramp / 2},            // 6 - purple
          {heatramp, heatramp / 4, heatramp * 2 / 3}, // 7 - pink
      };
      int Pixel = n;
      if (downUp == 2)
      {
        Pixel = (NUM_LEDS - 1) - n;
      }
      if (t192 > 0x80) // Set Pixels according to the three regions of the flame:
      {                // hottest (bottom of flame, heatramp between yellow and white)
        leds[Pixel].setRGB(round(BottomColor[Color][0]), round(BottomColor[Color][1]), round(BottomColor[Color][2]));
      }
      else if (t192 > 0x40)
      { // middle (midde of flame, heatramp with analogous Color)
        leds[Pixel].setRGB(round(MiddleColor[Color][0]), round(MiddleColor[Color][1]), round(MiddleColor[Color][2]));
      }
      else
      { // coolest (top of flame, heatramp heatramp with monochromatic Color)
        leds[Pixel].setRGB(round(TopColor[Color][0]), round(TopColor[Color][1]), round(TopColor[Color][2]));
      }
    }
    FastLED.show();
    FastLED.delay(DelayDuration);
  }
}

void FireDownwards(int Cooling, int Sparks, int DelayDuration, int downUp)
{
  int Color = random(0, 8);                // Zufällige Farbe zwischen 0 und 7 auswählen
  while (millis() - timeOut < PIR_TIMEOUT) // Solange das PIR-Timeout nicht erreicht ist
  {
    // Schritt 1: Abkühlung jeder Zelle
    for (int i = 0; i < NUM_LEDS; i++)
    {
      // Zufällige Abkühlung je nach Cooling-Einstellung
      cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);
      if (cooldown > heat[i])
      {
        heat[i] = 0;
      }
      else
      {
        heat[i] -= cooldown;
      }
    }
    // Schritt 2: Ausbreitung der Hitze nach unten und leichtes Diffundieren
    for (int k = 0; k < NUM_LEDS - 2; k++)
    {
      heat[k] = (heat[k + 1] + heat[k + 2] + heat[k + 2]) / 3;
    }
    // Schritt 3: Zufälliges Entzünden neuer Funken nahe dem unteren Teil der Flamme
    if (random(255) < Sparks)
    {
      int y = NUM_LEDS - random(7) - 1;
      heat[y] = heat[y] + random(160, 255);
    }
    // Schritt 4: Umwandlung der Hitze in LED-Farben
    for (int n = 0; n < NUM_LEDS; n++)
    {
      byte temperature = heat[n];                     // Temperaturwert der aktuellen LED-Zelle
      byte t192 = round((temperature / 255.0) * 191); // Skalierung der Hitze auf den Bereich 0-191
      byte heatramp = t192 & 0x3F;                    // Hitzerampe für die Farbverläufe
      heatramp <<= 2;                                 // Skalierung der Hitzerampe auf 0-252
      // Arrays für die Farben der unteren, mittleren und oberen Teile der Flamme
      int BottomColor[8][3] = {
          {heatramp / 2, heatramp / 2, 255},
          {255, 255, heatramp},
          {255, heatramp, heatramp},
          {heatramp, heatramp, 255},
          {heatramp, heatramp, 255},
          {255, 255, heatramp},
          {255, heatramp, heatramp},
          {255, heatramp, heatramp},
      };
      int MiddleColor[8][3] = {
          {heatramp / 2, heatramp / 2, heatramp},
          {255, heatramp, 0},
          {heatramp, heatramp, 0},
          {0, 255, heatramp / 2},
          {0, heatramp, heatramp},
          {0, heatramp, 255},
          {heatramp / 3, 0, heatramp / 2},
          {heatramp, heatramp / 4, heatramp * 2 / 3},
      };
      int TopColor[8][3] = {
          {heatramp, heatramp, heatramp},
          {heatramp, 0, 0},
          {heatramp, heatramp, 0},
          {0, heatramp, 0},
          {0, heatramp, heatramp},
          {0, 0, heatramp},
          {heatramp / 3, 0, heatramp / 2},
          {heatramp, heatramp / 4, heatramp * 2 / 3},
      };
      int Pixel = n;
      // Abhängig von der downUp-Variable, um die Richtung des Effekts zu steuern
      if (downUp == 2)
      {
        Pixel = (NUM_LEDS - 1) - n; // Umgekehrte Richtung (von unten nach oben)
      }
      // Festlegen der Farbe basierend auf der Temperatur der LED-Zelle
      if (t192 > 0x80)
      {
        leds[Pixel].setRGB(round(BottomColor[Color][0]), round(BottomColor[Color][1]), round(BottomColor[Color][2]));
      }
      else if (t192 > 0x40)
      {
        leds[Pixel].setRGB(round(MiddleColor[Color][0]), round(MiddleColor[Color][1]), round(MiddleColor[Color][2]));
      }
      else
      {
        leds[Pixel].setRGB(round(TopColor[Color][0]), round(TopColor[Color][1]), round(TopColor[Color][2]));
      }
    }
    FastLED.show();               // Anzeigen der aktualisierten LED-Farben
    FastLED.delay(DelayDuration); // Wartezeit bis zur nächsten Aktualisierung
  }
}

void FireDiagonal(int Cooling, int Sparks, int DelayDuration, bool downUp)
{
  int Color = random(0, 8); // Zufällige Farbe zwischen 0 und 7 auswählen

  while (millis() - timeOut < PIR_TIMEOUT) // Solange das PIR-Timeout nicht erreicht ist
  {
    // Schritt 1: Abkühlung jeder Zelle
    for (int i = 0; i < NUM_LEDS; i++)
    {
      // Zufällige Abkühlung je nach Cooling-Einstellung
      cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);
      if (cooldown > heat[i])
      {
        heat[i] = 0;
      }
      else
      {
        heat[i] -= cooldown;
      }
    }
    // Schritt 2: Diagonale Ausbreitung der Hitze
    for (int k = NUM_LEDS - 1; k >= 2; k--)
    {
      if (k % 2 == 0)
      {
        heat[k] = (heat[k - 2] + heat[k - 1]) / 2;
      }
      else
      {
        heat[k] = (heat[k - 1] + heat[k - 3]) / 2;
      }
    }
    // Schritt 3: Zufälliges Entzünden neuer Funken
    if (random(255) < Sparks)
    {
      int y = random(7);
      heat[y] = heat[y] + random(160, 255);
    }
    // Schritt 4: Umwandlung der Hitze in LED-Farben
    for (int n = 0; n < NUM_LEDS; n++)
    {
      byte temperature = heat[n];                     // Temperaturwert der aktuellen LED-Zelle
      byte t192 = round((temperature / 255.0) * 191); // Skalierung der Hitze auf den Bereich 0-191

      byte heatramp = t192 & 0x3F; // Hitzerampe für die Farbverläufe
      heatramp <<= 2;              // Skalierung der Hitzerampe auf 0-252

      // Arrays für die Farben der unteren, mittleren und oberen Teile des Feuers
      int BottomColor[8][3] = {
          {heatramp / 2, heatramp / 2, 255},
          {255, 255, heatramp},
          {255, heatramp, heatramp},
          {heatramp, heatramp, 255},
          {heatramp, heatramp, 255},
          {255, 255, heatramp},
          {255, heatramp, heatramp},
          {255, heatramp, heatramp},
      };
      int MiddleColor[8][3] = {
          {heatramp / 2, heatramp / 2, heatramp},
          {255, heatramp, 0},
          {heatramp, heatramp, 0},
          {0, 255, heatramp / 2},
          {0, heatramp, heatramp},
          {0, heatramp, 255},
          {heatramp / 3, 0, heatramp / 2},
          {heatramp, heatramp / 4, heatramp * 2 / 3},
      };
      int TopColor[8][3] = {
          {heatramp, heatramp, heatramp},
          {heatramp, 0, 0},
          {heatramp, heatramp, 0},
          {0, heatramp, 0},
          {0, heatramp, heatramp},
          {0, 0, heatramp},
          {heatramp / 3, 0, heatramp / 2},
          {heatramp, heatramp / 4, heatramp * 2 / 3},
      };
      int Pixel = n;
      // Abhängig von der downUp-Variable die Pixelrichtung umkehren oder nicht
      if (downUp)
      {
        Pixel = (NUM_LEDS - 1) - n;
      }
      // Festlegen der Farbe basierend auf der Temperatur der LED-Zelle
      if (t192 > 0x80)
      {
        leds[Pixel].setRGB(round(BottomColor[Color][0]), round(BottomColor[Color][1]), round(BottomColor[Color][2]));
      }
      else if (t192 > 0x40)
      {
        leds[Pixel].setRGB(round(MiddleColor[Color][0]), round(MiddleColor[Color][1]), round(MiddleColor[Color][2]));
      }
      else
      {
        leds[Pixel].setRGB(round(TopColor[Color][0]), round(TopColor[Color][1]), round(TopColor[Color][2]));
      }
    }
    FastLED.show();               // Anzeigen der aktualisierten LED-Farben
    FastLED.delay(DelayDuration); // Wartezeit bis zur nächsten Aktualisierung
  }
}

void twinkle(uint16_t wait, int FadeToBlk)
{
  while (millis() - timeOut < PIR_TIMEOUT) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (random8() < 25) // 25% Chance, dass die LED funkelt
      {
        leds[i] = CHSV(random8(), 255, random8(128, 255)); // Zufällige Farb- und Helligkeitswerte
      }
      else
      {
        leds[i].fadeToBlackBy(FadeToBlk); // LED abdunkeln
      }
    }
    FastLED.show();      // LEDs anzeigen
    FastLED.delay(wait); // Wartezeit
  }
}

void sparkleEffect(uint16_t wait, uint8_t numSparks, bool useRandomColors)
{
  FastLED.clear(); // Setze alle LEDs auf Schwarz
  FastLED.show();  // LEDs anzeigen

  while (millis() - timeOut < PIR_TIMEOUT)
  {
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
    FastLED.delay(wait);
    FastLED.clear(); // Setze alle LEDs auf Schwarz
    FastLED.show();  // LEDs anzeigen
  }
}

void WaveRipple(int Color, int DelayDuration)
{
  int rippleCenter = 0;
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    rippleCenter = (rippleCenter + 1) % NUM_LEDS;
    for (int i = 0; i < NUM_LEDS; i++)
    {
      int distance = abs(i - rippleCenter);
      byte brightness = 255 - (distance * 20);
      if (brightness < 0)
        brightness = 0;
      leds[i] = CHSV(Color, 255, brightness);
    }
    FastLED.show();
    FastLED.delay(DelayDuration);
  }
}

void wavePattern(CRGB color1, CRGB color2, uint16_t wait)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = blend(color1, color2, sin8(i * 255 / NUM_LEDS)); // Mische zwei Farben basierend auf einer Sinuswelle
    }
    FastLED.show();
    FastLED.delay(wait); // Geschwindigkeit des Effekts
  }
}

void waveEffect(CRGB color, int waveDelay)
{
  // Speichere die aktuelle Helligkeit
  originalBrightness = FastLED.getBrightness();
  // Setze die gewünschte Helligkeit für den Effekt
  FastLED.setBrightness(100);
  while (millis() - timeOut < PIR_TIMEOUT)
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
  // Stelle die ursprüngliche Helligkeit wieder her
  FastLED.setBrightness(originalBrightness);
}

// Funktion für dynamische Farben
void comet(CRGB startColor, CRGB endColor, int cometSize, int speedDelay, int cometTrailDecay, int downUp)
{
  FastLED.clear(); // Alle LEDs werden zu Beginn gelöscht
  if (downUp == 1)
  {
    // Komet in Abwärtsrichtung
    for (int i = 0; i < NUM_LEDS * 2; i++) // Schleife zur Steuerung der Bewegung des Kometen
    {
      for (int j = 0; j < NUM_LEDS; j++) // Schleife zur Steuerung der LEDs
      {
        if ((j < i) && (j > i - cometSize)) // Überprüfung, ob sich die LED innerhalb des Kometenbereichs befindet
        {
          float ratio = float(j - (i - cometSize)) / float(cometSize); // Verhältnis zur Bestimmung der Farbmischung
          leds[j] = blend(startColor, endColor, ratio * 255);          // Farbmischung basierend auf dem Verhältnis
        }
        else
        {
          leds[j].fadeToBlackBy(cometTrailDecay); // LEDs außerhalb des Kometenbereichs verblassen
        }
      }
      FastLED.show();            // Anzeige der LEDs aktualisieren
      FastLED.delay(speedDelay); // Verzögerung für die Bewegungsgeschwindigkeit
    }
  }
  else if (downUp == 2)
  {
    // Komet in Aufwärtsrichtung
    for (int i = NUM_LEDS * 2 - 1; i >= 0; i--) // Schleife zur Steuerung der Bewegung des Kometen in umgekehrter Richtung
    {
      for (int j = 0; j < NUM_LEDS; j++) // Schleife zur Steuerung der LEDs
      {
        if ((j < i) && (j > i - cometSize)) // Überprüfung, ob sich die LED innerhalb des Kometenbereichs befindet
        {
          float ratio = float(j - (i - cometSize)) / float(cometSize); // Verhältnis zur Bestimmung der Farbmischung
          leds[j] = blend(startColor, endColor, ratio * 255);          // Farbmischung basierend auf dem Verhältnis
        }
        else
        {
          leds[j].fadeToBlackBy(cometTrailDecay); // LEDs außerhalb des Kometenbereichs verblassen
        }
      }
      FastLED.show();            // Anzeige der LEDs aktualisieren
      FastLED.delay(speedDelay); // Verzögerung für die Bewegungsgeschwindigkeit
    }
  }
}

void smoothComet(CRGB color, int cometSize, int speedDelay, int cometTrailDecay, int downUp)
{
  FastLED.clear();

  // Eine vollständige Schleife ohne erneute Überprüfung des Zeitlimits
  for (int i = 0; i < NUM_LEDS + cometSize; i++)
  {
    FastLED.clear(); // Clear the LED array at the beginning of each frame

    for (int j = 0; j < NUM_LEDS; j++)
    {
      int pixel;
      if (downUp == 1)
      {
        pixel = j; // Normal, oben nach unten
      }
      else if (downUp == 2)
      {
        pixel = (NUM_LEDS - 1) - j; // Unten nach oben
      }
      else
      {
        pixel = j; // Standardfall für ungültigen Wert von downUp
      }

      if (j >= i - cometSize && j < i)
      {
        leds[pixel] = color;
      }
      else
      {
        leds[pixel].fadeToBlackBy(cometTrailDecay);
      }
    }

    FastLED.show();
    FastLED.delay(speedDelay);
  }
}

void glowingComet(CRGB color, int cometSize, int speedDelay, int cometTrailDecay, int downUp)
{
  FastLED.clear();
  if (downUp == 1)
  {
    // Abwärtsrichtung
    for (int i = 0; i < NUM_LEDS * 2; i++)
    {
      for (int j = 0; j < NUM_LEDS; j++)
      {
        if ((j < i) && (j > i - cometSize))
        {
          // Gloweinstellung und Limitierung
          float distance = float(i - j) / cometSize;
          int glow = max(0, min(255, 255 - int(255 * distance))); // Begrenzung des Glow-Werts
          leds[j] = color;
          leds[j].fadeLightBy(glow);
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
  else if (downUp == 2)
  {
    // Aufwärtsrichtung
    for (int i = NUM_LEDS * 2 - 1; i >= 0; i--)
    {
      for (int j = 0; j < NUM_LEDS; j++)
      {
        if ((j < i) && (j > i - cometSize))
        {
          // Gloweinstellung und Limitierung
          float distance = float(i - j) / cometSize;
          int glow = max(0, min(255, 255 - int(255 * distance))); // Begrenzung des Glow-Werts
          leds[j] = color;
          leds[j].fadeLightBy(glow);
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

void colorWipe(CRGB color, uint16_t wait)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = color;
      FastLED.show();
      FastLED.delay(wait);
    }
    FastLED.delay(500); // Ersetze delay(500) durch FastLED.delay(500)

    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
      FastLED.show();
      FastLED.delay(wait);
    }
    FastLED.delay(500); // Ersetze delay(500) durch FastLED.delay(500)
  }
}

// Farbzyklus: Wechselt alle LEDs durch den gesamten Farbraum
void colorCycle(uint16_t wait)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int hue = 0; hue < 256; hue++)
    {
      for (int i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CHSV(hue, 255, Brightness); // Setze Farbe basierend auf dem aktuellen Farbwert
      }
      FastLED.show();
      FastLED.delay(wait); // Geschwindigkeit des Farbwechsels
    }
  }
}

void colorExplosion(CRGB baseColor, int speedDelay)
{
  while (millis() - timeOut < PIR_TIMEOUT)
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

// Starburst-Funktion mit zufälliger Farbe
void Starburst(int speedDelay, int FadeToBlk)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    int burst = random(NUM_LEDS);
    // Zufällige Farbe auswählen
    int colorIndex = random(90);
    CRGB color = getColorFromPROGMEM(colorIndex);

    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (i == burst)
      {
        leds[i] = color; // Setze die zufällige Farbe an der Burst-Position
      }
      else
      {
        leds[i].fadeToBlackBy(FadeToBlk); // Alle anderen LEDs werden gedimmt
      }
    }
    FastLED.show();
    FastLED.delay(speedDelay);
  }
}

void BouncingBall(int Color, int BallCount, int DelayDuration, bool individualColors, int downUp)
{
  float gravity = -9.81; // Schwerkraftkonstante, negative Werte für abwärts gerichtete Schwerkraft
  float startHeight = 1; // Anfangshöhe der Bälle

  float height[BallCount];                                      // Array zur Speicherung der aktuellen Höhe jedes Balls
  float impactVelocityStart = sqrt(-2 * gravity * startHeight); // Anfangsgeschwindigkeit bei Aufprall
  float impactVelocity[BallCount];                              // Array zur Speicherung der Aufprallgeschwindigkeit jedes Balls
  float timeSinceLastBounce[BallCount];                         // Array zur Speicherung der Zeit seit dem letzten Aufprall jedes Balls
  int position[BallCount];                                      // Array zur Speicherung der aktuellen LED-Position jedes Balls
  long clockTimeSinceLastBounce[BallCount];                     // Array zur Speicherung der Uhrzeit seit dem letzten Aufprall jedes Balls
  float dampening[BallCount];                                   // Array zur Speicherung des Dämpfungsfaktors jedes Balls

  // Initialisierung der Bälle
  for (int i = 0; i < BallCount; i++)
  {
    clockTimeSinceLastBounce[i] = millis();             // Setze die aktuelle Zeit
    height[i] = startHeight;                            // Setze die Anfangshöhe
    position[i] = 0;                                    // Setze die Anfangsposition
    impactVelocity[i] = impactVelocityStart;            // Setze die Anfangsgeschwindigkeit
    timeSinceLastBounce[i] = 0;                         // Setze die Zeit seit dem letzten Aufprall auf 0
    dampening[i] = 0.90 - float(i) / pow(BallCount, 2); // Setze den Dämpfungsfaktor, variiert für jeden Ball
  }
  while (millis() - timeOut < PIR_TIMEOUT)
  { // Führe die Animation aus, solange die Zeit nicht abgelaufen ist
    for (int i = 0; i < BallCount; i++)
    {
      timeSinceLastBounce[i] = millis() - clockTimeSinceLastBounce[i];                                                             // Berechne die Zeit seit dem letzten Aufprall
      height[i] = 0.5 * gravity * pow(timeSinceLastBounce[i] / 1000.0, 2.0) + impactVelocity[i] * timeSinceLastBounce[i] / 1000.0; // Berechne die aktuelle Höhe
      if (height[i] < 0)
      {                                                       // Wenn der Ball den Boden erreicht
        height[i] = 0;                                        // Setze die Höhe auf 0
        impactVelocity[i] = dampening[i] * impactVelocity[i]; // Verringere die Aufprallgeschwindigkeit durch Dämpfung
        clockTimeSinceLastBounce[i] = millis();               // Setze die Zeit des letzten Aufpralls auf die aktuelle Zeit
        if (impactVelocity[i] < 0.01)
        {                                          // Wenn die Geschwindigkeit sehr gering ist
          impactVelocity[i] = impactVelocityStart; // Setze die Aufprallgeschwindigkeit zurück
        }
      }
      position[i] = round(height[i] * (NUM_LEDS - 1) / startHeight); // Berechne die LED-Position basierend auf der Höhe
    }
    // Alle LEDs ausschalten
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Black;
    }
    // Bälle anzeigen
    for (int i = 0; i < BallCount; i++)
    {
      int displayPosition = position[i];
      if (downUp == 2)
      {
        // Invertiere die Position für die Aufwärtsrichtung
        displayPosition = NUM_LEDS - 1 - position[i];
      }
      if (individualColors)
      {
        leds[displayPosition] = CHSV(Color + (i * (256 / BallCount)), 255, 255); // Jeder Ball erhält eine eigene Farbe
      }
      else
      {
        leds[displayPosition] = CHSV(Color, 255, 255); // Alle Bälle haben die gleiche Farbe
      }
    }
    FastLED.show();               // Aktualisiere die LEDs
    FastLED.delay(DelayDuration); // Warte für die angegebene Verzögerungsdauer
  }
}

// Funktion für das Feuerwerk
void fireworks(uint16_t wait, int FadeToBlk)
{
  // Speichere die aktuelle Helligkeit
  uint8_t originalBrightness = FastLED.getBrightness();
  // Setze die gewünschte Helligkeit für den Effekt
  FastLED.setBrightness(255);
  unsigned long timeOut = millis(); // Setze den Startzeitpunkt für das Timeout

  while (millis() - timeOut < PIR_TIMEOUT) // Solange die Zeit innerhalb des PIR-Timeouts liegt
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i].fadeToBlackBy(FadeToBlk); // Alle LEDs leicht abdunkeln
    }
    int pos = random(NUM_LEDS);                   // Zufällige Position für das Feuerwerk
    int colorIndex = random(90);                  // Zufälliger Index für die Farbe
    CRGB color = getColorFromPROGMEM(colorIndex); // Zufällige Farbe auswählen
    leds[pos] = color;                            // LED an der zufälligen Position auf die ausgewählte Farbe setzen
    FastLED.show();                               // LEDs anzeigen
    FastLED.delay(wait);                          // Wartezeit
  }
  // Stelle die ursprüngliche Helligkeit wieder her
  FastLED.setBrightness(originalBrightness);
}

void fadeToColor(CRGB targetColor, int steps, int delayTime)
{
  CRGB currentColor = leds[0]; // Initialisiere die aktuelle Farbe mit der ersten LED

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < steps; i++)
    {
      uint8_t blendAmount = 255 * i / steps;
      // Setze die Farbe für jede LED basierend auf dem blendAmount
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = blend(currentColor, targetColor, blendAmount);
      }
      // Zeige die LEDs an
      FastLED.show();
      // Warte für die Verzögerung
      FastLED.delay(delayTime);
    }
    // Beende die Schleife, wenn der Fade-Vorgang abgeschlossen ist
    break;
  }
  // Stelle sicher, dass die LEDs auf die endgültige Ziel-Farbe eingestellt sind
  fill_solid(leds, NUM_LEDS, targetColor);
  FastLED.show();
}

void bottomup_colorchange()
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int stair = STAIR - 1; stair >= 0; stair--)
    {
      // Auswahl einer zufälligen Farbe aus dem definierten Farbspektrum
      CRGB color = getColorFromPROGMEM(random16(sizeof(colors) / sizeof(colors[0])));

      // Setze die Farbe für alle LEDs auf der aktuellen Stufe
      for (int led = 0; led < LEDS_PER_STAIR; led++)
      {
        leds[stair * LEDS_PER_STAIR + led] = color;
      }

      // Zeige die Änderung nach der aktuellen Schleife an
      FastLED.show();

      // Verzögerung für den Übergangseffekt
      FastLED.delay(50); // Wartezeit von 50 Millisekunden
    }
  }
}

void colourWipeDownUp(uint32_t color, int wait, int r, int downUp)
{
  // Wenn r 1 ist, wähle eine zufällige Farbe.
  if (r == 1)
  {
    color = colors[random8(0, sizeof(colors) / sizeof(colors[0]))];
  }

  // Wisch-Effekt von unten nach oben und oben nach unten
  for (int j = 0; j < STAIR; j++)
  {
    int start, end;
    // Effekt von unten nach oben
    if (downUp != 2)
    {
      start = NUM_LEDS / STAIR * j;
      end = start + LEDS_PER_STAIR;
      for (int k = 0; k <= 50; k++)
      {
        for (int i = start; i < end; i++)
        {
          leds[i] = blend(CRGB::Black, color, k * 5); // Direkte Multiplikation für Farbblendung
        }
        FastLED.show();      // Zeige die LEDs an
        FastLED.delay(wait); // Verzögere den nächsten Schritt
      }
    }
    // Effekt von oben nach unten
    if (downUp != 1)
    {
      start = NUM_LEDS - (NUM_LEDS / STAIR * (j + 1));
      end = start + LEDS_PER_STAIR;
      for (int k = 0; k <= 50; k++)
      {
        for (int i = start; i < end; i++)
        {
          leds[i] = blend(CRGB::Black, color, k * 5); // Direkte Multiplikation für Farbblendung
        }
        FastLED.show();      // Zeige die LEDs an
        FastLED.delay(wait); // Verzögere den nächsten Schritt
      }
    }
  }
}

void efekt(uint16_t wait, uint32_t color, int downUp)
{
  int halfStairs = STAIR / 2;
  int halfLedsPerStair = LEDS_PER_STAIR / 2;

  // Helper function to handle LED coloring with boundary checks
  auto colorStep = [&](int start, bool reverse)
  {
    for (int i = 0; i < halfLedsPerStair; i++)
    {
      int left = reverse ? start - i - 1 : start + i;
      int right = reverse ? start - LEDS_PER_STAIR + i : start + LEDS_PER_STAIR - i - 1;

      // Ensure indices are within valid range
      if (left >= 0 && left < NUM_LEDS)
        leds[left] = color;
      if (right >= 0 && right < NUM_LEDS)
        leds[right] = color;
    }
  };

  auto colorReverseStep = [&](int start, bool reverse)
  {
    for (int i = halfLedsPerStair; i > 0; i--)
    {
      int left = reverse ? start - LEDS_PER_STAIR - i : start + LEDS_PER_STAIR + i - 1;
      int right = reverse ? start - 2 * LEDS_PER_STAIR + i - 1 : start + 2 * LEDS_PER_STAIR - i;

      // Ensure indices are within valid range
      if (left >= 0 && left < NUM_LEDS)
        leds[left] = color;
      if (right >= 0 && right < NUM_LEDS)
        leds[right] = color;
    }
  };

  // Effekt von unten nach oben
  if (downUp == 1)
  {
    for (int j = 0; j < halfStairs; j++)
    {
      int start = LEDS_PER_STAIR * j * 2;
      colorStep(start, false);
      FastLED.show();
      FastLED.delay(wait);
      colorReverseStep(start, false);
      FastLED.show();
      FastLED.delay(wait);
    }
  }

  // Effekt von oben nach unten
  else if (downUp == 2)
  {
    for (int j = halfStairs; j > 0; j--)
    {
      int start = LEDS_PER_STAIR * (j * 2 - 2);
      colorStep(start, true);
      FastLED.show();
      FastLED.delay(wait);
      colorReverseStep(start, true);
      FastLED.show();
      FastLED.delay(wait);
    }
  }
}

// Diese Funktion erzeugt einen Effekt, bei dem die LEDs einer Treppenstufe nacheinander von den Enden zur Mitte hin eingefärbt werden.
void efekt1(uint16_t wait, uint32_t color, int downUp)
{
  auto colorStep = [&](int start, bool reverse)
  {
    for (uint16_t i = 0; i < LEDS_PER_STAIR / 2; i++)
    {
      int left = start + i;
      int right = start + LEDS_PER_STAIR - i - 1;

      if (reverse)
      {
        left = start - i - 1;
        right = start - LEDS_PER_STAIR + i;
      }

      leds[left] = color;
      leds[right] = color;
    }
  };

  if (downUp != 2)
  {
    // Effekt von unten nach oben
    for (uint16_t j = 0; j < STAIR; j++)
    {
      uint16_t start = LEDS_PER_STAIR * j;
      colorStep(start, false);
      FastLED.show();      // Zeige die LEDs an
      FastLED.delay(wait); // Verzögere den nächsten Schritt
    }
  }

  if (downUp != 1)
  {
    // Effekt von oben nach unten
    for (uint16_t j = STAIR; j > 0; j--)
    {
      uint16_t start = LEDS_PER_STAIR * j;
      colorStep(start, true);
      FastLED.show();      // Zeige die LEDs an
      FastLED.delay(wait); // Verzögere den nächsten Schritt
    }
  }
}

// Diese Funktion kombiniert den efekt1-Effekt mit einem zusätzlichen Weißblitz am Ende.
void efekt2(uint16_t wait, uint32_t color, int white, int downUp)
{
  // Effekt von unten nach oben oder oben nach unten, abhängig von `downUp`.
  for (int j = 0; j < STAIR; j++)
  {
    uint16_t start;

    // Bestimme den Startindex basierend auf der Richtung
    if (downUp == 1)
    {
      start = LEDS_PER_STAIR * j; // Effekt von unten nach oben
    }
    else if (downUp == 2)
    {
      start = LEDS_PER_STAIR * (STAIR - j - 1); // Effekt von oben nach unten
    }
    else
    {
      continue; // Falls `downUp` weder 1 noch 2 ist, überspringe diese Iteration
    }

    // Färbe die LEDs von beiden Enden zur Mitte hin ein
    for (uint16_t i = 0; i < LEDS_PER_STAIR / 2; i++)
    {
      int left, right;

      if (downUp == 1)
      {
        left = start + i;
        right = start + LEDS_PER_STAIR - i - 1;
      }
      else
      {
        left = start - i - 1;
        right = start - LEDS_PER_STAIR + i;
      }

      // Setze die Farben
      leds[left] = color;
      leds[right] = color;

      // Zeige die LEDs an
      FastLED.show();
      FastLED.delay(wait);
    }
  }
  // Weißblitz-Effekt, falls `white` 1 ist
  if (white == 1)
  {
    uint16_t steps = 20;    // Anzahl der Schritte für den Weißblitz
    uint16_t delayTime = 3; // Verzögerung zwischen den Schritten

    for (uint16_t k = 0; k <= steps; k++)
    {
      // Berechne den Blendfaktor
      uint8_t blendFactor = map(k, 0, steps, 0, 255);

      // Färbe alle LEDs von der aktuellen Farbe zu Weiß
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = blend(color, CRGB::White, blendFactor);
      }

      // Zeige die LEDs an
      FastLED.show();
      FastLED.delay(delayTime);
    }
  }
}

void starrySky(CRGB color, int flashDelay, int numberOfStars)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    fill_solid(leds, NUM_LEDS, CRGB::Black); // Setze den Hintergrund auf schwarz

    // Setze die LEDs für die Anzahl der Sterne
    for (int i = 0; i < numberOfStars; i++)
    {
      int pos = random(NUM_LEDS); // Wähle eine zufällige Position für den Stern
      leds[pos] = color;          // Setze die Farbe des Sterns
    }

    FastLED.show();            // Zeige die LEDs an
    FastLED.delay(flashDelay); // Verzögere die nächste Aktualisierung
  }
}

void stepColorChange(uint32_t color, int wait, int direction)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    if (direction == 1) // Von unten nach oben
    {
      for (int stair = 0; stair < STAIR; stair++)
      {
        fill_solid(leds + (LEDS_PER_STAIR * stair), LEDS_PER_STAIR, color);
        FastLED.show();
        FastLED.delay(wait);
        fill_solid(leds + (LEDS_PER_STAIR * stair), LEDS_PER_STAIR, CRGB::Black); // Löschen der vorherigen Stufe
      }
    }
    else if (direction == 2) // Von oben nach unten
    {
      for (int stair = STAIR - 1; stair >= 0; stair--)
      {
        fill_solid(leds + (LEDS_PER_STAIR * stair), LEDS_PER_STAIR, color);
        FastLED.show();
        FastLED.delay(wait);
        fill_solid(leds + (LEDS_PER_STAIR * stair), LEDS_PER_STAIR, CRGB::Black); // Löschen der vorherigen Stufe
      }
    }
  }
}

// Die Raindrop-Funktion erzeugt einen Regentropfen-Effekt mit den angegebenen Parametern
void Raindrop(CRGB color, int speed, int spawnProbability, int tailLength)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    // Verschiebe alle LEDs um eine Position nach unten
    for (int i = NUM_LEDS - 1; i > 0; i--)
    {
      leds[i] = leds[i - 1];
    }
    // Füge einen neuen Regentropfen hinzu basierend auf der Wahrscheinlichkeit
    if (random(100) < spawnProbability)
    {
      // Setze die erste LED auf die angegebene Farbe
      leds[0] = color;
      // Zeichne den Schweif des Regentropfens
      for (int j = 1; j <= tailLength; j++)
      {
        if (j < NUM_LEDS)
        {
          CRGB tailColor = color;
          tailColor.fadeLightBy(255 - (255 * j / tailLength)); // Fade out des Schweifs
          leds[j] = tailColor;
        }
      }
    }
    else
    {
      leds[0] = CRGB::Black; // Setze die erste LED auf Schwarz, wenn kein Tropfen erscheint
    }
    FastLED.show();       // Zeige die LEDs an
    FastLED.delay(speed); // Verzögerung basierend auf der angegebenen Geschwindigkeit
  }
}

void Heartbeat(CRGB color, int BeatDelay, int FadeDelay)
{

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    unsigned long timeInCycle = millis() % BeatDelay;
    float phase = (float)timeInCycle / BeatDelay;

    // Sinusfunktion zur Berechnung der Helligkeit
    float brightness = sin(phase * PI) * 255;

    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = color;
      leds[i].fadeLightBy(255 - (int)brightness);
    }

    FastLED.show();
    FastLED.delay(FadeDelay); // Warten, um den Effekt zu verlangsamen
  }
}

void NoiseEffect2(uint8_t scale, uint8_t speed, uint8_t colorSpeed)
{
  // Globale Variablen für das Rauschen
  uint16_t noiseX = 0;
  uint16_t noiseY = 0;
  uint16_t noiseZ = 0;

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int stair = 0; stair < STAIR; stair++)
    {
      for (int led = 0; led < LEDS_PER_STAIR; led++)
      {
        int index = stair * LEDS_PER_STAIR + led; // Berechne den Index basierend auf der Stufe und LED-Position

        // Berechnung des Rauschwertes
        uint8_t noiseValue = inoise8(noiseX + index * scale, noiseY + index * scale, noiseZ);

        // Umwandlung des Rauschwertes in eine Farbe
        leds[index] = CHSV(noiseValue + millis() / colorSpeed, 255, 255);
      }
    }

    // Aktualisierung der Rauschkoordinaten
    noiseX += speed;
    noiseY += speed / 2;
    noiseZ += 1;

    FastLED.show();
    FastLED.delay(20);
  }
}

void NoiseEffect(uint16_t scale)
{
  static uint16_t x = random16();
  static uint16_t y = random16();
  static uint16_t z = random16();

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int stair = 0; stair < STAIR; stair++)
    {
      for (int led = 0; led < LEDS_PER_STAIR; led++)
      {
        int index = stair * LEDS_PER_STAIR + led; // Berechne den Index basierend auf der Stufe und LED-Position
        uint8_t noise = inoise8(x + index * scale, y + index * scale, z);
        leds[index] = CHSV(noise, 255, 255);
      }
    }
    z += 1; // Ändere die Geschwindigkeit des Effekts
    FastLED.show();
    delay(20); // Verzögerung zwischen den Aktualisierungen
  }
}

void flashingBlitz(uint32_t color, int flashDelay, int downUp)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    if (downUp == 1) // Von unten nach oben
    {
      for (int j = 0; j < STAIR; j++)
      {
        fill_solid(leds + (LEDS_PER_STAIR * j), LEDS_PER_STAIR, color);
        FastLED.show();
        FastLED.delay(flashDelay);
        fill_solid(leds + (LEDS_PER_STAIR * j), LEDS_PER_STAIR, CRGB::Black);
      }
    }
    else if (downUp == 2) // Von oben nach unten
    {
      for (int j = STAIR - 1; j >= 0; j--)
      {
        fill_solid(leds + (LEDS_PER_STAIR * j), LEDS_PER_STAIR, color);
        FastLED.show();
        FastLED.delay(flashDelay);
        fill_solid(leds + (LEDS_PER_STAIR * j), LEDS_PER_STAIR, CRGB::Black);
      }
    }
  }
}

void NoiseStorm(int speed, int intensity, int fadeAmount)
{
  static uint8_t noise[NUM_LEDS];

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      noise[i] = random8();
    }
    for (int i = 0; i < NUM_LEDS; i++)
    {
      uint8_t index = (i + millis() / speed) % NUM_LEDS;
      leds[i] = CHSV(noise[index], 255, intensity);
    }
    fadeToBlackBy(leds, NUM_LEDS, fadeAmount);
    FastLED.show();
    FastLED.delay(50);
  }
}

void Aurora(int waveSpeed, uint8_t baseColor, uint8_t hueRange, int intensity, int delayDuration)
{
  static uint8_t position = 0; // Aktuelle Position der Welle
  // Haupt-Effekt-Schleife
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    // Durchlaufe alle LEDs und berechne die Farbe basierend auf der Wellenform
    for (int i = 0; i < NUM_LEDS; i++)
    {
      // Berechne den Wellenwert für die aktuelle LED
      uint8_t waveValue = sin8(i * intensity + position);
      // Setze die Farbe der LED, basierend auf der Basisfarbe und dem Wellenwert
      leds[i] = CHSV(baseColor + waveValue / 2, 255, waveValue);
    }
    // Aktualisiere die Wellenposition
    position += waveSpeed;
    // Zeige die Änderungen auf den LEDs an
    FastLED.show();
    // Verzögere die nächste Aktualisierung
    FastLED.delay(delayDuration);
  }
}

void Aurora2()
{
  static uint16_t angle = 0;
  static uint16_t hue = 0;

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(hue + sin8(angle + (i % 15) * 2), 255, 255);
    }
    angle += 4;
    hue += 1;
    FastLED.show();
    FastLED.delay(20);
  }
}

void Plasma(int speed, int colorShift, int delayDuration)
{
  static uint8_t xOffset = 0;
  static uint8_t yOffset = 0;

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    // Berechne die Farben für jede LED basierend auf den Sinuswerten
    for (int i = 0; i < NUM_LEDS; i++)
    {
      // Berechne die Farbe basierend auf den Sinuswerten und den Offsets
      uint8_t wave1 = sin8(i * 2 + xOffset);
      uint8_t wave2 = sin8(i * 2 + yOffset);
      uint8_t colorValue = (wave1 + wave2) / 2;
      leds[i] = CHSV(colorValue + colorShift, 255, 255);
    }
    // Aktualisiere die Offsets für die nächste Iteration
    xOffset += speed;
    yOffset += speed;
    // Zeige die aktualisierten Farben an
    FastLED.show();
    // Verzögere die nächste Aktualisierung
    FastLED.delay(delayDuration);
  }
}

void Plasma2()
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      int colorIndex = (sin8((i % 15) * 16 + millis() / 2) + cos8((i % 15) * 32 + millis() / 3)) / 2;
      leds[i] = CHSV(colorIndex, 255, 255);
    }
    FastLED.show();
    FastLED.delay(50);
  }
}

void NoiseStorm()
{
  uint8_t scale = 30; // Skala des Rauschens
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      uint8_t noise = inoise8(i * scale, millis() / 10);
      leds[i] = CHSV(noise, 255, 255);
    }
    FastLED.show();
    FastLED.delay(50);
  }
}

void Sinelon()
{
  fadeToBlackBy(leds, NUM_LEDS, 20);
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    int pos = beatsin16(13, 0, NUM_LEDS - 1);
    leds[pos] += CHSV(millis() / 10, 255, 192);
    FastLED.show();
    FastLED.delay(20);
  }
}

void Juggle(int waitDuration, int hueIncrement)
{
  static uint8_t currentHue = 0;

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    // Setze jede LED auf eine Farbe, die von der aktuellen Hue und ihrer Position abhängt
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(currentHue + (i * 16), 255, 255);
    }

    // Erhöhe die Hue für die nächste Iteration
    currentHue += hueIncrement;

    // Zeige die aktualisierten Farben an
    FastLED.show();

    // Verzögere die nächste Aktualisierung
    FastLED.delay(waitDuration);
  }
}

void LavaLamp()
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(millis() / 10 + i * 8, 200, 255);
    }
    FastLED.show();
    FastLED.delay(30);
  }
}

void StarryNight(int fadeAmount, int sparkleChance, int minBrightness, int maxBrightness, int delayDuration)
{
  // Fade the LEDs slightly
  fadeToBlackBy(leds, NUM_LEDS, fadeAmount);
  while (millis() - timeOut < PIR_TIMEOUT)
  {

    // Add random white stars
    if (random8() < sparkleChance)
    {
      int pos = random16(NUM_LEDS);
      leds[pos] = CHSV(0, 0, random8(minBrightness, maxBrightness));
    }

    FastLED.show();
    FastLED.delay(delayDuration);
  }
}

void ColorPulse(CRGB color1, CRGB color2, unsigned long pulseSpeed, int pulseRange)
{
  static uint8_t pulseValue = 0;
  static int8_t pulseDirection = 1;
  static unsigned long lastUpdate = 0; // Zeitstempel der letzten Aktualisierung

  while (millis() - timeOut < PIR_TIMEOUT)
  {
    unsigned long currentTime = millis(); // Aktuelle Zeit

    // Überprüfen, ob es Zeit für die nächste Puls-Aktualisierung ist
    if (currentTime - lastUpdate >= pulseSpeed)
    {
      pulseValue += pulseDirection;
      if (pulseValue <= 0 || pulseValue >= pulseRange)
      {
        pulseDirection = -pulseDirection;
      }

      // Interpolieren zwischen color1 und color2
      CRGB currentColor = color1;
      currentColor.lerp8(color2, pulseValue);

      // LEDs auf die interpolierte Farbe setzen
      fill_solid(leds, NUM_LEDS, currentColor);
      FastLED.show();

      // Aktualisiere den Zeitstempel
      lastUpdate = currentTime;
    }

    // Warten, um die Schleife nicht zu schnell durchlaufen zu lassen
    FastLED.delay(10);
  }
}

void Fireflies()
{
  fadeToBlackBy(leds, NUM_LEDS, 20);
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    int pos = random16(NUM_LEDS);
    leds[pos] += CHSV(random8(0, 255), 200, 255);
    FastLED.show();
    FastLED.delay(50);
  }
}

void Fireflies2(uint8_t brightness, uint8_t density, uint8_t fadeSpeed)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    fadeToBlackBy(leds, NUM_LEDS, fadeSpeed);

    for (int i = 0; i < density; i++)
    {
      int pos = random16(NUM_LEDS);
      leds[pos] += CHSV(90, 255, brightness); // Grün-gelbes Licht
    }

    FastLED.show();
    FastLED.delay(50);
  }
}

void FlashAndFade()
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
    FastLED.delay(30);
    fadeToBlackBy(leds, NUM_LEDS, 50);
    FastLED.show();
    FastLED.delay(80);
  }
}

void OceanWaves()
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CHSV(160, 255, beatsin8(10, 0, 255, 0, i * 10));
    }
    FastLED.show();
    FastLED.delay(30);
  }
}

void OceanWaves2(uint8_t waveSpeed, uint8_t waveRange, uint8_t hue)
{
  while (millis() - timeOut < PIR_TIMEOUT)
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      // Der Effekt nutzt eine Sinuswelle, um den Helligkeitswert der LEDs zu modulieren
      leds[i] = CHSV(hue, 255, beatsin8(waveSpeed, 0, waveRange, 0, i * 10));
    }
    FastLED.show();
    FastLED.delay(30);
  }
}

void GlimmeringTwilight()
{
  fadeToBlackBy(leds, NUM_LEDS, 10);
  while (millis() - timeOut < PIR_TIMEOUT)
  {

    if (random8() < 80)
    {
      int pos = random16(NUM_LEDS);
      leds[pos] = CHSV(160, 200, random8(100, 255));
    }
    FastLED.show();
    FastLED.delay(50);
  }
}
