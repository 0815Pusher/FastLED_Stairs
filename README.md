Treppenbeleuchtung mit LED-Effekten mit FastLED

Dieses Projekt nutzt einen BH1750 Lichtsensor, PIR-Sensoren und WS2812b LEDs zur Steuerung der Beleuchtung einer Treppe. Die Beleuchtung wird durch verschiedene visuelle Effekte gesteuert, die auf Lichtverhältnissen und Bewegung basieren. Hier sind die Hauptmerkmale und Funktionen des Codes:

Hauptmerkmale:

Lichtsensor (BH1750): Misst die Umgebungshelligkeit, um die LED-Helligkeit anzupassen und die LEDs nur bei Bedarf einzuschalten. Die Helligkeit wird basierend auf dem Lux-Wert skaliert, um eine sanfte Anpassung der LED-Leistung zu gewährleisten.

const unsigned long LDRThreshold = 380; // Only switch on LED's at night!
bool useLDR = true; // Ob der LDR (Lichtabhängiger Widerstand) verwendet werden soll
bool readPIRInputs = true; // Ob PIR-Eingaben gelesen werden sollen
const int maxBrightness = 250;
const int minBrightness = 30;
const float minLux = 0.0; // Minimaler Lux-Wert
const float maxLux = 200.0; // Maximaler Lux-Wert ab da, maping maxBrightness

PIR-Sensoren: Zwei PIR-Sensoren (oben und unten an der Treppe) erkennen Bewegungen. Je nach Bewegung werden 45 verschiedene LED-Effekte random ausgelöst, z.B. von oben nach unten oder von unten nach oben. Variablen Konfigurierbar in der "case" Funktion, 11 random Effekte zum Ausschalten nach PIR_TIMEOUT

Zeitkonstanten:
DETECTION_TIME: Zeit, nach der ein PIR-Sensor als ausgelöst gilt.
PIR_TIMEOUT: Zeit, nach der die LEDs ausgeschaltet werden, wenn keine Bewegung mehr erkannt wird.
TOTAL_OFF_TIMEOUT: Zeit, nach der die LEDs komplett ausgeschaltet bleiben, wenn keine Bewegung erfolgt.
shortLDRInterval und longLDRInterval: Zeitintervalle für die Lichtmessung durch den Lichtsensor.

LED-Konfiguration: Der Sketch verwendet WS2812b LEDs, die an einem bestimmten Datenpin angeschlossen sind. Verschiedene LED-Effekte wie Wellen- oder Spiralbewegungen werden genutzt, um die Beleuchtung dynamisch zu steuern.

// LED-Konfiguration
constexpr int DATA_PIN = 6; // Pin für das Datenkabel der WS2812b LEDs
constexpr int STAIR = 8; // Anzahl der Stufen
constexpr int LEDS_PER_STAIR = 15; // Anzahl der LEDs pro Stufe
constexpr int NUM_LEDS = STAIR * LEDS_PER_STAIR; // Gesamtzahl der LEDs
constexpr int VOLTS = 5; // Volt der LEDs einstellen
constexpr int MAX_MA = 7000; // Maximalleistung der LEDs einstellen in MilliAmpere

Funktionen:
setup(): Initialisiert die PIR-Sensoren, die LEDs und den Lichtsensor.
initializePIRSensors(): Konfiguriert die PIR-Sensor-Pins.
initializeLEDs(): Richtet die LEDs ein und setzt die maximale Leistung.
initializeLDR(): Initialisiert den Lichtsensor und prüft die Lichtverhältnisse.
loop(): Überprüft regelmäßig die Lichtverhältnisse und verarbeitet die PIR-Sensoren.
checkLDR(): Prüft den Lux-Wert und passt die LED-Helligkeit an.
handlePIR(): Verarbeitet die Eingaben von den PIR-Sensoren und löst entsprechende LED-Effekte aus.
clearLEDs(): Schaltet alle LEDs aus.
topdown(int mode): Führt verschiedene LED-Effekte basierend auf dem Modus aus, z.B. farbige Wischbewegungen, Feuer usw.
ledoff(int mode): Funktion zur Handhabung des Ausschaltens der LEDs

Debugging:
Die Debugging-Makros ermöglichen das Ausgeben von Informationen zur Überwachung des Systemstatus und der Sensordaten.

Farben:
Eine vordefinierte Palette von 90 Farben ist im Programm gespeichert und wird für die verschiedenen LED-Effekte verwendet.

