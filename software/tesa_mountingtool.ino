/**
 * @file tesa_mountingtool.ino
 * @brief Cross Innovation Class Projekt für Tesa zur Orientierungserkennung und Bewegungskontrolle bei dem Aufkleben von Tesa Powerstrips mit dem BNO055-Sensor.
 * 
 * Dieses Projekt dient zum Leveling und als Führungssystem für gerade Bewegungen entlang einer Wand.
 * LEDs zeigen Abweichungen an. Zwei Betriebsmodi können über einen Button umgeschaltet werden.
 */
 
 #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/// @brief Pin für LED bei Neigung nach oben
#define LED_UP     19
/// @brief Pin für LED bei Neigung nach unten
#define LED_DOWN   2
/// @brief Pin für LED bei Neigung nach links
#define LED_LEFT   18
/// @brief Pin für LED bei Neigung nach rechts
#define LED_RIGHT  4
/// @brief Pin für den Modus-Button
#define BUTTON_PIN 13

/// @brief Toleranz in Grad für LED-Anzeige im LEVEL_MODE
#define ANGLE_TOLERANCE_DEG 5
/// @brief Engere Toleranz in Grad für Bewegungskontrolle im GUIDE_MODE
#define MOVEMENT_TOLERANCE_DEG 2

/// @brief BNO055 Sensorinstanz
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/// @brief Referenzwinkel für Pitch (Grad)
float refPitch = 0;
/// @brief Referenzwinkel für Roll (Grad)
float refRoll = 0;
/// @brief Referenzwinkel für Yaw (Grad)
float refYaw = 0;

/// @brief Letzte vertikale Beschleunigung (Y-Achse)
float lastAccelY = 0;
/// @brief Integrierte vertikale Beschleunigung zur Trendbestimmung
float accelYIntegral = 0;
/// @brief Trend der vertikalen Bewegung (geglättet)
float movementTrend = 0;


/**
 * @brief Betriebsmodi für das System
 * 
 * - LEVEL_MODE: normale Wasserwaage
 * - GUIDE_MODE: Führung für horizontale Bewegung
 */
enum MovementMode {
  LEVEL_MODE,
  GUIDE_MODE
};

/// @brief Aktueller Betriebsmodus
MovementMode currentMode = LEVEL_MODE;


/// @brief Zeitstempel des Button-Drücks
unsigned long buttonPressTime = 0;
/// @brief Button-Status (true, wenn gedrückt)
bool buttonPressed = false;

/**
 * @brief Holt remappte Euler-Winkel aus der aktuellen Sensororientierung.
 * 
 * Remapping ist notwendig, da der Sensor vertikal montiert ist.
 * Es wird ein Quaternion gelesen, normalisiert, remappt und in Euler-Winkel umgerechnet.
 * 
 * @return imu::Vector<3> Remappte Euler-Winkel in Grad: (Yaw, Roll, Pitch)
 */
imu::Vector<3> getRemappedEuler() {
  imu::Quaternion q = bno.getQuat();
  q.normalize();

  /// Remapping der Achsen
  float temp = q.x();  
  q.x() = -q.y();  
  q.y() = temp;
  q.z() = -q.z();

  imu::Vector<3> euler = q.toEuler();

  return imu::Vector<3>(
    -180 / M_PI * euler.x(),  ///< Yaw in Grad
    -180 / M_PI * euler.y(),  ///< Roll in Grad
    -180 / M_PI * euler.z()   ///< Pitch in Grad
  );
}

/**
 * @brief Initialisierung des Systems, Sensorstarts und Referenzsetzung.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial);

  /// Pins konfigurieren
  pinMode(LED_UP, OUTPUT);
  pinMode(LED_DOWN, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  /// Sensor intialisieren
  if (!bno.begin()) {
    Serial.println("Kein BNO055 gefunden!");
    while (1);
  }

  bno.setExtCrystalUse(true);
  delay(2000); // Extra-Wartezeit für vertikale Haltung
  
  /// Kalibrierung Status ausgeben (Debug)
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("Kalibration - System: "); Serial.print(system);
  Serial.print(" Gyro: "); Serial.print(gyro);
  Serial.print(" Accel: "); Serial.print(accel);
  Serial.print(" Mag: "); Serial.println(mag);

  /// Referenz setzen
  imu::Vector<3> eulerInit = getRemappedEuler();
  refYaw = eulerInit.x();
  refRoll = eulerInit.y();
  refPitch = eulerInit.z();

  Serial.print("Referenz - Yaw: "); Serial.print(refYaw, 1);
  Serial.print("° Roll: "); Serial.print(refRoll, 1);
  Serial.print("° Pitch: "); Serial.print(refPitch, 1);
  Serial.println("°");
  Serial.println("Modus: LEVEL_MODE");
}

/**
 * @brief Hauptprogrammschleife: verarbeitet Sensorwerte, steuert LEDs und Moduswechsel.
 */
void loop() {
  /// --- Buttonhandling ---
  bool buttonState = digitalRead(BUTTON_PIN) == LOW;
  
  if (buttonState && !buttonPressed) {
    buttonPressTime = millis();
    buttonPressed = true;
  }
  
  if (!buttonState && buttonPressed) {
    unsigned long pressDuration = millis() - buttonPressTime;
    
    if (pressDuration < 1000) {
      /// Kurzer Druck: Referenz neu setzen
      imu::Vector<3> eulerRef = getRemappedEuler();
      refYaw = eulerRef.x();
      refRoll = eulerRef.y();
      refPitch = eulerRef.z();

      Serial.println("Referenz zurückgesetzt.");
      Serial.print("Referenz - Yaw: "); Serial.print(refYaw, 1);
      Serial.print("° Roll: "); Serial.print(refRoll, 1);
      Serial.print("° Pitch: "); Serial.print(refPitch, 1);
      Serial.println("°");
    } else {
      /// Langer Druck: Modus wechseln
      if (currentMode == LEVEL_MODE) {
        currentMode = GUIDE_MODE;
        Serial.println("Modus: GUIDE_MODE - Führung für gerade Bewegung");
        
        /// Neue Referenz setzen
        imu::Vector<3> eulerRef = getRemappedEuler();
        refYaw = eulerRef.x();
        refRoll = eulerRef.y();
        refPitch = eulerRef.z();
      } else {
        currentMode = LEVEL_MODE;
        Serial.println("Modus: LEVEL_MODE - Normale Wasserwaage");
      }
    }
    
    buttonPressed = false;
    delay(300); // Entprellzeit
  }

  /// --- Sensorwerte lesen ---
  imu::Vector<3> euler = getRemappedEuler();
  float yaw = euler.x();
  float roll = euler.y();
  float pitch = euler.z();
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  /// Bewegungsanalyse an der Wand also vertikal
  float accelY = accel.y(); // Y-Achse = vertikal bei vertikaler Haltung
  movementTrend = movementTrend * 0.70 + accelY * 0.30; // Gleitender Durchschnitt

  /// Abweichungen berechnen
  float pitchDiff = pitch - refPitch;
  float rollDiff = roll - refRoll;
  float yawDiff = yaw - refYaw;

  /// Yaw-Differenz normalisieren (-180 bis +180)
  while (yawDiff > 180) yawDiff -= 360;
  while (yawDiff < -180) yawDiff += 360;

  /// --- Debug-Ausgabe ---
  Serial.print("Modus: "); Serial.print(currentMode == LEVEL_MODE ? "LEVEL" : "GUIDE");
  if (currentMode == GUIDE_MODE) {
    digitalWrite(LED_UP, HIGH);
    digitalWrite(LED_DOWN, HIGH);
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, HIGH);
    Serial.print(" | Trend: "); Serial.print(movementTrend, 3);
    Serial.print(" AccelY: "); Serial.print(accelY, 3);
  }
  Serial.print(" | Yaw: "); Serial.print(yaw, 1);
  Serial.print("° (Δ"); Serial.print(yawDiff, 1);
  Serial.print("°)  Roll: "); Serial.print(roll, 1);
  Serial.print("° (Δ"); Serial.print(rollDiff, 1);
  Serial.print("°)  Pitch: "); Serial.print(pitch, 1);
  Serial.print("° (Δ"); Serial.print(pitchDiff, 1);
  Serial.println("°)");

  /// --- LEDs steuern ---
  digitalWrite(LED_UP, LOW);
  digitalWrite(LED_DOWN, LOW);
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);

  if (currentMode == LEVEL_MODE) {
    /// Pitch = Neigung nach oben/unten
    if (pitchDiff > ANGLE_TOLERANCE_DEG)
      digitalWrite(LED_UP, HIGH);
    else if (pitchDiff < -ANGLE_TOLERANCE_DEG)
      digitalWrite(LED_DOWN, HIGH);

    /// Roll = Neigung nach links/rechts hat höhere Priorität über Yaw
    if (rollDiff > ANGLE_TOLERANCE_DEG)
      digitalWrite(LED_RIGHT, HIGH);
    else if (rollDiff < -ANGLE_TOLERANCE_DEG)
      digitalWrite(LED_LEFT, HIGH);
    /// Yaw = Drehung um vertikale Achse also im Uhrzeigersinn, nur wenn Roll im Toleranzbereich
    else if (yawDiff > ANGLE_TOLERANCE_DEG)
      digitalWrite(LED_LEFT, HIGH);
    else if (yawDiff < -ANGLE_TOLERANCE_DEG)
      digitalWrite(LED_RIGHT, HIGH);
      
  } else {
    /// GUIDE_MODE: Horizontale Bewegungsführung
    
    /// Hauptziel: Horizontale Bewegung (kein vertikaler Trend)
    bool verticalMovement = abs(movementTrend) > 0.05; // m/s² Schwelle
    
    if (verticalMovement) {
      /// Vertikale Bewegung erkannt - Korrektur anzeigen
      if (movementTrend > 0.08) {
        digitalWrite(LED_DOWN, HIGH);  /// Nach oben bewegt -> nach unten korrigieren
      } else if (movementTrend < -0.08) {
        digitalWrite(LED_UP, HIGH);    /// Nach unten bewegt -> nach oben korrigieren
      }
    }
    
    /// Orientierungs-Stabilität (sekundär)
    if (!verticalMovement) {
      /// Nur wenn keine vertikale Bewegung, dann Orientierung prüfen
      
      /// Yaw-Kontrolle (Richtung halten)
      if (abs(yawDiff) > MOVEMENT_TOLERANCE_DEG) {
        if (yawDiff > MOVEMENT_TOLERANCE_DEG)
          digitalWrite(LED_LEFT, HIGH);
        else if (yawDiff < -MOVEMENT_TOLERANCE_DEG)
          digitalWrite(LED_RIGHT, HIGH);
      }
      
      /// Roll-Kontrolle (seitliche Neigung)
      else if (abs(rollDiff) > MOVEMENT_TOLERANCE_DEG) {
        if (rollDiff > MOVEMENT_TOLERANCE_DEG)
          digitalWrite(LED_RIGHT, HIGH);
        else if (rollDiff < -MOVEMENT_TOLERANCE_DEG)
          digitalWrite(LED_LEFT, HIGH);
      }
      
      /// Pitch-Kontrolle (vertikale Neigung)
      else if (abs(pitchDiff) > MOVEMENT_TOLERANCE_DEG) {
        if (pitchDiff > MOVEMENT_TOLERANCE_DEG)
          digitalWrite(LED_UP, HIGH);
        else if (pitchDiff < -MOVEMENT_TOLERANCE_DEG)
          digitalWrite(LED_DOWN, HIGH);
      }
    }
  }

  delay(30);
}