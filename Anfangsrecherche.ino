// Die genaue Bibliothek muss ich noch auswählen, brauche erstmal aber den ESP32 und UM7 um auszuprobieren, laut Internet aber die meisten auzulesen mit um7.read(); und dann mit variablen wie ax_local=... benennen
// Zeitvariablen
unsigned long lastTime = 0; // unsigned heißt dass die Zahl nicht negativ werden kann, da Zeit auch nötig/logisch
float dt = 0; // float = fließkommazahl, da keine Kommazahlen benötigt

// Geschwindigkeit & Position
float vx = 0, vy = 0;
float x = 0, y = 0;

void setup() {
  Serial.begin(9600);
  // IMU Setup ...
  lastTime = millis();
}

void loop() {
  // Beispielwerte! ersetzen durch echte UM7-Werte, abhängig davon was die Bibliothek brauch
  float ax_local = 0.2; // Vorwärtsbeschleunigung
  float ay_local = 0.0; // Seitliche Beschleunigung
  float yaw_deg = 45.0; // Ausrichtung in Grad

  // Zeitdifferenz in Sekunden
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // dt ist die Zeitdifferenz zwischen 2 Messungen, / 1000 für Sekunden, da Milli
  lastTime = currentTime;

  // Yaw in Radiant umrechnen
  float yaw_rad = yaw_deg * PI / 180.0;

  // Koordinatentransformation: lokal → global
  float ax_global = ax_local * cos(yaw_rad) - ay_local * sin(yaw_rad);
  float ay_global = ax_local * sin(yaw_rad) + ay_local * cos(yaw_rad);

// Bis hierhin erstmal Werte eingelesen und interpretiert

  // Integration: Geschwindigkeit über Euler Methode, vlt nochmal überdenken, da bei Sensorrauschen umso ungenauer, Glättung der Integration soll ggf. über den internen Kalman Filter möglich sein?
  vx += ax_global * dt;
  vy += ay_global * dt;

  // Integration: Weg
  x += vx * dt;
  y += vy * dt;

  // Ausgabe
  Serial.print("Yaw: "); Serial.print(yaw_deg); Serial.print("°\t");

  Serial.print("a_x_global: "); Serial.print(ax_global); Serial.print("\t");
  Serial.print("v_x: "); Serial.print(vx); Serial.print("\t");
  Serial.print("x: "); Serial.print(x); Serial.print("\t");

  Serial.print("a_y_global: "); Serial.print(ay_global); Serial.print("\t");
  Serial.print("v_y: "); Serial.print(vy); Serial.print("\t");
  Serial.print("y: "); Serial.println(y);

  delay(50); // ~20 Hz
}
