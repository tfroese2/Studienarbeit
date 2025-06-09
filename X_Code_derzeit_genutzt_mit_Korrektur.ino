#include <Wire.h>
#include <U8g2lib.h>
#include <MYUM7.h>

// OLED Display (SH1106, I2C)
U8G2_SH1106_128X64_NONAME_1_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// UM7 über Hardware Serial des Arduino Nano
MYUM7 imu(Serial);  // Serial = D0/RX, D1/TX

// Dynamische Offsets
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;

// Kursverfolgung / Wendeerkennung
float yaw_ref = 0.0;
float delta_yaw = 0.0;
String kurs_status = "Init...";

// Zeitsteuerung für automatische Aktualisierung
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 5000; // alle 5 Sekunden

//Für Beschleunigungs-Offsets
const int samples = 1000;
float sum_ax = 0, sum_ay = 0, sum_az = 0;
int collected = 0;


// Dynamische Kalibrierung der Beschleunigungs-Offsets
void calibrateAccelerometer() {

  display.firstPage();
  do {
    display.drawStr(0, 28, "Kalibriere Accel...");
  } while (display.nextPage());

  while (collected < samples) {
    if (Serial.available()) {
      if (imu.decode(Serial.read())) {
        sum_ax += imu.accel_x;
        sum_ay += imu.accel_y;
        sum_az += imu.accel_z;
        collected++;
      }
    }
  }

  ax_offset = (sum_ax / samples) * 9.80665;
  ay_offset = (sum_ay / samples) * 9.80665;
  az_offset = (sum_az / samples) * 9.80665;
}

void showDataOnDisplay() {
  char buf[12];

  float ax = imu.accel_x * 9.80665 - ax_offset;
  float ay = imu.accel_y * 9.80665 - ay_offset;
  float az = imu.accel_z * 9.80665 - az_offset;

  display.firstPage();
  do {
    display.drawStr(0, 0, "UM7 Sensorwerte:");

    display.drawStr(0, 12, "aX:");
    dtostrf(ax, 6, 2, buf); display.drawStr(30, 12, buf);

    display.drawStr(0, 22, "aY:");
    dtostrf(ay, 6, 2, buf); display.drawStr(30, 22, buf);

    display.drawStr(0, 32, "aZ:");
    dtostrf(az, 6, 2, buf); display.drawStr(30, 32, buf);

    display.drawStr(0, 42, "Yaw:");
    dtostrf(imu.yaw, 6, 2, buf); display.drawStr(30, 42, buf);

    display.drawStr(64, 42, "dYaw:");
    dtostrf(delta_yaw, 6, 2, buf); display.drawStr(90, 42, buf);

    display.drawStr(0, 52, "Status:");
    display.drawStr(50, 52, kurs_status.c_str());

    display.drawStr(0, 62, "Roll:");
    dtostrf(imu.roll, 6, 2, buf); display.drawStr(30, 62, buf);
    
    //display.drawStr(64, 12, "Orient:");
    //dtostrf(imu., 6, 2, buf); display.drawStr(30, 62, buf);

  } while (display.nextPage());
}

void setup() {
  display.begin();
  display.setFont(u8g2_font_5x7_tr);

  Serial.begin(115200);
  delay(100);

  imu.set_sensor_baud_rate(115200);
  delay(100);
  imu.set_all_processed_rate(255);
  delay(100);

  // Gyros nullen (Sensor muss ruhig liegen) vorher extremes nachregeln gehabt, hier also scheinbar nötig
  imu.zero_gyros();
  delay(1000);  // kurz Zeit für den Vorgang geben

  display.firstPage();
  do {
    display.drawStr(0, 28, "UM7 Initialisierung...");
  } while (display.nextPage());

  delay(1000);
  calibrateAccelerometer();

  // Referenz einmal initial setzen
  yaw_ref = imu.yaw;
  lastUpdateTime = millis();
}



void loop() {
  while (Serial.available()) {
    if (imu.decode(Serial.read())) {

      // Referenzkurs alle 5 Sekunden neu setzen
      if (millis() - lastUpdateTime >= updateInterval) {
        yaw_ref = imu.yaw;
        lastUpdateTime = millis();
      }

      // Delta-Yaw berechnen (mit Wrap-Around)
      delta_yaw = imu.yaw - yaw_ref;
      if (delta_yaw > 180) delta_yaw -= 360;
      if (delta_yaw < -180) delta_yaw += 360;

      // Kursstatus bestimmen
      if (abs(delta_yaw) < 5.0) {
        kurs_status = "Geradeaus";
      } else if (abs(delta_yaw) > 30.0) {
        kurs_status = "Wende erkannt!";
      } else {
        kurs_status = "Kursabweichung";
      }

      showDataOnDisplay();
    }
  }
}
