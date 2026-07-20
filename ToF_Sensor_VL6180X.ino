// Written by Mahmud Hasan Saikot

#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!vl.begin()) {
    Serial.println("Failed to find VL6180X sensor!");
    while (1);
  }
  Serial.println("VL6180X ready.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'r') {  // command from Python
      float avg = getAverageDistance(10);
      Serial.print("AVG,");
      Serial.println(avg, 2);
    }
  }
}

float getAverageDistance(int N) {
  float sum = 0;
  int valid = 0;

  for (int i = 0; i < N; i++) {
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    // Valid reading: status 0 means "no error"
    if (status == 0 && range > 0 && range < 255) {
      sum += range;
      valid++;
    }

    delay(100);
  }

  if (valid == 0) {
    Serial.println("No valid readings.");
    return 0.0;
  }

  return sum / valid;
}
