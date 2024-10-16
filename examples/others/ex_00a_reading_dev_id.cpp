#include <Arduino.h>
#include <SPI.h>
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "port.h"

void setup() {

  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  pinMode(DW_CS_PIN, OUTPUT);
  digitalWrite(DW_CS_PIN, HIGH);

  reset_DW3000();

  // Initialize DW3000
  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
    Serial.println("Failed to probe DW3000");
    while (1);
  }

  delay(100);

  /* Reads and validate device ID returns DWT_ERROR if it does not match expected else DWT_SUCCESS */
  if (dwt_check_dev_id() == DWT_SUCCESS)
  {
    Serial.println("Device ID is correct");
  }
  else
  {
    Serial.println("Device ID is incorrect");
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
