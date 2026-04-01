// Basic demo for configuring the MCP4728 4-Channel 12-bit I2C DAC
#include <Adafruit_MCP4728.h>
#include <Wire.h>

Adafruit_MCP4728 mcp;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 
    
  Serial.println("Adafruit MCP4728 test!");

  // Try to initialize!
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }
  //voltage (max voltage)
  mcp.setChannelValue(MCP4728_CHANNEL_A, 4095);
  //current (1/2 max)
  mcp.setChannelValue(MCP4728_CHANNEL_B, 2048);

  //extra channels
  mcp.setChannelValue(MCP4728_CHANNEL_C, 1024);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0);
}

void loop() { delay(1000); }