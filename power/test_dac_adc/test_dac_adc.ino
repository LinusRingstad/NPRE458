#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS7830.h>
#include <Wire.h>
#include "ADS1X15.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);


// Initialize DAC and ADC
Adafruit_MCP4728 mcp;
Adafruit_ADS7830 ad7830;


#define button 8 // Button is on digital pin 8
 
ADS1115 ADS(0x49);
static char Ps[8]; // Pressure array
float voltage_factor;
float v_ccpg;
float p_ccpg;
int16_t val_0;
float voltage_true;
float current_true;

float calc_p_ccpg(float voltage) 
{
  return pow(10, (1.666667 * voltage) - 11.46);
}

float calc_v(float inputv)
{
  return 1490.0/4095*inputv;
}

float calc_i(float inputi)
{
  return 0.0497/4095*inputi;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for console to open

  Serial.println("MCP4728 and ADS7830 Test");

  Serial.println(__FILE__);
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
  lcd.init();
  lcd.backlight();
  
  ADS.begin();

  // Initialize MCP4728 DAC
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) delay(10);
  }

  // Initialize ADS7830 ADC
  if (!ad7830.begin()) {
    Serial.println("Failed to initialize ADS7830!");
    while (1);
  }
}

void loop() {
  ADS.setGain(0);
  val_0 = ADS.readADC(0);  
  voltage_factor = ADS.toVoltage(1);  // voltage factor
  v_ccpg = val_0 * voltage_factor * 2; // Factor of 2 is to adjust for voltage divider
  p_ccpg = calc_p_ccpg(v_ccpg);

  // Set DAC values for current and voltage
  uint16_t voltageValue = 1800; // Example: Set voltage channel to ~3/4 max 0.0497  A
  uint16_t currentValue = 4065; // Example: Set current channel to ~1/3 max 1490 A

  mcp.setChannelValue(MCP4728_CHANNEL_A, voltageValue); // Voltage channel
  mcp.setChannelValue(MCP4728_CHANNEL_B, currentValue); // Current channel

  // Read ADC values for current and voltage
  uint8_t voltageADC = ad7830.readADCsingle(0); // Read voltage on channel 0 5:  0.46 kV ==5
  uint8_t currentADC = ad7830.readADCsingle(1); // Read current on channel 1: 49.4 mA == 17


  voltage_true = calc_v((float)voltageValue);
  current_true = calc_i((float)currentValue);

  // Print the results
  Serial.print(voltage_true);
  Serial.print(",");
  Serial.print(voltageADC);
  Serial.print(",");
  Serial.print(current_true);
  Serial.print(",");
  Serial.print(currentADC);
  Serial.print(",");
  Serial.println(p_ccpg);


  lcd.clear();
  lcd.setCursor(2, 0);
      
  dtostre(p_ccpg, Ps, 2, DTOSTR_UPPERCASE);
  for (int i=0; i<8; i++){
    lcd.print(Ps[i]);
  }
  
  //lcd.setCursor(0, 1); // This is because the 16x1 is weird
  lcd.print(" Torr");
  lcd.setCursor(0, 1);
  lcd.print("Voltage:");
  lcd.print(v_ccpg, 3);
  lcd.print("V");
  delay(1000);
}