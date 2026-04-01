#include "ADS1X15.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define button 8 // Button is on digital pin 8
 
ADS1115 ADS(0x48);
static char Ps[8]; // Pressure array
float voltage_factor;
float v_ccpg;
float p_ccpg;
int16_t val_0;

float calc_p_ccpg(float voltage) 
{
  return pow(10, (1.666667 * voltage) - 11.46);
}
 
void setup() 
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("ADS1X15_LIB_VERSION: ");
  Serial.println(ADS1X15_LIB_VERSION);
 
  lcd.init();
  lcd.backlight();
  
  ADS.begin();
}
 
void loop() 
{
  ADS.setGain(0);
 
  val_0 = ADS.readADC(0);  
  //int16_t val_1 = ADS.readADC(1);   
 
  voltage_factor = ADS.toVoltage(1);  // voltage factor
  v_ccpg = val_0 * voltage_factor * 2; // Factor of 2 is to adjust for voltage divider
  p_ccpg = calc_p_ccpg(v_ccpg);
  Serial.print(p_ccpg);
 
  //Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t'); Serial.println(val_0 * f, 3);
  //Serial.print("\tAnalog1: "); Serial.print(val_1); Serial.print('\t'); Serial.println(val_1 * f, 3);
  //Serial.println();

  /*
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ADC Val:");
  lcd.print(val_0);
  lcd.setCursor(0, 1);
  lcd.print("Voltage:");
  lcd.print(v_ccpg, 3);
  lcd.print("V");
  */

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
