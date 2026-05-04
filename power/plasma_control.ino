/*
 * plasma_control.ino
 * ==================
 * Arduino Uno firmware for the Plasma Control System.
 *
 * HARDWARE:
 *   - MCP4728  quad DAC (I2C) — sets voltage and current on power supply
 *   - ADS7830  8-ch ADC (I2C) — reads back voltage and current from supply
 *   - ADS1115  16-bit ADC (I2C, addr 0x49) — reads pressure gauge voltage
 *   - 16x2 I2C LCD (addr 0x27) — local display
 *
 * SERIAL OUTPUT (115200 baud, once per loop):
 *   voltage_set, voltageADC, current_set, currentADC, pressure
 *   e.g.  "655.15,5,0.04884,17,2.31e-05"
 *
 *   Fields:
 *     voltage_set   float  DAC-implied voltage in Volts  (calc_v result)
 *     voltageADC    uint8  raw ADC count from ADS7830 ch0 (read-back)
 *     current_set   float  DAC-implied current in Amps   (calc_i result)
 *     currentADC    uint8  raw ADC count from ADS7830 ch1 (read-back)
 *     pressure      float  chamber pressure in Torr
 *
 * SERIAL INPUT (from Raspberry Pi):
 *   Command format:  "V<dac_v>,I<dac_i>\n"
 *   e.g.  "V1800,I4065\n"
 *
 *   dac_v  uint16  DAC counts for voltage channel (0–4095)
 *   dac_i  uint16  DAC counts for current channel (0–4095)
 *
 *   Both values are clamped to 0–4095 on receipt.
 *   If no command is received, the last setpoint is held.
 *   On startup both DAC channels are set to 0 (safe state).
 *
 * DAC SCALING (MCP4728, Vref = internal 2.048 V, x2 gain → 4.096 V FS):
 *   voltage_V  = 1490.0 / 4095.0 * dac_v    (max ~1490 V via HV supply)
 *   current_A  = 0.0497 / 4095.0 * dac_i    (max ~49.7 mA)
 *
 * ADC READ-BACK CALIBRATION (ADS7830, 8-bit):
 *   voltage_kV = voltageADC * (0.46 / 5.0)   kV/count
 *   current_mA = currentADC * (49.4 / 17.0)  mA/count
 *
 * PRESSURE CALIBRATION (CCPG gauge via ADS1115):
 *   v_ccpg = ADS1115_raw * voltage_factor * 2  (factor of 2 for divider)
 *   pressure_Torr = 10 ^ (1.666667 * v_ccpg - 11.46)
 */

#include <Adafruit_MCP4728.h>
#include <Adafruit_ADS7830.h>
#include <Wire.h>
#include "ADS1X15.h"
#include <LiquidCrystal_I2C.h>

// ── Hardware objects ────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MCP4728  mcp;
Adafruit_ADS7830  ad7830;
ADS1115           ADS(0x49);

// ── DAC setpoints (updated by serial commands from Pi) ──────────────────────
volatile uint16_t g_dac_voltage = 0;   // MCP4728 channel A — voltage
volatile uint16_t g_dac_current = 0;   // MCP4728 channel B — current

// ── Safety limits (DAC counts) ──────────────────────────────────────────────
// Voltage:  max 1.0 kV  → dac = 1000 * 4095/1490 ≈ 2748
// Current:  max 90 mA   → dac = 0.090 * 4095/0.0497 ≈ 7415  (but DAC max = 4095)
//           At full scale (4095) current = 0.0497 A ≈ 49.7 mA — already under 90 mA
//           so current is naturally limited by the DAC range.
const uint16_t DAC_MAX          = 4095;
const uint16_t DAC_VOLTAGE_MAX  = 2748;   // ~1.0 kV
const uint16_t DAC_CURRENT_MAX  = 4095;   // ~49.7 mA (hardware limit)

// ── LCD update cadence ──────────────────────────────────────────────────────
const unsigned long LCD_INTERVAL_MS = 1000;
unsigned long       g_last_lcd_ms   = 0;

// ── Pressure scratch buffer ─────────────────────────────────────────────────
static char Ps[8];


// ── Calibration functions ───────────────────────────────────────────────────

float calc_p_ccpg(float voltage)
{
  return pow(10.0f, (1.666667f * voltage) - 11.46f);
}

float calc_v(float dac_counts)
{
  return 1490.0f / 4095.0f * dac_counts;   // Volts
}

float calc_i(float dac_counts)
{
  return 0.0497f / 4095.0f * dac_counts;   // Amps
}


// ── Serial command parser ───────────────────────────────────────────────────
/*
 * Reads any complete "V<n>,I<n>\n" lines that have arrived in the serial
 * buffer and updates g_dac_voltage / g_dac_current.
 * Non-matching lines (e.g. startup noise) are silently discarded.
 */
void process_serial_commands()
{
  while (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Expect format: V<uint16>,I<uint16>
    if (cmd.length() < 5)      continue;
    if (cmd.charAt(0) != 'V')  continue;

    int comma = cmd.indexOf(',');
    if (comma < 2)             continue;
    if (cmd.charAt(comma + 1) != 'I') continue;

    uint16_t new_v = (uint16_t) constrain(
        cmd.substring(1, comma).toInt(), 0, DAC_VOLTAGE_MAX);

    uint16_t new_i = (uint16_t) constrain(
        cmd.substring(comma + 2).toInt(), 0, DAC_CURRENT_MAX);

    g_dac_voltage = new_v;
    g_dac_current = new_i;

    // Apply to DAC immediately
    mcp.setChannelValue(MCP4728_CHANNEL_A, g_dac_voltage);
    mcp.setChannelValue(MCP4728_CHANNEL_B, g_dac_current);
  }
}


// ── Setup ───────────────────────────────────────────────────────────────────

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  // ── LCD ──
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Plasma Control");
  lcd.setCursor(0, 1);
  lcd.print("Initialising...");

  // ── ADS1115 (pressure gauge) ──
  ADS.begin();

  // ── MCP4728 DAC ──
  if (!mcp.begin())
  {
    Serial.println("ERROR: MCP4728 DAC not found");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERR: DAC missing");
    while (1) delay(10);
  }

  // Safe state — both outputs at zero
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0);

  // ── ADS7830 ADC ──
  if (!ad7830.begin())
  {
    Serial.println("ERROR: ADS7830 ADC not found");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERR: ADC missing");
    while (1) delay(10);
  }

  lcd.clear();
}


// ── Main loop ───────────────────────────────────────────────────────────────

void loop()
{
  // ── 1. Process any incoming Pi commands first ──────────────────────────
  process_serial_commands();

  // ── 2. Read pressure (ADS1115) ─────────────────────────────────────────
  ADS.setGain(0);
  int16_t val_0          = ADS.readADC(0);
  float   voltage_factor = ADS.toVoltage(1);
  float   v_ccpg         = val_0 * voltage_factor * 2.0f;
  float   p_ccpg         = calc_p_ccpg(v_ccpg);

  // ── 3. Read back voltage and current (ADS7830) ─────────────────────────
  uint8_t voltageADC = ad7830.readADCsingle(0);   // ch0: voltage read-back
  uint8_t currentADC = ad7830.readADCsingle(1);   // ch1: current read-back

  // ── 4. Compute set-values for reporting ────────────────────────────────
  float voltage_set = calc_v((float) g_dac_voltage);   // Volts
  float current_set = calc_i((float) g_dac_current);   // Amps

  // ── 5. Serial output to Pi ─────────────────────────────────────────────
  //   Format: voltage_set, voltageADC, current_set, currentADC, pressure
  Serial.print(voltage_set,  4);  Serial.print(",");
  Serial.print(voltageADC);       Serial.print(",");
  Serial.print(current_set,  6);  Serial.print(",");
  Serial.print(currentADC);       Serial.print(",");
  Serial.println(p_ccpg,     6);

  // ── 6. LCD update (throttled to once per second) ───────────────────────
  unsigned long now = millis();
  if (now - g_last_lcd_ms >= LCD_INTERVAL_MS)
  {
    g_last_lcd_ms = now;

    lcd.clear();

    // Row 0: pressure
    lcd.setCursor(0, 0);
    dtostre(p_ccpg, Ps, 2, DTOSTR_UPPERCASE);
    for (int i = 0; i < 8; i++) lcd.print(Ps[i]);
    lcd.print(" Torr");

    // Row 1: set voltage (kV) and current (mA)
    lcd.setCursor(0, 1);
    lcd.print("V:");
    lcd.print(voltage_set / 1000.0f, 2);   // kV
    lcd.print("kV ");
    lcd.print("I:");
    lcd.print(current_set * 1000.0f, 1);   // mA
    lcd.print("mA");
  }
}
