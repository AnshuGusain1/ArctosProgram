#include <Wire.h>

// AS5600 I2C address (fixed, cannot be changed)
#define AS5600_ADDR       0x36

// Register addresses
#define REG_RAW_ANGLE_H   0x0C  // High byte of raw 12-bit angle
#define REG_RAW_ANGLE_L   0x0D  // Low byte
#define REG_ANGLE_H       0x0E  // Filtered/processed angle high byte
#define REG_ANGLE_L       0x0F  // Filtered/processed angle low byte
#define REG_STATUS        0x0B  // Magnet detection status
#define REG_AGC           0x1A  // Automatic gain control value
#define REG_MAGNITUDE_H   0x1B  // CORDIC magnitude high byte
#define REG_MAGNITUDE_L   0x1C  // CORDIC magnitude low byte

// Reads a 16-bit value from two consecutive registers (high byte first)
uint16_t readRegister16(uint8_t regHigh) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(regHigh);
  Wire.endTransmission(false);          // Repeated START, don't release bus
  Wire.requestFrom(AS5600_ADDR, 2);     // Request 2 bytes

  uint16_t high = Wire.read();
  uint16_t low  = Wire.read();
  return (high << 8) | low;
}

// Reads a single byte from a register
uint8_t readRegister8(uint8_t reg) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 1);
  return Wire.read();
}

// Returns the filtered angle in the range [0, 4095]
// Multiply by (360.0 / 4096.0) to get degrees
uint16_t readAngle() {
  return readRegister16(REG_ANGLE_H) & 0x0FFF;  // Mask to 12 bits
}

// Returns the raw (unfiltered) angle [0, 4095]
uint16_t readRawAngle() {
  return readRegister16(REG_RAW_ANGLE_H) & 0x0FFF;
}

// Checks magnet presence via the STATUS register
// Bit 5 (MD): magnet detected
// Bit 4 (ML): magnet too weak
// Bit 3 (MH): magnet too strong
void printMagnetStatus() {
  uint8_t status = readRegister8(REG_STATUS);
  bool detected = (status >> 5) & 0x01;
  bool tooWeak  = (status >> 4) & 0x01;
  bool tooStrong = (status >> 3) & 0x01;

  Serial.print("Magnet: ");
  if (detected)   Serial.print("DETECTED ");
  if (tooWeak)    Serial.print("TOO WEAK ");
  if (tooStrong)  Serial.print("TOO STRONG ");
  if (!detected && !tooWeak && !tooStrong) Serial.print("NOT FOUND");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz fast mode (AS5600 supports up to 1 MHz)

  Serial.println("AS5600 Magnetic Angle Sensor");
  Serial.println("----------------------------");
  printMagnetStatus();

  uint8_t agc = readRegister8(REG_AGC);
  Serial.print("AGC value: ");
  Serial.println(agc);  // ~128 is ideal; lower = strong magnet, higher = weak
}

void loop() {
  uint16_t raw   = readRawAngle();
  uint16_t angle = readAngle();

  float degrees    = angle * (360.0 / 4096.0);
  float degreesRaw = raw   * (360.0 / 4096.0);

  Serial.print("Raw: ");
  Serial.print(degreesRaw, 2);
  Serial.print(" deg  |  Filtered: ");
  Serial.print(degrees, 2);
  Serial.println(" deg");

  delay(50);
}