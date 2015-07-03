#include <Wire.h>

//DEFINE CONSTANTS
#define BNO055_SAMPLERATE_DELAY_MS (100)

void setup()
{
  //Start serial debugger
  Serial.begin(9600);
  Serial.println("Serial debugger functional.");

  //Start I2C  
  Wire.begin();
  Serial.println("I2C functional.");
  
  delay(500);
  
  Serial.println("Reading device config. on address 0x29:");
  //Read configuration registers of BNO055 at address 0x29
  
  Wire.beginTransmission(0x29);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(0x29, (uint8_t) 1);
  Serial.print("BNO055_CHIP_ID_ADDR (0x00): ");
  Serial.println(Wire.read(), HEX);
  
  Serial.print("BNO055_CHIP_ID_ADDR (0x00): ");
  Serial.println(read8(0x00), BIN);
  Serial.print("BNO055_CHIP_ID_ADDR (0x00): ");
  Serial.println(read8(0x00), DEC);
  Serial.print("BNO055_CHIP_ID_ADDR (0x00): ");
  Serial.println(read8(0x00), HEX);
  Serial.print("BNO055_ACCEL_REV_ID_ADDR (0x01): ");
  Serial.println(read8(0x01), HEX);
  Serial.print("BNO055_MAG_REV_ID_ADDR (0x02): ");
  Serial.println(read8(0x02), HEX);
  Serial.print("BNO055_GYRO_REV_ID_ADDR (0x03): ");
  Serial.println(read8(0x03), HEX);
  Serial.print("BNO055_SW_REV_ID_LSB_ADDR (0x04): ");
  Serial.println(read8(0x04), HEX);
  Serial.print("BNO055_SW_REV_ID_MSB_ADDR (0x05): ");
  Serial.println(read8(0x05), HEX);
  Serial.print("BNO055_BL_REV_ID_ADDR (0x06): ");
  Serial.println(read8(0x06), HEX);
  Serial.print("BNO055_PAGE_ID_ADDR (0x07): ");
  Serial.println(read8(0x07), HEX);

  Serial.print("BNO055_PAGE_ID_ADDR (0x3D): ");
  Serial.println(read8(0x3D), BIN);
  
  Serial.print("BNO055_PAGE_ID_ADDR (0x3E): ");
  Serial.println(read8(0x3E), HEX);
  
  //Set Config mode to CONFIG MODE
  write8(0x3D,0x00);
    
  Serial.print("BNO055_PAGE_ID_ADDR (0x3D): ");
  Serial.println(read8(0x3D), BIN);
  
  //Set Config mode to NDOF MODE
  write8(0x3D,0x0C);
    
  Serial.print("BNO055_PAGE_ID_ADDR (0x3D): ");
  Serial.println(read8(0x3D), BIN);

} 

void loop()
{
  // put your main code here, to run repeatedly:
    /* Read vector data (6 bytes) */
  uint8_t buffer[6];
  
  readLen(0x08, buffer, 6);
  int x, y, z = 0;      
    
  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
  
 
  
  Serial.print("X: "); Serial.print(x);
  Serial.print(", Y: "); Serial.print(y);
  Serial.print(", Z: "); Serial.print(z);
  Serial.println(9);
  
   delay(50);
}

byte read8(byte reg)
{
  byte value = 0;
  
  Wire.beginTransmission(0x29);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x29, (byte)1);
  value = Wire.read();
 
  return value;
}


void write8(byte reg, byte value)
{
  Wire.beginTransmission(0x29);

  Wire.write(reg);
  Wire.write(value);

  Wire.endTransmission();
  
}

void readLen(byte reg, byte * buffer, uint8_t len)
{
  Wire.beginTransmission(0x29);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x29, (byte)len);

  /* Wait until data is available */
  while (Wire.available() < len) {};
    
  for (uint8_t i = 0; i < len; i++)
  {
      buffer[i] = Wire.read();
  }
  
}
