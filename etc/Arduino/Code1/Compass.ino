/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
  
//  Serial.println();
//  Serial.print("- Compass_Headi - ");
//  Serial.print("- magnetom[0]: ");  Serial.print(magnetom[0]);
//  Serial.print("- magnetom[1]: ");  Serial.print(magnetom[1]);
//  Serial.print("- magnetom[2]: ");  Serial.print(magnetom[2]);
//
//  Serial.print("- cos_roll: ");  Serial.print(cos_roll);
//  Serial.print("- sin_roll: ");  Serial.print(sin_roll);
//  Serial.print("- cos_pitch: ");  Serial.print(cos_pitch);
//  Serial.print("- sin_pitch: ");  Serial.print(sin_pitch);
//  
//  Serial.print("- mag_x: ");  Serial.print(mag_x);
//  Serial.print("- mag_y: ");  Serial.print(mag_y);
//  Serial.print("- MAG_Heading: ");  Serial.print(MAG_Heading);
//
//  Serial.println();
}
