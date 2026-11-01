/*
  Calibrate HMC5883L. 
  Based on the work of Korneliusz Jarzebski: https://github.com/jarzebski/Arduino-HMC5883L
  Modified by Gabriel Lopes.
*/

#include <Wire.h>
#include <HMC5883L.h>

#define address 0x1E

HMC5883L compass;

float minX = 0;
float maxX = 0;
float minY = 0;
float maxY = 0;
float minZ = 0;
float maxZ = 0;
float offX = 0;
float offY = 0;
float offZ = 0;
float scaleX = 0;
float scaleY = 0;
float scaleZ = 0;

void setup()
{
  Serial.begin(115200);

  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_15HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  Serial.println("Setup Complete");
}

void loop()
{
  Vector mag = compass.readValues();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;
  if (mag.ZAxis < minZ) minZ = mag.ZAxis;
  if (mag.ZAxis > maxZ) maxZ = mag.ZAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;
  offZ = (maxZ + minZ)/2;
  scaleX = (maxX - minX)/2;
  scaleY = (maxY - minY)/2;
  scaleZ = (maxZ - minZ)/2;

  double scaleTotal = (scaleX + scaleY + scaleZ)/3;

  scaleX = scaleTotal/scaleX;
  scaleY = scaleTotal/scaleY;
  scaleZ = scaleTotal/scaleZ;


  Serial.print(minX);
  Serial.print(":");
  Serial.print(maxX);
  Serial.print(" | ");
  Serial.print(minY);
  Serial.print(":");
  Serial.print(maxY);
  Serial.print(" | ");
  Serial.print(minZ);
  Serial.print(":");
  Serial.print(maxZ);
  Serial.print(" | ");
  Serial.print(offX);
  Serial.print(":");
  Serial.print(offY);
  Serial.print(":");
  Serial.print(offZ);
  Serial.print(" | ");
  Serial.print(scaleX);
  Serial.print(":");
  Serial.print(scaleY);
  Serial.print(":");
  Serial.println(scaleZ);
  Serial.print("\n");
}
