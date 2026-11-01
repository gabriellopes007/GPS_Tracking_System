/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Digital Compass Arduino Library.
Created by Korneliusz Jarzebski - www.jarzebski.pl
Modifications made by Gabriel Lopes to fit the project.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "HMC5883L.h"

bool HMC5883L::begin()
{	
	Serial.begin(115200);
		Wire.begin();

    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48)
    || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34)
    || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33))
    {
	Serial.println("ERROR");
	return false;
    }

    setRange(HMC5883L_RANGE_1_3GA);
	setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);

    mgPerDigit = 0.92f;
	
    return true;
}

Vector HMC5883L::readRaw(void)
{
    v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M);// - xOffset;
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M);// - yOffset;
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M);

    return v;
}

Vector HMC5883L::readNormalize(void)
{
    v.XAxis = ((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(HMC5883L_REG_OUT_Z_M) * mgPerDigit;

    return v;
}

void HMC5883L::setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

void HMC5883L::setRange(hmc5883l_range_t range)
{
    switch(range)
    {
	case HMC5883L_RANGE_0_88GA:
	    mgPerDigit = 0.073f;
	    break;

	case HMC5883L_RANGE_1_3GA:
	    mgPerDigit = 0.92f;
	    break;

	case HMC5883L_RANGE_1_9GA:
	    mgPerDigit = 1.22f;
	    break;

	case HMC5883L_RANGE_2_5GA:
	    mgPerDigit = 1.52f;
	    break;

	case HMC5883L_RANGE_4GA:
	    mgPerDigit = 2.27f;
	    break;

	case HMC5883L_RANGE_4_7GA:
	    mgPerDigit = 2.56f;
	    break;

	case HMC5883L_RANGE_5_6GA:
	    mgPerDigit = 3.03f;
	    break;

	case HMC5883L_RANGE_8_1GA:
	    mgPerDigit = 4.35f;
	    break;

	default:
	    break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t HMC5883L::getRange(void)
{
    return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

	value = readRegister8(HMC5883L_REG_MODE);
	value &= 0b11111100;
    value |= mode;
	

    writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t HMC5883L::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t HMC5883L::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void HMC5883L::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L::getSamples(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

// Write byte to register
void HMC5883L::writeRegister8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
}

// Read byte to register
uint8_t HMC5883L::fastRegister8(uint8_t reg)
{
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Read byte from register
uint8_t HMC5883L::readRegister8(uint8_t reg)
{
    uint8_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    //#if ARDUINO >= 100
    Wire.write(reg);
    //#else
        //Wire.send(reg);
    //#endif
    Wire.endTransmission();
	
	

    //Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    while(!Wire.available()){};
	value = Wire.read();
    //#if ARDUINO >= 100
        
    //#else
        //value = Wire.receive();
    //#endif;
    Wire.endTransmission();

    return value;
}

// Read word from register
int16_t HMC5883L::readRegister16(uint8_t reg)
{
    int16_t value;
    Wire.beginTransmission(HMC5883L_ADDRESS);
    //#if ARDUINO >= 100
    Wire.write(reg);
    //#else
        //Wire.send(reg);
    //#endif
    Wire.endTransmission();

    //Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 2, true);
    while(!Wire.available()) {};
    //#if ARDUINO >= 100
    uint8_t vha = Wire.read();
	//Serial.println(vha);
    uint8_t vla = Wire.read();
	//Serial.println(vla);
    //#else
        //uint8_t vha = Wire.receive();
        //uint8_t vla = Wire.receive();
    //#endif;
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

Vector HMC5883L::readValues(void){
    Vector mag;
    int16_t mx_raw, my_raw, mz_raw;

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.write(0x03); //select register 3, X MSB register
    Wire.endTransmission();
 
    //Read data from each axis, 2 registers per axis
    Wire.requestFrom(HMC5883L_ADDRESS, 6);
    if(6<=Wire.available()){
        mx_raw = Wire.read()<<8 | Wire.read(); //X msb
        //x |= Wire.read(); //X lsb
        mz_raw = Wire.read()<<8 | Wire.read(); //Z msb
        //z |= Wire.read(); //Z lsb
        my_raw = Wire.read()<<8 | Wire.read(); //Y msb
        //y |= Wire.read(); //Y lsb
    }

    mag.XAxis = mx_raw;
    mag.YAxis = my_raw;
    mag.ZAxis = mz_raw;

    return mag;
}
