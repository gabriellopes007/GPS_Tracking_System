/* 
 * Main program - GPS Tracking System.
 * Created by Gabriel Lopes.
 */

#include <TinyGPS++.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Haversine.h>
#include <Wire.h>
#include <GPSKalmanFilter.h>
#include <HMC5883L.h>
#include <MPU6050.h>

#define address 0x1E

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 40 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

//-------------------------------- LORA COMMUNICATION CONFIGURATION ---------------------//

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x3B, 0x4D, 0x5B, 0x36, 0x35, 0x16, 0xA2, 0x49, 0xF2, 0x6B, 0x30, 0xF2, 0x37, 0x40, 0xB9, 0x4E };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xF1, 0xEA, 0x43, 0x0E, 0x4F, 0xB2, 0xAA, 0xA3, 0xAC, 0x2A, 0x9D, 0xB6, 0xEC, 0xA9, 0xBC, 0xE1 };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260CBF50; // <-- Change this address for every node!



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

uint8_t payload[11] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins ={
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23,
  .dio = {26, 33, 32},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        Serial.println("Enviando Payload...");
        int x = 0;
        for (x=0; x <=sizeof(payload); x++){
          Serial.print(payload[x], HEX);
        }
        Serial.println(" ");
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}



//--------------------------Kalman Filter initial variables --------------------------//

TinyGPSPlus gps;
bool signalLost = true;


bool first = true;
double lat_old;
double lng_old;
double lat_cur;
double lng_cur;
uint32_t LatitudeBinary = 0;
uint32_t LongitudeBinary = 0;
uint32_t AltitudeBinary = 0;
double velocity;


double N = 0;
double E = 0;

double dt = 0.20;
//x -> state vector
double x_N[2][1] = {{0},{0}};
//P -> Covariance Matrix
double P_N[2][2] = {{5, 0},{0, 1}};
//x -> state vector
double x_E[2][1] = {{0},{0}};
//P -> Covariance Matrix
double P_E[2][2] = {{5, 0},{0, 1}};

GPSKF gpsKalmanFilter(dt, N, E, x_N, P_N, x_E, P_E);


//-------------------- IMU initial variables --------------------------//
HMC5883L compass;
MPU6050lib mpu;

float aRes, gRes; // scale resolutions per LSB for the sensors
int16_t accelCount[3];                                   // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];                                    // Stores the 16-bit signed gyro sensor output
float gyrox, gyroy, gyroz;                               // Stores the real gyro value in degrees per seconds
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
int16_t tempCount;                                       // Stores the real internal chip temperature in degrees Celsius
float temperature;
float SelfTest[6];
float delt_t = 0;                                     // used to control display output rate
float count = 0;                                      // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f;                                     // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;                // used to calculate integration interval
uint32_t Now = 0;                                        // used to calculate integration interval
float ax, ay, az, gx, gy, gz, m_x, m_y, m_z; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float axg, axl;

float previousTimeLoRa;
int med = 0;
double latitude_filter = 0; 
double longitude_filter = 0;
uint8_t nSatelite;

//---------------------------------------------------------------------//

/*
 * Configuração de variáveis do GPS
 */
#define SERIAL1_RX 34 // GPS_TX -> 34
#define SERIAL1_TX 12 // 12 -> GPS_RX

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
  delay(3000);

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

  Serial.println("Setup Complete for Mag");

  Serial.println("Start MPU6050 Configuration");
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t c = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
  Serial.print("I AM ");
  Serial.print(c, HEX);
  Serial.print(" I Should Be ");
  Serial.println(0x68, HEX);

  if (c == 0x68) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU6050 is online...");

    mpu.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    
    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
      Serial.println("Pass Selftest!");

      mpu.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
      Serial.println("MPU6050 bias");
      Serial.println(" x\t  y\t  z  ");
      Serial.print((int)(1000 * accelBias[0])); Serial.print('\t');
      Serial.print((int)(1000 * accelBias[1])); Serial.print('\t');
      Serial.print((int)(1000 * accelBias[2]));
      Serial.println(" mg");

      Serial.print(gyroBias[0], 1); Serial.print('\t');
      Serial.print(gyroBias[1], 1); Serial.print('\t');
      Serial.print(gyroBias[2], 1);
      Serial.println(" o/s");


      mpu.initMPU6050(); Serial.println("MPU6050 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      while (1) ; // Loop forever if communication doesn't happen
    }
  }

  Serial.println("Finish MPU6050 Configuration");
  delay(1000);

    
  Serial.print("Start LoRa Configuration");

  #ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif
  
  //SPI pin mapping ESP32 (LILYGO Board V1.1)
  //SPI.begin(5, 19, 27, 18);
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
  #else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  Serial.println("AU915");
  LMIC_selectSubBand(1);
  

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7,14);

  delay(2000);
  Serial.println("Setup done");

  // Start job
  do_send(&sendjob);
  previousTimeLoRa = millis();
  count = millis();
}

void loop() {
// Uncomment this part to execute the LoRa communication
/*    os_runloop_once();
    if(millis() - previousTimeLoRa > 5000){
      Serial.println("Enviei aqui");
      do_send(&sendjob);
      Serial.println("Sending...");
      previousTimeLoRa = millis();
    }
*/
  
    if(!first){

       // If data ready bit set, all data registers have new data
       if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { // check if data ready interrupt
        mpu.readAccelData(accelCount);  // Read the x/y/z adc values
        aRes = mpu.getAres();
    
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0] * aRes; // get actual g value, this depends on scale being set
        ay = (float)accelCount[1] * aRes;
        az = (float)accelCount[2] * aRes;
    
        mpu.readGyroData(gyroCount);  // Read the x/y/z adc values
        gRes = mpu.getGres();
    
        // Calculate the gyro value into actual degrees per second
        gyrox = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
        gyroy = (float)gyroCount[1] * gRes;
        gyroz = (float)gyroCount[2] * gRes;
      }

      Vector mag = compass.readValues(); //175.00:29.50:21.00 | 1.29:0.85:0.95


    
      m_x = ((float)mag.XAxis) * 0.92f;
      m_x = (m_x - 175) * 1.29;
      m_y = ((float)mag.YAxis) * 0.92f;
      m_y = (m_y - 29.5) * 0.85;
      m_z = ((float)mag.ZAxis) * 0.92f;
      m_z = (m_z - 21) * 0.95;
    
      double heading = atan2(m_y, m_x);
      //float declinationAngle = -(23.0 + (50.0 / 60.0)) / (180 / M_PI);
      double declinationAngle = 0;//336.15534 / (180 / M_PI);
      heading += declinationAngle;
    
      // Correct for heading < 0deg and heading > 360deg
      if (heading < 0)
      {
        heading += 2 * PI;
      }
     
      if (heading > 2 * PI)
      {
        heading -= 2 * PI;
      }
    
      double headingDegrees = heading * 180/M_PI;

      Now = micros();
      deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;
      //Serial.println(deltat, 6);
      // Pass gyro rate as rad/s  
      MadgwickQuaternionUpdate(ax, ay, az, gyrox * PI / 180.0f, gyroy * PI / 180.0f, gyroz * PI / 180.0f, m_x, m_y, m_z);

      

        
      float NowKF = millis();
      delt_t = NowKF - count;
      //count = NowKF;
      //Serial.println(delt_t, 6);
      if (delt_t > 195) { // update LCD once per half-second independent of read rate
        count = NowKF;
        yaw  = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
        yaw += 0.41597;

        if (yaw < 0)
        {
          yaw += 2 * PI;
        }
     
        if (yaw > 2 * PI)
        {
          yaw -= 2 * PI;
        }
        pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
        roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      
        axg = 2*(q[1]*q[3] - q[0]*q[2]);
        axl = (ax - axg);

        nSatelite = gps.satellites.value();
        lat_cur = gps.location.lat();
        lng_cur = gps.location.lng();
        velocity = gps.speed.mps();


        gpsKalmanFilter.filterCalculation(lat_old, lng_old, lat_cur, lng_cur, velocity, axl, nSatelite, heading, latitude_filter, longitude_filter);
        lat_old = lat_cur;
        lng_old = lng_cur;
      }

      gps_payload(latitude_filter, longitude_filter, gps.altitude.meters());

      smartDelay(5);
    }else{
      lat_old = gps.location.lat();
      lng_old = gps.location.lng();
      first = false;
    }
}


void gps_payload(double lat, double lon, float alt){
  
  long l_lat = lat*1000000;
  long l_lon = lon*1000000;
  long l_alt = alt*100;
  
  if(lat != 0){
    LatitudeBinary = (l_lat);
    LongitudeBinary = (l_lon);
    AltitudeBinary = (l_alt);
  }

  payload[0] = (LatitudeBinary & 0xFF000000) >> 24;
  payload[1] = (LatitudeBinary & 0x00FF0000) >> 16;
  payload[2] = (LatitudeBinary & 0x0000FF00) >> 8;
  payload[3] = (LatitudeBinary & 0x000000FF);

  payload[4] = (LongitudeBinary & 0xFF000000) >> 24;
  payload[5] = (LongitudeBinary & 0x00FF0000) >> 16;
  payload[6] = (LongitudeBinary & 0x0000FF00) >> 8;
  payload[7] = (LongitudeBinary & 0x000000FF);

  payload[8] = (AltitudeBinary & 0xFF00) >> 8;
  payload[9] = (AltitudeBinary);

  for(int x = 0; x < sizeof(payload); x++){
    //Serial.print(payload[x], HEX);
  }//Serial.println("");
  
}


static void smartDelay(unsigned long ms)                
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available()){
      gps.encode(Serial1.read());
      }
      //getAccelerometer();
      
  } while (millis() - start < ms);
}


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}
