#include <Wire.h>
#include <MS5611.h>
#include <HMC5883L.h>
#include <MPU6050.h>

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_RST 11
#define RFM95_CS  10
#define RFM95_INT 1

#define RF95_FREQ 434.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
MS5611 ms5611;
HMC5883L compass;
MPU6050 mpu;

double referencePressure, realTemperature;
double wave_power;
float relativeAltitude;
float wave_period;
long realPressure;
long pressure;
double min_height, max_height, wave_height, mid_point, smudge_factor;
byte escaped, started;
unsigned long period_start, period_end;
float avg_period = -1;
double altitude;
int len = 0;

void setup() 
{

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while(!ms5611.begin()) {
    delay(500);
  }

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    delay(500);
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    delay(500);
  }

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  compass.setOffset(0, 0); 
  referencePressure = 101320;
  checkSettings();

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}

void checkSettings()
{
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());
}



void loop()
{
  // Read raw values
  uint32_t rawTemp = ms5611.readRawTemperature();
  uint32_t rawPressure = ms5611.readRawPressure();

  // Read true temperature & Pressure
  realTemperature = ms5611.readTemperature();
  realPressure = ms5611.readPressure();
  float realPressureInHPa = (float)(realPressure / 100.0);

  // Calculate altitude
  float absoluteAltitude = ms5611.getAltitude(realPressure);
  relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);

  Vector norm = compass.readNormalize();

  float heading = atan2(norm.YAxis, norm.XAxis);

  float declinationAngle = (-4.0 + (44.0/60.0)) / (180 / PI);
  heading += declinationAngle;

  if(heading < 0) heading += 2 * PI;
  if(heading > 2 * PI) heading -= 2 * PI;

  get_wave_power(realPressure, referencePressure);

  char radiopacket[50] = "";
  char buf[20] = "";
  itoa((int)realPressureInHPa, radiopacket, 10); //0000

  
  strcat(radiopacket, "/");
  Serial.print("relative Altitude = ");
  Serial.println(relativeAltitude);
  itoa((int)relativeAltitude, buf, 10);
  strcat(radiopacket, buf);

  buf[0] = '\n';

  strcat(radiopacket, "/");
  itoa((int)wave_period, buf, 10);
  strcat(radiopacket, buf);

  buf[0] = '\n';

  strcat(radiopacket, "/");
  itoa((int)wave_power, buf, 10);
  strcat(radiopacket, buf);

  buf[0] = '\n';

  strcat(radiopacket, "/");
  itoa((int)heading, buf, 10); // 0000/0000/0000/0000/0000 
  strcat(radiopacket, buf);

  buf[0] = '\n';

  rf95.send((uint8_t *)radiopacket, sizeof(radiopacket));
  rf95.waitPacketSent();

  Serial.print("content : ");
  Serial.println(radiopacket);
  
  
  
}

void get_wave_power(double real_pressure, double reference_pressure){

  unsigned long start_time = millis();
  pressure = ms5611.readPressure();
  altitude = ms5611.getAltitude(real_pressure, reference_pressure);
  max_height = altitude;
  min_height = altitude;


  //  for 15 seconds try and get wave height
  while(millis() - start_time < 5000){
    pressure = ms5611.readPressure();
    altitude = ms5611.getAltitude(pressure, reference_pressure);
    if (altitude < min_height) min_height = altitude;
    if (altitude > max_height) max_height = altitude;
  }
  mid_point = (max_height + min_height)/2.0;
  wave_height = (max_height - mid_point) / 2.0;
  Serial.print("min_height = ");
  Serial.println(min_height);
  Serial.print("max_height = ");
  Serial.println(max_height);
  Serial.print("WaveHeight = ");
  Serial.println(wave_height);
  smudge_factor = wave_height * 0.15;
  
  start_time = millis();
  //  for 15 seconds try and get wave period
  while(millis() - start_time < 5000){
    pressure = ms5611.readPressure();
    altitude = ms5611.getAltitude(pressure, reference_pressure);
    //    if within a range of 30% of wave height from the mid point
    //    start the timer else stop it
    if (altitude < mid_point + smudge_factor && altitude > mid_point - smudge_factor){
      if ( !started){
        period_start = millis();
        started = true;
      }
      else{
        if ( escaped ){
          // if it has started and escaped and is now returning
          period_end = millis();
          started = false;
          escaped = false;
          // if the variable has already been assigned
          // use its previous value and new value to work out avg
          if(avg_period != -1){
            avg_period = (avg_period + (period_end-period_start)*2)  / 2.0;
          }
          // assign it
          else{
            avg_period =  (period_end-period_start)*2; 
            Serial.print("period end = ");
            Serial.println(period_end);
            Serial.print("period start = ");
            Serial.println(period_start);
          }

        }
      }
    }
    else{
      escaped = true;
    } 
  }
  wave_period = avg_period/1000;
  Serial.print("WavePeriod = ");
  Serial.println(wave_period);
  // work out wave power according to https://en.wikipedia.org/wiki/Wave_power wave power formula
  wave_power = 0.5 * wave_height * wave_height * (avg_period / 1000);
}
