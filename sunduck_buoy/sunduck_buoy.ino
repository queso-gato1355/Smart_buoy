/*********************************************************
 * 2020년 선덕고등학교 과학영재학급 프로젝트
 * 프로젝트명 : 아두이노를 이용한 부표
 * 
 * 사용할 센서 : GY-86, DS18B20(오류가 가장 많은 센서이니 주의!)
 * 사용할 모듈 : NEO-6M, 3색 LED, 블루투스, SD카드 리더 모듈, Wi-Fi모듈
 * 
 * 센서 와이어링
 * 
 * GY-86(지자기계, 가속도계, 기압계 내장)
 * 일반적인 I2C연결법
 *****************************************************************************/

#include <Wire.h>
#include <MS5611.h>
#include <LiquidCrystal_I2C.h>
#include <HMC5883L.h>
#include <MPU6050.h>

#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,4,5,6,7);


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

void setup() 
{
  Serial.begin(9600);
  lcd.begin(16, 2);

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

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Direction:");
  if(degrees(heading) < 30) lcd.print('N');
  else if(degrees(heading) < 60) lcd.print("NE");
  else if(degrees(heading) < 150) lcd.print("E");
  else if(degrees(heading) < 180) lcd.print("SE");
  else if(degrees(heading) < 210) lcd.print("S");
  else if(degrees(heading) < 240) lcd.print("SW");
  else if(degrees(heading) < 300) lcd.print("W");
  else if(degrees(heading) < 330) lcd.print("NW");
  else if(degrees(heading) < 360) lcd.print("N");

  get_wave_power(realPressure, referencePressure);

  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Temp:");
  lcd.print(realTemperature);
  lcd.write(223);
  lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Prss:");
  lcd.print(realPressureInHPa);
  lcd.print("hPa");

  delay(10000);

  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Altd:");
  lcd.print(relativeAltitude);
  lcd.print("m");

  lcd.setCursor(0,1);
  lcd.print("Power:");
  lcd.print(wave_power);
  lcd.print("kW/m");

  delay(10000);
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
