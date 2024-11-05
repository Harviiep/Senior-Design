#include <Arduino.h>
#include <SensirionI2cSht3x.h>
#include <Wire.h>
#include <sps30.h>

SensirionI2cSht3x sensor;

static char errorMessage[64];
static int16_t error;

//ADD GLOBAL VARIABLE TO HOLD SENSOR READINGS TO BE SENT VIA BLUTOOTH -
//Serial send as its ready or use working memory stack?
//Use timers to trigger sensor reading
//Check current consumption


void setup() 
{
  Serial.begin(9600);

  while (!Serial) 
  {
    delay(100);
  }

    Wire.begin();
    
}
void SPS30_Function()
  {
    struct sps30_measurement m;
    char serial[SPS30_MAX_SERIAL_LEN];
    uint16_t data_ready;
    int16_t ret;
    uint8_t auto_clean_days = 4;          //SPS30
    uint32_t auto_clean;                  //SPS30

    sensirion_i2c_init();                   

     while (sps30_probe() != 0) 
    {
      Serial.print("SPS sensor probing failed\n");
      delay(100);
    }
    
    //SPS30
    #ifndef PLOTTER_FORMAT
      Serial.print("SPS sensor probing successful\n");
    #endif /* PLOTTER_FORMAT */

    //SPS30
    ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
    if (ret) 
    {
      Serial.print("error setting the auto-clean interval: ");
      Serial.println(ret);
    }

    //SPS30
    ret = sps30_start_measurement();
    if (ret < 0)
    {
      Serial.print("error starting measurement\n");
    }

    //SPS30
    #ifndef PLOTTER_FORMAT
      Serial.print("measurements started\n");
    #endif /* PLOTTER_FORMAT */

    //SPS30
    #ifdef SPS30_LIMITED_I2C_BUFFER_SIZE
      Serial.print("Your Arduino hardware has a limitation that only\n");
      Serial.print("  allows reading the mass concentrations. For more\n");
      Serial.print("  information, please check\n");
      Serial.print("  https://github.com/Sensirion/arduino-sps#esp8266-partial-legacy-support\n");
      Serial.print("\n");
      delay(1000);
    #endif

    do 
    {
      ret = sps30_read_data_ready(&data_ready);
      if (ret < 0) 
      {
        Serial.print("error reading data-ready flag: ");
        Serial.println(ret);
      }   
      else if (!data_ready)
        Serial.print("data not ready, no new measurement available\n");
      else
        break;
      delay(200); /* retry in 100ms*/
    } 
    while (1);

    ret = sps30_read_measurement(&m);
    if (ret < 0) 
    {
      Serial.print("error reading measurement\n");
    } 
    else 
      {

        #ifndef PLOTTER_FORMAT
            Serial.print("PM  1.0: ");
            Serial.println(m.mc_1p0);
            Serial.print("PM  2.5: ");
            Serial.println(m.mc_2p5);
            Serial.print("PM  4.0: ");
            Serial.println(m.mc_4p0);
            Serial.print("PM 10.0: ");
            Serial.println(m.mc_10p0);

        #ifndef SPS30_LIMITED_I2C_BUFFER_SIZE
            Serial.print("NC  0.5: ");
            Serial.println(m.nc_0p5);
            Serial.print("NC  1.0: ");
            Serial.println(m.nc_1p0);
            Serial.print("NC  2.5: ");
            Serial.println(m.nc_2p5);
            Serial.print("NC  4.0: ");
            Serial.println(m.nc_4p0);
            Serial.print("NC 10.0: ");
            Serial.println(m.nc_10p0);

          Serial.print("Typical particle size: ");
          Serial.println(m.typical_particle_size);
        #endif

      Serial.println();

    #else
      // since all values include particles smaller than X, if we want to create buckets we 
      // need to subtract the smaller particle count. 
      // This will create buckets (all values in micro meters):
      // - particles        <= 0,5
      // - particles > 0.5, <= 1
      // - particles > 1,   <= 2.5
      // - particles > 2.5, <= 4
      // - particles > 4,   <= 10

      Serial.print(m.nc_0p5);
      Serial.print(" ");
      Serial.print(m.nc_1p0  - m.nc_0p5);
      Serial.print(" ");
      Serial.print(m.nc_2p5  - m.nc_1p0);
      Serial.print(" ");
      Serial.print(m.nc_4p0  - m.nc_2p5);
      Serial.print(" ");
      Serial.print(m.nc_10p0 - m.nc_4p0);
      Serial.println();


    #endif /* PLOTTER_FORMAT */
    }

  
    return;
}

void SHT30_Function()
  {
    uint16_t aStatusRegister = 0u; 
    sensor.begin(Wire, SHT30_I2C_ADDR_44);    //SHT30
    
    sensor.stopMeasurement();
    delay(1);
    sensor.softReset();
    delay(100);

    error = sensor.readStatusRegister(aStatusRegister);

    if (error != NO_ERROR) 
    {
      Serial.print("Error trying to execute readStatusRegister(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }

    //SHT30 Print Status Register after Check
    Serial.print("Status Register: ");
    Serial.print(aStatusRegister);
    Serial.println();
    
    float aTemperature = 0.0;
    float aHumidity = 0.0;

    error = sensor.measureSingleShot(REPEATABILITY_LOW, false, aTemperature,
                                     aHumidity);
    if (error != NO_ERROR) 
    {
      Serial.print("Error trying to execute measureSingleShot(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
    }
    
    
    //SHT30 - Temperature Readout
    Serial.print("Temperature: ");
    Serial.print(aTemperature);
    Serial.print("\t");

    //SHT30 - Humidity Readout
    Serial.print("Humidity: ");
    Serial.print(aHumidity);
    Serial.println();
    
    return;

  }


//Main Loop
void loop() 
{
  int command;
  command = Serial.read();
  if (command == 48)
  {
    SHT30_Function();
    SPS30_Function();
  }
}


 