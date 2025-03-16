#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <SparkFun_BMP581_Arduino_Library.h>

const int chipSelect = BUILTIN_SDCARD;  // Change this if using a different pin

Adafruit_MPU6050 mpu;  // Use Wire1 for the alternate I2C bus
BMP581 pressureSensor;


File file;
float initialPressure = 0.0;
void setup() {
    Serial.begin(9600);

    Wire1.begin();
    Wire2.begin();

    Serial.print("Initializing SD card... ");
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present.");
        return;
    }
    Serial.println("Card initialized.");


    // Initialize MPU6050
//     if (!mpu.begin(0x68, &Wire1)) {  // Pass the I2C address and the TwoWire instance
//       Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
//       while (1);
//   }

    // Initialize BMP581
    int8_t err = pressureSensor.beginI2C(0x47, Wire2);
    if (err != BMP5_OK) {
        Serial.println("Could not find a valid BMP581 sensor, check wiring!"+String(err));
        while (1);
    }
    bmp5_sensor_data data = {0,0};
   
    bmp5_iir_config config =
    {
        .set_iir_t = BMP5_IIR_FILTER_COEFF_3, // Set filter coefficient
        .set_iir_p = BMP5_IIR_FILTER_COEFF_3, // Set filter coefficient
        .shdw_set_iir_t = BMP5_ENABLE, // Store filtered data in data registers
        .shdw_set_iir_p = BMP5_ENABLE, // Store filtered data in data registers
        .iir_flush_forced_en = BMP5_DISABLE // Flush filter in forced mode
    };
    err = pressureSensor.setFilterConfig(&config);
    
    if(err)
    {
        // Setting coefficient failed, most likely an invalid coefficient (code -12)
        Serial.print("Error setting filter coefficient! Error code: ");
        Serial.println(err);
    }

    err = pressureSensor.getSensorData(&data);
    delay(100);
    err = pressureSensor.getSensorData(&data);
    if (err == BMP5_OK) {
        initialPressure = data.pressure/100;
        Serial.print("Initial pressure: ");
        Serial.println(initialPressure);
    } else {
        Serial.print("Error getting initial pressure! Error code: ");
        Serial.println(err);
    }
    // Open file for writing
    char filename[] = "LOGGER00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        file = SD.open(filename, FILE_WRITE); 
        break;  
      }
    }
    
    if (!file) {
      Serial.println("couldnt create file");
    }

    Serial.println("opened: "+String(filename));

    if (file) {
        file.println("Hello, Teensy SD!");
        file.flush();
        Serial.println("Write successful.");
    } else {
        Serial.println("Write failed.");
    }

}
float pressure_to_altitude(float pressure_pa)
{
  double pressure_mb = pressure_pa / 100;
  double first_term = pow( (pressure_mb/initialPressure) , (1/5.255) );
  float altitude = 44330 * (1 - first_term);
  return altitude;
}

long interval = 500;
static unsigned long previousMillis = 0;
bool started_logging = false;
void loop() {
    unsigned long currentMillis = millis();
    bmp5_sensor_data data = {0,0};
    int8_t err = pressureSensor.getSensorData(&data);
    float altitude = pressure_to_altitude(data.pressure);

    if(err != BMP5_OK)
    {
        Serial.println(err);
        file.println(err);
        file.flush();
    }

    if (!started_logging && altitude>1) started_logging = true;

    if (started_logging) {
        interval = 100;
        if (file) {
            Serial.println(String(currentMillis)+","+String(altitude)+","+String(data.pressure/100));
            file.println(String(currentMillis)+","+String(altitude)+","+String(data.pressure/100));
            file.flush();
        } else {
            Serial.println("Write failed.");
        }
    }
    else {
        Serial.println("p:"+String(altitude)+","+String(data.pressure/100));
    }

    // Blink the onboard LED every 500 ms
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    delay(20);  // Delay for a short period
}