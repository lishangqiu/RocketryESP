#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Servo.h>

Servo myServo; // Create a Servo object

const int chipSelect = BUILTIN_SDCARD;  // Change this if using a different pin

Adafruit_MPU6050 mpu;  // Use Wire1 for the alternate I2C bus
BMP581 pressureSensor;


File file;
float initialPressure = 0.0;
void setup() {
    Serial.begin(9600);

    Wire.begin();
    Wire2.begin();

    Serial.print("Initializing SD card... ");
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present.");
        return;
    }
    Serial.println("Card initialized.");


    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected! Check connections.");
        while (1);
    }

    Serial.println("MPU6050 initialized.");

    // Configure sensor settings
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

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

    myServo.attach(22);  // Attach the servo to pin 9
    // myServo.write(90);  // Set servo to 90 degrees

    while(1) {
        myServo.write(90);  // Set servo to 90 degrees
        delay(1000);
        myServo.write(50);
        delay(1000);
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

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    String dataString = "," + String(accel.acceleration.x, 3) + "," +
                        String(accel.acceleration.y, 3) + "," +
                        String(accel.acceleration.z, 3) + "," +
                        String(gyro.gyro.x, 3) + "," +
                        String(gyro.gyro.y, 3) + "," +
                        String(gyro.gyro.z, 3) + "," +
                        String(temp.temperature, 2);

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
            Serial.println(String(currentMillis)+","+String(altitude)+","+String(data.pressure/100)+dataString);
            file.println(String(currentMillis)+","+String(altitude)+","+String(data.pressure/100)+dataString);
            file.flush();
        } else {
            Serial.println("Write failed.");
        }
    }
    else {
        Serial.println(String(altitude)+","+String(data.pressure/100)+dataString);
    }

    // Blink the onboard LED every 500 ms
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    delay(10);  // Delay for a short period
}