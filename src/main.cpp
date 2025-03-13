#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <SparkFun_BMP581_Arduino_Library.h>

const int chipSelect = BUILTIN_SDCARD;  // Change this if using a different pin

Adafruit_MPU6050 mpu;  // Use Wire1 for the alternate I2C bus
BMP581 pressureSensor;

void setup() {
    Serial.begin(9600);
    while (!Serial) {}  // Wait for Serial Monitor to open (for some boards)

    Serial.print("Initializing SD card... ");
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present.");
        return;
    }
    Serial.println("Card initialized.");


    // Initialize MPU6050
    if (!mpu.begin(0x68, &Wire2)) {  // Pass the I2C address and the TwoWire instance
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      while (1);
  }

    // Initialize BMP581
    // if (pressureSensor.beginI2C((uint8_t)71U, Wire2) != BMP5_OK) {
    //     Serial.println("Could not find a valid BMP581 sensor, check wiring!");
    //     while (1);
    // }

    // Open file for writing
    File file = SD.open("test.txt", FILE_WRITE);
    if (file) {
        file.println("Hello, Teensy SD!");
        file.close();
        Serial.println("Write successful.");
    } else {
        Serial.println("Write failed.");
    }

    // Open file for reading
    file = SD.open("test.txt");
    if (file) {
        Serial.println("Reading file:");
        while (file.available()) {
            Serial.write(file.read());
        }
        file.close();
    } else {
        Serial.println("Read failed.");
    }
}

void loop() {
    // Read BMP581 data
    bmp5_sensor_data data = {0,0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if(err == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature (C): ");
        Serial.print(data.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure (Pa): ");
        Serial.println(data.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor! Error code: ");
        Serial.println(err);
    }

    // Read MPU6050 data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.println(a.acceleration.z);

    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.println(g.gyro.z);

    delay(1000);  // Delay for a second
}