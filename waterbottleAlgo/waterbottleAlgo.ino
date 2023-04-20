#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ThingSpeak.h>
#include <TimeLib.h>
#include <RTClib.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
//#include <MPU6050.h>



#define led D0

// Wi-Fi credentials
const char* ssid = "Jackson's Galaxy S22 Ultra";                 //"R15-8FB3";
const char* password = "Redwine21";             //"aukqh47834";

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// ThingSpeak details
unsigned long myChannelNumber = 2036323;
const char* myWriteAPIKey = "BRDOZW0TN4B3TTF5";

// Ultrasonic sensor
const int trigPin = 12;
const int echoPin = 14;
const int tiltSensorPin = A0;  // analog pin connected to the tilt sensor

// Select SDA and SCL pins for I2C communication
const uint8_t scl = D4;
const uint8_t sda = D3;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
//MPU6050 mpu;
// Water consumption parameters
float consumptionGoal = 700.0;
float bottles = 0.0;
float waterLevel = 0.0;
float previousWaterLevel;  // Set initial water level to full
float consumption = 0;
float height = 11;  // The height or volume of the bottle (depends on the units being used)
float distance = 0;
float toreachGoal = 0;
int idletime = 0;
float bottleConsumption = 0;
float actualconsumption = 0;
// Set your NTP Server and Timezone
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 0;
RTC_DS1307 rtc;
struct tm timeinfo;
WiFiClient client;
float volume = 600;
// Calculate the radius of the cylinder using the formula for the volume of a cylinder
float radius = sqrt(volume / (PI * height));
// Define a flag variable to ensure that previous_water_level is only initialized once
bool initialized = false;
void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  MPU6050_Init();
  //mpu.initialize();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Connected to Wi-Fi");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Set pin modes for the ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led, OUTPUT);

  //previousWaterLevel = 12;
  // Initialize the RTC module
  Wire.begin();
  rtc.begin();

  // Synchronize with NTP server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Wait for time to be set
  while (!time(nullptr)) {
    delay(1000);
    Serial.println("Waiting for time synchronization...");
  }

  Serial.println("Time synchronized");
}

void sendtoThings() {
  ThingSpeak.setField(1, previousWaterLevel);
  ThingSpeak.setField(2, waterLevel);
  ThingSpeak.setField(3, consumption);
  ThingSpeak.setField(4, toreachGoal);
  ThingSpeak.setField(5, actualconsumption);
  ThingSpeak.setField(6, bottles);
  ThingSpeak.setField(7, 0);
  ThingSpeak.setField(8, asctime(&timeinfo));
  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
}

void loop() {

  double Ax, Ay, Az, T, Gx, Gy, Gz;
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX / AccelScaleFactor;
  Ay = (double)AccelY / AccelScaleFactor;
  Az = (double)AccelZ / AccelScaleFactor;
  T = (double)Temperature / 340 + 36.53;  //temperature formula
  Gx = (double)GyroX / GyroScaleFactor;
  Gy = (double)GyroY / GyroScaleFactor;
  Gz = (double)GyroZ / GyroScaleFactor;
  float pitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180 / PI;
  float roll = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180 / PI;
  Serial.print("Pitch ");
  Serial.print(pitch);
  Serial.print(" Roll");
  Serial.println(roll);
  // check for bottle orientation
  if (abs(pitch) > 30 || abs(roll) > 30 || abs(Gx) > 10 || abs(Gy) > 10) {  //Pitch -ve means back +ve means front, roll -ve means left and roll +ve means right.
    Serial.println("Bottle not upright or is shaking");

  } else {
    Serial.println("Good to take measurements since the bottle is stable and upright");
    time_t currentTime = time(nullptr);

    localtime_r(&currentTime, &timeinfo);

    Serial.print("Current time is: ");
    Serial.println(asctime(&timeinfo));


    digitalWrite(led, LOW);

    // Measure water level
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    float duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    Serial.print("DISTANCE");
    Serial.println(distance);
    // Calculate the volume of the empty space in the cylinder using the formula for the volume of a frustrum
    //float empty_volume = (PI/3) * distance * (pow(radius,2) + radius * distance + pow(distance,2));
    // calculate the current water level
    float h_water = height -  distance;
    waterLevel = PI * pow(radius,2) * h_water;
    Serial.print("WaterLevel: ");
    Serial.println(waterLevel);
    // Initialize the previous water level to the current water level on the first loop iteration
    if (!initialized) {
    float previousWaterLevel = waterLevel;
      initialized = true;
    }
    if ((waterLevel < previousWaterLevel) && (waterLevel >= 0)) {
      if (timeinfo.tm_hour == 00 && timeinfo.tm_sec >= 0 && timeinfo.tm_sec <= 19) {
        consumption = 0;
        toreachGoal = 0;
        bottles = 0;
      }
      consumption += previousWaterLevel - waterLevel;
      actualconsumption = previousWaterLevel - waterLevel;
      float currentConsumption = consumption - bottleConsumption;

      if (currentConsumption >= volume) {
        bottles = 1;
        bottleConsumption = consumption;
      } else {
        bottles = 0;
      }
      Serial.println(bottles);
      Serial.println("Drink more water ");
      Serial.print("Consumption: ");
      Serial.println(consumption);
      toreachGoal = consumptionGoal - consumption;
      if (toreachGoal < 0) {
        //toreachGoal = 0;
      }
      // Update ThingSpeak with water level and consumption data
      sendtoThings();

      if (consumption < consumptionGoal) {
        Serial.print(toreachGoal);
        Serial.println(" left to reach your goal.");
      }
      if (consumption >= consumptionGoal) {
        Serial.println("Goal achieved, congs! Goal reset!");
        consumptionGoal = 700 + consumption;
      }
      previousWaterLevel = waterLevel;
      idletime = 0;

    } else if (waterLevel <= 0) {
      Serial.println("You have no water in the bottle");
      Serial.println("Please refill the bottle");

    } else if (waterLevel == 0) {
      Serial.println("You have no water in the bottle");
      Serial.println("Please refill the bottle");
      bottles = 0;
      actualconsumption = 0;
      sendtoThings();

    } else if (waterLevel > previousWaterLevel) {
      idletime = 0;
      bottles = 0;
      actualconsumption = 0;
      Serial.println("Bottle refilled, more energy!");
      // Update ThingSpeak with water level and consumption data
      sendtoThings();
      previousWaterLevel = waterLevel;
    } else if (waterLevel >= previousWaterLevel - 0.00015 && waterLevel <= previousWaterLevel + 0.00015) {
      idletime += 15;  // increment by 15 seconds (delay time)
      Serial.println("Please take water");
      digitalWrite(led, HIGH);
      // Update ThingSpeak with water level and consumption data
      ThingSpeak.setField(1, previousWaterLevel);
      ThingSpeak.setField(2, waterLevel);
      ThingSpeak.setField(3, 0);
      ThingSpeak.setField(4, 0);
      ThingSpeak.setField(5, 0);
      ThingSpeak.setField(6, 0);
      ThingSpeak.setField(7, idletime);
      ThingSpeak.setField(8, asctime(&timeinfo));
      ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    }
    // Wait for 1 second before taking another measurement
    
  }
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);

  Serial.print("Ax: ");
  Serial.print(Ax);
  Serial.print(" Ay: ");
  Serial.print(Ay);
  Serial.print(" Az: ");
  Serial.print(Az);
  Serial.print(" T: ");
  Serial.print(T);
  Serial.print(" Gx: ");
  Serial.print(Gx);
  Serial.print(" Gy: ");
  Serial.print(Gy);
  Serial.print(" Gz: ");
  Serial.println(Gz);
  delay(15000);
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  Temperature = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init() {
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);   //set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);  // set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
