#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <Ticker.h>

// Select SDA and SCL pins for I2C communication 
#define I2C_SCL 5                    //Connect SCL pin GPIO5(D1) of Nodemcu
#define I2C_SDA 4                    //Connect SDA pin GPIO4(D2) of Nodemcu

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
Ticker flipper;

const char* ssid     = "arsalan-multilive-03219231052";
const char* password = "300saeed003";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,113);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter subscriber

void messageCb( const std_msgs::String& msg){
  Serial.print(msg.data);
}

ros::Subscriber<std_msgs::String> sub("/chatter", &messageCb );


void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); //Initializing the I2C connection
  MPU6050_Init();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Start to be polite
  nh.subscribe(sub);
  flipper.attach_ms(500, Test); // Test function will be call after every 500 milli seconds
}

void loop() {
nh.spinOnce();
  // Loop exproximativly at 0.1Hz
delay(100);
}
void Test()
{
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  
  //divide each with their sensitivity scale factor
  //Ax = (double)AccelX/AccelScaleFactor;
  //Ay = (double)AccelY/AccelScaleFactor;
  //Az = (double)AccelZ/AccelScaleFactor;
  //T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  //Gy = (double)GyroY/GyroScaleFactor;
  //Gz = (double)GyroZ/GyroScaleFactor;
/*
  Serial.print("Ax: "); Serial.print(Ax);
  Serial.print(" Ay: "); Serial.print(Ay);
  Serial.print(" Az: "); Serial.print(Az);
  Serial.print(" T: "); Serial.print(T);
  Serial.print(" Gx: "); Serial.print(Gx);
  Serial.print(" Gy: "); Serial.print(Gy);
  Serial.print(" Gz: "); Serial.println(Gz);
*/
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}
