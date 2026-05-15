#include "I2Cdev.h"                      //I2C
#include <Wire.h>                        //I2C
#include "MPU6050_6Axis_MotionApps20.h"  //IMU
#include "AS5600.h"                      //Encoder
#include <HCSR04.h>                      //ultrasonic
#define OUTPUT_READABLE_YAWPITCHROLL     //IMU
#define M_PI 3.14159265358979323846      // pi defination
#define AS5600_RAW_TO_RADIANS (2.0 * M_PI / 4096.0)  // get angle in Radian

volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;}

//defination 
MPU6050 mpu;   //IMU
AS5600L as5600;          //Encoder
int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin

const byte triggerPin = 5;  // ultrasonic
const byte echoPin = 18;     // ultrasonic
const float safeDistance = 20 ; // ultrasonic

// kinematics constants
const float L1 = 0.138 ;
const float L2 = 0.106 ; 
const float L3 = 0.077 ;
const float L4 = 0.075 ;
const float L23 = 0.050;
const float m1 = 0.025 ;
const float m2 = 0.014 ;
const float m3 = 0.011 ;
const float m23= 0.008 ;
// variables 
float Lleg = 0;
float L0 = 0 ;
float Lw = 0 ;     // effective Length (pendulum Length)

float yaw, pitch, roll;    //IMU
float yawRate, pitchRate, rollRate; //IMU

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// Encoder 
float theta_1 = 0;    // (Radian)
float theta_offset = 0;  
// Ultrasonic 
float distance = 0;

void setup() {
 
  // (I2C & Serial)
  Wire.begin();   //I2C  (IMU & Encoder)
  Wire.setClock(400000);  // 400 KHz  
  Serial.begin(115200); 

  // (Pins & Modes)(Sensors)
  as5600.begin(4);        // Encoder (pin 4)
  as5600.setDirection(AS5600_CLOCK_WISE);   // direction of increment 
  delay(500); // تأخير بسيط للتأكد من استقرار القراءة
  theta_offset = as5600.rawAngle() * AS5600_RAW_TO_RADIANS;   // Encoder Zero
  Serial.print("Encoder Zeroed at: ");
  Serial.println(theta_offset);


  HCSR04.begin(triggerPin, echoPin);      // ultrasonic
  pinMode(INTERRUPT_PIN, INPUT);          // interrupt pin of IMU

  // DMP
  Serial.println(F("Initializing DMP..."));
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    // Calibration
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    //DMPDataReady
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    DMPReady = true;    
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("System Ready!"));
  } else {
    Serial.print(F("DMP Error: "));
    Serial.println(devStatus);
  }
}


void readIMU() {
  if (!DMPReady) return; 

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
    
    // (Yaw, Pitch, Roll) angles in Radian
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    yaw   = ypr[0]; 
    pitch = ypr[1]; 
    roll  = ypr[2];

    // (Gyro Rates) 
    mpu.dmpGetGyro(&gy, FIFOBuffer);
    // from deg/s  to  rad/s  
    yawRate   = (gy.z / 16.4) * (M_PI / 180.0);
    pitchRate = (gy.y / 16.4) * (M_PI / 180.0);
    rollRate  = (gy.x / 16.4) * (M_PI / 180.0);
    // Print 
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) { 
      Serial.print("Yaw:"); Serial.print(yaw);
      Serial.print(" Pitch:"); Serial.print(pitch);
      Serial.print(" Roll:"); Serial.print(roll);
      Serial.print(" Yaw_Rate:"); Serial.print(yawRate);
      Serial.print(" Pitch_Rate:"); Serial.print(pitchRate);
      Serial.print(" Roll_Rate:"); Serial.println(rollRate);
      lastPrint = millis();
    }

  }
}
void readEncoder() {

  float currentRaw = as5600.rawAngle() * AS5600_RAW_TO_RADIANS;
  theta_1 = currentRaw - theta_offset;

}

void readUltrasonic() {
  
  double* distances = HCSR04.measureDistanceCm();
  if (distances != nullptr) {
    distance = (float)distances[0]; 
  }

  if (distance > 0 && distance < safeDistance) {
     Serial.println("Warning: Obstacle Detected!");
  }
}

void calculateKinematics() {
  
  float L5 = sqrt(pow(L2, 2) + pow(L4, 2) - 2 * L2 * L4 * cos(theta_1 + M_PI / 4.0));
  // معادلة (24): theta_2
  float theta_2 = asin((L4 * sin(theta_1 + M_PI / 4.0)) / L5);
  // معادلة (25): theta_3
  float theta_3 = acos((pow(L23, 2) + pow(L5, 2) - pow(L3, 2)) / (2 * L23 * L5));
  // معادلة (26): theta_4
  float theta_4 = M_PI - theta_1 - theta_2 - theta_3;
  // معادلة (27): Lleg الفعلي
  Lleg = L2 * sin(theta_1) + L1 * sin(theta_4);
  // معادلة (29): theta_5
  float theta_5 = M_PI - theta_4;
  // معادلة (30): theta_6
  float theta_6 = asin((L5 * sin(theta_3)) / L3) - theta_4;

  float xc2  = (L2 * sin(theta_1)) / 2.0;
  float xc1  = L2 * sin(theta_1) + (L1 * sin(theta_5)) / 2.0;
  float xc3  = -L4 * sin(M_PI / 4.0) + (L3 * sin(theta_6)) / 2.0;
  float xc23 = L2 * sin(theta_1) - (L23 * sin(theta_4)) / 2.0;
  // معادلة (35): حساب ارتفاع مركز الثقل الإجمالي L0
  L0 = (m1 * xc1 + m2 * xc2 + m3 * xc3 + m23 * xc23) / (m1 + m2 + m3 + m23);
  // معادلة (36): طول البندول الفعال Lw المستخدم في الـ LQR
  Lw = Lleg - L0;
  static unsigned long lastKinPrint = 0;
  if (millis() - lastKinPrint > 100) {
    Serial.print("Lleg: "); Serial.print(Lleg);
    Serial.print("L0: "); Serial.print(L0);
    Serial.print("Lw: "); Serial.println(Lw);
    lastKinPrint = millis();
  }
}


void loop() {
  
  readIMU();        
  readEncoder();    
  readUltrasonic(); 
  calculateKinematics(); 

}

