// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements (Angular Velocity)
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container
///////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////LQR ////////////////////////////
const float Fixed_Theta1 = 1.0;  //Rad
//////////(LQR Gain)//////////
float p_k1[] = {0.074114, -0.173704, -1.929676}; 
float p_k2[] = {-0.021690, 0.050877, -0.223459};
////////(Lleg & Lw)///////
float a1 = -0.072848, b1 = 0.205408, c1 = 0.094574;
float a3 = -0.037861, b3 = 0.088834, c3 = 0.113857;
//////////variables/////////
float Lleg = 0;
float Lw = 0 ;     // effective Length (pendulum Length)
float k_pitch, k_pitch_rate;
////////States variable from IMU //////////
float current_pitch, current_gyro_y;
float target_pitch = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////Stepper ////////////////
// Left wheel motor
byte step_L =5;
byte dir_L  =6;
// Right wheel motor
byte step_R= 7;
byte dir_R  =8;

long stepDelay = 0;         
unsigned long lastStepTime = 0; 
const int min_delay = 200;    
const int max_delay = 5000;

/////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////
void setup() {
    // Arduino Nano uses pins A4 (SDA) and A5 (SCL) automatically
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
    Serial.begin(115200);
    while (!Serial); 
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();
    // Verify connection
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
    if (devStatus == 0) {
        // Calibration
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      
    }
     ///////////////Stepper////////////////
  pinMode(step_L, OUTPUT);
  pinMode(dir_L, OUTPUT);
  pinMode(step_R, OUTPUT);
  pinMode(dir_R, OUTPUT);
  
  //////////////////////LQR/////////////////////////
  k_pitch = p_k1[0]*pow(Fixed_Theta1, 2) + p_k1[1]*Fixed_Theta1 + p_k1[2];
  k_pitch_rate = p_k2[0]*pow(Fixed_Theta1, 2) + p_k2[1]*Fixed_Theta1 + p_k2[2];
  Lleg = a1*pow(Fixed_Theta1, 2) + b1*Fixed_Theta1 + c1;
  Lw = a3*pow(Fixed_Theta1, 2) + b3*Fixed_Theta1 + c3;

}
void loop() {
    if (!dmpReady) return;
    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        // Get orientation data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Get angular velocity data (Gyro)
        mpu.dmpGetGyro(&gy, fifoBuffer);

        // Send data to serial (Yaw, Pitch, Roll, and Gyro X,Y,Z)
        sendReadings(ypr[0], ypr[1], ypr[2], gy.x, gy.y, gy.z);

       float torque = LQR_Calculation();
        stepper_control(torque);
    }
///////////////////////////////////////////////////////////////////////////////////////////////////

}
 float LQR_Calculation() {
    current_pitch = ypr[1];         
    current_gyro_y = gy.y / 131.0;  
    // Error
    float pitch_error = current_pitch - target_pitch;
    float rate_error = current_gyro_y; 
    //Torque 
    float T_bal = (k_pitch * pitch_error) + (k_pitch_rate * rate_error);
    return T_bal;
}

void sendReadings(float yaw_rad, float pitch_rad, float roll_rad, int16_t gx, int16_t gy, int16_t gz) {
    // Convert radians to degrees
    float yaw_deg = yaw_rad * 180.0 / M_PI;
    float pitch_deg = pitch_rad * 180.0 / M_PI;
    float roll_deg = roll_rad * 180.0 / M_PI;
    // Printing Angles
    Serial.print("Yaw:"); Serial.print(yaw_deg);
    Serial.print("\tPitch:"); Serial.print(pitch_deg);
    Serial.print("\tRoll:"); Serial.print(roll_deg);
    // Printing Angular Velocities (Raw values from Gyro)
    Serial.print("\t| GX:"); Serial.print(gx);
    Serial.print("\tGY:"); Serial.print(gy);
    Serial.print("\tGZ:"); Serial.println(gz);
}

void stepper_control(float torque) {
  
    if (torque > 0) {
        digitalWrite(dir_L, HIGH); 
        digitalWrite(dir_R, LOW);  
    } else {
        digitalWrite(dir_L, LOW);
        digitalWrite(dir_R, HIGH);
    }
    float abs_torque = abs(torque);

    if (abs_torque < 0.05) { 
        stepDelay = 0; 
    } else {
        
        stepDelay = 80000 / abs_torque;
        if (stepDelay < min_delay) stepDelay = min_delay;
        if (stepDelay > max_delay) stepDelay = max_delay;
    }
    run_motors();
}



void run_motors() {
    if (stepDelay == 0) return; 

    if (micros() - lastStepTime >= stepDelay) {
        lastStepTime = micros();
      
        digitalWrite(step_L, HIGH);
        digitalWrite(step_R, HIGH);
        delayMicroseconds(10);
        digitalWrite(step_L, LOW);
        digitalWrite(step_R, LOW);
        Serial.println("motor run");
    }
}