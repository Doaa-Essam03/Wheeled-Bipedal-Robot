#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 gy;
VectorFloat gravity;
float ypr[3];

void setup() {
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200);

    mpu.initialize();

    Serial.println(mpu.testConnection() ? "MPU6050 Connected" : "MPU6050 Connection Failed");

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);

        mpu.setDMPEnabled(true);

        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        Serial.println("DMP Ready");
    }
    else {
        Serial.print("DMP Initialization Failed: ");
        Serial.println(devStatus);
    }
}

void loop() {

    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

        mpu.dmpGetQuaternion(&q, fifoBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        mpu.dmpGetGyro(&gy, fifoBuffer);

        float yaw   = ypr[0] * 180/M_PI;
        float pitch = ypr[1] * 180/M_PI;
        float roll  = ypr[2] * 180/M_PI;

        Serial.print("Yaw: ");
        Serial.print(yaw);

        Serial.print("  Pitch: ");
        Serial.print(pitch);

        Serial.print("  Roll: ");
        Serial.print(roll);

        Serial.print("  Gyro Y: ");
        Serial.println(gy.y / 131.0);
    }
}