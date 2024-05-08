#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float rpy[3];


// ================================================================
// ===               Varialble for Reciever                     ===
// ================================================================

int i = 0;

// Pins associated with Each channel of Reciever
int ReceiverPins[] = {13, 14, 16, 17};

// PID values of each channel R P Y
float P[3] = {1,1,1};
float I[3] = {0,0,0};
float D[3] = {0,0,0};

float RPY_Setpoint[3] = {0,0,0};

// PID control variables
float dt;
float t_n, t_b; // time now, time before
float e[3];
float Prev_e[3] = {0, 0, 0};
float integral[3] = {0, 0, 0};
float g[3];
float Motor_Speed[4] = {0, 0, 0, 0};
int Motor_Pins[4] = {26, 27, 32, 33};

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
#include <ESP32Servo.h>
Servo MotorFL;  // create servo object to control a ESC
Servo MotorFR;
Servo MotorBL;
Servo MotorBR;

void setup() {
    MotorFL.attach(Motor_Pins[0]);  // create servo object to control a ESC
    MotorFR.attach(Motor_Pins[1]);
    MotorBL.attach(Motor_Pins[2]);
    MotorBR.attach(Motor_Pins[3]);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//    while (Serial.available() && Serial.read()); // empty buffer
//    while (!Serial.available());                 // wait for data
//    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // calculate dt
    t_n = micros();
    dt = t_n - t_b;
    t_b = t_n;

    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

    // Serial.print("Roll Pitch Yaw");
    for(int i=0; i<3; i++){
        rpy[i] = ypr[2-i] * 180/M_PI;
        // Serial.print(", ");
        // Serial.print(rpy[i]);
    }
    // Serial.println("");
    
    // get setpoint from receiver
    // ReadReceiver(ReceiverPins, RPY_Setpoint[0], RPY_Setpoint[1], RPY_Setpoint[2]);

    // Serial.print("calculate Error");
    for(int i=0; i<3; i++){
        e[i] = RPY_Setpoint[i] - rpy[i];
        // Serial.print(", ");
        // Serial.print(e[i]);
    }
    // Serial.println("");

    Serial.print("calculate PID ctl cmd ");
    for(int i=0; i<3; i++){
        integral[i] += e[i] * dt;
        g[i] = P[i]*e[i] + I[i]*(integral[i]) + D[i]*(e[i]-Prev_e[i])/dt;
        Serial.print(", g: ");
        Serial.print(g[i]);
    }
    // Serial.println("");

    // convert PID ctl cmd to motor ctl cmd
    //! check max and min value of Motor_Speed by experiments
    Motor_Speed[0] = (-g[0] + g[1] + g[2]);
    Motor_Speed[1] = (g[0] + g[1] - g[2]);
    Motor_Speed[2] = (-g[0] - g[1] - g[2]);
    Motor_Speed[3] = (g[0] - g[1] + g[2]);

    Serial.print(", Motor before mapping: ");
    Serial.print(Motor_Speed[0]);

    for(int i=0; i<4; i++){
        Motor_Speed[i] = map(constrain(Motor_Speed[i], -180, 180), -180, 180, 0, 180);
    }

    Serial.print(", Motor after mapping: ");
    Serial.println(Motor_Speed[0]);

    MotorFL.write(Motor_Speed[0]);
    MotorFR.write(Motor_Speed[1]);
    MotorBL.write(Motor_Speed[2]);
    MotorBR.write(Motor_Speed[3]);
}

// void ReadReceiver(int ReceiverPin[], float& ROLL, float& PITCH, float& YAW) {
//     /*
//     reads setpoint from receiver via pulseIn()
//     and returns data by reference
//     */
//     //!!Each channels range should be tested
//     int ch1_raw = constrain(pulseIn(ReceiverPin[0], HIGH), 1044, 1885);
//     int ch2_raw = constrain(pulseIn(ReceiverPin[1], HIGH), 1135, 1800);
//     int ch3_raw = constrain(pulseIn(ReceiverPin[2], HIGH, 25000), 1013, 1941);
//     int ch4_raw = constrain(pulseIn(ReceiverPin[3], HIGH), 1044, 1885);
//     int ch5_raw = constrain(pulseIn(ReceiverPin[4], HIGH), 1135, 1800);

//     // Invert or trim each channel here
//     float roll  = map(ch1_raw, 1044, 1885, -254, 254);
//     float pitch = map(ch2_raw, 1135, 1800, 254, -254);
//     float yaw   = map(ch3_raw, 1013, 1941, -254, 254);

//     // Setting Dead jone of joystick
//     //Serial.print(ROLL);
//     if(abs(roll) < 24){
//     roll = 0;
//     }
//     //Serial.println(ROLL);
//     if(abs(pitch) < 24){
//     pitch = 0;
//     }
//     if(abs(yaw) < 30){
//     yaw = 0;
//     }

//     ROLL  = roll;
//     PITCH = pitch;
//     YAW   = yaw;
// }