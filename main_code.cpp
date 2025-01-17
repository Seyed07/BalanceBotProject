/***********PID************/
struct PIDController {
    double kp;
    double ki;
    double kd;
    double setpoint;
    double integral;
    double previous_error;
    double output;
    double integral_max;
    double integral_min;
    
    PIDController(double Kp, double Ki, double Kd, double i_max, double i_min) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
        setpoint = 0.0;
        integral = 0.0;
        previous_error = 0.0;
        output = 0.0;
        integral_max = i_max;
        integral_min = i_min;
    }
    
    double compute(double input, double dt) {
        double error = setpoint - input;
        integral += error * dt;
        
        // Integral Clamping (Anti-Windup)
        if (integral > integral_max) {
            integral = integral_max;
        }
        else if (integral < integral_min) {
            integral = integral_min;
        }
        
        double derivative = (error - previous_error) / dt;
        output = (kp * error) + (ki * integral) + (kd * derivative);
        
        // Output Clamping
        if (output > 255) {
            output = 255;
        }
        else if (output < -255) {
            output = -255;
        }
        
        previous_error = error;
        return output;
    }
};

// Initialize PID with Anti-Windup
double kp = 20; // To be Tuned
double ki = 9; // To be Tuned (Set to a non-zero value for integral action)
double kd = 1; // To be Tuned
double integral_max = 1000.0; // Maximum integral term
double integral_min = -1000.0; // Minimum integral term
PIDController pid(kp, ki, kd, integral_max, integral_min);



/********** L298N **********/
#include <L298N.h>

//Pin Definition:
const unsigned int EN_A = 9;
const unsigned int IN1_A = 8;
const unsigned int IN2_A = 7;

const unsigned int IN1_B = 5;
const unsigned int IN2_B = 4;
const unsigned int EN_B = 3;

// Create motor instances
L298N rightMotor(EN_A, IN1_A, IN2_A);
L298N leftMotor(EN_B, IN1_B, IN2_B);

// motor speeds
int speedLeft = 0;
int speedRight = 0;

/****************************/
/************MPU************/

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;
long velocity;

float trimPot;
float trimAngle;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

/****Interrupt Handling****/
void dmpDataReady() {
    mpuInterrupt = true;
}


/*************************/

/*************Filtered_output***********/
double lowPassFilter(double input, double outputPrev) {
    // Low-pass filter equation
    double output = 0.95 * input + (1 - 0.95) * outputPrev;

    return output;
}


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    
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


        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;


        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {

        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    // PID Initialization
    pid.setpoint = 0.0; // Desired setpoint (e.g., level position)
    
    pinMode(LED_PIN, OUTPUT);



}
  static unsigned long lastTime = 0;

void loop() {
  // Timing for PID (assuming loop runs every 10 ms)
  unsigned long currentTime = millis();
  double dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
  
  if (dt >= 0.01) { // 10 ms
    lastTime = currentTime;

//  }

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
//     

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        #endif

//       
            // blink LED to indicate activity
            blinkState = !blinkState;
            digitalWrite(LED_PIN, blinkState);
    }

  pitch = (ypr[1] * 180/M_PI); // adjust to degrees

//  
    pid.setpoint = 0.00; // You can adjust the setpoint as needed

    if (pitch > 0 && pitch < 5){
      pitch = 5.1;
    }
    else if (pitch < 0 && pitch > -5){
      pitch = -5.1;
    }
    else{
      pitch = pitch;
    }
    double input = pitch;

    // Compute PID output with anti-windup
    double output = pid.compute(input, dt);

    // set motor speed with adjusted turn values
    double output_mod;
    if (output > 0 && output < 20) {
        output_mod = output;
    }
    else if (output < 0 && output > -20) { // Corrected condition for negative outputs
        output_mod = -100;
    }
    else {
        output_mod = output;
    }

    speedLeft = output_mod;
    speedRight = output_mod;

    if (pitch > 30 || pitch < -30) { // angle threshold
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0); 
    } else {
        leftMotor.setSpeed(abs(speedLeft));
        rightMotor.setSpeed(abs(speedRight));
    }

    // move motors
    if (speedLeft > 0) { 
        leftMotor.backward();
    } else {
        leftMotor.forward();
    }
    
    if (speedRight > 0) { 
        rightMotor.backward();
    } else {
        rightMotor.forward();
    }
    

    Serial.print(pitch);

    Serial.println();
    

  }

}
