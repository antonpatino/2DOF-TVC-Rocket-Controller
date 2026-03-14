#include <Wire.h>
#include <ESP32Servo.h>
#include "ITG3200.h"
#include "DFRobot_ADXL345.h"

#define DEG2RAD(x) (x)*71.0/4068.0
#define RAD2DEG(x) (x)*4068.0/71.0

//Gains (use matlab script "Gain_calculate.m" to get them)
float Kx[2][4] = { {0.0, -1.694, 0.0, -0.0782}, {-1.694, 0.0, -0.0782, 0.0} };
float Ki[2][2] = { {0.0, -3.3516}, {-3.3516, 0.0} };

// Objects
ITG3200 gyro;
DFRobot_ADXL345_I2C ADXL345(&Wire, 0x53);
Servo servoPitch;
Servo servoYaw;

// Calibration constants
float gyro_bias_dps[3] = { -4.2154f, 0.8157f, -0.6550f };
float accel_offset[3] = { 4.969f, -6.378f, -37.069f };
float accel_g[3] = { 1.0/262.561f, 1.0/262.578f, 1.0/259.099f };

// Sensor alignment offsets
float theta_mount_rad = (90.0-16.2) * (PI / 180.0);
float pitch_zero_offset = 0.0;
float roll_zero_offset = 92.0;

// Pins
const int PIN_SERVO_PITCH = 25;   // TVC pitch axis
const int PIN_SERVO_YAW   = 26;   // TVC yaw axis
const int PIN_BUTTON_LQR  = 14;   // Button to enable LQR control
const int PIN_LED_OK      = 12;   // Status OK LED pin
const int PIN_LED_ERR     = 13;   // Error LED pin
const int SDA_PIN         = 21;   // IMU SDA pin
const int SCL_PIN         = 22;   // IMU SCL pin

// Debugging and timing
bool debug = true;
unsigned long t0 = 0, th = 0;
const int h = 100;                // Output time step in ms (100ms)

// Complementary Filter
const float alpha = 0.02;         // Accelerometer weight

// IMU Gyroscope Filter
float fc_g = 10.0;                // Filter cutoff frequency
float tau_g = 1.0 / (2.0 * PI * fc_g);

// IMU Accelerometer Filter
float fc_a = 2.0;                 // Filter cutoff frequency
float tau_a = 1.0 / (2.0 * PI * fc_a);

// Reference Angles (Setpoints)
float setpoint[2]={0.0,0.0};

//System output
float out_servos_rad[2];

// Angle variable definitions
float pitch = 0.0;
float roll  = 0.0;
float pitch_rocket = 0.0;         // Pitch in rocket frame
float yaw_rocket = 0.0;           // Yaw in rocket frame


// Servo Limits
const float SERVO_CENTER_P = 85.0;
const float SERVO_CENTER_Y = 85.0; // Corrected offset from manufacturing error
const float SERVO_LIMIT_DEG = 12.0; // +-12° from center

// IMU Filter variables
float gx_filt = 0.0;
float gy_filt = 0.0;
float gz_filt = 0.0;
float ax_filt = 0.0;
float ay_filt = 0.0;
float az_filt = 0.0;

// Integral term variable
float integral[2] = {0, 0}; 

// ###############################   FUNCTIONS   ###############################

//Calculates the angle in the rocket reference frame accounting for the mounting rotation of the sensor.
void computeRocketAngles(float pitch_s_deg, float yaw_s_deg) {
  // Rotation Matrix
  pitch_rocket =  cos(theta_mount_rad) * pitch_s_deg + sin(theta_mount_rad) * yaw_s_deg;
  yaw_rocket   = -sin(theta_mount_rad) * pitch_s_deg + cos(theta_mount_rad) * yaw_s_deg;
}

void LQRCompute(float dt, float x[4], float output[2]) {
  float error[2];

  for (int i = 0; i < 2; i++) {
    //Calculate error and integral term
    error[i] = setpoint[i] - x[i]; 
    
    //leaky integrator to prevent saturation before launch (0.99)
    integral[i] = (integral[i] * 0.99f) + (error[i] * dt);
    
    //reset output
    output[i] = 0;

    // Ki (2x2) * integral term (2x1)
    for (int j = 0; j < 2; j++) {
      output[i] += Ki[i][j] * integral[j];
    }

    //- Kx (2x4) * x (4x1)
    for (int j = 0; j < 4; j++) {
      output[i] -= Kx[i][j] * x[j];
    }

    // Anti-Windup and saturation
    if (output[i] > DEG2RAD(SERVO_LIMIT_DEG)) {
      integral[i] -= error[i] * dt; 
      output[i] = DEG2RAD(SERVO_LIMIT_DEG);
    } else if (output[i] < DEG2RAD(-SERVO_LIMIT_DEG)) {
      integral[i] -= error[i] * dt;
      output[i] = DEG2RAD(-SERVO_LIMIT_DEG);
    }
  }
}

//Commands the servo to a specific degree offset from its center
void moveServoDegrees(Servo &s, float degrees, float SERVO_CENTER){

  // Calculate final angle for the servo
  float ang = SERVO_CENTER + degrees;

  // Sature output if it exceeds limits
  if (ang < SERVO_CENTER - SERVO_LIMIT_DEG) ang = SERVO_CENTER - SERVO_LIMIT_DEG;
  if (ang > SERVO_CENTER + SERVO_LIMIT_DEG) ang = SERVO_CENTER + SERVO_LIMIT_DEG;

  s.write((int)ang);
}

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  delay(200);

  // Pin Modes
  pinMode(PIN_BUTTON_LQR, INPUT); 
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_ERR, OUTPUT);

  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_ERR, LOW);

  // Initialize I2C 
  Wire.begin(SDA_PIN, SCL_PIN, 400000); // 400kHz

  // Initialize IMU
  ADXL345.begin();
  ADXL345.powerOn();
  gyro.init();

  // Force ±8g range for high-acceleration scenarios
  #ifdef ADXL345_SET_RANGE_8G
  ADXL345.setRange(ADXL345_RANGE_8G);
  #endif

  delay(500);

  // Calibrate gyro bias using 500 samples
  int GYRO_SAMPLES = 500;
  int SAMPLE_DELAY_MS = 3;

  float sumx = 0.0, sumy = 0.0, sumz = 0.0;
  for (int i = 0; i < GYRO_SAMPLES; ++i) {
    float gx, gy, gz;
    gyro.getAngularVelocity(&gx, &gy, &gz); // in degrees
    sumx += gx;
    sumy += gy;
    sumz += gz;
    delay(SAMPLE_DELAY_MS);
  }

  // Calculate Gyro Bias
  gyro_bias_dps[0] = sumx / GYRO_SAMPLES;
  gyro_bias_dps[1] = sumy / GYRO_SAMPLES;
  gyro_bias_dps[2] = sumz / GYRO_SAMPLES;

  // Initialize Servos to center position
  servoPitch.setPeriodHertz(50);  // 50Hz 
  servoYaw.setPeriodHertz(50);    // 50Hz
  servoPitch.attach(PIN_SERVO_PITCH);
  servoYaw.attach(PIN_SERVO_YAW);
  servoPitch.write((int)SERVO_CENTER_P);
  servoYaw.write((int)SERVO_CENTER_Y);

  // Status OK Signal
  digitalWrite(PIN_LED_OK, HIGH);
  delay(200);
  digitalWrite(PIN_LED_OK, LOW);
}

void loop() {
  
  // Calculate Delta Time (dt)
  unsigned long t = micros();
  float dt = (t - t0) * 1e-6;
  t0 = t;

  // -------------------------   Read IMU   -------------------------
  
  // ACCELEROMETER
  int accraw[3] = {0,0,0};
  ADXL345.readAccel(accraw);
  int16_t ay_raw = (int16_t)accraw[0];
  int16_t ax_raw = (int16_t)accraw[1];
  int16_t az_raw = (int16_t)accraw[2];

  // Apply offsets and scaling
  float ax_g = ((float)ax_raw - accel_offset[1]) * accel_g[1];
  float ay_g = ((float)ay_raw - accel_offset[0]) * accel_g[0];
  float az_g = ((float)az_raw - accel_offset[2]) * accel_g[2];

  // Apply Low-Pass Filtering
  float a_a = dt / (tau_a + dt);
  ax_filt = a_a * ax_g + (1.0 - a_a) * ax_filt;
  ay_filt = a_a * ay_g + (1.0 - a_a) * ay_filt;
  az_filt = a_a * az_g + (1.0 - a_a) * az_filt;

  // Calculate angles from accelerometer
  float roll_acc_rad = atan2f(ay_g, az_g);
  float roll_acc_deg = roll_acc_rad * 180.0f / PI;
  float pitch_acc_rad = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g));
  float pitch_acc_deg = pitch_acc_rad * 180.0f / PI;

  // GYROSCOPE
  float gx_dps = 0, gy_dps = 0, gz_dps = 0;
  gyro.getAngularVelocity(&gy_dps, &gx_dps, &gz_dps);  
  
  // Apply bias offsets
  gx_dps -= gyro_bias_dps[1];
  gy_dps -= gyro_bias_dps[0];
  gz_dps -= gyro_bias_dps[2];

  // Apply Low-Pass Filtering
  float a_g = dt / (tau_g + dt);
  gx_filt = a_g * gx_dps + (1.0 - a_g) * gx_filt;
  gy_filt = a_g * gy_dps + (1.0 - a_g) * gy_filt;
  gz_filt = a_g * gz_dps + (1.0 - a_g) * gz_filt;

  // Calculate angles from gyroscope integration
  float pitchFromGyro = pitch + gy_dps * dt;
  float rollFromGyro  = roll  + gz_dps * dt;

  // Complementary Filter Fusion
  pitch = (1.0 - alpha) * pitchFromGyro + alpha * pitch_acc_deg;
  roll  = (1.0 - alpha) * rollFromGyro  + alpha * roll_acc_deg;

  // Apply calibration zero-offsets
  float pitch_off = pitch + pitch_zero_offset;
  float roll_off  = roll  + roll_zero_offset;
  
  // Transform angles to rocket reference frame
  computeRocketAngles(pitch_off, roll_off);

  // Build state vector (Converting EVERYTHING to Radians)
  float x[4] = {
    DEG2RAD(pitch_rocket),
    DEG2RAD(yaw_rocket),
    DEG2RAD(gy_filt),
    DEG2RAD(gz_filt)};

  // Enable LQR Control if button is pressed (Active LOW)
  if (digitalRead(PIN_BUTTON_LQR) == LOW) {

      //Apply LQR control  
      LQRCompute(dt, x, out_servos_rad);

      //Convert the output in radians of LQRCompute to degrees and move servos
      moveServoDegrees(servoPitch, RAD2DEG(out_servos_rad[0]), SERVO_CENTER_P);
      moveServoDegrees(servoYaw, RAD2DEG(out_servos_rad[1]), SERVO_CENTER_Y);
  }

  // Serial Debug Output
  if ((t - th >= h * 1e3) && (debug == true)) {
    Serial.print("ax: "); Serial.print(ax_filt, 2);
    Serial.print("  ay: "); Serial.print(ay_filt, 2);
    Serial.print("  az: "); Serial.print(az_filt, 2);
    Serial.print("  gx: "); Serial.print(gx_filt, 2);
    Serial.print("  gy: "); Serial.print(gy_filt, 2);
    Serial.print("  gz: "); Serial.print(gz_filt, 2);
    Serial.print("  pitch_sensor: "); Serial.print(pitch, 2);
    Serial.print("  roll_sensor: "); Serial.print(roll, 2);
    Serial.print("  roll_sensor_offset: "); Serial.print(roll_off, 2);
    Serial.print("  pitch_rocket: "); Serial.print(pitch_rocket, 2);
    Serial.print("  yaw_rocket: "); Serial.print(yaw_rocket, 2);
    Serial.print("  servo_pitch: "); Serial.print(RAD2DEG(out_servos_rad[0]), 2);
    Serial.print("  servo_yaw: "); Serial.println(RAD2DEG(out_servos_rad[1]), 2);

    th = t;
  }
}
