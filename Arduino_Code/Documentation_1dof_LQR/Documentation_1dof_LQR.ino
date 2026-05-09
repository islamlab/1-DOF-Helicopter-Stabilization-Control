#include <Wire.h>   // I2C communication library for MPU6500
#include <Servo.h>  // Library to control the ESC (Electronic Speed Controller)
#include <avr/wdt.h> // Watchdog timer to prevent system freeze



#Documentation_1dof_LQR
/* ================= USER PARAMETERS ================= */

// --- Control Loop Timing ---
#define LOOP_HZ 100               // Running the control loop at 100Hz (every 10ms)
float Ts = 1.0 / LOOP_HZ;         // Sample time in seconds (0.01s)

// --- Desired Angle (Setpoint) ---
float theta_ref_deg = 20.0;       // Target angle in degrees
float theta_ref;                  // Target angle in radians (converted later)
float theta_integral = 0;         // Sum of error over time for Integral control
float theta_offset = 0.0;         // Calibration value to zero the sensor at rest

// --- LQR & INTEGRAL GAINS ---
float k1 = 3.0;                   // Proportional gain (Attitude stiffness)
float k2 = 2.0;                   // Derivative gain (Damping/Velocity control)
float ki = 0.7;                   // Integral gain (Eliminates steady-state error)

// --- ESC & ACTUATION ---
int escPin = 9;                   // Digital pin connected to ESC signal
int pwm_hover = 1100;             // PWM value needed to hold heli level at 0 degrees
int pwm = 1000;                   // Current PWM value being sent to motor

// --- MPU6500 I2C ADDRESS ---
#define MPU_ADDR 0x68             // Default I2C address for the MPU6050/6500

/* ================= GLOBAL VARIABLES ================= */

// Kalman Filter State Estimates
float theta = 0.0;                // Estimated angle (rad)
float theta_dot = 0.0;            // Estimated angular velocity (rad/s)

// Kalman Covariance Matrix (Uncertainty Tracking)
float P[2][2] = {{1, 0}, {0, 1}}; // Initial certainty; gets updated every loop

// Kalman Noise Tuning Parameters
float Q_angle = 0.001;            // Process noise for angle (trust in gyro integration)
float Q_rate  = 0.003;            // Process noise for gyro bias
float R_angle = 0.03;             // Measurement noise (trust in accelerometer)

// Raw Sensor Values
float gyro_rate;                  // Direct gyro reading (rad/s)
float accel_angle;                // Calculated angle from accelerometer (rad)

Servo esc;                        // Create Servo object to control ESC
unsigned long lastTime = 0;       // Timer for loop frequency control

/* ================= MPU FUNCTIONS ================= */

// Initialize the MPU6500 sensor
void mpuInit() {
  Wire.begin();                   // Start I2C
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);               // Power Management register
  Wire.write(0x00);               // Wake up the sensor
  Wire.endTransmission();

  // Configure Gyroscope scale (+/- 250 deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure Accelerometer scale (+/- 2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

// Read raw data and convert to physical units
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);               // Starting register for Accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  // Combine high/low bytes for Accel X, Y, Z
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();       // Skip Temperature data
  int16_t gx = Wire.read() << 8 | Wire.read(); // Gyro X (pitch axis)

  // Convert raw to G-force (16384 LSB/g)
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  // Calculate angle using gravity vector: atan2(y, z)
  accel_angle = atan2(accY, accZ);          
  // Convert raw gyro to rad/s (131 LSB per deg/s)
  gyro_rate = (gx / 131.0) * DEG_TO_RAD;     
}

// Model the torque change due to gravity
float gravityComp(float theta) {
  return cos(theta); // Gravity torque is max at 0 rad (horizontal) and min at pi/2 (vertical)
}

/* ================= KALMAN FILTER ================= */

// Merges Accel (stable but noisy) and Gyro (clean but drifts)
void kalmanUpdate(float gyro_rate, float accel_angle) {
  
  // 1. Prediction Step: Use Gyro to predict new state
  theta += Ts * gyro_rate;       // Integrate gyro to get angle
  theta_dot = gyro_rate;         // Rate is taken directly from gyro

  // 2. Update Covariance: Predict how much our uncertainty grew
  P[0][0] += Ts * (Ts * P[1][1] - P[0][1] - P[1][0] + Q_angle); // The += ensures that every time the loop runs, we are adding a small amount of "doubt" ($Q_{angle}$) to our estimate.
  P[0][1] -= Ts * P[1][1];
  P[1][0] -= Ts * P[1][1];
  P[1][1] += Q_rate * Ts;

  // 3. Innovation: Calculate difference between Accel measurement and prediction
  float y = accel_angle - theta; 
  float S = P[0][0] + R_angle;   // Estimation error + Measurement noise
  float K0 = P[0][0] / S;        // Kalman Gain for angle
  float K1 = P[1][0] / S;        // Kalman Gain for rate

  // 4. Correction: Update state estimate based on Gain and Innovation
  theta     += K0 * y;
  theta_dot += K1 * y;

  // 5. Update Covariance: Narrow down uncertainty for next loop
  float P00 = P[0][0];
  float P01 = P[0][1];
  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;
}

/* ================= STATE SPACE CONTROL ================= */

float stateControl(float theta, float theta_dot) {
  // Calculate Errors
  float e_theta = theta - theta_ref;      // Angle error (State 1) in rad
  float e_theta_dot = theta_dot;          // Angular velocity error (State 2)

  // Integral Term: Sum up error over time to fix offsets
  theta_integral += e_theta * Ts;

  // Feedback (LQR Logic): u = -(K*x)
  float u_feedback = -(k1*e_theta + k2*e_theta_dot + ki*theta_integral);

  // Feedforward (Gravity Logic): Adjust thrust based on physics model
  // We subtract 1.0 because pwm_hover already accounts for gravity at 0 deg (cos(0)=1)
  float u_feedforward = gravityComp(theta_ref) - 1.0; 

  // Combined control effort
  return u_feedback + u_feedforward;
}

/* ================= ESC ACTUATION ================= */

void writeESC(float u) {
  // Scale normalized 'u' to PWM change. 18.0 is the experimental gain factor.
  pwm = pwm_hover + (int)(u * 18.0);
  
  // Safety constrain: Keep PWM within ESC limits
  pwm = constrain(pwm, 1000, 1250);
  
  // Send signal to ESC
  esc.writeMicroseconds(pwm);
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);           // High speed Serial for debugging
  while(!Serial); 
  delay(100);
  Wire.setWireTimeout(3000, true);

  mpuInit();                      // Setup sensor registers
  
  // Calibration: Assume the heli is at rest on startup
  readMPU();
  theta_offset = accel_angle;     // Capture the 'zero' position
  
  esc.attach(escPin);             // Setup ESC pin
  esc.writeMicroseconds(1000);    // Send minimum throttle
  delay(3000);                    // Wait for ESC to arm (beeps)

  theta_ref = theta_ref_deg * DEG_TO_RAD; // Initial deg to rad conversion

  lastTime = millis();
  wdt_enable(WDTO_250MS);         // Setup Watchdog (Auto-reset if code hangs > 250ms)
}

/* ================= MAIN LOOP ================= */

void loop() {
  wdt_reset();                    // Tell Watchdog the system is alive

  // --- Serial Interface ---
  if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      float new_angle = input.toFloat();
      if (new_angle >= 0 && new_angle <= 90) {
          theta_ref_deg = new_angle; // Update setpoint via Serial Monitor
      }
  }

  // --- Control Loop Execution ---
  if (millis() - lastTime >= 1000 / LOOP_HZ) {
    lastTime = millis();          // Maintain 100Hz frequency

    theta_ref = theta_ref_deg * DEG_TO_RAD; // Convert current target to radians

    readMPU();                    // Get raw data

    kalmanUpdate(gyro_rate, accel_angle); // Filter the data
    
    // Adjust the estimate by the initial calibration offset
    float corrected_theta = theta - theta_offset;

    // Calculate control signal
    float u = stateControl(corrected_theta, theta_dot);

    // Command the motor
    writeESC(u);

    // --- Telemetry (10Hz) ---
    static int print_cnt = 0;
    if (++print_cnt >= 10) { 
        print_cnt = 0;
        Serial.print("Theta:"); Serial.print(corrected_theta * RAD_TO_DEG);
        Serial.print(" Ref:");   Serial.print(theta_ref_deg);
        Serial.print(" u:");     Serial.print(u);
        Serial.print(" PWM:");   Serial.println(pwm);
    }
  }
}



/*
1. Waking up the Sensor (Register 0x6B) 
Purpose: The MPU6500 starts in "Sleep Mode" by default to save power.

Action: Writing 0x00 to the PWR_MGMT_1 register (0x6B) clears the "Sleep" bit (bit 6).

Result: This "wakes up" the sensor so it can begin measuring motion data. 

2. Setting Gyroscope Scale (Register 0x1B)
Purpose: To define the sensitivity range for rotational measurements.

Action: Writing 0x00 to the GYRO_CONFIG register (0x1B) sets the full-scale range to its lowest setting.

Result: The scale is set to ±250 degrees per second (°/s), providing the highest resolution (approx. 131 LSB/°/s). 

3. Setting Accelerometer Scale (Register 0x1C)
Purpose: To define the range for linear acceleration measurements.

Action: Writing 0x00 to the ACCEL_CONFIG register (0x1C) sets the full-scale range to its lowest setting.

Result: The scale is set to ±2g, providing the most sensitive measurement range (approx. 16,384 LSB/g). 


*/

/*
How $S$ controls the "Trust"Because $S$ is in the denominator, 
it decides who the filter believes more:If $S$ is large because $R_{angle}$ is huge: 
The denominator becomes very big, making the Kalman Gain ($K$) very small.Result: 
The filter thinks, "The sensor is too noisy right now, I'll mostly ignore it and stick to my gyro prediction.
"If $S$ is small because $P[0][0]$ is huge: The Kalman Gain ($K$) becomes large (closer to 1.0).Result: 
The filter thinks, "I have no idea where I am based on my gyro, but the accelerometer seems clear, 
so I will jump to the accelerometer's value."
*/

/*
2. The Physical Intuition: "The Reality Check"Think of $y$ as the error signal for your filter.If $y = 0$: Your prediction was perfect. 
The gyroscope's integration matches the accelerometer's gravity vector exactly. No correction is needed.If $y$ is positive: 
The accelerometer says the heli is tilted higher than the gyro predicted.If $y$ is negative: 
The accelerometer says the heli is tilted lower than the gyro predicted.Because gyroscopes drift over time, 
$y$ will almost never be zero. $y$ is the tool the Kalman Filter uses to "pull" the drifting gyroscope back to the truth.
*/