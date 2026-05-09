#include <Wire.h>
#include <Servo.h>
#include <avr/wdt.h>



#LQR_control_1dof_heli
/* ================= USER PARAMETERS ================= */

// --- Control Loop ---
#define LOOP_HZ 100
float Ts = 1.0 / LOOP_HZ;

// --- Desired Angle (CETA) ---
float theta_ref_deg = 20.0;    // <<< CHANGE THIS (e.g. 10.0 deg)
float theta_ref;
float theta_integral = 0;  // global variable
float theta_offset = 0.0;  // global variable


// --- LQR GAINS (TUNE LATER) ---
float k1 = 3.0;   // angle gain
float k2 = 4.0;    // angular velocity gain damping
float ki = 0.5;   // start small

float k_g = 0.85;   // <<< TUNE THIS

// --- ESC ---
int escPin = 9;
int pwm_hover = 1100;   // hover PWM (find experimentally)
int pwm = 1000;
// --- MPU6500 ---
#define MPU_ADDR 0x68

/* ================= GLOBAL VARIABLES ================= */

// Kalman states
float theta = 0.0;
float theta_dot = 0.0;

// Kalman covariance
float P[2][2] = {{1, 0}, {0, 1}};

// Kalman noise tuning
float Q_angle = 0.001;
float Q_rate  = 0.003;
float R_angle = 0.03;

// Raw sensor values
float gyro_rate;
float accel_angle;

// ESC object
Servo esc;

// Timing
unsigned long lastTime = 0;

/* ================= MPU FUNCTIONS ================= */

void mpuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);    // Power management
  Wire.write(0x00);    // Wake up
  Wire.endTransmission();

  // Gyro config ±250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Accel config ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t gx = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();

  // Convert
  float accX = ax / 16384.0;
  float accY = ay / 16384.0;
  float accZ = az / 16384.0;

  accel_angle = atan2(accY, accZ);          // rad
  gyro_rate = (gx / 131.0) * DEG_TO_RAD;     // rad/s
}

float gravityComp(float theta) {
  float g_comp = k_g * sin(theta);   // normalized gravity torque
  return g_comp;
}

/* ================= KALMAN FILTER ================= */

void kalmanUpdate(float gyro_rate, float accel_angle) {
  // Prediction
  theta += Ts * gyro_rate;
  theta_dot = gyro_rate;

  // Coveriance Matrix
  P[0][0] += Ts * (Ts * P[1][1] - P[0][1] - P[1][0] + Q_angle); //angle uncertainty
  P[0][1] -= Ts * P[1][1]; // correlation
  P[1][0] -= Ts * P[1][1]; // correlation
  P[1][1] += Q_rate * Ts; //rate uncertainty

  // Update
  float y = accel_angle - theta; // Innovation
  float S = P[0][0] + R_angle;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  theta     += K0 * y;
  theta_dot += K1 * y;

  float P00 = P[0][0];
  float P01 = P[0][1];

  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;
}

/* ================= STATE SPACE CONTROL ================= */

float stateControl(float theta, float theta_dot) {
  float e_theta = theta - theta_ref;
  float e_theta_dot = theta_dot;

  // accumulate integral
  theta_integral += e_theta * Ts;

  // Calculate the total control output (u is deviation from pwm_hover)
  float u_feedback = -(k1*e_theta + k2*e_theta_dot + ki*theta_integral);
  float u_feedforward = gravityComp(theta_ref) - 1.0; 

  float u = u_feedback + u_feedforward;

  // float u = gravityComp(theta_ref)
  //           - (k1 * e_theta + k2 * e_theta_dot + ki * theta_integral);

  // float u = -(k1*e_theta + k2*e_theta_dot + ki*theta_integral);

  return u;
}

/* ================= ESC CONTROL ================= */

void writeESC(float u) {
  // u = constrain(u, -1.0, 1.0);
  pwm = pwm_hover + (int)(u * 18.0);
  pwm = constrain(pwm, 1000, 1350);
  esc.writeMicroseconds(pwm);
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  while(!Serial); // wait for Serial (optional on UNO)
  delay(100);
  Wire.setWireTimeout(3000, true);

  mpuInit();
  // Read current angle as zero reference
  readMPU();
  theta_offset = accel_angle;   // store the rest angle in radians
  Serial.print("MPU zero offset (deg): ");
  Serial.println(theta_offset * RAD_TO_DEG);

  esc.attach(escPin);
  esc.writeMicroseconds(1000);
  delay(3000); // arm ESC

  theta_ref = theta_ref_deg * DEG_TO_RAD;

  lastTime = millis();
  wdt_enable(WDTO_250MS);  // reset if loop stalls
}

/* ================= LOOP ================= */

void loop() {
  wdt_reset();

  // -------- Read Serial input for desired angle --------
  if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      float new_angle = input.toFloat();
      if (new_angle >= 0 && new_angle <= 140) {
          theta_ref_deg = new_angle;
          Serial.print("New desired angle set: ");
          Serial.println(theta_ref_deg);
      } else {
          Serial.println("Invalid angle! Enter 0-90 deg.");
      }
  }

  if (millis() - lastTime >= 1000 / LOOP_HZ) {
    lastTime = millis();

    // Update reference angle dynamically
    theta_ref = theta_ref_deg * DEG_TO_RAD;

    // Read sensors
    readMPU();

    // Kalman filter
    kalmanUpdate(gyro_rate, accel_angle);
    // Apply offset to correct theta
    float corrected_theta = theta - theta_offset;

    // State-space control
    float u = stateControl(corrected_theta, theta_dot);

    // ESC output
    writeESC(u);

    static int print_cnt = 0;
    print_cnt++;
    if (print_cnt >= 10) {   // 10 Hz printing
      print_cnt = 0;
    // Debug
    Serial.print("Theta(deg): ");
    Serial.print(corrected_theta * RAD_TO_DEG);

    Serial.print("  Ref(deg): ");
    Serial.print(theta_ref_deg);

    Serial.print("  u: ");
    Serial.print(u);

    Serial.print("  PWM(us): ");
    Serial.println(pwm);
    }

  }
}
