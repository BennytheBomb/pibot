#include <Arduino.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define DEBUG 0

#define PWMB 9
#define BIN2 10
#define BIN1 11  // right
#define STBY 12
#define AIN1 13  // left
#define AIN2 14
#define PWMA 15

#define MPU_SDA 16
#define MPU_SCL 17

#define ENC_LEFT_A  18
#define ENC_LEFT_B  19
#define ENC_RIGHT_A 20
#define ENC_RIGHT_B 21

// ---------------------------------------------------------------------------
// Robot physical constants
// ---------------------------------------------------------------------------
const float WHEEL_DIAMETER_M = 0.065f;
const float TICKS_PER_REV    = 2140.0f;
const float DIST_PER_TICK    = PI * WHEEL_DIAMETER_M / TICKS_PER_REV;
const float MAX_WHEEL_VEL    = (160.0f / 60.0f) * PI * WHEEL_DIAMETER_M;  // ~0.272 m/s

// ---------------------------------------------------------------------------
// PID timing
// ---------------------------------------------------------------------------
const unsigned long PID_INTERVAL_MS  = 10;   // 100 Hz
const unsigned long SEND_INTERVAL_MS = 20;   // 50 Hz

// ---------------------------------------------------------------------------
// PID gains — tuned for ticks-per-interval units
// At 0.15 m/s target = ~15.7 ticks per 10ms interval
// Kp=1.5 means: error of 1 tick → PWM correction of 1.5
// ---------------------------------------------------------------------------
float KP =  1.5f;
float KI =  0.8f;
float KD =  0.05f;

// ---------------------------------------------------------------------------
// Motor driver
// ---------------------------------------------------------------------------
const int offsetA = -1;
const int offsetB = -1;
const int MIN_PWM =  30;

Motor leftMotor  = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// ---------------------------------------------------------------------------
// Encoder state
// ---------------------------------------------------------------------------
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
int last_l_dir = 1;
int last_r_dir = 1;

void leftEncoderISR()  { left_ticks  += last_l_dir; }
void rightEncoderISR() { right_ticks += last_r_dir; }

// ---------------------------------------------------------------------------
// PID state per wheel
// ---------------------------------------------------------------------------
float left_integral    = 0.0f;
float left_prev_error  = 0.0f;
float right_integral   = 0.0f;
float right_prev_error = 0.0f;

// Max integral clamp in ticks units — prevents windup
const float MAX_INTEGRAL = 50.0f;

float computePID(float target_ticks, float actual_ticks,
                 float &integral, float &prev_error) {
    float error  = target_ticks - actual_ticks;
    integral    += error;
    integral     = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
    float deriv  = error - prev_error;
    prev_error   = error;
    return KP * error + KI * integral + KD * deriv;
}

// ---------------------------------------------------------------------------
// Target velocities in m/s — set by serial parser
// ---------------------------------------------------------------------------
float target_left_vel  = 0.0f;
float target_right_vel = 0.0f;

// ---------------------------------------------------------------------------
// Serial input
// ---------------------------------------------------------------------------
const int MAX_CHARS = 64;
char input_buf[MAX_CHARS];
int  buf_idx = 0;

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------
Adafruit_MPU6050 mpu;

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
unsigned long last_pid_time  = 0;
unsigned long last_send_time = 0;

// ---------------------------------------------------------------------------
// Apply PID output to a motor
// ---------------------------------------------------------------------------
void applyMotor(Motor &motor, int &last_dir,
                float target_vel, float pwm_output,
                float &integral, float &prev_error) {
    if (abs(target_vel) < 1e-4f) {
        // Stop — reset PID state
        last_dir   = 1;
        integral   = 0.0f;
        prev_error = 0.0f;
        motor.drive(0);
        return;
    }

    int direction = target_vel > 0.0f ? 1 : -1;

    // Reset integrator on direction change to prevent windup fighting
    if (direction != last_dir) {
        integral   = 0.0f;
        prev_error = 0.0f;
    }
    last_dir = direction;

    int pwm = (int)constrain(abs(pwm_output), 0.0f, 255.0f);
    if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
    motor.drive(pwm * direction);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    pinMode(ENC_LEFT_A,  INPUT_PULLUP);
    pinMode(ENC_LEFT_B,  INPUT_PULLUP);
    pinMode(ENC_RIGHT_A, INPUT_PULLUP);
    pinMode(ENC_RIGHT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

    Wire.setSDA(MPU_SDA);
    Wire.setSCL(MPU_SCL);
    Wire.begin();
    if (!mpu.begin(0x68, &Wire)) {
        pinMode(25, OUTPUT);
        while (1) {
            digitalWrite(25, HIGH); delay(200);
            digitalWrite(25, LOW);  delay(200);
        }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    last_pid_time  = millis();
    last_send_time = millis();
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop() {

    // 1. Read incoming velocity commands: "left_vel,right_vel\n" in m/s
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            input_buf[buf_idx] = '\0';
            buf_idx = 0;
            char* token = strtok(input_buf, ",");
            if (token != NULL) {
                float lv = atof(token);
                token = strtok(NULL, ",");
                if (token != NULL) {
                    float rv = atof(token);
                    if (abs(lv) <= MAX_WHEEL_VEL * 1.1f &&
                        abs(rv) <= MAX_WHEEL_VEL * 1.1f) {
                        target_left_vel  = lv;
                        target_right_vel = rv;
                    }
                }
            }
        } else if (c != '\r' && buf_idx < MAX_CHARS - 1) {
            input_buf[buf_idx++] = c;
        }
    }

    // 2. PID velocity control at 100Hz
    unsigned long now = millis();
    if (now - last_pid_time >= PID_INTERVAL_MS) {
        last_pid_time = now;

        noInterrupts();
        long lt   = left_ticks;
        long rt   = right_ticks;
        float tlv = target_left_vel;
        float trv = target_right_vel;
        interrupts();

        // Ticks counted since last PID interval
        static long prev_lt = 0;
        static long prev_rt = 0;
        float actual_left_ticks  = (float)(lt - prev_lt);
        float actual_right_ticks = (float)(rt - prev_rt);
        prev_lt = lt;
        prev_rt = rt;

        // Convert m/s target to ticks per interval
        // ticks_per_interval = vel * PID_INTERVAL_MS/1000 / DIST_PER_TICK
        float target_left_ticks  = tlv * (PID_INTERVAL_MS / 1000.0f) / DIST_PER_TICK;
        float target_right_ticks = trv * (PID_INTERVAL_MS / 1000.0f) / DIST_PER_TICK;

        float left_pwm  = computePID(target_left_ticks,  actual_left_ticks,
                                     left_integral,  left_prev_error);
        float right_pwm = computePID(target_right_ticks, actual_right_ticks,
                                     right_integral, right_prev_error);

        applyMotor(leftMotor,  last_l_dir, tlv, left_pwm,
                   left_integral,  left_prev_error);
        applyMotor(rightMotor, last_r_dir, trv, right_pwm,
                   right_integral, right_prev_error);

#if DEBUG
        Serial.print("L tgt:"); Serial.print(target_left_ticks, 2);
        Serial.print(" act:"); Serial.print(actual_left_ticks, 2);
        Serial.print(" pwm:"); Serial.print(left_pwm, 1);
        Serial.print(" | R tgt:"); Serial.print(target_right_ticks, 2);
        Serial.print(" act:"); Serial.print(actual_right_ticks, 2);
        Serial.print(" pwm:"); Serial.println(right_pwm, 1);
#endif
    }

    // 3. Send sensor data at 50Hz
    now = millis();
    if (now - last_send_time >= SEND_INTERVAL_MS) {
        last_send_time = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        noInterrupts();
        long lt = left_ticks;
        long rt = right_ticks;
        interrupts();

#if !DEBUG
        Serial.print(lt);                  Serial.print(",");
        Serial.print(rt);                  Serial.print(",");
        Serial.print(a.acceleration.x, 4); Serial.print(",");
        Serial.print(a.acceleration.y, 4); Serial.print(",");
        Serial.print(a.acceleration.z, 4); Serial.print(",");
        Serial.print(g.gyro.x, 4);         Serial.print(",");
        Serial.print(g.gyro.y, 4);         Serial.print(",");
        Serial.println(g.gyro.z, 4);
#endif
    }
}
