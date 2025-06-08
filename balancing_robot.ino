/*
Code includes:
 Real-time balancing using MPU6050 IMU and wheel encoders
 Incremental teleop controls (W/X for speed, A/D for turning)
*/

#include "MPU6050.h"
#include "Pins.h"
#include "My_encoders.h"


// Timing Constants
#define RUN_DURATION 3000000      // milliseconds
#define MAX_ULONG 0xFFFFFFFF

#define SERVO_INTERVAL 4000       // computation/tick microseconds
#define SERVO_INTERVAL_F (SERVO_INTERVAL*(1e-6))  // in seconds
#define ONE_OVER_SERVO_INTERVAL (1.0/SERVO_INTERVAL_F)

// Startup timing (milliseconds after start)
#define START_COLLECT  5000       // start data collection
#define START_BALANCE   100       // start balancer (after START_COLLECT)
#define START_SAFETY    1000      // start safety checks (after START_COLLECT)
#define START_MOVEMENT  5000      // start movements (after START_BALANCE)

#define DEBUG_PRINT_INTERVAL 500  // milliseconds between debug prints

// Physical Constants and Limits
#define MAX_COMMAND 1000          // maximum motor command
#define ENCODER_TO_RADIANS 0.004027682889218
#define GSCALE_TO_ACC 0.001575
#define ASCALE 5.575380291904716e-05  // convert Y accelerometer to radians
#define GSCALE 1.352011295799561e-04  // convert gyro to radians/second
#define HALF_TS_GSCALE 2.704022591599122e-07  // 0.5*Ts*GSCALE for Ts=0.004
#define BODY_ANGLE_LIMIT 0.5      // safety limit (radians)


// Teleop Control Parameters
#define VELOCITY_INCREMENT 0.5    // velocity change per W/X press
#define TURN_INCREMENT 0.1        // turn rate change per A/D press
#define MAX_TELEOP_SPEED 10.0     // maximum forward/backward speed (rad/s)
#define MAX_TURN_RATE 2.0         // maximum turning rate (rad/s)
#define VELOCITY_DECAY 1.00       // gradual decay when no input

// Array Sizes and Data Collection
#define NN 10                     // generic array size
#define DO_NOT_COLLECT_DATA 0
#define PLEASE_COLLECT_DATA 1
#define MIN_JERK_MOVEMENT_SIZE (4.0*PI)

// Check vector indices
#define ENCODER_DIFFS_SUM2 0
#define ACCELEROMETER_SUM 1
#define GYRO_SUM 2


// Global Variables

bool go = false;                  // enable/disable servo

// Button control
bool last_button_state = HIGH;   // assume button is normally HIGH (pullup)
unsigned long last_button_time = 0;
#define BUTTON_DEBOUNCE_MS 50     // debounce time

// Encoder readings
long left_angle = 0;
long right_angle = 0;

// Wheel control
float drive_frequency = 0.0;
float freq_2PI_over_1000 = 0;
float desired_angle_amplitude = 0;
float desired_velocity_amplitude = 0;
float wheel_desired_angle = 0;
float wheel_desired_velocity = 0;

// Teleop state
float teleop_forward_velocity = 0.0;
float teleop_turn_rate = 0.0;
bool teleop_mode = false;
unsigned long last_command_time = 0;

// Wheel velocity estimation
float wheel_angle = 0;
float last_wheel_angle = 0;

// Command filtering
float past_raw_commands[NN];
float past_filtered_commands[NN];

MPU6050 accelgyro;

// IMU bias corrections
int ax0 = 0;
int ay0 = 550.0;              // hand-adjusted
int gx0 = 16.47;
int gy0 = 91.23;
int gz0 = -75.55;

// Kalman filter for body angle
float Kf_body = 0.005;        // body angle Kalman filter gain
float last_body_angle = 0;
long last_gxx = 0;            // last zero-adjusted X gyro reading

// Gyro saturation handling
long last_gx = 0;             // last raw X gyro reading
long last_last_gx = 0;        // previous to last raw X gyro reading

// Performance monitoring
unsigned long servo_late = 0;
unsigned long max_servo_late = 0;

// Control gains (LQR design)
float wheel_angular_velocity_gain = 22.82;
float body_angle_gain = 584.61;
float body_angular_velocity_gain = 92.69;
float past_command_gain = 0.038;
float wheel_angle_gain = 10;

// Low pass filter coefficients (Matlab butter(1, 0.1))
float a_filter[2] = {1.000000000000000, -0.726542528005361};
float b_filter[2] = {0.136728735997319,  0.136728735997319};

int launch_duration = 0;
long checks[NN];              // diagnostic checks
int check_counter = 0;
int dead_zone = 0;            
float movement_duration = 1;
float one_over_movement_duration = 1.0/movement_duration;
float scaled_time = 0;
int past_command = 0;

// Motor Control Functions
void motor_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void motor_stop()
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(BIN1, LOW);
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void motor_left_command(int speed)
{
  if (speed >= 0) {
    digitalWrite(AIN1, 1);
    analogWrite(PWMA_LEFT, speed);
  } else {
    digitalWrite(AIN1, 0);
    analogWrite(PWMA_LEFT, -speed);
  }
}

void motor_right_command(int speed)
{
  // Note: reverses the sign of speed
  if (speed >= 0) {
    digitalWrite(BIN1, 1);
    analogWrite(PWMB_RIGHT, speed);
  } else {
    digitalWrite(BIN1, 0);
    analogWrite(PWMB_RIGHT, -speed);
  }
}

// Differential motor control for turning
void motor_differential_command(int base_command, float turn_command)
{
  int left_cmd = base_command + (int)(turn_command);
  int right_cmd = base_command - (int)(turn_command);
  
  // Apply command limits
  if (left_cmd > MAX_COMMAND) left_cmd = MAX_COMMAND;
  if (left_cmd < -MAX_COMMAND) left_cmd = -MAX_COMMAND;
  if (right_cmd > MAX_COMMAND) right_cmd = MAX_COMMAND;
  if (right_cmd < -MAX_COMMAND) right_cmd = -MAX_COMMAND;
  
  motor_left_command(left_cmd);
  motor_right_command(right_cmd);
}

// Setup
void setup()
{
  Wire.begin();
  Serial.begin(1000000);
  Wire.setClock(1000000UL);
  analogReference(INTERNAL1V1);

  while(!Serial);  // wait for serial port
  
  Serial.println("Balance Controller v2 with Incremental Teleoperation");
  delay(1000);

  motor_init();
  Serial.println("Motors initialized");
  delay(1000);

  // Initialize button with pullup resistor
  pinMode(KEY_MODE, INPUT_PULLUP);
  Serial.println("Button initialized");
  delay(500);

  Encoder_init(&left_angle, &right_angle);
  Serial.println("Encoders initialized");
  delay(1000);

  accelgyro.initialize();
  Serial.println("IMU initialized");
  delay(1000);

  Serial.println("Robot should be upright with training wheels.");
  Serial.println("Commands:");
  Serial.println("  g = start/run, s = stop");
  Serial.println("  BUTTON = start/stop balancing");
  Serial.println("  W/X = increase/decrease forward speed");
  Serial.println("  A/D = increase left/right turn rate");
  Serial.println("  Q = zero all velocities");

  go = false;  // start disabled
}


// Button and Command Processing

void check_button()
{
  bool current_button_state = digitalRead(KEY_MODE);
  
  // Check for button press (HIGH to LOW transition with pullup)
  if (last_button_state == HIGH && current_button_state == LOW) {
    // Debounce the button
    if (millis() - last_button_time > BUTTON_DEBOUNCE_MS) {
      // Toggle go state
      go = !go;
      if (go) {
        Serial.println("Button pressed - Starting balance!");
        teleop_mode = false;  // start in manual balance mode
        teleop_forward_velocity = 0;
        teleop_turn_rate = 0;
      } else {
        Serial.println("Button pressed - Stopping!");
        teleop_mode = false;
        teleop_forward_velocity = 0;
        teleop_turn_rate = 0;
      }
      last_button_time = millis();
    }
  }
  
  last_button_state = current_button_state;
}

void ProcessCommand()
{
  if (Serial.available() <= 0) return;

  int c = Serial.read();
  
  switch (c) {
    case 'S': case 's':
      Serial.println("Stop!");
      go = false;
      teleop_mode = false;
      teleop_forward_velocity = 0;
      teleop_turn_rate = 0;
      break;
      
    case 'G': case 'g':
      Serial.println("Go!");
      go = true;
      break;
      
    // Forward/backward control
    case 'W': case 'w':
      teleop_forward_velocity += VELOCITY_INCREMENT;
      if (teleop_forward_velocity > MAX_TELEOP_SPEED) {
        teleop_forward_velocity = MAX_TELEOP_SPEED;
      }
      teleop_mode = true;
      last_command_time = millis();
      Serial.print("Forward velocity: ");
      Serial.println(teleop_forward_velocity, 2);
      break;
      
    case 'X': case 'x':
      teleop_forward_velocity -= VELOCITY_INCREMENT;
      if (teleop_forward_velocity < -MAX_TELEOP_SPEED) {
        teleop_forward_velocity = -MAX_TELEOP_SPEED;
      }
      teleop_mode = true;
      last_command_time = millis();
      Serial.print("Forward velocity: ");
      Serial.println(teleop_forward_velocity, 2);
      break;
      
    // Turn control
    case 'D': case 'd':
      teleop_turn_rate -= TURN_INCREMENT;
      if (teleop_turn_rate < -MAX_TURN_RATE) {
        teleop_turn_rate = -MAX_TURN_RATE;
      }
      teleop_mode = true;
      last_command_time = millis();
      Serial.print("Turn rate: ");
      Serial.println(teleop_turn_rate, 2);
      break;
      
    case 'A': case 'a':
      teleop_turn_rate += TURN_INCREMENT;
      if (teleop_turn_rate > MAX_TURN_RATE) {
        teleop_turn_rate = MAX_TURN_RATE;
      }
      teleop_mode = true;
      last_command_time = millis();
      Serial.print("Turn rate: ");
      Serial.println(teleop_turn_rate, 2);
      break;
      
    case 'Q': case 'q':
      Serial.println("Zero all velocities!");
      teleop_mode = false;
      teleop_forward_velocity = 0;
      teleop_turn_rate = 0;
      last_command_time = millis();
      break;
      
    default:
      break;
  }
}

// Main Balancer Control Loop

void run_balancer(int launch_duration,
                  float wheel_angle_gain, float wheel_angular_velocity_gain,
                  float body_angle_gain, float body_angular_velocity_gain,
                  float past_command_gain,
                  float some_kind_of_integral_gain,  // placeholder
                  unsigned long run_time_limit, int collect_data)
{
  // Time management
  unsigned long last_micros = micros();
  unsigned long last_millis = millis();
  int servo_time = 0;
  unsigned long run_time = 0;
  unsigned long debug_print_time = 0;
  run_time_limit += START_COLLECT;

  // State initialization
  int started = 0;
  last_body_angle = 0;
  last_gx = 0;
  last_last_gx = 0;
  last_gxx = 0;
  int in_dead_zone = 0;
  last_wheel_angle = 0;
  
  // Initialize filter history
  for (int i = 0; i < NN; i++) {
    past_raw_commands[i] = 0;
    past_filtered_commands[i] = 0;
  }
   
  check_counter = 0;
  float movement_start_time = START_COLLECT + START_BALANCE + START_MOVEMENT;
  wheel_desired_angle = 0;
  wheel_desired_velocity = 0;

  // Print startup info for data collection mode
  if (collect_data) {
    int voltage_raw = analogRead(VOL_MEASURE_PIN);
    double voltage = (voltage_raw*1.1/1024)*((10+1.5)/1.5);
    Serial.print("Voltage: ");
    Serial.println(voltage);
    
    Serial.print("Gains: ");
    Serial.print(wheel_angle_gain, 8); Serial.print(" ");
    Serial.print(wheel_angular_velocity_gain, 8); Serial.print(" ");
    Serial.print(body_angle_gain, 8); Serial.print(" ");
    Serial.print(body_angular_velocity_gain, 8); Serial.print(" ");
    Serial.print(past_command_gain, 8); Serial.print(" ");
    Serial.println(some_kind_of_integral_gain, 8);
    
    Serial.print("SERVO_INTERVAL: "); Serial.println(SERVO_INTERVAL);
    Serial.print("Kf_body: "); Serial.println(Kf_body, 8);
    Serial.print("IMU biases: ");
    Serial.print(ax0); Serial.print(" ");
    Serial.print(ay0); Serial.print(" ");
    Serial.print(gx0); Serial.print(" ");
    Serial.print(gy0); Serial.print(" ");
    Serial.println(gz0);
  }
 
  // Main servo loop
  while (true) {
    // Process commands and button every loop
    ProcessCommand();
    check_button();

    // Timing Management
    unsigned long current_micros = micros();
    if (current_micros > last_micros) {
      servo_time += current_micros - last_micros;
    } else {  // handle rollover
      servo_time += (MAX_ULONG - last_micros) + current_micros;
    }
    last_micros = current_micros;

    // Wait for servo interval
    if (servo_time < SERVO_INTERVAL) continue;

    servo_time -= SERVO_INTERVAL;

    // Track timing performance
    if (max_servo_late < servo_time) max_servo_late = servo_time;
    if (servo_time > 50) servo_late += 1;
    
    // Update millisecond clocks
    unsigned long current_millis = millis();
    int millis_increment;
    if (current_millis > last_millis) {
      millis_increment = current_millis - last_millis;
    } else {  // handle rollover
      millis_increment = (MAX_ULONG - last_millis) + current_millis;
    }
    last_millis = current_millis;
    
    run_time += millis_increment;
    debug_print_time += millis_increment;

     // Safety and Stop Conditions
    if ((run_time > run_time_limit) || (!go)) {
      motor_stop();
      Serial.println("Motor stopped - time limit or stop command");
      return;
    }
    
    // Read wheel encoders
    Read_encoders(&left_angle, &right_angle);
    int enc_diff = left_angle - right_angle;
    checks[ENCODER_DIFFS_SUM2] += enc_diff*enc_diff;

    // Convert to radians (note sign flip)
    left_angle = left_angle * -1;
    right_angle = right_angle * -1;
    wheel_angle = -0.5*(ENCODER_TO_RADIANS)*((float)(left_angle + right_angle));

    // Estimate wheel velocity
    float wheel_velocity = ONE_OVER_SERVO_INTERVAL*(wheel_angle - last_wheel_angle);
    last_wheel_angle = wheel_angle;

    // Read IMU
    long ay = (long)(accelgyro.getAccelerationY());
    long gx = (long)(accelgyro.getRotationX());

    // Handle gyro saturation issue
    if ((last_gx <= last_last_gx) && (last_gx < 0) && (gx > 32000)) {
      gx = -32761;
    }
    last_last_gx = last_gx;
    last_gx = gx;

    // Apply IMU bias corrections
    long ayy = (long)ay - ay0;
    long gxx = (long)gx - gx0;

    checks[ACCELEROMETER_SUM] += ayy;
    checks[GYRO_SUM] += gxx;
    check_counter++;

    // Body angle Kalman filter
    float body_angle = last_body_angle + HALF_TS_GSCALE*((float)(gxx + last_gxx));
    body_angle -= Kf_body*(body_angle - (ASCALE)*((float)ayy));
    
    last_body_angle = body_angle;
    last_gxx = gxx;

    // Safety checks - stop if robot falls over
    if (run_time > START_COLLECT + START_BALANCE + START_SAFETY) {
      if (body_angle < -BODY_ANGLE_LIMIT) {
        motor_stop();
        Serial.print("Robot fell backward: ");
        Serial.println(body_angle);
        return;
      }
      if (body_angle > BODY_ANGLE_LIMIT) {
        motor_stop();
        Serial.print("Robot fell forward: ");
        Serial.println(body_angle);
        return;
      }
    }

    float body_angular_velocity = (GSCALE)*gxx;

    /* Control Algorithm */
    
    float command_float = 0;
    int command = 0;

    if (run_time < START_COLLECT + START_BALANCE) {
      // Do nothing during startup
      command = 0;
      past_raw_commands[1] = command;
      past_raw_commands[0] = command;
      past_filtered_commands[0] = command;
    } else if (run_time < START_COLLECT + START_BALANCE + launch_duration) {
      // Launch behavior - quick backward pulse
      command = -MAX_COMMAND;
      past_raw_commands[1] = command;
      past_raw_commands[0] = command;
      past_filtered_commands[0] = command;
    } else {
      // Main balancing control
      wheel_desired_angle = 0;
      
      // Set desired velocity from teleop
      if (teleop_mode) {
        wheel_desired_velocity = teleop_forward_velocity;
      } else {
        wheel_desired_velocity = 0;
      }

      // LQR control law
      command_float = 
        - wheel_angle_gain * (wheel_angle - wheel_desired_angle)
        - wheel_angular_velocity_gain * (wheel_velocity - wheel_desired_velocity)
        - body_angle_gain * body_angle
        - body_angular_velocity_gain * body_angular_velocity
        - past_command_gain * past_command;
      
      // Apply low-pass filter to command
      past_raw_commands[1] = past_raw_commands[0];
      past_raw_commands[0] = command_float;
      float filtered_command = b_filter[0]*past_raw_commands[0]
                             + b_filter[1]*past_raw_commands[1]
                             - a_filter[1]*past_filtered_commands[0];
      past_filtered_commands[0] = filtered_command;

      // Apply command limits
      if (filtered_command > MAX_COMMAND) filtered_command = MAX_COMMAND;
      if (filtered_command < -MAX_COMMAND) filtered_command = -MAX_COMMAND;
      command = (int)filtered_command;

      // Dead zone implementation for stiction compensation
      if (dead_zone > 0) {
        if (!in_dead_zone) {
          if (command > 0) {
            if (command <= (dead_zone >> 1)) {
              in_dead_zone = 1;
              command = 0;
            }
          } else {
            if (command >= (-dead_zone >> 1)) {
              in_dead_zone = 1;
              command = 0;
            }
          }
        } else {
          if (command > 0) {
            if (command <= dead_zone) {
              command = 0;
            } else {
              in_dead_zone = 0;
            }
          } else {
            if (command >= -dead_zone) {
              command = 0;
            } else {
              in_dead_zone = 0;
            }
          }
        }
      }
    }

    past_command = command;
    
    // Apply differential steering for turning
    float turn_command = teleop_turn_rate * 50.0;
    motor_differential_command(command, turn_command);

    /* Debug Output and Data Collection */
    
    // Debug output during non-collection periods
    if ((debug_print_time > DEBUG_PRINT_INTERVAL) && 
        ((run_time < START_COLLECT) || (run_time > run_time_limit))) {
      Serial.print(servo_late); Serial.print(" ");
      Serial.print(max_servo_late); Serial.print(" ");
      Serial.print(ayy); Serial.print(" ");
      Serial.print(body_angle, 4);
      
      if (teleop_mode) {
        Serial.print(" [TELEOP: V="); Serial.print(teleop_forward_velocity, 2);
        Serial.print(" T="); Serial.print(teleop_turn_rate, 2);
        Serial.print(" cmd="); Serial.print(command);
        Serial.print("]");
      }
      Serial.println();
      debug_print_time -= DEBUG_PRINT_INTERVAL;
    }

    // Data collection output
    if ((run_time >= START_COLLECT) && (run_time <= run_time_limit) && collect_data) {
      if (!started) {
        servo_late = 0;
        max_servo_late = 0;
        started = 1;
        Serial.println("Data");
      }
      Serial.print(current_micros); Serial.print(" ");
      Serial.print(left_angle); Serial.print(" ");
      Serial.print(right_angle); Serial.print(" ");
      Serial.print(command); Serial.print(" ");
      Serial.print(command); Serial.print(" ");
      Serial.print(ayy); Serial.print(" ");
      Serial.print(gxx); Serial.print(" ");
      Serial.print(1000*body_angle); Serial.print(" ");  // more digits
      Serial.println(wheel_velocity);
    }
  }
}

// Main loop

void loop()
{
  if (!go) {
    motor_stop();
    delay(500);
    ProcessCommand();
    check_button();  // Check button even when stopped
    return;
  }

  // Initialize diagnostics
  for (int i = 0; i < NN; i++) {
    checks[i] = 0;
  }

  // Read and report battery voltage
  int voltage_raw = analogRead(VOL_MEASURE_PIN);
  double voltage = (voltage_raw*1.1/1024)*((10+1.5)/1.5);
  Serial.print("Battery: ");
  Serial.print(voltage);
  Serial.println("V");


  // Reset wheel encoders and start balancing
  Encoder_init(&left_angle, &right_angle);
  
  run_balancer(launch_duration,
               wheel_angle_gain, wheel_angular_velocity_gain,
               body_angle_gain, body_angular_velocity_gain,
               past_command_gain, 0.0,
               RUN_DURATION, DO_NOT_COLLECT_DATA);

  Serial.println("Balancer stopped");
  go = false;
}
