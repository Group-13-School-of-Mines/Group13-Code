#include <Encoder.h>
#include <Wire.h>
#include <math.h>

// Motor constants
const int D2 = 4;
const int STATUS_FLAG = 12;
const int MOTOR1_SIGN = 7;
const int MOTOR2_SIGN = 8;
const int MOTOR1_VELOCITY = 9;
const int MOTOR2_VELOCITY = 10;

// Timing globals
const int SAMPLE_COUNT = 100;
const int PERIOD = 10; // In milliseconds
unsigned long int time_start = 0; // In milliseconds

// Data globals
long old_position = 0;
double old_angular_position = 0;
double desired_theta = 0.0;

// Feedback globals
const double KP = 1.0;
const double KI = 1.0;

// Encoder
Encoder the_encoder(2, 3);

// Controller
int control_result = 0;
double total_error = 0.0;

int get_control_result(double new_angular_position) {
  double new_error = desired_theta - new_angular_position;
  total_error += new_error;

  double control_signal = (KP * new_error) + KI * (PERIOD / 1000.0) * total_error;
  int control_signal_final = (int) control_signal;

  // Clamp between 0..255
  if (control_signal_final < 0) {
    return 0;
  } else  if (control_signal_final > 255) {
    return 255;
  } else {
    return control_signal_final;
  }
}

unsigned long time_end() {
  return time_start + PERIOD;
}

void calc_stats() {
  long new_position = the_encoder.read();

  if (new_position == old_position) {
    // Don't need to update any stats as nothing has changed
    return;
  }

  old_position = new_position;
  double new_angular_position = (2.0 * PI * new_position) / 3200.0;

  double d_angular_position = new_angular_position - old_angular_position;
  double angular_velocity = (d_angular_position / PERIOD) * 1000.0;

  old_angular_position = new_angular_position;

  Serial.print(time_start);
  Serial.print("\t");
  Serial.print(new_angular_position);
  Serial.print("\t");
  Serial.print(angular_velocity);
  Serial.print("\n");
}

void receive_data_handler(int byte_count) {
  int data;
  
  while (Wire.available()) {
    data = Wire.read();
  }

  total_error = 0.0;

  switch (data) {
  case 1:
    desired_theta = (2.0 * PI) + old_angular_position;
  case 2:
    desired_theta = (1.0 / 2.0 * PI) + old_angular_position;
  case 3:
    desired_theta = (PI) + old_angular_position;
  case 4:
    desired_theta = (3.0 / 2.0 * PI) + old_angular_position;
  case 5:
    {
      long new_position = the_encoder.read();
      double new_angular_position = (2.0 * PI * new_position) / 3200.0;
      new_position = 3200 - (new_position % 3200);
      desired_theta = new_angular_position + (2.0 * PI * new_position) / 3200.0;
    }
  default:
    desired_theta = 0.0;
  }
}

void request_data_handler() {
  // Use mod to normalize
  double position_in_rads = 2.0 * PI * (old_position % 3200) / 3200;
  Wire.write((uint8_t) position_in_rads);
}

void setup() {
  pinMode(D2, OUTPUT);

  pinMode(MOTOR1_SIGN, OUTPUT);
  pinMode(MOTOR2_SIGN, OUTPUT);

  pinMode(MOTOR1_VELOCITY, OUTPUT);
  pinMode(MOTOR2_VELOCITY, OUTPUT);

  pinMode(STATUS_FLAG, INPUT);

  digitalWrite(D2, HIGH);
  digitalWrite(MOTOR1_SIGN, HIGH);
  digitalWrite(MOTOR2_SIGN, HIGH);

  Serial.begin(9600);
  Wire.begin(0x08);
  Serial.println("Performing encoder test...");
  Wire.onReceive(receive_data_handler);
  Wire.onRequest(request_data_handler);
}

void loop() {
  time_start = millis();

  if (time_start < PERIOD * SAMPLE_COUNT) {
    analogWrite(MOTOR2_VELOCITY, 0);
  } else {
    analogWrite(MOTOR2_VELOCITY, control_result);
    desired_theta = 1.0;
  }

  calc_stats();

  control_result = get_control_result(old_angular_position);

  // Wait 
  if (millis() > time_end()) {
    Serial.println("Can't keep up!");
    return;
  }
  
  while (millis() < time_end()) { }
}
