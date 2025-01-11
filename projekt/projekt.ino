#include "HardwareSerial.h"
#include <math.h>
#include "Wire.h"
#include "I2C_MPU6886.h"

I2C_MPU6886 imu(0x68, Wire1);
float angle;
uint16_t distance, speed;

#define PWM_A1 13 // Right motor
#define PWM_A2 12 // Right motor
#define PWM_B1 26 // Left motor
#define PWM_B2 25 // Left motor
// Constants for serial communication
#define HEADER 0x54
#define TOTAL_DATA_LENGTH 44

#define MAX_SPEED 800
#define NORMAL_SPEED 500
#define MIN_SPEED 10
#define LOOKAHEAD_DISTANCE 100  // Distance ahead to calculate pathing
#define THRESHOLD 40

uint8_t buffer_left_ready = 0;
uint8_t buffer_right_ready = 0;
uint8_t buffer_straight_ready = 0;

#define BUFFER_SIZE 40
uint16_t buff_left[BUFFER_SIZE] = {0};
int buff_index_left = 0;
uint16_t buff_right[BUFFER_SIZE] = {0};
int buff_index_right = 0;
uint16_t buff_straight[BUFFER_SIZE] = {0};
int buff_index_straight = 0;

uint8_t data[45];
volatile uint8_t bytes_read = 0;

float calculate_effective_distance(uint16_t distance, float angle_offset) {
  float scale_factor = sin(fabs(radians(angle_offset))); // Symmetric scaling
  scale_factor = constrain(scale_factor, 0.5, 1.0); // Avoid overly small scaling
  return distance * scale_factor;
}

static const uint8_t CrcTable[256] = {
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
  0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
  0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
  0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
  0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
  0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
  0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
  0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
  0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
  0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
  0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
  0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
  0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
  0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
  0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
  0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

void setup() {
  Serial.begin(230400);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);

  pinMode(PWM_A1, OUTPUT);
  pinMode(PWM_A2, OUTPUT);
  pinMode(PWM_B1, OUTPUT);
  pinMode(PWM_B2, OUTPUT);

  Wire1.begin(22, 23);
  if (imu.begin()) {
    Serial.println("Failed to detect gyro sensor!");
  } else {
    Serial.printf("WhoAmI() = 0x%02x\n", imu.whoAmI());
  }

  Serial.print("Setup Done");
}

float constrainAngle(float x) {
  x = fmod(x, 360);
  if (x < 0)
    x += 360;
  return x;
}

float calculateStep(float startangle, float endangle, int num_distances) {
  float angular_diff = endangle - startangle;
  return angular_diff / (num_distances - 1);
}

uint8_t CalCRC8(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
  uint8_t crc = 0xD8;
  for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {
    crc = CrcTable[(crc ^ *p++) & 0xff];
  }
  return crc;
}

void read_lidar() {
  uint8_t cs, calculated_crc;
  uint16_t startangle, endangle;
  float endangle_f, startangle_f;
  uint8_t data_length = 44;

  uint8_t byte = Serial2.read();
  if (bytes_read == 0) {
    if (byte == HEADER) {
      bytes_read++;
    } else {
      bytes_read = 0;
      return;
    }
  } else if (bytes_read == 1) {
    if (byte == 0x2C) {
      bytes_read++;
    } else {
      bytes_read = 0;
      return;
    }
  } else if (bytes_read < data_length + 2) {
    data[bytes_read - 2] = byte;
    bytes_read++;
    return;
  } else if (bytes_read == data_length + 2) {
    cs = byte;
    bytes_read = 0;

    calculated_crc = CalCRC8(data, data_length);

    if (calculated_crc != cs) {
      Serial.println("CRC mismatch!");
      return;
    }

    startangle = (data[3] << 8 | data[2]) / 100.0;
    endangle = (data[41] << 8 | data[40]) / 100.0;

    int num_distances = (data_length - 8) / 3;
    float step = calculateStep(startangle, endangle, num_distances);
    for (int i = 0; i < num_distances; i++) {
      uint16_t lsb = data[4 + i * 3];
      uint16_t msb = data[4 + i * 3 + 1];
      uint16_t confidence = data[4 + i * 3 + 2];
      uint16_t distance = ((msb << 8) | lsb) / 10.0;
      if(confidence < 200 && distance > 1200){
        distance = 0;
      }
      
      
      float angle = startangle + step * i;

      if (35 < angle && angle < 105) {
        buff_right[buff_index_right++] = distance;
        if (buff_index_right == BUFFER_SIZE) {
          buff_index_right = 0;
          buffer_right_ready = 1;
        }
      } else if (245 < angle && angle < 345) {
        buff_left[buff_index_left++] = distance;
        if (buff_index_left == BUFFER_SIZE) {
          buff_index_left = 0;
          buffer_left_ready = 1;
        }
      } else if ((angle >= 0 && angle < 10) || (angle > 350 && angle <= 359)) {
        buff_straight[buff_index_straight++] = distance;
        //Serial.printf("straight distance %d\n", distance);
        if (buff_index_straight == BUFFER_SIZE) {
          buff_index_straight = 0;
          buffer_straight_ready = 1;
        }
      }
    }
  }
}


void drive_motor(int speed, int direction, int pin1, int pin2) {
  switch (direction) {
    case 0:  // FORWARD
      analogWrite(pin1, speed);
      analogWrite(pin2, 0);
      break;
    case 1:  // BACKWARD
      analogWrite(pin1, 0);
      analogWrite(pin2, speed);
      break;
    default:
      Serial.print("Function incorrectly used. drive_motor()");
  }
}

void turn_left(int speed) {
  drive_motor(speed, 0, PWM_A1, PWM_A2);
  drive_motor(speed * 0.1, 0, PWM_B1, PWM_B2);
}

void turn_right(int speed) {
  drive_motor(speed, 0, PWM_B1, PWM_B2);
  drive_motor(speed * 0.1, 0, PWM_A1, PWM_A2);
}

void go_straight(int speed) {
  drive_motor(speed, 0, PWM_A1, PWM_A2);
  drive_motor(speed, 0, PWM_B1, PWM_B2);
}

void steer_based_on_buffers() {
  uint32_t sumleft = 0, sumright = 0, sumstraight = 0;
  int valid_left = 0, valid_right = 0, valid_straight = 0;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (buff_left[i] != 0) {
      sumleft += buff_left[i];
      valid_left++;
    }
    if (buff_right[i] != 0) {
      sumright += buff_right[i];
      valid_right++;
    }
    if (buff_straight[i] != 0) {
      sumstraight += buff_straight[i];
      valid_straight++;
    }
  }

  // Weight angles to prioritize central directions
  uint32_t weighted_left = sumleft * 1.2;  // Slightly prioritize left
  uint32_t weighted_right = sumright * 1.2; // Slightly prioritize right
  uint32_t distance_left = valid_left > 0 ? weighted_left / valid_left : 0;
  uint32_t distance_right = valid_right > 0 ? weighted_right / valid_right : 0;
  uint32_t distance_straight = valid_straight > 0 ? sumstraight / valid_straight : 0;

  if (distance_left > distance_right && distance_straight < LOOKAHEAD_DISTANCE) {
    int turn_speed = NORMAL_SPEED + (distance_left - distance_right) / 10; // Increase turn speed based on difference
    turn_left(constrain(turn_speed, NORMAL_SPEED, MAX_SPEED)); // Limit to MAX_SPEED
    Serial.printf("LEFT turn: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
  } else if (distance_right > distance_left && distance_straight < LOOKAHEAD_DISTANCE) {
  int turn_speed = NORMAL_SPEED + (distance_right - distance_left) / 10; // Increase turn speed based on difference
    turn_right(constrain(turn_speed, NORMAL_SPEED, MAX_SPEED));
    Serial.printf("RIGHT turn: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
  } else if(distance_straight > THRESHOLD){
    go_straight(NORMAL_SPEED);
    Serial.printf("STRAIGHT: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
  }
}


void loop() {
  read_lidar();
  if (buffer_left_ready || buffer_right_ready || buffer_straight_ready) {
    buffer_left_ready = 0;
    buffer_straight_ready = 0;
    buffer_right_ready = 0;

    steer_based_on_buffers();
  }
}
