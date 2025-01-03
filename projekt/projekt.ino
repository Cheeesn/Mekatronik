#include "HardwareSerial.h"
#include <math.h>
#include "Wire.h"
#include "I2C_MPU6886.h"

I2C_MPU6886 imu(0x68, Wire1);
float angle;
uint16_t distance,speed;

#define PWM_A1 13 // Right motor
#define PWM_A2 12 // Right motor
#define PWM_B1 26 //Left motor
#define PWM_B2 25 // Left motor
// Constants for serial communication
#define HEADER 0x54
#define TOTAL_DATA_LENGTH 48

#define MAX_SPEED 150
#define MIN_SPEED 10
#define MAX_DISTANCE 30  // Maximum distance to consider speed adjustments
#define THRESHOLD 20

volatile uint8_t buffer_ready = 0;
int last_turn = 0;

#define BUFFER_SIZE 14 
uint16_t buff_left[BUFFER_SIZE] = {0};
int buff_index_left = 0;
uint16_t buff_right[BUFFER_SIZE] = {0};
int buff_index_right = 0;
uint16_t buff_straight[BUFFER_SIZE] = {0};
int buff_index_straight = 0;

volatile uint8_t data[48];
volatile uint8_t bytes_read = 0;

int calculate_speed(uint16_t distance) {
  if (distance > MAX_DISTANCE) return MAX_SPEED;  // No obstacle, full speed
  
  // Linearly scale speed based on distance
  int speed = MIN_SPEED + 2 * distance;
  return constrain(speed, MIN_SPEED, MAX_SPEED);
}
static const uint8_t CrcTable[] =
{
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

volatile size_t sent_bytes = 0, received_bytes = 0;

// void onReceiveFunction(void) {
//   // This is a callback function that will be activated on UART RX events
//   size_t available = Serial2.available();
//   received_bytes = received_bytes + available;
//   Serial.printf("onReceive Callback:: There are %d bytes available: ", available);
//   while (available--) {
//     read_lidar();
//   }
//   Serial.println();
// }

void setup() {
  Serial.begin(230400);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);
  //Serial2.onReceive(onReceiveFunction, false);  // sets a RX callback function for Serial 1

  pinMode(PWM_A1, OUTPUT);
  pinMode(PWM_A2, OUTPUT);
  pinMode(PWM_B1, OUTPUT);
  pinMode(PWM_B2, OUTPUT);

  
  while (!Serial);

  Wire1.begin(22, 23);
  if (imu.begin()) {
    Serial.println("Failed to detect gyro sensor!");
  } else {
    Serial.printf("WhoAmI() = 0x%02x\n", imu.whoAmI());
  }


  Serial.print("Setup Done");
}
float constrainAngle(float x){
  x = fmod(x,360);
  if (x < 0)
      x += 360;
  return x;
}

float calculateStep(float startangle, float endangle, int num_distances) {
  // Calculate angular difference (handling wrap-around at 360 degrees)
  float angular_diff = endangle - startangle;
  // Calculate step size
  return angular_diff / (num_distances - 1);
}
uint8_t CalCRC8(volatile uint8_t *p, uint8_t len)
{
 uint8_t crc = 0;
 

  for (int i = 0; i < len; i++){
  crc = CrcTable[(crc ^ *(p++)) & 0xff];
  }
  return crc;
}

void read_lidar() {
  uint8_t data_length, cs, calculated_crc;
  uint16_t speed, startangle, endangle, timestamp;
  uint16_t buffer[48];  // Buffer to hold data (adjust size based on your need)
  float endangle_f, startangle_f;
  uint8_t byte = Serial2.read();  // Read first byte
  
  if(bytes_read == 0){
    if (byte == HEADER) {
      // Read the data length byte
      data[bytes_read++] = byte;
    }
    else
    {
      bytes_read = 0;
      return;
    }
  }
  else if(bytes_read==1){
    if(byte == 0x2c){
      data[bytes_read++] = byte; 
      data_length = byte;
    }
    else{
      bytes_read = 0;
    }
  }
  else if(bytes_read < 47) {
    data[bytes_read++] = byte; 
    return;
  }
  else if(bytes_read == 47){
    cs = data[data_length-1];

    bytes_read = 0;
    // Calculate the CRC for the data (excluding the checksum byte)
    calculated_crc = CalCRC8(data, data_length - 1);

    // Compare the calculated CRC with the received checksum
    if (calculated_crc != cs) {
      //Serial.println("CRC mismatch!");
      buffer_ready = 0;
      
      return; // Skip frame if CRC mismatch
    }
    buffer_ready = 1;
  }  

    for (int i = 0; i < data_length; i++) {
      //Serial.printf("byte:%d %x\n", i,data[i]);
    }

    // Read speed (2 bytes)
    uint8_t speedbytes[2];
    speedbytes[0] = data[2];
    speedbytes[1] = data[3];
    speed = (speedbytes[1] << 8) | speedbytes[0];  // Convert to speed value

    // Read start angle (2 bytes)
    uint8_t startanglebytes[2];
    startanglebytes[0] = data[4];
    startanglebytes[1] = data[5];
    startangle = (startanglebytes[1] << 8) | startanglebytes[0];
    
    // Read distances (3 bytes per distance)
    int data_index = 6;  // Starting index for distance data in the buffer
    int num_distances = (data_length - 6) / 3; // 3 bytes per distance (LSB, MSB, confidence)

    for (int i = 0; i < num_distances; i++) {
      uint16_t lsb = data[data_index++];
      uint16_t msb = data[data_index++];
      uint8_t confidence = data[data_index++];
      
      distance = (msb << 8) | lsb;  // Combine MSB and LSB to get the full distance value
      
      // Apply confidence threshold (example: only accept values with confidence > 150)
      if (confidence < 200 || distance > 1200) { // Retry reading this distance if confidence is low
        buffer[i] = 0;
        continue;
      }
      distance = distance/10.0;
      // Store the distance in the buffer or process it as needed
      buffer[i] = distance;
    }

    // Read end angle (2 bytes)
    uint8_t endanglebytes[2];
    endanglebytes[0] = data[data_index++];
    endanglebytes[1] = data[data_index++];
    endangle = (endanglebytes[1] << 8) | endanglebytes[0];

    // Read timestamp (2 bytes)
    uint8_t timestampbytes[2];
    timestampbytes[0] = data[data_index++];
    timestampbytes[1] = data[data_index++];
    timestamp = (timestampbytes[1] << 8) | timestampbytes[0];
    endangle_f = endangle / 100.0;
    startangle_f = startangle/100.0;
    if ((startangle_f > 270 && startangle_f <= 90) && (endangle_f > 270 && endangle_f <= 90)) {
        // The range is behind the radar, skip processing
        return;
    }

    double step = calculateStep(startangle_f, endangle_f, num_distances);
    buff_index_right = 0;
    buff_index_left = 0;
    buff_index_straight = 0;
    for (int i = 0; i < num_distances; i++) {
      if(buffer[i] == 0){
        continue;
      }
      angle = startangle_f + step*i;
      
      //Serial.printf("startangle %d endangle %d num %d\n",startangle, endangle, num_distances);
      //Serial.printf("angle: %f distance: %d \n", angle, buffer[i]);
      if ((angle > 350 && angle <= 360) || (angle >= 0 && angle < 10)){
        buff_straight[buff_index_straight++] = buffer[i];
        /*if(buff_index_straight == BUFFER_SIZE){
          buff_index_straight = 0;
        }*/
      }
      else if(15 < angle && angle < 105){
        buff_right[buff_index_right++] = buffer[i];
        /*if(buff_index_right == BUFFER_SIZE){
          buff_index_right = 0;
        }*/
        //Serial.printf("angle: %f distance: %d Left turn turn \n ", angle, buffer[i]);
      }
      else if(245 < angle && angle < 345){
        buff_left[buff_index_left++] = buffer[i];
        /*if(buff_index_left == BUFFER_SIZE){
          buff_index_left = 0;
        }*/
        //Serial.printf("angle: %f distance: %d Right turn\n", angle, buffer[i]);
      }

      //Serial.printf("i: %d angle: %f distance: %d startangle: %d endangle: %d\n", i, angle, buffer[i], startangle, endangle);
    }
    
  
  
  // Optionally print or process other data fields
}

void read_angle(float* roll, float* pitch){
  float ax = 0;
  float ay = 0;
  float az = 0;
  imu.getAccel(&ax, &ay, &az);
  *roll = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / M_PI);
  *pitch = atan2(-ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI);
}

void drive_motor(int speed, int direction, int pin1, int pin2) {
  
  switch (direction) {
    case 0:  //FORWARDS
      analogWrite(pin1, speed);
      analogWrite(pin2, 0);
      break;
    case 1:  //BACKWARDS
      analogWrite(pin1, 0);
      analogWrite(pin2, speed);
      break;
    default:
      Serial.print("Function incorrectly used. drive_motor()");
  }
}

void turn_left(int speed){
  drive_motor(speed, 0,PWM_A1,PWM_A2);
  drive_motor(speed*0.6, 0,PWM_B1,PWM_B2);
}
void turn_right(int speed){
  drive_motor(speed, 0,PWM_B1,PWM_B2);
  drive_motor(speed*0.6, 0,PWM_A1,PWM_A2);
}
void go_straight(int speed){
  drive_motor(speed, 0,PWM_A1,PWM_A2);
  drive_motor(speed, 0,PWM_B1,PWM_B2);
}
void go_backwards(int speed){
  drive_motor(speed, 1,PWM_A1,PWM_A2);
  drive_motor(speed, 1,PWM_B1,PWM_B2);
}

// In the main loop, read the frame and process it
void loop() {
  /*if(currentTime >= Timer_read_IMU)
  {
    float roll = 0;
    float pitch = 0;
    read_angle(&roll, &pitch);
    Serial.printf("ROLL=%f,PITCH=%f\n", roll, pitch);
    Timer_read_IMU = millis() + 1000;
  }*/

  read_lidar();

  if(buffer_ready){
    buffer_ready = 0;

    uint32_t sumleft = 0;
    uint32_t sumright = 0;
    uint32_t sumstraight = 0;
    for(uint32_t i = 0; i<BUFFER_SIZE; i++){
      sumleft += buff_left[i];
      sumright += buff_right[i];
      sumstraight += buff_straight[i];
    }
    
    uint32_t distance_left = sumleft / BUFFER_SIZE;
    uint32_t distance_right = sumright / BUFFER_SIZE;
    uint32_t distance_straight = sumstraight / BUFFER_SIZE;


    if(abs((int32_t)(distance_left - distance_right)) >= THRESHOLD){
      if(distance_left < distance_right){
        turn_right(MAX_SPEED);
        memset(buff_right, 0 , sizeof(buff_right));
        Serial.printf("right turn: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
        last_turn = 0;
      }
      else if(distance_left > distance_right){
        turn_left(MAX_SPEED);
        memset(buff_left, 0 , sizeof(buff_left));
        Serial.printf("left turn: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
        last_turn = 1;
      }
    }
    // else if(distance_straight < 10){
    //   go_backwards(MAX_SPEED);
    //   memset(buff_straight, 0 , sizeof(buff_straight));
    //   Serial.printf("backwards: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
    // }
    else if(distance_straight > 30){
      go_straight(MAX_SPEED);
      memset(buff_straight, 0 , sizeof(buff_straight));
      Serial.printf("straight: left: %d right: %d straight: %d\n", distance_left, distance_right, distance_straight);
      last_turn = 2;
    }
    
    delay(400);
    go_straight(0);
    Serial.println("stop:");
    delay(200);
  }
}
