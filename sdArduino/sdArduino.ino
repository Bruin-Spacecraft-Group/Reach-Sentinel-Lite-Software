#include <SoftwareSerial.h>

// 1421 Right
// 1411 Left 

struct datapacket {
  unsigned long timestamp;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float GPS_latitude;
  float GPS_longitude;
  float GPS_altitude;
  uint8_t GPS_hour;
  uint8_t GPS_minute;
  uint8_t GPS_seconds;
  float temp_tempC; // TODO: remove or keep depending on if the Adafruit_MCP9808 sensor is used
  float baro_pressure; // in mmHg
  float baro_altitude; // in meters
  float baro_tempC;
} currentPacket;

SoftwareSerial mySerial(3, 2); // RX, TX
byte tx_buf[sizeof(datapacket)] = {0};

String unpack_packet(uint8_t* recvbuf) {
     memcpy(&currentPacket, recvbuf, sizeof(currentPacket));
     String ret;
     ret += String(currentPacket.timestamp) + ",";
     ret += String(currentPacket.accel_x) + "," + String(currentPacket.accel_y) + "," + String(currentPacket.accel_z) + ",";
     ret += String(currentPacket.gyro_x) + "," + String(currentPacket.gyro_y) + "," + String(currentPacket.gyro_z) + ",";
     ret += String(currentPacket.GPS_latitude) + "," + String(currentPacket.GPS_longitude) + "," + String(currentPacket.GPS_altitude) + ",";
     ret += String(currentPacket.GPS_hour) + "," + String(currentPacket.GPS_minute) + "," + String(currentPacket.GPS_seconds) + ",";
     ret += String(currentPacket.temp_tempC) + ",";
     ret += String(currentPacket.baro_pressure) + "," + String(currentPacket.baro_altitude) + "," + String(currentPacket.baro_tempC);
     return ret;
}

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial.println("SD Script:");
  Serial.println(sizeof(datapacket));
}

int i = 0;
byte rx_buf[sizeof(datapacket)] = {0};

void loop() {
  if(mySerial.available()) {
    char c = mySerial.read();    
    if (c == '\n') {
      Serial.println(unpack_packet(rx_buf));
      i = 0;
    } else {
      rx_buf[i] = c;
      i++;
    }
    //Serial.print(c);
  }
}
