/*
 teensyUDP
 this code is udp connection for teensy++ 2.0
 
 */

#include <Arduino.h>
#include <teensy_udp.h>
#include <vector>


std::vector<int> localIp{192, 168, 1, 176};
std::vector<int> remortIp{192, 168, 1, 177};

unsigned int localPort = 8887;      // local port to listen on
unsigned int remortPort = 8888;



void setup() {
  Serial.begin(115200);
  
}

eth::teensy_udp udp(localIp, remortIp, localPort, remortPort, 100);

void loop() {
  std::vector<int> intData{0,0};
  std::vector<float> floatData{1, 0, 0, 0};
  
  udp.advertiseUDP(intData, floatData);
  delay(10);
}


