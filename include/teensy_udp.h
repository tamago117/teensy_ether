#pragma once

/*
this code is udp connection for teensy++ 2.0
 array : [number of int array(1byte)][number of float array(1byte)][int content1(4byte)].....[int contentN(4byte)][float content1(4byte)].....[float contentN(4byte)]
*/
#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <string>
#include <vector>

namespace eth{

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
}; //suitable number, it is unnecessary

union Data{ 
    float f;
    int32_t i; 
};

class teensy_udp
{
private:
    int localPort;
    int remortPort;
    IPAddress loIp;
    IPAddress reIp;
    EthernetUDP Udp;
    int bufferSize;
    char* ReplyBuffer;
    std::string se_message;
    std::vector<std::string> se_message_array;
    std::string int_toString(int32_t Int);
    std::string float_toString(int32_t Float);
    int byte_toInt(char bytearr[4]);
    float byte_toFloat(char bytearr[4]);

public:
    teensy_udp(std::vector<int> localIp, std::vector<int> remortIp, int localPort_, int remortPort_, int bufferSize_=200);
    void advertiseUDP(const std::vector<int>& se_message_i, const std::vector<float>& se_message_f);
    void advertiseUDP(const std::vector<int>& se_message_i);
    void advertiseUDP(const std::vector<float>& se_message_f);
    void subscribeUDP(std::vector<int>& re_message_i, std::vector<int>& re_message_f);
};

teensy_udp::teensy_udp(std::vector<int> localIp, std::vector<int> remortIp, int localPort_, int remortPort_, int bufferSize_=200)
{
    IPAddress loIp(localIp[0], localIp[1], localIp[2], localIp[3]);
    IPAddress reIp(remortIp[0], remortIp[1], remortIp[2], remortIp[3]);
    localPort = localPort_;
    remortPort = remortPort_;
    bufferSize = bufferSize_;
    ReplyBuffer[bufferSize];

    Ethernet.init(20);
    Ethernet.begin(mac, loIp);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true) {
          delay(1); // do nothing, no point running without Ethernet hardware
        }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }

    // start UDP
    Udp.begin(localPort);
}

std::string teensy_udp::int_toString(int32_t Int){
    char Byte[4];
    Byte[0] = (char)((Int >> 24) & 0xFF);
    Byte[1] = (char)((Int >> 16) & 0xFF);
    Byte[2] = (char)((Int >>  8) & 0xFF);
    Byte[3] = (char)((Int >>  0) & 0xFF);

    return Byte;
}

//shift operator can't use float,double.
//So this problem is able to solve by union.
//union assign variables to one memory.
//on one memory : int32_t = float
std::string teensy_udp::float_toString(int32_t Float){
    char Byte[4];
    Byte[0] = (char)((Float >> 24) /*& 0xFF*/);
    Byte[1] = (char)((Float >> 16) /*& 0xFF*/);
    Byte[2] = (char)((Float >>  8)/* & 0xFF*/);
    Byte[3] = (char)((Float >>  0)/* & 0xFF*/);

    return Byte;
}

int teensy_udp::byte_toInt(char bytearr[4]){
  return (int32_t) (
            ((((int32_t)(bytearr[0])) << 24) & 0xFF000000)
          | ((((int32_t)(bytearr[1])) << 16) & 0x00FF0000)
          | ((((int32_t)(bytearr[2])) <<  8) & 0x0000FF00)
          | ((((int32_t)(bytearr[3])) <<  0) & 0x000000FF));
}

float teensy_udp::byte_toFloat(char bytearr[4]){
  return (float) (
            ((((int32_t)(bytearr[0])) << 24) & 0xFF000000)
          | ((((int32_t)(bytearr[1])) << 16) & 0x00FF0000)
          | ((((int32_t)(bytearr[2])) <<  8) & 0x0000FF00)
          | ((((int32_t)(bytearr[3])) <<  0) & 0x000000FF));
}

void teensy_udp::advertiseUDP(const std::vector<int>& se_message_i, const std::vector<float>& se_message_f)
{
    se_message.clear();
    //number of int array
    std::string intNumber(1, (char)se_message_i.size());
    se_message += intNumber;
    //number of float array
    std::string floatNumber(1, (char)se_message_f.size());
    se_message += floatNumber;

    //int
    for(const auto& message : se_message_i){
        se_message += int_toString(message);
    }
    //float
    union Data data;
    for(const auto& message : se_message_f){
        data.f = message;
        se_message += float_toString(data.i);
    }
    Udp.beginPacket(reIp, remortPort);
    //Udp.write(se_message.c_str());
    Udp.write("abcde");
    Udp.endPacket();

    char Byte[4];
    for(int j=0; j<4; j++){
        Byte[j] = se_message.c_str()[j+2];
        //Serial.print((int8_t)se_message.c_str()[j+2]);
    }
    Serial.println("");
    

    //Serial.println(se_message.c_str());
    //Serial.println((int8_t)se_message.c_str()[0]);
    Serial.println(byte_toInt(Byte));
    //Serial.println((int32_t)Byte);
}

void teensy_udp::advertiseUDP(const std::vector<int>& se_message_i)
{
    se_message.clear();
    //number of int array
    std::string intNumber(1, (char)se_message_i.size());
    se_message += intNumber;
    //number of float array
    std::string floatNumber(1, (char)0);
    se_message += floatNumber;

    //int
    for(const auto& message : se_message_i){
        se_message += int_toString(message);
    }
    Udp.beginPacket(reIp, remortPort);
    Udp.write(se_message.c_str());
    Udp.endPacket();
}

void teensy_udp::advertiseUDP(const std::vector<float>& se_message_f)
{
    se_message.clear();
    //number of int array
    std::string intNumber(1, (char)0);
    se_message += intNumber;
    //number of float array
    std::string floatNumber(1, (char)se_message_f.size());
    se_message += floatNumber;

    //float
    union Data data;
    for(const auto& message : se_message_f){
        data.f = message;
        se_message += float_toString(data.i);
    }
    Udp.beginPacket(reIp, remortPort);
    Udp.write(se_message.c_str());
    Udp.endPacket();
}

void teensy_udp::subscribeUDP(std::vector<int>& re_message_i, std::vector<int>& re_message_f)
{
    if (Udp.parsePacket()){
        re_message_i.clear();
        re_message_f.clear();

        Udp.read(ReplyBuffer, bufferSize);
        int iNumber = (int8_t)ReplyBuffer[0];
        int fNumber = (int8_t)ReplyBuffer[1];
        
        //int
        for(int i=0; i<iNumber; i++){
            char Byte[4];
            for(int j=0; j<4; j++){
                Byte[j] = ReplyBuffer[i*4+j+2];
            }
            re_message_i.push_back(byte_toInt(Byte));
        }
        //float
        for(int i=iNumber; i<iNumber+fNumber; i++){
            char Byte[4];
            for(int j=0; j<4; j++){
                Byte[j] = ReplyBuffer[i*4+j+2];
            }
            re_message_f.push_back(byte_toInt(Byte));
        }
    }
    
}



}//end namespace eth