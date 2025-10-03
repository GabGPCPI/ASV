// Software: usv push boat
// Autor: Emerson Andrade
// Date: Jul 2022
// Version: 1.0.6

/*
ESP8266 GPIO pin numbers:
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;
*/

#include <WiFiUdp.h>
#include <ESP8266WiFi.h>

const char* ssid = "swarm"; //Enter your wifi network SSID
const char* password = "loc123456"; //Enter your wifi network password

//const char* ssid = "iphonemrson"; //Enter your wifi network SSID
//const char* password = "bitcoin10"; //Enter alphayour wifi network password

IPAddress CONTROL_IP = IPAddress(10,122,116,148);

const int SERVER_PORT = 2222;
const int BAUD_RATE = 115200;

byte incomingByte = 0;
byte packetBuffer[512];
//char[100] bufferReply;

//char robot_ip[] = "";
IPAddress robot_ip;
String robot_ID = "08";

char data [11];
byte byte_received;
int flag;
int cont;
char dig0,dig1,dig2,dig3,dig4,dig5,dig6;
char dig7,dig8,dig9,dig10;


// motor reduction (from 4mm to 21mm of diam.) = 5.25:1
// motor min and max output rpm = [0 , 9600] RPM at 6V
int main_motor_rpm = 0; // estimated rpm value based on the sensor
int main_motor_rpm_target = 2500; // from 1000 (390 pwm) to 3950 (1023 pwm) (considering the reduction)
int main_motor_pwm_input = 0; // pwm input given to the controller (0 to 1023)
const int main_motor_pwm_KP_gain_numerator = 1; // pwm numerator gain used in a proportional way
const int main_motor_pwm_KP_gain_denominator = 5; // pwm denominator gain used in a proportional way

int auxiliary_motor_pwm_input = 0; // pwm input given to the controller (0 to 1023)

int first_turn_on_main_motor_flag = 0;
const int first_turn_on_main_motor_pwm = 800;

WiFiUDP udp;
WiFiServer server(80);

// rele
int rele_pin = 13;

// RPM reading
int rpm_pin = 12;
volatile byte pulses;
unsigned long timeold;
unsigned long timeold_udp;
unsigned long dt;
void IRAM_ATTR pulses_counter(); //ICACHE_RAM_ATTR or IRAM_ATTR

void pulses_counter()
{
  pulses++;
}

void connectWifi() {
  // connecting to WIFI network
  Serial.print("Connecting to WIFI network");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
  robot_ip = WiFi.localIP();
}

// Set your Static IP Address Settings
//IPAddress local_IP(192, 168, 30, 50);
//IPAddress gateway(192, 168, 1, 1);
//IPAddress subnet(255, 255, 0, 0);
//IPAddress primaryDNS(8, 8, 8, 8); // this is optional
//IPAddress secondaryDNS(8, 8, 4, 4); // this is optional

void setup() {
  Serial.begin(BAUD_RATE);

  analogWriteRange(1023); // define the PWM range

  //Define os pinos de controle do motor como saida
  pinMode(5, OUTPUT); // saída A- PWM
  pinMode(0, OUTPUT); // saída A+ DIR
  pinMode(4, OUTPUT); // saída B- PWM
  pinMode(2, OUTPUT); // saída B+ DIR

  pinMode(rpm_pin, INPUT); //INPUT_PULLUP
  pinMode(rele_pin, OUTPUT);
  digitalWrite(rele_pin, LOW);
  attachInterrupt(digitalPinToInterrupt(rpm_pin), pulses_counter, FALLING);
  pulses = 0;
  timeold = 0;
  
  delay(10);
  // Print feedback if the settings are not configured
//  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
//    Serial.println("STA Failed to configure");
//  }
  WiFi.mode(WIFI_STA);//Define o ESP8266 como Station.
  connectWifi();
  server.begin();
  udp.begin(SERVER_PORT);
  Serial.println("UDP and server ok!");
  
  digitalWrite(rele_pin, HIGH);
}

void loop() {

  ////////////////////////////////////////////////////////////////////////
  // sending the IP address
  ////////////////////////////////////////////////////////////////////////
  if (millis() - timeold_udp >= 10000){
    Serial.print("Sending IP:");
    Serial.println(robot_ip);
    udp.beginPacket(CONTROL_IP, 9543);
    char finalChar[80];
    sprintf(finalChar, "%s,%s", robot_ID, robot_ip.toString().c_str());
    udp.write(finalChar);
    udp.endPacket();
    timeold_udp = millis();
  }
    
  ////////////////////////////////////////////////////////////////////////
  // updates the RPM value
  ////////////////////////////////////////////////////////////////////////
  if (millis() - timeold >= 1000) {
    detachInterrupt(digitalPinToInterrupt(rpm_pin));
    dt = millis() - timeold;
    main_motor_rpm = (main_motor_rpm + ((pulses*6000) / dt)) / 2;
    timeold = millis();
    Serial.print("\tPul=");
    Serial.print(pulses);
    Serial.print("\tdt = ");
    Serial.print(dt);
    Serial.print("\tRPM = ");
    Serial.print(main_motor_rpm, DEC);
    Serial.print("\tRPMT = ");
    Serial.print(main_motor_rpm_target, DEC);
    Serial.print("\tPWM = ");
    Serial.println(main_motor_pwm_input, DEC);
    pulses = 0;
    attachInterrupt(digitalPinToInterrupt(rpm_pin), pulses_counter, FALLING);

    ////////////////////////////////////////////////////////////////////////
    // reduces the difference between the PWM and its target
    ////////////////////////////////////////////////////////////////////////
    if (main_motor_rpm<main_motor_rpm_target){
      if (first_turn_on_main_motor_flag==0){
        main_motor_pwm_input = first_turn_on_main_motor_pwm;
        first_turn_on_main_motor_flag = 1;
      }
      else {
        main_motor_pwm_input = main_motor_pwm_input + ((main_motor_rpm_target-main_motor_rpm)*main_motor_pwm_KP_gain_numerator)/main_motor_pwm_KP_gain_denominator;
        if (main_motor_pwm_input>1023){main_motor_pwm_input=1023;}
      }
      analogWrite(5, main_motor_pwm_input);
      digitalWrite(0, HIGH);
    }
    else {
      main_motor_pwm_input = main_motor_pwm_input - ((main_motor_rpm-main_motor_rpm_target)*main_motor_pwm_KP_gain_numerator)/main_motor_pwm_KP_gain_denominator;
      if (main_motor_pwm_input>1023){main_motor_pwm_input=1023;}
      analogWrite(5, max(main_motor_pwm_input, 400)); // lower value
      digitalWrite(0, HIGH);
    }
  }
  
  int noBytes = udp.parsePacket();
  //String received_command = "";
  
  flag=1; // activate the reading
  if ( noBytes ) {
    
    /*Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());*/

    udp.read(packetBuffer,noBytes);
    /*Serial.println();
    Serial.println(packetBuffer[0]);
    incomingByte = packetBuffer[0];
    Serial.println();
    byte_received = packetBuffer[0]; //udp.read();
    Serial.println(byte_received,HEX);*/
    
    for (int i=1;i<=noBytes;i++){
      Serial.print((char) packetBuffer[i-1]);
      if(packetBuffer[i] == '{'){
      cont=0;
      }
      if(cont <=11){
        data[cont] = byte_received;
        cont+=1;
      }
      
      // {-10231023} auxiliary_motor (-1023) + main_motor (1023) 
      // left/right; motor side; motor forward
      dig0 = packetBuffer[0];
      dig1 = packetBuffer[1];
      dig2 = packetBuffer[2];   dig3 = packetBuffer[3];   dig4 = packetBuffer[4];   dig5 = packetBuffer[5];
      dig6 = packetBuffer[6];   dig7 = packetBuffer[7];   dig8 = packetBuffer[8];   dig9 = packetBuffer[9];
      dig10 = packetBuffer[10];
      
      if(dig10 == '}'){
        flag=0;
      }
      if(dig0=='{' && dig10 =='}'){
        String sv1;
        String sv2;
        for(int x = 2; x<=5; x++){sv2 += (char) packetBuffer[x];}
        for(int x = 6; x<=9; x++){sv1 += (char) packetBuffer[x];}
        main_motor_rpm_target = sv1.toInt();
        auxiliary_motor_pwm_input = sv2.toInt();

        Serial.println(main_motor_rpm_target);
        Serial.print(auxiliary_motor_pwm_input);

        if (dig1=='+'){
          analogWrite(4, auxiliary_motor_pwm_input);
          digitalWrite(2, HIGH);
          }
        else {
          analogWrite(4, auxiliary_motor_pwm_input);
          digitalWrite(2, LOW);
          }
      }
    }
  }   
}
