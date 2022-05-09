#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <ArduinoJson.h>
#include <Servo.h>

#define channel 1
#define ssid_hidden 0
#define max_connection 1

#define PWM_DIR 13

#define PWM_VEL 19

#define INA 17
#define INB 18
#define EN 16



const char* ssid     = "Carrinho_1";
const char* password = "554e4b55e8cc57f39f16f335b0a63825";

// UDP and CoAP class
WiFiUDP udp;
Coap coap(udp);

Servo myservo;

// Allocate the JSON document
StaticJsonDocument<200> json;

// CoAP server endpoint url callback
void callback_car1(CoapPacket &packet, IPAddress ip, int port);

// CoAP server endpoint URL
void callback_car1(CoapPacket &packet, IPAddress ip, int port) {
  // send response
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(json, p);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  int vel = (1023/90)*(-round(json["data"][0]["data"]));
  byte dir = -round(json["data"][1]["data"]) + 85;


  // Limitador de Guinada
  if (dir < 60) // Direita
    dir = 60;
  else if (dir > 122) // Esquerda
    dir = 122;

  // DIR
  myservo.write(dir);
  //Serial.println(dir);
  //Serial.println(vel);
  
  // vel
  if(vel <= -180 || vel >= 180){
    digitalWrite(EN, HIGH);
    if(vel > 0){
      digitalWrite(INA, HIGH); 
      digitalWrite(INB, LOW);
      ledcWrite(3, vel);
    }else if(vel < 0){
      digitalWrite(INA, LOW); 
      digitalWrite(INB, HIGH);
      ledcWrite(3, -vel);
    }
  }else{
      digitalWrite(EN, LOW);
  }
}

IPAddress local_IP(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

void setup()
{
  Serial.begin(115200);

  pinMode(PWM_VEL, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(EN, OUTPUT);

  ledcAttachPin(PWM_VEL, 3); //Speed control of Motor
  ledcSetup(3, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password, channel, ssid_hidden, max_connection);
  Serial.println("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  Serial.println("Setup Callback Device Man");
  coap.server(callback_car1, "car-1");
  
  coap.start();
  myservo.attach(PWM_DIR);
}

bool state;

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;

void loop()
{
  startMillis = millis();
  while(!coap.loop()){
    if (millis() - startMillis <= period)  //test whether the period has elapsed
    {
      digitalWrite(EN, LOW);
    }
  }

  delay(200);
}