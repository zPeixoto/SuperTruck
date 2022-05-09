#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <SoftwareSerial.h>

// UDP and CoAP class
WiFiUDP udp;
Coap coap(udp);

SoftwareSerial CMPS12(2, 3); // RX TX

const char* ssid     = "Carrinho_1";
const char* password = "554e4b55e8cc57f39f16f335b0a63825";

char angles[256];
unsigned char high_byte, low_byte;

unsigned int port = 5683;
int flagDir = 0;
int16_t regis_16b[6];

int Ang_AcX, Ang_AcZ;

IPAddress ip(192, 168, 10, 1);

void connect_to_network() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);   // Connect to WiFi

  while (WiFi.status() != WL_CONNECTED) {
    yield();
    Serial.print(".");
    delay(100);
  }

  Serial.println("");
  Serial.println("WiFi connected");

  Serial.println(WiFi.localIP()); // Print the IP address of client;
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());

  WiFi.setAutoReconnect(true); // Reconeta-se à rede quando a mesma estiver disponivel;
  WiFi.persistent(true);
}

void getAngles() {
  Ang_AcX = round( (180 / PI) * atan(regis_16b[3] / sqrt(pow(regis_16b[5], 2) + pow(regis_16b[4], 2))) );
  Ang_AcZ = round( (180 / PI) * atan(regis_16b[4] / sqrt(pow(regis_16b[5], 2) + pow(regis_16b[3], 2))) );
}

void read_data_CMPS12() {
  
  // Request and read Bearing (16 bit = 2 bytes);
  CMPS12.write(0x13);   
  while (CMPS12.available() < 2);
  high_byte = CMPS12.read();
  low_byte = CMPS12.read();
  regis_16b[0] = high_byte;           
  regis_16b[0] <<= 8;
  regis_16b[0] += low_byte;

  // Request and read Raw accelerometer data, 16 bit signed: X high, X low, Y high, Y low, Z high, Z low;
  CMPS12.write(0x20);  
  while (CMPS12.available() < 6);
  high_byte = CMPS12.read();
  low_byte = CMPS12.read();
  
  // X ACELL
  regis_16b[3] = high_byte;      
  regis_16b[3] <<= 8;
  regis_16b[3] += low_byte;

  // Y ACELL
  high_byte = CMPS12.read();
  low_byte = CMPS12.read();
  regis_16b[4] = high_byte;      
  regis_16b[4] <<= 8;
  regis_16b[4] += low_byte;

  // Z ACELL
  high_byte = CMPS12.read();
  low_byte = CMPS12.read();
  regis_16b[5] = high_byte;      
  regis_16b[5] <<= 8;
  regis_16b[5] += low_byte;

}

// CoAP client response callback
void callback_response(CoapPacket &packet, IPAddress ip, int port) {
  Serial.println("[Coap Response got]");
  
  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;
  
  Serial.println(p);
}

void setup() {
  Serial.begin(115200);  // Start serial port;
  CMPS12.begin(9600);    // O baud rate do CMPS12 deve ser 9600 (segundo indicado no datasheet);
  connect_to_network();
  
  coap.response(callback_response);

  coap.start();  // start coap client
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {

    read_data_CMPS12();
    getAngles();

    sprintf(angles, "{'data':[{'name':'Costas_Frente_Tras', 'data':'%d'},{'name':'Costas_Esq_Dir', 'data':'%d'}]}", Ang_AcX, Ang_AcZ);
    coap.put(ip, port, "car-1", angles, strlen(angles));
    
    coap.loop();
  }
  delay(200);
}
