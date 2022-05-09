#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <coap-simple.h>
#include <Wire.h>

#define CMPS12_ADDRESS 0x60

// UDP and CoAP class
WiFiUDP udp;
Coap coap(udp);

const char* ssid     = "Carrinho_1";
const char* password = "554e4b55e8cc57f39f16f335b0a63825";

char angles[256];
unsigned char high_byte, low_byte;

unsigned int port = 5683;
int8_t regis_8b[3];
int16_t regis_16b[6];

double Ang_AcX, Ang_AcZ, Ang_Rot;

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

  WiFi.setAutoReconnect(true); // Reconeta-se à rede quando a mesma estiver disponive;
  WiFi.persistent(true);
}

void getAngles() {
  Ang_AcX = (180 / PI) * atan(regis_16b[3] / sqrt(pow(regis_16b[5], 2) + pow(regis_16b[4], 2)));
  Ang_AcZ = (180 / PI) * atan(regis_16b[4] / sqrt(pow(regis_16b[5], 2) + pow(regis_16b[3], 2)));
}

void read_data_CMPS12() {
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(0);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 31 bytes from the CMPS12
  Wire.requestFrom(CMPS12_ADDRESS, 18);
  while (Wire.available() < 18);       // Wait for all bytes to come back

  regis_8b[0] = Wire.read(); // 0x00 (COMMAND RESITER)
  regis_8b[1] = Wire.read(); // 0x01 (COMPASS BEARING 8 Bit)
  regis_16b[0] = Wire.read() << 8 | Wire.read(); // 0x02 (COMPASS BEARING 8 BITS HIGH) & 0x03 (COMPASS BEARING 8 BITS LOW)
  regis_8b[2] = Wire.read(); // 0x04 (PITCH ANGLE +/-90º)
  regis_8b[3] = Wire.read(); // 0x05 (ROLL ANGLE +/-90º)
  regis_16b[1] = Wire.read() << 8 | Wire.read(); // 0x06 (MAGNETOMETER X AXIS RAW - 8 BITS HIGH) & 0x07 (MAGNETOMETER X AXIS RAW - 8 BITS LOW)
  regis_16b[2] = Wire.read() << 8 | Wire.read(); // 0x08 (MAGNETOMETER Y AXIS RAW - 8 BITS HIGH) & 0x09 (MAGNETOMETER Y AXIS RAW - 8 BITS LOW)
  regis_16b[3] = Wire.read() << 8 | Wire.read(); // 0x0A (MAGNETOMETER Z AXIS RAW - 8 BITS HIGH) & 0x0B (MAGNETOMETER Z AXIS RAW - 8 BITS LOW)
  regis_16b[4] = Wire.read() << 8 | Wire.read(); // 0x0C (ACCELEROMETER X AXIS RAW - 8 BITS HIGH) & 0x0D (ACCELEROMETER X AXIS RAW - 8 BITS LOW)
  regis_16b[5] = Wire.read() << 8 | Wire.read(); // 0x0E (ACCELEROMETER Y AXIS RAW - 8 BITS HIGH) & 0x0F (ACCELEROMETER Y AXIS RAW - 8 BITS LOW)
  regis_16b[6] = Wire.read() << 8 | Wire.read(); // 0x10 (ACCELEROMETER Z AXIS RAW - 8 BITS HIGH) & 0x11 (ACCELEROMETER Z AXIS RAW - 8 BITS LOW)

}

void setup() {
  Serial.begin(115200);  // Start serial port;
  Wire.begin();   // O baud rate do CMPS12 deve ser 9600 (segundo indicado no datasheet);
  connect_to_network();

  coap.start();  // start coap client
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    read_data_CMPS12();
    getAngles();

    sprintf(angles, "{'data':[{'name':'Costas_Frente_Tras', 'data':'%f'},{'name':'Costas_Esq_Dir', 'data':'%f'}]}", Ang_AcX, Ang_AcZ);
    Serial.println(angles);
    coap.put(ip, port, "car-1", angles, strlen(angles));

    coap.loop();
  }
  delay(200);
}
