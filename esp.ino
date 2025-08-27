#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// === Настройки Wi-Fi ===
const char* ssid = "MikroTik-32182D";
const char* password = "";

// === Статичный IP ESP ===
IPAddress localIP(192, 168, 88, 222); //192.168.88.223
IPAddress gateway(192, 168, 88, 1);
IPAddress subnet(255, 255, 255, 0);

// === UDP ===
WiFiUDP Udp;
const unsigned int localUdpPort = 4312;   // порт для приёма на ESP (с ПК)
const IPAddress pcIP(192, 168, 88, 228);   // IP ПК
const unsigned int pcUdpPort = 4211;      // порт ПК, куда ESP шлёт данные

// === Буферы ===
uint8_t uartBuffer[64];
uint8_t uartIndex = 0;
const uint8_t expectedPacketSize = 12;

void setup() {
  Serial.begin(115200);  // UART для связи с Mega 
  delay(1000);

  // Статический IP
  WiFi.config(localIP, gateway, subnet);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Udp.begin(localUdpPort);  // Порт для приёма с ПК

  Serial.println("\nWiFi подключено!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // === 1. Приём UDP от ПК → Отправка в UART (в Mega) ===
  int packetSize = Udp.parsePacket();
  if (packetSize == expectedPacketSize) {
    uint8_t buf[expectedPacketSize];
    int len = Udp.read(buf, sizeof(buf));

    if (len == expectedPacketSize && buf[0] == 0xAA) {
      Serial.write(buf, expectedPacketSize);
    }
  }

  // === 2. Приём UART от Mega → Отправка в UDP (на ПК) ===
  while (Serial.available()) {
    uint8_t byteIn = Serial.read();

    uartBuffer[uartIndex++] = byteIn;

    if (uartIndex == expectedPacketSize) {
      Udp.beginPacket(pcIP, pcUdpPort);
      Udp.write(uartBuffer, expectedPacketSize);
      Udp.endPacket();

      uartIndex = 0;
    }
  }
}
