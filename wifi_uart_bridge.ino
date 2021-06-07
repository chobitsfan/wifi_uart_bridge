
/*
  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using the WiFi module.
 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 created 30 December 2012
 by dlf (Metodo2 srl)

 */


#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "ardupilotmega/mavlink.h"

int status = WL_IDLE_STATUS;
#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 14550;      // local port to listen on

#define PKT_BUF_SZ 512
uint8_t packetBuffer[PKT_BUF_SZ]; //buffer to hold incoming packet
uint8_t expectedSeq = 0;
bool got_client = false;

WiFiUDP Udp;

void setup() {
  Serial.begin(9600);
  Serial1.begin(256000);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  IPAddress ip;
  ip.fromString(MY_IP_ADDR);
  WiFi.config(ip);
  WiFi.noLowPowerMode();

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Udp.begin(localPort);
}

void loop() {
  unsigned long cur = millis();
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    got_client = true;
    // read the packet into packetBufffer
    Udp.read(packetBuffer, PKT_BUF_SZ);
    Serial1.write(packetBuffer, packetSize);
  }

  mavlink_message_t msg;
  mavlink_status_t status;
  while (Serial1.available() > 0) {
    if (mavlink_parse_char(0, Serial1.read(), &msg, &status)) {
      if (msg.seq != expectedSeq) {
        Serial.print(msg.seq);
        Serial.print(" expect ");
        Serial.println(expectedSeq);
      }
      if (msg.seq == 255) expectedSeq = 0; else expectedSeq = msg.seq + 1;
      if (got_client) {
        packetSize = mavlink_msg_to_send_buffer(packetBuffer, &msg);
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(packetBuffer, packetSize);
        Udp.endPacket();
      }
    }
  }
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
