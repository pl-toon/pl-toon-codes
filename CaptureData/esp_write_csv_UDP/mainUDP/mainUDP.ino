/*
 Name:		testESP_UDP.ino
 Author:	Felipe Villenas
*/

#include <esp_wifi_types.h>
#include <esp_wifi_internal.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <math.h>


#define PAYLOAD_SIZE	3     // numero de doubles
#define Ts            20    // tiempo en ms
#define PI            3.1415926535

/* UDP Settings */
WiFiUDP udp;
uint32_t udp_port = 3333;
const char* server = "192.168.1.100";
double udp_buffer[PAYLOAD_SIZE];

uint32_t t0;
uint32_t k = 0;
double x = 0;   // output

/* MQTT Settings */
const char* ssid = "fvp";
const char* password = "nomeacuerdo";

void setup_wifi_mqtt();

// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	WiFi.mode(WIFI_STA);


	/* ----------- Setup MQTT -------------- */
	setup_wifi_mqtt();
	/* ------------------------------------- */
	udp.begin(WiFi.localIP(), udp_port);

	uint8_t ch; wifi_second_chan_t ch2;
	esp_wifi_get_channel(&ch, &ch2);
	float freq = 2.407 + 0.005 * (int)ch;
	Serial.printf("Conectado al canal Wi-Fi: %d (%.3f GHz)\n", ch, freq);

	Serial.println("------ Setup Completado! ------");
	t0 = millis();
}

// the loop function runs over and over again until power down or reset
void loop() {

	if (millis() - t0 >= Ts) {
    
    int a = k % 2;
    udp_buffer[0] = a;
    udp_buffer[1] = (1 - a)*cos(2*PI*0.005*(double)k) + a*sin(2*PI*0.005*(double)k);
    udp_buffer[2] = (1 - a)*sin(2*PI*0.005*(double)k) + a*cos(2*PI*0.005*(double)k);

	udp.beginPacket(server, udp_port);
    udp.write((uint8_t*)&udp_buffer, sizeof(udp_buffer));     // buffer y el size en bytes
    udp.endPacket();

    t0 = millis();
    k++;

    //Serial.println(temp);
	}

  /* para leer por udp
  int packetSize = udp.parsePacket();
  if(packetSize){
      int len = udp.read(udp_buffer, 32);
      if(len > 0){
          uint32_t tx = 0;
          uint32_t tv = micros();
          memcpy(&tx, udp_buffer, 4);
          rtt_sum += (tv - tx);
          N_rcv++;
      } 
  } */

}

//////////////////////////////////////////////////////////////////////////

void setup_wifi_mqtt() {

	Serial.println("Connecting to " + String(ssid));
	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(3000);
		Serial.print(".");
		WiFi.begin(ssid, password);
	}

	Serial.print("\nWi-Fi Connected!, IP Address: ");
	Serial.println(WiFi.localIP());
}