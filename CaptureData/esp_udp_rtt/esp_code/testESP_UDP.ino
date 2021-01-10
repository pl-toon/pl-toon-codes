/*
 Name:		testESP_UDP.ino
 Created:	10/31/2020 6:52:51 PM
 Author:	Felipe Villenas
*/

#include <esp_now.h>
#include <esp_wifi_types.h>
#include <esp_wifi_internal.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUDP.h>


#define PAYLOAD_SIZE	8
#define CHANNEL			8
#define DATA_RATE		WIFI_PHY_RATE_6M
#define CUSTOM_WIFI_CFG true
#define SET_ACTION(action, name) if(action == ESP_OK) { Serial.println(String(name) + " OK!"); } else{ Serial.println("Error with: "+String(name)); }

static uint8_t mac_broadcast[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
//static uint8_t mac_A[] = { 0xc4, 0x4f, 0x33, 0x15, 0xb0, 0x99 };

uint32_t T_MS = 20;
uint32_t TOTAL_PACKETS = 500;
uint32_t t0, t1;	// para controlar la tasa de envio
uint32_t timestamp;
String esp_mqtt_id = "ESP";
bool resend = true;
int N_packets = 0;
int N_rcv = 0;
float rtt_sum = 0;

esp_now_peer_info_t peer_info;

/* UDP Settings */
WiFiUDP udp;
uint32_t udp_port = 3333;
uint8_t udp_buffer[32];

/* MQTT Settings */
const char* ssid = "VTR-6351300";
const char* password = "zkd2bxhcHqHm";
const char* mqtt_server = "192.168.0.7";	// laptop local IPv4
String topic[] = { "ESP_command", "ESP_ts", "ESP_tpackets" };
WiFiClient espClient;
PubSubClient client(espClient);

/* ESPNOW functions */
void setup_custom_wifi();
void setup_espnow();
void add_peer(uint8_t* mac, int channel, bool encrypt);
void OnDataSent(const uint8_t* mac, esp_now_send_status_t status) { /* nothing for now */ };

/* MQTT functions */
void setup_wifi_mqtt();
void mqtt_reconnect();
void mqtt_callback(char* ftopic, uint8_t* msg, uint32_t len);

// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	WiFi.mode(WIFI_STA);

	/* ---------- Setup ESP NOW ------------ */
	/*if (CUSTOM_WIFI_CFG) {
		WiFi.disconnect();
		setup_custom_wifi();
		esp_wifi_set_ps(WIFI_PS_NONE);	// power saving mode none
	}
	setup_espnow();
	add_peer(mac_broadcast, CHANNEL, false);
	/* ------------------------------------- */

	/* ----------- Setup MQTT -------------- */
	setup_wifi_mqtt();
	client.setServer(mqtt_server, 1883);
	client.setCallback(mqtt_callback);
	if (!client.connected()) {
		mqtt_reconnect();
		Serial.println("Sub a los topicos: ");
		for (int i = 0; i < sizeof(topic) / sizeof(String); i++)
			Serial.println("> " + topic[i]);
	}
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

	if (resend) {
		client.disconnect();
	}

	if (N_packets < TOTAL_PACKETS) {
		t1 = millis();
		if (t1 - t0 >= T_MS) {
			timestamp = micros();
			udp.beginPacket(mqtt_server, udp_port);
			udp.write((uint8_t*)&timestamp, 4);
			udp.endPacket();    
			//esp_now_send(mac_broadcast, (uint8_t*)&timestamp, sizeof(timestamp));
			N_packets++;
			t0 = millis();
		}
	}
	else if (N_packets == TOTAL_PACKETS) {
		Serial.println("------ Finished Sending " + String(TOTAL_PACKETS) + " Packets -------");
		mqtt_reconnect();
		Serial.println("MQTT restored");
		resend = false;
		N_packets++;
	}

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
    }

	client.loop();
}

//////////////////////////////////////////////////////////////////////////

/* Custom WiFi Settings for 'better' ESPNOW */
void setup_custom_wifi() {

	SET_ACTION(esp_wifi_stop(), "Stop WiFi");
	SET_ACTION(esp_wifi_deinit(), "De-init WiFi");

	wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
	wifi_cfg.ampdu_tx_enable = 0;
	SET_ACTION(esp_wifi_init(&wifi_cfg), "Setting custom config");
	SET_ACTION(esp_wifi_start(), "Starting WiFi");
	SET_ACTION(esp_wifi_set_promiscuous(true), "Setting promiscuous mode");
	SET_ACTION(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE), "Setting channel");
	SET_ACTION(esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, DATA_RATE), "Setting datarate");

}

void setup_espnow() {

	SET_ACTION(esp_now_init(), "Initializing ESPNOW");
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);
}

void add_peer(uint8_t* mac, int channel, bool encrypt) {

	memcpy(peer_info.peer_addr, mac, 6);
	peer_info.channel = channel;
	peer_info.encrypt = encrypt;

	if (esp_now_add_peer(&peer_info) != ESP_OK)
		Serial.println("Failed to add peer");
	else
		Serial.println("Peer added successfully");
}

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {

	uint32_t t1 = micros();
	uint32_t t0;
	memcpy(&t0, incomingData, 4);
	rtt_sum += (t1 - t0);
	N_rcv++;
}
//////////////////////////////////////////////////////////////////////////
/* MQTT Setup */
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

void mqtt_reconnect() {
	while (!client.connected()) {
		if (client.connect(esp_mqtt_id.c_str())) {
			for (int i = 0; i < sizeof(topic) / sizeof(String); i++)
				client.subscribe(topic[i].c_str());
		}
		else { delay(500); }
	}
}

void mqtt_callback(char* ftopic, uint8_t* msg, uint32_t len) {

	char* rcv_msg = (char*)malloc(len + 1);
	memcpy(rcv_msg, msg, len);
	rcv_msg[len] = '\0';

	if (String(ftopic) == topic[0]) {

		if (String(rcv_msg) == "resend") {
			N_packets = 0;
			N_rcv = 0;
			rtt_sum = 0;
            bzero(udp_buffer,32);
			resend = true;
			Serial.println("Resending packets...");
		}
		else if (String(rcv_msg) == "rcv") {
			Serial.println("Packets received from other ESPs: " + String(N_rcv));
		}
		else if (String(rcv_msg) == "reset rcv") {
			Serial.println("Resetting 'rcv' counter.");
			N_rcv = 0;
		}
		else if (String(rcv_msg) == "rtt") {
			float rtt_avg = rtt_sum / N_rcv;
			Serial.println("Recovered " + String(N_rcv) + " packets.");
			Serial.println("Average RTT = " + String(rtt_avg) + " [us], Average Latency = " + String(rtt_avg / 2) + " [us]");
		}
	}
	else if (String(ftopic) == topic[1]) {
		T_MS = atoi(rcv_msg);
		Serial.println("T_MS changed to = " + String(T_MS));
	}
	else if (String(ftopic) == topic[2]) {
		TOTAL_PACKETS = atoi(rcv_msg);
		Serial.println("TOTAL_PACKETS changed to = " + String(TOTAL_PACKETS));
	}

	free(rcv_msg);
}
