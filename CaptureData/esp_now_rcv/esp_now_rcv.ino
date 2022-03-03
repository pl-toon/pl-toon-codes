#include <esp_wifi_types.h>
#include <esp_wifi_internal.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>

#define MAC_2_MSBytes(MAC)  MAC == NULL ? 0 : (MAC[0] << 8) | MAC[1]
#define MAC_4_LSBytes(MAC)  MAC == NULL ? 0 : (((((MAC[2] << 8) | MAC[3]) << 8) | MAC[4]) << 8) | MAC[5]

#define N_8BYTES	5

#define DATARATE        WIFI_PHY_RATE_6M   // Para cambiar el bitrate de ESP-NOW a 24Mbps
#define CHANNEL         1                   // Canal WiFi
#define DELAY_CAM		800

#define CARRO_ID		0

uint8_t mac_leader[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};		// custom MAC
uint8_t mac_addr_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool received, resend;      // control variables
uint32_t rcv_time = 0;

typedef struct {
	uint32_t timestamp;
	double position;
	double velocity;
} ESPNOW_payload;

//ESPNOW_payload monitor_data;
double monitor_data[5];

esp_now_peer_info_t peerInfo;   // struct to add peers to ESPNOW


typedef struct {
	uint64_t mac[10];
	int n_esp = 0;
	uint32_t t0_esp[10];
	int get_esp_id(const uint8_t* mac);
} ESP_MAC_list;

ESP_MAC_list mac_list;		// struct para almacenar la lista de MACs

void setup() {
	Serial.begin(115200);
	Serial.println("");
	/////////////////////////////////////

	/////////////////////////////////////
	///  	  Communication Start     ///
	/////////////////////////////////////

	WiFi.mode(WIFI_STA);
	// Only use the lines below to activate 'custom wifi settings'
	//WiFi.disconnect();
	//setup_custom_wifi(&my_config);
	esp_wifi_set_ps(WIFI_PS_NONE);		// No power-saving mode
	//add_peer(mac_addr_A, CHANNEL, false);     // (const char* mac, int channel, bool encryption)
	//add_peer(mac_addr_broadcast, CHANNEL, false);

    esp_wifi_set_promiscuous(true);
	if ( esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK )
		Serial.println("Error channel");
	/* MQTT */

	/* ESP-NOW */
	//add_peer(mac_addr_A, CHANNEL, false);     // (const char* mac, int channel, bool encryption)
	setup_espnow();
	add_peer(mac_addr_broadcast, CHANNEL, false);
    
}

void loop(){
    // chill
    //esp_now_send(mac, (uint8_t*) payload, sizeof(payload));
}

/* ESPNOW Sent Callback Function */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	/*if ( status == ESP_NOW_SEND_SUCCESS ) {
		//Serial.println("Delivery Success");
		//Serial.printf("sent = %u (CPU%d)\n",sent,xPortGetCoreID());
		//resend = false;
	}
	else {
		Serial.println("Delivery Fail");
		resend = true;
	}*/
}
double rcv_data[N_8BYTES];

/* ESPNOW Receive Callback Function */
void OnDataRecv(const uint8_t *mac, const uint8_t *Data, int len)
{
    int id = mac_list.get_esp_id(mac);
	/* Copia los datos recibidos */
	memcpy(&rcv_data, Data, len);
	
    /*Serial.print(id);
    Serial.print(",");
    Serial.print(millis());
	Serial.print(",");
	Serial.print(rcv_data[1]);
	Serial.print(",");
    Serial.println(rcv_data[2]);*/
    Serial.println(rcv_data[0]);
}

///////////////////////////////////////////////


///////////////////////////////////////////////////
/* Custom WiFi Settings for 'better' ESPNOW */
void setup_custom_wifi()
{
	if ( esp_wifi_stop() != ESP_OK )
		Serial.println("Error in esp_wifi_stop");
	if ( esp_wifi_deinit() != ESP_OK )
		Serial.println("Error in esp_wifi_deinit");

	wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
	my_config.ampdu_tx_enable = 0;

	if ( esp_wifi_init(&my_config) != ESP_OK )
		Serial.println("Error in my_config");
	if ( esp_wifi_start() != ESP_OK )
		Serial.println("Error stating");
	//esp_wifi_set_promiscuous(true);
	/*if ( esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK )
		Serial.println("Error channel");*/
	if ( esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, DATARATE) != ESP_OK )
		Serial.println("Error fix rate");

}

/* ESPNOW Setup */
void setup_espnow()
{
	if (esp_now_init() != ESP_OK)
		Serial.println("Error initializing ESP-NOW");
		
	esp_now_register_send_cb(OnDataSent);
	esp_now_register_recv_cb(OnDataRecv);
}

/* Add ESPNOW Peer */
void add_peer(uint8_t *mac, int channel, bool encrypt)
{
	memcpy(peerInfo.peer_addr, mac, 6);
	peerInfo.channel = channel;
	peerInfo.encrypt = encrypt;

	if (esp_now_add_peer(&peerInfo) != ESP_OK)
		Serial.println("Failed to add peer");
	else
		Serial.println("Peer added successfully");
}

int ESP_MAC_list::get_esp_id(const uint8_t* mac){

	int id = -1;
	uint64_t LSB = MAC_4_LSBytes(mac);
	uint64_t MSB = MAC_2_MSBytes(mac);
	uint64_t mac_tr = (MSB << 32) + LSB;
	for(int i=0; i < (int)(sizeof(mac_list.mac)/sizeof(uint64_t)); i++){
		if(mac_tr == mac_list.mac[i]){
			id = i;
		}
	}
	if(id == -1){
		id = mac_list.n_esp;
		mac_list.mac[id] = mac_tr;
		mac_list.n_esp++;
		//flag_t0_esp = true;
	}
	return id;
}
