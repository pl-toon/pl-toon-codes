/*
 *	Last modified: 03-27-2021
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include "SPI.h"
#include <Wire.h>   // ???
#include <VL53L0X.h>  // Pololu 1.0.2
#include <esp_wifi_types.h>
#include <esp_wifi_internal.h>
#include <esp_wifi.h>
#include <esp_now.h>

#define DATARATE        WIFI_PHY_RATE_6M    // Para cambiar el bitrate de ESP-NOW a 6Mbps
#define CHANNEL         6                   // Canal WiFi
#define DELAY_CAM		800                 // Delay para consulta a camara

/* Filtering */

typedef struct filterIIR {
	double x_vec[20];
	double y_vec[20];
	double b[20];
	double a[20];
	int N;
	filterIIR(double *b_coeff, double *a_coeff, int N_points){
		N = N_points;
		bzero(x_vec, N*sizeof(double));
		bzero(y_vec, N*sizeof(double));
		bzero(b, N*sizeof(double));
		bzero(a, N*sizeof(double));
		if(b_coeff != NULL)
			memcpy(b, b_coeff, N*sizeof(double));
		if(a_coeff != NULL)
			memcpy(a, a_coeff, N*sizeof(double));
	}
	double filtering(double x){
		double ma = 0;
		double ar = 0;
		memmove(x_vec + 1, x_vec, (N-1)*sizeof(double));
		memmove(y_vec + 1, y_vec, (N-1)*sizeof(double));
		x_vec[0] = x;
		// moving average
		for(int n=0; n < N; n++){
			ma += b[n]*x_vec[n];
		}
		// auto regressive
		for(int n=1; n < N; n++){
			ar += (-1)*a[n]*y_vec[n];
		}
		y_vec[0] = ar + ma;
		return y_vec[0];
	}
} filterIIR;


double b_hann[20] = {0, 0.0013, 0.0063, 0.0164, 0.0318, 0.0514, 0.0729, 0.0932, 0.1090, 0.1176, 0.1176, 0.1090, 0.0932,
                    0.0729, 0.0514, 0.0318, 0.0164, 0.0063, 0.0013, 0};
double b_hamm[20] = {0.0005, 0.0022, 0.0065, 0.0153, 0.0298, 0.0492, 0.0716, 0.0934, 0.1109, 0.1206, 0.1206, 0.1109, 0.0934,
                    0.0716, 0.0492, 0.0298, 0.0153, 0.0065, 0.0022, 0.0005};
double b_win[5] = {0.2, 0.2, 0.2, 0.2, 0.2};

filterIIR filterPos(b_win, NULL, 5);
filterIIR filterVel(b_hamm, NULL, 20);

///////////////////////////////////////
//     MQTT Configuration   ///////////
///////////////////////////////////////
//const char* ssid = "fvp";
//const char* password = "nomeacuerdo";
//const char* ssid = "VTR-6351300";
//const char* password= "zkd2bxhcHqHm";
const char* ssid = "MOVISTAR_7502";
const char* password = "X27JgSvWteS2US4";

//const char* mqtt_server = "192.168.1.100";  // IP fvp
const char* mqtt_server = "192.168.1.114";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  
char msg[50];  

////////////////////////////////////////////
//      ESP-NOW Configuration       ////////
////////////////////////////////////////////
uint8_t mac_leader[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};		// custom MAC
uint8_t mac_A[] = {0xc8, 0x2b, 0x96, 0xb4, 0xe6, 0xcc};
uint8_t mac_addr_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct {
	uint32_t timestamp;
	double position;
	double velocity;
} ESPNOW_payload;

esp_now_peer_info_t peerInfo;   // Struct to add peers to ESPNOW


////////////////////////////////////////////////////
//          DC Motor Configuration      ////////////
////////////////////////////////////////////////////
const int Control_fwd = 25;                //  Pin AIN1  [rotation direction + control]
const int Control_back = 26;            //  Pin AIN2   [rotation direction - control]
const int Control_v = 12;                 //  Pin PWMA    [rotation speed control]
int MotorSpeed = 0;                   // Motor speed from 0 to 1024
int MotorDirection = 1;               // Forward (1) or Backwards (0)

////////////////////////////////////////////////
//         Distance Sensor Variables	  //////
////////////////////////////////////////////////
VL53L0X SensorToF;
double pos_med = 10;  // position/distance measurement              
double temp_cal;     // temporary distance measurement [cm]      
uint32_t time_old = 0;   // time for measurements 
uint32_t t_old;     // time for esp-now tx
uint32_t t1;        // time for initial delay
uint32_t t2;

//////////////////////////////////////////////
//          Velocity Sensor Variables      //
/////////////////////////////////////////////
struct MD {              // struct for camera measurements
	byte motion;
	char dx, dy;
	byte squal;
	word shutter;
	byte max_pix;
	int over;
};
double last_vel = 0;
double vel_lim = 50;
double scale;           // scale factor for the ADNS [cm/counts] 

////////////////////////////////////////////
//      	General Variables		  //////
////////////////////////////////////////////
int dead = 1;     // flag for the deadzone routine
int deadband = 300;     // deadband value
int tiempo_inicial = 0;
bool flag = true; // sync flag
bool start = false; // flag to run the experiment
bool run_test = false;

////////////////////////////////////////////
//		 Motor & PID Variables 		////////
////////////////////////////////////////////
double v_leader = 0, mierror, v_medida, v_min = 1000;
double u = 1;       // Motor Actuation

//////////////////////////////////////////////////////
double READINGS[5];
double SUM = 0;
unsigned int INDEX = 0;
double VALUE = 0;


void loop() {

   
	if(!flag){   // offset to synchronize the data of the experiment
		String delta = String((millis()-tiempo_inicial)*0.001);
		delta.toCharArray(msg, delta.length() + 1);                                                                           
		client.publish("trenes/carroA/desfase", msg);
		flag = true;
		//start = true;
		Serial.println("Sync!");
	}
	
	if(run_test){	// start the test
		client.disconnect();
		start = true;
		t1 = millis();
		run_test = false;
	}
	
	if(!start){	// while the experiment hasn't started, check for the MQTT conection
		if (!client.connected()) 
		{
			esp_now_deinit();
			reconnect();
			esp_now_init();
		}
		client.loop();
		return;
	}
	
	// Measurements 
	pos_med = 0;
	int count_cam=0;
	int last_dy = 0;
	MD cam_med;
	
	/* Routine 1 Measurements */
	for (int i = 0; i < 20 ; i++) {
		if (i % 5 == 1) {   
			uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
			pos_med += range;
		}
		mousecam_read_motion(&cam_med);
		int curr_dy = (int8_t) cam_med.dy;
		if(curr_dy*last_dy < 0 && abs(curr_dy) > abs(last_dy)){
			curr_dy = last_dy;
		}
		count_cam += curr_dy;
		last_dy = curr_dy;
		uint32_t cam_t = micros();
		while(micros() - cam_t < DELAY_CAM);	// using while instead of delay (?)
	}
	mousecam_read_motion(&cam_med);
	count_cam += (int8_t) cam_med.dy;
	v_medida= (scale*1000)*count_cam/(millis()-time_old);   

	if(abs(v_medida) > vel_lim){
		v_medida = last_vel;
	}
	last_vel = v_medida;
	time_old = millis();
		
	pos_med = pos_med / 40; // average of 4 distance measurements [cm]
	pos_med = filterPos.filtering(pos_med);		// testing filtering the data

	Serial.println(pos_med);

	/* TX ESP-NOW */
	uint32_t time_now = millis() - t_old;
	if( time_now > 50 ){	// send data every 50 [ms]
		ESPNOW_payload.timestamp = time_now;
		ESPNOW_payload.velocity = v_medida;
		ESPNOW_payload.position = 20;
		esp_now_send(mac_addr_broadcast, (uint8_t*) &ESPNOW_payload, sizeof(ESPNOW_payload));         // 'True' broadcast, no hay ACK del receptor
		t_old = millis();
	}

	////////////////////////////////////////////////////////
	//         Agent Routine                   /////////////
	////////////////////////////////////////////////////////

	if(millis() - t1 < 3000)	return;     // initial delay to start capturing the data with the train idle

	if (abs(u) != 0 && pos_med > 12.0)
	{
		if (u >= 0){
			MotorDirection = 1;
			MotorSpeed = u;
		}
		else{
			MotorDirection = 0;
			MotorSpeed = -u;
		} 
	}
	else  
	{
		if (MotorSpeed > 20)
			MotorSpeed =  MotorSpeed - 20; 
		else
		{
			//start = false;
			MotorSpeed = 0;
			u = 1;                            // if it stays at 0, after restarting the experiment it will enter this loop inmediately					
			run_test = false;
			//reconnect();
		} 
	}
	SetMotorControl();
}

///////////////////////////////////////////////////////

void SetMotorControl()
{
	if (MotorDirection == 1)            // move forward
	{
		digitalWrite(Control_fwd, LOW);
		digitalWrite(Control_back, HIGH);
	}
	else                                // move backwards
	{
		digitalWrite(Control_fwd, HIGH);
		digitalWrite(Control_back, LOW);
	}

	ledcWrite(Control_v, MotorSpeed); // esp32 (PIN, duty cycle PWM)
}

// MQTT callback function
void callback(char* topic, byte* payload, unsigned int length) {
	String message;
	for (int i = 0; i < length; i++) {
		char nuevo = (char)payload[i];
		message.concat(nuevo);
	}
	Serial.println("message Recibido por MQTT");
	////////////////////////////////
	// 		RECEIVING DATA		 ///
	////////////////////////////////
	if (String(topic) == "trenes/sync") {
		if (String(message)=="True") {
			tiempo_inicial = millis();
			Serial.println("Sync Recibido");
			flag = false;
		}
		else if (String(message)=="False"){
			Serial.println("Detener");
			flag = true;
			start = false;
			MotorSpeed = 0;
			SetMotorControl();      
		}
	}
	if (String(topic) == "trenes/start"){
		if(String(message) == "True"){
			run_test = true;
			Serial.println("Starting Test");
		}
	}
	if (strcmp(topic , "trenes/carrol/u") == 0) {
		u = message.toFloat();
		Serial.println(u);
	}

}

/////////////////////////////////////////////////////

/* ESPNOW Sent Callback Function */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
	// nothing
}

/* ESPNOW Receive Callback Function */
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
	// nothing
}
