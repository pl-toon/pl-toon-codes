/*
 * Last Modified: 04-19-2022
 */

#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SPI.h"
#include <Wire.h>   // ???
#include <VL53L0X.h>  // Pololu 1.0.2
#include <esp_wifi_types.h>
#include <esp_wifi_internal.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFiUDP.h>
#include "math.h"

#include "KickFiltersRT.h"

#define DATARATE        WIFI_PHY_RATE_6M   // Para cambiar el bitrate de ESP-NOW a 24Mbps
#define CHANNEL         3                   // Canal WiFi
#define DELAY_CAM		800

#define ONBOARD_LED  2

#define PAYLOAD_SIZE	3     // numero de doubles para monitoreo
#define Ts            	20    // tiempo en ms

#define n_carro 0     // lider = 0


////////////////////////////////////////////
///////////////////////////////////////////
String carro = "carro" + String(random(100));
//const char carro[] = "carroD2";
char treneserror[] = "trenes/carroD/error";
char trenesp[] = "trenes/carroD/p";
char trenesi[] = "trenes/carroD/i";
char trenesd[] = "trenes/carroD/d";
char trenesp_v[] = "trenes/carroD/p_v";
char trenesi_v[] = "trenes/carroD/i_v";
char trenesd_v[] = "trenes/carroD/d_v";
char trenesdesfase[] = "trenes/carroD/desfase";
char trenesestado[] = "trenes/estado/carroD"; 
char trenestest[] = "trenes/carroD/test";     // topic to send the data to analyze separated by "," (deprecated?)
char trenessampletime[] = "trenes/carroD/ts"; // topic to modify the PID sample time
char trenestransmision[] = "trenes/envio";    // topic to modify the transmission rate
int t_envio = 50;                             // default tx-rate;
///////////////////////////////////////////
/////////////////////////////////////////// 

typedef struct filterIIR {
	double x_vec[20];
	double y_vec[20];
	double b[20];
	double a[20];
	double G;
	int N;
	filterIIR(int N_points, double *b_coeff, double *a_coeff, double gain = 1){
		N = N_points;
		G = gain;
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
		y_vec[0] = ar + G*ma;
		return y_vec[0];
    }
} filterIIR;

typedef struct butterworth {
  double x_vec[3];
  double y_vec[3];
  double b[3];
  double a[3];
  double b_coeff[3];
  double a_coeff[3];
  double G;
  int N = 3;
  butterworth(double wn){
    G = wn*wn/(4/(PI*PI)-2*sqrt(2)*wn/PI+wn*wn);
    bzero(x_vec, N*sizeof(double));
    bzero(y_vec, N*sizeof(double));
    bzero(b, N*sizeof(double));
    bzero(a, N*sizeof(double));
    b_coeff[0] = 1;
    b_coeff[1] = 2;
    b_coeff[2] = 1;
    a_coeff[2] = 1;
    a_coeff[1] = (2*pow(wn,2)-8/(pow(M_PI,2)))/(4/(pow(M_PI,2))-2*sqrt(2)*wn/M_PI+pow(wn,2));
    a_coeff[0] = (4/(pow(M_PI,2))+2*sqrt(2)*wn/M_PI+pow(wn,2))/(4/(pow(M_PI,2))-2*sqrt(2)*wn/M_PI+pow(wn,2));
    double aa = M_PI;
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
    y_vec[0] = (ar + G*ma)/a_coeff[0];
    return y_vec[0];
    }
} butterworth;



double Gain_butter = 0.0201;
double b_butter[3] = {1, 2, 1};
double a_butter[3] = {1, -1.5610, 0.6414};

//filterIIR filterPos(b_win, NULL, 5);
//filterIIR filterPos(3, b_butter, a_butter, Gain_butter);
butterworth filterPosB(0.15);
//filterIIR filterVel(b_hamm, NULL, 20);


///////////////////////////////////////
//     MQTT Configuration   ///////////
///////////////////////////////////////
const char* ssid = "fvp";
const char* password = "nomeacuerdo";
//const char* ssid = "GTD-3813230";
//const char* password= "g6yWsKswphfs";
//const char* ssid = "KATRINA";
//const char* password = "wlan70634f";
//const char* ssid = "MOVISTAR_7502";
//const char* password = "X27JgSvWteS2US4";

//const char* mqtt_server = "192.168.1.100";  // IP fvp
//const char* mqtt_server = "192.168.1.114";
const char* mqtt_server = "192.168.1.101";

/* UDP Settings */
WiFiUDP udp;
uint32_t udp_port = 3333;
const char* udp_server = "192.168.1.103";
double udp_buffer[PAYLOAD_SIZE];
uint32_t t_udp = 0;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  
char msg[50];

////////////////////////////////////////////
//      ESP-NOW Configuration       ////////
////////////////////////////////////////////
 

uint8_t mac_leader[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};		// custom MAC
uint8_t mac_list[][6] = { {0x24, 0x0A, 0xC4, 0x31, 0x34, 0x18},   // lider
                          {0x24, 0x0A, 0xC4, 0x32, 0x0E, 0x20},   // 2do
                          {0x24, 0x0A, 0xC4, 0x32, 0x14, 0x80},   // 3ro
                          {0x24, 0x0A, 0xC4, 0x32, 0x37, 0xAC}    // 4to
                        };  
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
double data_com[2];
ESPNOW_payload rcv_data;

esp_now_peer_info_t peerInfo;   // struct to add peers to ESPNOW

////////////////////////////////////////////////////
//          DC Motor Configuration      ////////////
////////////////////////////////////////////////////
const int Control_fwd = 25;                //  Pin AIN1  [rotation direction + control]
const int Control_back = 26;            //  Pin AIN2   [rotation direction - control]
const int Control_v = 12;                 //  Pin PWMA    [rotation speed control]
int MotorSpeed = 0;                   // Motor speed from 0 to 1024
int MotorDirection = 1;               // Forward (1) or Backwards (0)

////////////////////////////////////////////////
//         Distance Sensor Variables	   /////
////////////////////////////////////////////////
VL53L0X SensorToF;
double pos_med = 25;	// position/distance measurement
double temp_cal;		// temporary distance measurement [cm]     
double last_distance = 0;               
double time_old = 0;  	// time for measurements
double t_old;		// time for esp-now tx
double t1; // para temporizar la medición de la velocidad
double t2;

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

/*-------------------------------------------------------------------*/
void averageDistance(double* x_0, int N){
  for (int i = 0; i < N; i++) {
    uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
    *x_0 += (double) range;
    delay(100);
  }
  *x_0 = *x_0 / (10*N); //[cm]
}

/*-------------------------------------------------------------------*/


////////////////////////////////////////////
//      	General Variables		  //////
////////////////////////////////////////////
int dead = 1;     // flag for the deadzone routine
int deadband = 300;     // deadband value
int tiempo_inicial = 0;
bool flag = true; // sync flag
bool start = false; // flag to run the experiment
bool run_test = false;
String estado;

////////////////////////////////////////////
//      Motor & PID Variables       ////////
////////////////////////////////////////////
double v_leader = 0, mierror, v_medida = 0;
double x_ref = 20;							// position setpoint
double u_distancia;							// actuation calculated by distance                           
double u_velocidad;							// actuation calculated by velocity
double u;									// weighted actuation
int umax = 400, umin = -umax;				// actuation limits
double Kp = 20, Ki = 20, Kd = 0;			// Position PID gains
double Kp_v = 20, Ki_v = 20, Kd_v = 0;      // Velocity PID gains   
int SampleTime = 50;						// Controllers sampling time [ms]
double alpha = 0.0;							// constant spacing policy
double h = 0.5;								// time headway
double error_distance;
double error_velocity;
double etha = 0.5;							// weight of pos and vel errors
double weighted;							// weighted error
double rf = 0;
int lim = 10;								// lower limit for the train to not move

double R_acumulado = 0;
double sum_s = 0;
double lambda = 0.5;

PID myPID(&error_distance, &u_distancia, &rf, Kp, Ki, Kd, DIRECT);
PID myPID_v(&error_velocity, &u_velocidad, &rf, Kp_v, Ki_v, Kd_v, DIRECT);

uint32_t time_trigger = millis();

//////////////////////////////////////////////////////
double frequencyCos = 0.02;

KickFiltersRT<float> filtersRT;
float cutFreq = 4.0;

int deltaT = 6; //ms
double velocity = 0;

void loop() {
    /*if(millis() >= time_trigger + 8000){
      time_trigger = millis();
      if (x_ref == 15) x_ref = 25;
      else x_ref = 15;
    }*/
	int offset_coseno = 20;
    int amplitude_coseno = 10;
    
  	if(n_carro==0){ 		
  		x_ref = offset_coseno + amplitude_coseno*cos(2*3.1415926335*frequencyCos*(0.001)*((double)millis()));
  		x_ref = x_ref >= offset_coseno ? offset_coseno + amplitude_coseno : offset_coseno - amplitude_coseno; // onda rectangular
  	}
	/*if(run_test){
		client.disconnect();
		start = true;
		run_test = false;
	}*/

	client.loop();
	
	if(!flag){   // offset to synchronize the data of the experiment
		String delta = String((millis()-tiempo_inicial)*0.001);
		delta.toCharArray(msg, delta.length() + 1);                                                                           
		client.publish(trenesdesfase, msg);
		flag = true;
		start = true;
		Serial.println("Sync!");
		//client.disconnect();
	}  // start the experiment

	if(!start){ // while the experiment hasn't started, check for the MQTT conection
		if (!client.connected()) {
				reconnect();
				estado = String(carro) + " Reconectando";
				estado.toCharArray(msg, estado.length() + 1);                                                                           
				client.publish(trenesestado, msg);
			}
		client.loop();
		return;
	}

	// Measurements 
	pos_med = 0;
	int count_cam=0;
	int last_dy = 0;
	MD camara;
	
	/* Routine 1 Measurements */
	for (int i = 0; i < 8 ; i++) {
		if (i % 2 == 1) {   
			uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
			pos_med += range;
		}
	}
	
  /*rutina medición*/
    /*---------------------------------------------------*/
    uint32_t time0 = millis(); 
     delay(deltaT);
      count_cam=0;
      do{
        //leer todos los datos del buffer
        mousecam_read_motion(&camara);
        count_cam += (int8_t)camara.dy;
        if(camara.motion & 0x10) //1 = Overflow has occurred
          Serial.println("Overflow");
      } while(camara.motion & 0x80); //1 = Motion occurred, data ready for reading in Delta_X and Delta_Y registers
      //distancewFilterB[1] = filterButter.butter_filtering((double)cuentasTotal*scale);
    
  velocity = filtersRT.lowpass((double)count_cam*scale*1000./(double)deltaT, cutFreq, 1000./deltaT);
 
  v_medida= velocity;
    
		
	pos_med = pos_med / 40; // average of 4 distance measurements [cm]

	if(pos_med > 90){	// to eliminate inconsistent measurements
		pos_med = last_distance;
	}
	last_distance = pos_med;
	double pos_med_filt = filterPosB.filtering(pos_med);
	//Serial.println(pos_med_filt);

	/* Time Headway */
	//double X_ref = x_ref + h*v_medida;

	error_distance = lambda*(x_ref - pos_med_filt) + (1 - lambda)*(R_acumulado + x_ref - (sum_s + pos_med_filt));  
  	if(n_carro==0)
  		error_distance = x_ref - pos_med_filt;
	error_velocity = -(v_medida - v_leader);

	//if(abs(error_distance < 0.6))	error_distance = 0;

	myPID.SetTunings(Kp, Ki, Kd);
	myPID.SetOutputLimits(umin, umax);
	myPID.Compute();
	myPID.SetSampleTime(SampleTime);
	myPID_v.SetTunings(Kp_v, Ki_v, Kd_v);
	myPID_v.SetOutputLimits(umin, umax);
	myPID_v.Compute();
	myPID_v.SetSampleTime(SampleTime);

	weighted = etha*abs(error_distance); //abs(v_medida - v_leader);
	//weighted = (1 - etha) * abs(x_ref - medi) + etha * abs(v_medida - v_leader);  // error weighted to avoid oscillation when the error is too small 
	
	u = u_distancia;
	if(n_carro != 0)
	  u = u_distancia + 0*alpha*u_velocidad;
    
	// TX ESP-NOW
	uint32_t time_now = millis() - t_old;
	if( time_now >= t_envio ){	// send data every 50 [ms]

	    if(n_carro == 0)
	      v_leader = v_medida;
    
			monitor_data[0] = x_ref;
			monitor_data[1] = pos_med_filt;
			monitor_data[2] = R_acumulado + x_ref;				// R_acumulado + referencia_local
		    monitor_data[3] = sum_s + pos_med_filt;		// SUMA de s_i + s_actual 
		    monitor_data[4] = pos_med;
		    Serial.println(monitor_data[2]);

		    udp_buffer[0] = (double) n_carro;
		    udp_buffer[1] = pos_med_filt;
		    udp_buffer[2] = error_distance;

		    esp_udp_send((uint8_t*) udp_buffer, sizeof(udp_buffer));
		    //Serial.printf(" ");
		    //Serial.printf(error_distance);
	    
			esp_now_send(mac_addr_broadcast, (uint8_t*) &monitor_data, sizeof(monitor_data));         // 'True' broadcast, no hay ACK del receptor
	       
			t_old = millis();
	}

	////////////////////////////////////////////////////////
	//       	  Agent Routine                /////////////
	////////////////////////////////////////////////////////

	if ((pos_med > 200))   // if there's nothing in front, stop or deaccelerate
	{
		if (MotorSpeed < 200) MotorSpeed -= 10 ; 
		else MotorSpeed = 0; 
		myPID.SetMode(MANUAL); // to stop integrating
		myPID_v.SetMode(MANUAL); // to stop integrating
	}

	else
	{
		if (u < -lim)  // lim is an arbitrary value to make the train not move if |u|<=lim
		{
			MotorDirection = 0;
			MotorSpeed = int(-u + deadband ); //- 40);
		}
		else if (u > lim)
		{
			MotorDirection = 1;
			MotorSpeed = int(u + deadband ); //- 40);
		}

		if (((u >= -lim)) && ((u <= lim)))
		{
			myPID.SetMode(MANUAL);            // turn off the distance PID
			myPID_v.SetMode(MANUAL);          // turn off the velocity PID
			MotorSpeed = 1.5 * int(abs(u));
			if (u > 0)  MotorDirection = 1;
			else  MotorDirection = 0;
		}

		if (((u >= -lim) && (u <= lim) && (abs(weighted) <= 0.75)))
		{
			MotorSpeed = 1;
			if (u < 0)  MotorDirection = 1;
			else  MotorDirection = 0;
		}
		if( (abs(weighted) > 0.75) && (pos_med < 200)){ // condition to turn on the PID again
			myPID.SetMode(AUTOMATIC);
			myPID_v.SetMode(AUTOMATIC);
		}        
	}
	SetMotorControl();
}

/////////////////////////////////////////////////////////////

void esp_udp_send(uint8_t* payload, uint32_t payload_size){

	udp.beginPacket(udp_server, udp_port);
    udp.write(payload, payload_size);     // buffer y el size en bytes
    udp.endPacket();
}

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

/* MQTT Callback */
void callback(char* topic, byte* payload, unsigned int length) {
	String message;
	for (int i = 0; i < length; i++) {
		char nuevo = (char)payload[i];
		message.concat(nuevo);
	}
	////////////////////////////////
	// 		RECEIVING DATA		 ///
	////////////////////////////////

	if (String(topic) == "trenes/sync") {
		if (String(message)=="True") {
			tiempo_inicial = millis();
			Serial.println("Sync Received");
			flag = false;
			myPID.SetMode(AUTOMATIC);            // start the distance PID  
			myPID_v.SetMode(AUTOMATIC);          // start the velocity PID  
		}
		else if (String(message)=="False"){
			Serial.println("Stop");
			flag = true;
			start = false;
			MotorSpeed = 0;
			SetMotorControl();
			myPID.SetMode(MANUAL);            // stop the distance PID 
			myPID_v.SetMode(MANUAL);          // stop the velocity PID   
			u_distancia = 0;
			u_velocidad = 0; 
		}
	}
	if (strcmp(topic ,trenesp) == 0) {
		Kp = message.toFloat();
	}
	if (strcmp(topic , trenesi) == 0) {
		Ki = message.toFloat();
	}
	if (strcmp(topic , trenesd) == 0) {
		Kd = message.toFloat();
	}
	if (strcmp(topic ,trenesp_v) == 0) {
		Kp_v = message.toFloat();
	}
	if (strcmp(topic , trenesi_v) == 0) {
		Ki_v = message.toFloat();
	}
	if (strcmp(topic , trenesd_v) == 0) {
		Kd_v = message.toFloat();
	}
  if (strcmp(topic , "trenes/carrol/u") == 0) {
    u = message.toFloat();
  }
	if (strcmp(topic , "trenes/etha") == 0) {
		etha = message.toFloat();
	}
	if (strcmp(topic, "trenes/h") == 0) {
		h = message.toFloat();
	}
	if (strcmp(topic, "trenes/alpha") == 0) {
		alpha = message.toFloat();
	}
  if (strcmp(topic, "trenes/lambda") == 0) {
    lambda = message.toFloat();
  }
	if(strcmp(topic, "trenes/start") == 0){
		run_test = true;
	}
	if (strcmp(topic , "trenes/u_lim") == 0) {
		umax = message.toInt();
		umin = -umax;
	}
	if (strcmp(topic , "trenes/carroL/v_leader") == 0) {
		v_leader = message.toFloat();
	}
	if (strcmp(topic , "trenes/ref") == 0) {
		x_ref = message.toFloat();
	}
	if (strcmp(topic , trenessampletime) == 0) {
		SampleTime = message.toInt();
	}
	if (strcmp(topic , trenestransmision) == 0) {
		t_envio = message.toInt();
	}
}

/////////////////////////////////////////////////


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

/* ESPNOW Receive Callback Function */
void OnDataRecv(const uint8_t *mac, const uint8_t *Data, int len)
{
  //Serial.println("llego");
	/*int mac_eq = (mac[0]==mac_leader[0])+(mac[1]==mac_leader[1])+(mac[2]==mac_leader[2]);
	mac_eq += (mac[3]==mac_leader[3])+(mac[4]==mac_leader[4])+(mac[5]==mac_leader[5]);
	if(mac_eq < 6){
		return;
	}
	memcpy(&rcv_data, Data, len);
	v_leader = rcv_data.velocity;
	uint32_t t_now = millis(); 
	rcv_time = t_now;*/

  if(n_carro == 0)
    return;
 
  int mac_eq = (mac[0]==mac_list[n_carro - 1][0])+(mac[1]==mac_list[n_carro - 1][1])+(mac[2]==mac_list[n_carro - 1][2]);
  mac_eq += (mac[3]==mac_list[n_carro - 1][3])+(mac[4]==mac_list[n_carro - 1][4])+(mac[5]==mac_list[n_carro - 1][5]);
  if(mac_eq < 6){
    return;
  }
  
  double temp[5];
  memcpy(temp, Data, len);  // temp[0] = R carro anterior, temp[1] = sum_s del carro anterior
  R_acumulado = temp[2]; 
  sum_s = temp[3];
  //v_leader = temp[3];
 
}
