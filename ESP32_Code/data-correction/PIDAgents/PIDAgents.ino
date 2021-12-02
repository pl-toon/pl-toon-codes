/*
 * Last Modified: 03-27-2021
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
#include "KickFiltersRT.h"


#ifndef M_PI 
#define M_PI  3.14159265358979323846
#endif
#define DATARATE        WIFI_PHY_RATE_6M   // Para cambiar el bitrate de ESP-NOW a 24Mbps
#define CHANNEL         6                   // Canal WiFi
#define DELAY_CAM		800

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



/* Funcion para corregir el dato si es necesario*/ 

double data_correction(double range, double prev_measure){
 static uint16_t range_buf[4]={0,0,0,0};
 static uint16_t change_counter=0;
 static uint16_t inv_counter;
 static uint16_t unmodified=0;
 static uint16_t last_okmes;  
 double range_corr; 

    if (range<1000) last_okmes = range;

       static uint16_t range_corrected;
    if(range<8000){                      //Corregir los casos en los que hay un error en la medición
      range_corrected = range;
      }
    range = range_corrected;
    if(range_buf[0]==0){
      range_buf[0] = range;
      range_buf[1] = range;
      range_buf[2] = range;
      range_buf[3] = range;
      }
      
      
      if(range!= range_buf[0]){
      memmove(range_buf + 1, range_buf, 3*sizeof(uint16_t));
      range_buf[0]= range;
      }

      double avg_range = (double(range_buf[1]) + double(range_buf[2]) + double(range_buf[3])) /3.0;
          
          bool valid_measure = 0;
          int coun = 0;

          if(abs(range-prev_measure)<50 && !unmodified){
            change_counter+=1;}
          else{
            change_counter=0;   
          }


          //Serial.println(last_okmes);
          if((abs(range_buf[0]-avg_range)>30)){
            unmodified = 0;
            inv_counter++;
            if(inv_counter<5 && change_counter<3){
              range_corr = 0.5*range_buf[1]+0.5*avg_range;
              range_buf[0] = range_buf[1];
            }
            //Serial.println("correg");
          }
          else{
            range_corr=range_buf[0];
            //Serial.println("bueno");
            inv_counter = 0;
            change_counter = 0;
            valid_measure = 1;
          }
          if(inv_counter==4){
                range_corr = last_okmes;
                range_buf[0]=last_okmes;
                inv_counter = 0;
                change_counter = 0;
          }
          if(change_counter==3){
                range_corr = last_okmes;
                range_buf[0]=last_okmes;
                inv_counter = 0;
                change_counter = 0;
          }
     return range_corr;
     }


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
  double butter_filtering(double x){
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


double b_hann[20] = {0, 0.0013, 0.0063, 0.0164, 0.0318, 0.0514, 0.0729, 0.0932, 0.1090, 0.1176, 0.1176, 0.1090, 0.0932,
                    0.0729, 0.0514, 0.0318, 0.0164, 0.0063, 0.0013, 0};
double b_hamm[20] = {0.0005, 0.0022, 0.0065, 0.0153, 0.0298, 0.0492, 0.0716, 0.0934, 0.1109, 0.1206, 0.1206, 0.1109, 0.0934,
                    0.0716, 0.0492, 0.0298, 0.0153, 0.0065, 0.0022, 0.0005};
double b_win[5] = {0.2, 0.2, 0.2, 0.2, 0.2};

double Gain_butter = 0.0201;
double b_butter[3] = {1, 2, 1};
double a_butter[3] = {1, -1.5610, 0.6414};

double Gain_butter03 = 0.1311;
double b_butter03[3] = {1, 2, 1};
double a_butter03[3] = {1,-0.747789178258503,0.272214937925007};

double Gain_butter05 = 0.2929;
double b_butter05[3] = {1, 2, 1};
double a_butter05[3] = {1,0,0.1716};

double b_orden1[2] = {0.8, 0};
double a_orden1[2] = {1, -0.2};

KickFiltersRT<float> filtroRT_dist;

//filterIIR filterPos(b_win, NULL, 5);
//filterIIR filterVel(b_hamm, NULL, 20);

filterIIR filterPos(3, b_butter05, a_butter05, Gain_butter05);
filterIIR filterVel(3, b_butter05, a_butter05, Gain_butter05);

butterworth filterPosB(0.3);

//filterIIR filterVel(2, b_orden1, a_orden1, 1);
//filterIIR filterPos(2, b_orden1, a_orden1, 1);

///////////////////////////////////////
//     MQTT Configuration   ///////////
///////////////////////////////////////
//const char* ssid = "fvp";
//const char* password = "nomeacuerdo";
//const char* ssid = "VTR-6351300";
//const char* password= "zkd2bxhcHqHm";
//const char* ssid = "MOVISTAR_7502";
//const char* password = "X27JgSvWteS2US4";
const char* ssid = "VTR-6720622";
const char* password= "vk5cqLvsKfwq";

//const char* ssid = "Cristobal";
//const char* password= "d78aeed77940";

//const char* mqtt_server = "192.168.1.100";  // IP fvp
//const char* mqtt_server = "192.168.1.114";
//const char* mqtt_server = "192.168.0.10";
const char* mqtt_server = "192.168.0.15";  



WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  
char msg[50];

////////////////////////////////////////////
//      ESP-NOW Configuration       ////////
////////////////////////////////////////////
uint8_t mac_leader[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};		// custom MAC
uint8_t mac_addr_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool received, resend;      // control variables
uint32_t rcv_time = 0;

typedef struct {
	uint32_t timestamp;
	double position;
	double velocity;
} ESPNOW_payload;

ESPNOW_payload monitor_data;
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
double time_trigger = 0;
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
double x_ref = 25;							// position setpoint
double u_distancia;							// actuation calculated by distance                           
double u_velocidad;							// actuation calculated by velocity
double u;									// weighted actuation
int umax = 400, umin = -umax;				// actuation limits
double Kp = 40, Ki = 20, Kd = 0;			// Position PID gains
double Kp_v = 40, Ki_v = 20, Kd_v = 0;      // Velocity PID gains   
int SampleTime = 50;						// Controllers sampling time [ms]
double alpha = 0;							// constant spacing policy
double h = 0.5;								// time headway
double error_distance;
double error_velocity;
double etha = 0.5;							// weight of pos and vel errors
double weighted;							// weighted error
double rf = 0;
int lim = 15;								// lower limit for the train to not move
PID myPID(&error_distance, &u_distancia, &rf, Kp, Ki, Kd, DIRECT);
PID myPID_v(&error_velocity, &u_velocidad, &rf, Kp_v, Ki_v, Kd_v, DIRECT);

static double tiempo_pr, ttt;

//////////////////////////////////////////////////////

void loop() {

	/*if(run_test){
		client.disconnect();
		start = true;
		run_test = false;
	}*/
  
      //Serial.println("Sync Received");
      flag = false;
      myPID.SetMode(AUTOMATIC);            // start the distance PID  
      myPID_v.SetMode(AUTOMATIC);          // start the velocity PID  
 
	if(!flag){   // offset to synchronize the data of the experiment
		String delta = String((millis()-tiempo_inicial)*0.001);
		delta.toCharArray(msg, delta.length() + 1);                                                                           
		client.publish(trenesdesfase, msg);
		flag = true;
		start = true;
		//Serial.println("Sync!");
		client.disconnect();
	}  // start the experiment

	/*if(!start){ // while the experiment hasn't started, check for the MQTT conection
		if (!client.connected()) {
				reconnect();
				estado = String(carro) + " Reconectando";
				estado.toCharArray(msg, estado.length() + 1);                                                                           
				client.publish(trenesestado, msg);
			}
		client.loop();
		return;
	}*/

	// Measurements 
  double x_ff = 0;
  double x_ff_sn = 0;
 
  static double pos_med_raw;
  static uint16_t prev_measure;
  double pos_med;
  
	pos_med = 0;
  pos_med_raw = 0;
	int count_cam=0;
	int last_dy = 0;
  int pos_inv = 0;
	MD cam_med;

 
	//Serial.println("Empieza medicion");
	/* Routine 1 Measurements */
	for (int i = 0; i < 8 ; i++) {
		if (i % 2 == 1) {
      //Serial.println("Tiempo:");
      //Serial.println(millis()-ttt);
      ttt = millis();
	  uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
 
      pos_med_raw += range;
      double range_corr = data_correction(range,prev_measure);
      pos_med += range_corr;
          /*Serial.print(pos_med);
          Serial.print(" ");
          Serial.println(pos_med_raw);*/
          delay(10);
        }
     //Serial.println(range);
     //Serial.println(last_distance);
     //Serial.println(pos_inv);
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
		
	pos_med = pos_med / 40;
  prev_measure = pos_med_raw / 4;
  pos_med_raw = pos_med_raw / 40;
  //Serial.println("velocidad entregada");
   
	if(pos_med > 90){	// to eliminate inconsistent measurements
		pos_med = last_distance;
	}
	last_distance = pos_med;
  double tiempo_act = millis()-tiempo_pr;
  tiempo_pr = millis();
  
	double pos_med_filt = filterPos.filtering(pos_med);
  double v_medida_filt = filterVel.filtering(v_medida);
  double pos_butter = filterPosB.butter_filtering(pos_med);
  double pos_med_filt_lib = filtroRT_dist.lowpass((double)pos_med_raw, 0.05*50, 50);

  Serial.print(pos_med);
  Serial.print(" ");
  Serial.println(pos_med_raw);
 
  
  tiempo_inicial = millis();
  /*if(millis() >= time_trigger + 10000){
    time_trigger = millis();
    if (x_ref == 35) x_ref = 25;
    else x_ref = 35;
    }*/
  //Serial.println(x_ref);
  
  
	/* Time Headway */
	double X_ref = x_ref + h*v_medida;

	error_distance = (x_ref - pos_med);  
	error_velocity = (v_medida_filt - v_leader);
  //Serial.println(error_distance);

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
	u = u_distancia + alpha*u_velocidad;
  //Serial.println(u);
    
	// TX ESP-NOW
	uint32_t time_now = millis() - t_old;
	if( time_now >= 50 ){	// send data every 50 [ms]
		monitor_data.timestamp = u;
		monitor_data.position = pos_med_filt;
		monitor_data.velocity = v_medida;
		esp_now_send(mac_addr_broadcast, (uint8_t*) &monitor_data, sizeof(monitor_data));         // 'True' broadcast, no hay ACK del receptor
		//esp_now_send(NULL, (uint8_t*) &v_leader, sizeof(v_leader));                     // Destino NULL para iteracion a todos los receptos. Sí hay ACK del receptor.
		t_old = millis();
	}

	////////////////////////////////////////////////////////
	//       	  Agent Routine                /////////////
	////////////////////////////////////////////////////////

	if ((pos_med > 200 || pos_med == 0))   // if there's nothing in front, stop or deaccelerate
	{
		if (MotorSpeed < 200) MotorSpeed -= 10 ; 
		else MotorSpeed = 0; 
    if (pos_med == 0)MotorSpeed = 0;
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
	if (strcmp(topic , "trenes/etha") == 0) {
		etha = message.toFloat();
	}
	if (strcmp(topic, "trenes/h") == 0) {
		h = message.toFloat();
	}
	if (strcmp(topic, "trenes/alpha") == 0) {
		alpha = message.toFloat();
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
	int mac_eq = (mac[0]==mac_leader[0])+(mac[1]==mac_leader[1])+(mac[2]==mac_leader[2]);
	mac_eq += (mac[3]==mac_leader[3])+(mac[4]==mac_leader[4])+(mac[5]==mac_leader[5]);
	if(mac_eq < 6){
		return;
	}
	memcpy(&rcv_data, Data, len);
	v_leader = rcv_data.velocity;
	uint32_t t_now = millis(); 
	rcv_time = t_now;
}
