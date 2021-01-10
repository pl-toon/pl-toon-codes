//#include "Adafruit_VL53L0X.h"
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

//--------- Definiciones ESPNOW ---------------//
#define PAYLOAD_SIZE    32                  // Tamano en Bytes de los paquetes a enviar por ESP-NOW
#define DATARATE        WIFI_PHY_RATE_6M   // Para cambiar el bitrate de ESP-NOW a 24Mbps
#define CHANNEL         1                   // Canal WiFi

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
char trenestest[] = "trenes/carroD/test";     // Donde se envian los datos a analizar separados por ","
char trenessampletime[] = "trenes/carroD/ts"; // Lista para modificar el tiempo de sampleo del PID
char trenestransmision[] = "trenes/envio";    // Lista para modificar el tiempo de transmision
int t_envio = 50;                             // Tiempo de transmision por defecto;
///////////////////////////////////////////
/////////////////////////////////////////// 

struct MD
{
	byte motion;
	char dx, dy;
	byte squal;
	word shutter;
	byte max_pix;
	int over;
};

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

//filter1 filterPos(0.7265, 0.1367, 0.1367);
//filterIIR filterPos(b_win, NULL, 5);
filterIIR filterPos(3, b_butter, a_butter, Gain_butter);
//filterIIR filterVel(b_hamm, NULL, 20);

int yy=0;  // coordenadas obtenidas por sensor ADNS3080 (dead reckoning)
double temp_cal;
double scale; // escalamiento de cuentas ADNS a cm 
double medi = 10;
double xx = 10;

///////////////////////////////////////
//     Configuración MQTT   ///////////
///////////////////////////////////////
//const char* ssid = "ac3e";  // Nombre WiFi a conectarse
//const char* password = "rac3e/07"; // Contraseña WiFi
//onst char* ssid = "stringUTEM";
//const char* password = "stringstable";
//const char* ssid = "trenesAC3E";
//const char* password = "stringstable";
//const char* ssid = "fvp";
//const char* password = "nomeacuerdo";
const char* ssid = "VTR-6351300";
const char* password= "zkd2bxhcHqHm";
//const char* ssid = "MOVISTAR_7502";
//const char* password = "X27JgSvWteS2US4";

//const char* mqtt_server = "10.1.28.117";  // IP de la Raspberry
//const char* mqtt_server = "192.168.137.1";
//const char* mqtt_server = "192.168.1.100";  // IP fvp
//const char* mqtt_server = "192.168.1.114";
const char* mqtt_server = "192.168.0.4";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  
char msg[50];

////////////////////////////////////////////
//      Configuracion ESP-NOW       ////////
////////////////////////////////////////////
uint8_t mac_leader[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};		// custom MAC
uint8_t mac_addr_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool received, resend;      // Variables de control
uint32_t rcv_time = 0;

typedef struct {
    uint32_t timestamp;
    double position;
    double velocity;
} ESPNOW_payload;

ESPNOW_payload monitor_data;
ESPNOW_payload rcv_data;

esp_now_peer_info_t peerInfo;   // Estructura para anadir peers a espnow
double last_vel = 0;
double vel_lim = 50;

////////////////////////////////////////////////////
//          Configuración motor DC      ////////////
////////////////////////////////////////////////////
const int Control_fwd = 25;                //  Pin AIN1  [Control del sentido de rotaciÃ³n +]
const int Control_back = 26;            //  Pin AIN2   [Control del sentido de rotaciÃ³n -]
const int Control_v = 12;                 //  Pin PWMA    [Control de la velocidad de rotaciÃ³n]
int MotorSpeed = 0;                   // Velocidad del motor  0..1024
int MotorDirection = 1;               // Avanzar (1) o Retroceder (0)

////////////////////////////////////////////////
//         Variables para Sensor Distancia /////
////////////////////////////////////////////////
VL53L0X SensorToF;
double distancia = 25;
double last_distancia = 0;
double old_d = 0;                         
double old = 0;  
double t_distancia = 0;
double t_old;
double t1; // para temporizar la medición de la velocidad
double t2;

////////////////////////////////////////////
//      Varaibles para código general //////
////////////////////////////////////////////
int dead = 1;     // flag buscar zona muerta
int deadband = 300;     //valor de deadband inicial en caso de no querer buscar deadband
int tiempo_inicial = 0;
bool flag = true; //flag para sincronizar
bool start = false; // variable para iniciar la prueba
String estado;      //Variable para saber el estado del carro
bool run_test = false;

////////////////////////////////////////////
// Variables y parametros PIDs /////////////
////////////////////////////////////////////
double v_lider = 0, mierror, v_medida = 0;
double x_ref = 20;
double u_distancia;                         //Actuacion calculada por distancia                           
double u_velocidad;                         //Actuacion calculada por velocidad
double u;                                   //Actuacion ponderada
int umax = 400, umin = -umax;				//C5
double Kp = 40, Ki = 20, Kd = 0;             //Ganancias PID control de distancia
double Kp_v = 40, Ki_v = 20, Kd_v = 0;       //Ganancias PID control de velocidad   
int SampleTime = 100;                       //Tiempo de muestreo ambos controladores
double etha = 0.5;
double alpha = 0;
double N = 10; //Constante Filtro Derivativa D(s)=s/(1+s/N)
double error_distancia;
double error_velocidad;
double ponderado;
double rf = 0;
int lim = 10;
PID myPID(&error_distancia, &u_distancia, &rf, Kp, Ki, Kd, DIRECT);
PID myPID_v(&error_velocidad, &u_velocidad, &rf, Kp_v, Ki_v, Kd_v, DIRECT);

//////////////////////////////////////////////////////

void loop() {

    //Verificación de conexion MQTT
    
    /*if(run_test){
    	client.disconnect();
    	start = true;
    	run_test = false;
    }*/
    if(flag==false){   // Envia desfase para sincronizar datos de la prueba
        String delta = String((millis()-tiempo_inicial)*0.001);
        delta.toCharArray(msg, delta.length() + 1);                                                                           
        client.publish(trenesdesfase, msg);
        flag = true;
        start = true;
        Serial.println("Sync!");
        client.disconnect();
    }  // Da comienzo a la prueba

    if(!start){
        if (!client.connected()) {
                reconnect();
                estado = String(carro) + " Reconectando";
                estado.toCharArray(msg, estado.length() + 1);                                                                           
                client.publish(trenesestado, msg);
            }
        client.loop();
        return;
    }

    // Si recibe la señal de inicio  
    // Medicion  y Envio de Variables
    // Mediciones  
    medi = 0;
    int count_cam=0;
    int last_dy = 0;
    MD md;
	
    /* Rutina 1 mediciones */
    for (int i = 0; i < 20 ; i++) {
        if (i % 5 == 1) {    // solo algunas mediciones de ToF. No se actualiza muy seguido. Pero ya no hay tiempo perdido entre mediciones.
            uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
            medi += range;
        }
        mousecam_read_motion(&md);
        int curr_dy = (int8_t) md.dy;
        if(curr_dy*last_dy < 0 && abs(curr_dy) > abs(last_dy)){
            curr_dy = last_dy;
        }
        count_cam += curr_dy;
        last_dy = curr_dy;
        uint32_t cam_t = micros();
        while(micros() - cam_t < DELAY_CAM);	//retardo en vez de utilizar delay
    }
    mousecam_read_motion(&md);
    count_cam += (int8_t) md.dy;
    v_medida= (scale*1000)*count_cam/(millis()-old);   // Problema: Bajé el foco del lento para aumentar la resolución
                                                        // Funciona pero hay que revisar cada lente para mejorarla.
    if(abs(v_medida) > vel_lim){
		v_medida = last_vel;
	}
	last_vel = v_medida;
    old = millis();

    /////////////////////////////////////////////////
	/* Rutina 2 mediciones */
	/*v_medida = 0;
	double vel_now = 0;
	for (int i = 0; i < 20; i++){
		// Sensor distancia
		if(i % 5 == 1){
			uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
			medi += range;
		}
		
		// Sensor velocidad
		uint32_t cam_t = micros();
		while(micros() - cam_t < DELAY_CAM);
		mousecam_read_motion(&md);
		int dy = (int8_t) md.dy;
		vel_now = 1000000*scale*dy/(micros() - cam_t);
		if(abs(vel_now) > vel_lim){
			vel_now = vel_last;
		}
		//Serial.println(vel_now);
		
		v_medida += filterVel.filtering(vel_now);
		vel_last = vel_now;		
	}

	v_medida = v_medida / 20;*/
	   
	medi = medi/40; // promediamos 4 mediciones del sensor de distancia. No se si sirve de algo
	if(medi > 90){
		medi = last_distancia;
	}
	last_distancia = medi;
	double medi_f = filterPos.filtering(medi);
	Serial.println(medi_f);
        
    error_distancia = (x_ref - medi_f);  
    error_velocidad = (v_medida - v_lider);

    //if(abs(error_distancia < 0.6))	error_distancia = 0;

    myPID.SetTunings(Kp, Ki, Kd);
    myPID.SetOutputLimits(umin, umax);
    myPID.Compute();
    myPID.SetSampleTime(SampleTime);
    myPID_v.SetTunings(Kp_v, Ki_v, Kd_v);
    myPID_v.SetOutputLimits(umin, umax);
    myPID_v.Compute();
    myPID_v.SetSampleTime(SampleTime);

    ponderado = etha*abs(error_distancia); //abs(v_medida - v_lider);
    //ponderado = (1 - etha) * abs(x_ref - medi) + etha * abs(v_medida - v_lider);  // error ponderado para evitar oscilación cuando el error es pequenho
    //u = (1 - etha) * u_distancia + etha * ( u_velocidad);                   // ponderacion de actuaciones 
    u = u_distancia + alpha*u_velocidad;
    
    // Envio cada 50 ms

    uint32_t time_now = millis() - t_old;
    if( time_now >= 50 ){
        monitor_data.timestamp = u;
        monitor_data.position = medi_f;
        monitor_data.velocity = v_medida;
        esp_now_send(mac_addr_broadcast, (uint8_t*) &monitor_data, sizeof(monitor_data));         // 'True' broadcast, no hay ACK del receptor
        //esp_now_send(NULL, (uint8_t*) &v_lider, sizeof(v_lider));                     // Destino NULL para iteracion a todos los receptos. Sí hay ACK del receptor.
        t_old = millis();
    }

    ////////////////////////////////////////////////////////
    //        Rutina del Carro                 /////////////
    ////////////////////////////////////////////////////////

    if ((medi > 200))   // si no hay nada enfrente: detenerse o desacelerar
    {
        if (MotorSpeed < 200) MotorSpeed = MotorSpeed -  10 ; 
        else MotorSpeed = 0; 
        myPID.SetMode(MANUAL); // para que no siga integrando
        myPID_v.SetMode(MANUAL); // para que no siga integrando
    }

    else
    {
        if (u < -lim)  //lim es un valor arbitrario donde se desea que el carro no se mueva si |u|<=lim
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
            myPID.SetMode(MANUAL);            // apaga el PID de distancia
            myPID_v.SetMode(MANUAL);          // apaga el PID de velocidad
            MotorSpeed = 1.5 * int(abs(u));
            if (u > 0)  MotorDirection = 1;
            else  MotorDirection = 0;
        }

        if (((u >= -lim) && (u <= lim) && (abs(ponderado) <= 0.75)))
        {
            MotorSpeed = 1;
            if (u < 0)  MotorDirection = 1;
            else  MotorDirection = 0;
        }
        if( (abs(ponderado) > 0.75) && (medi < 200)){ // condicion para revivir el PID
            myPID.SetMode(AUTOMATIC);
            myPID_v.SetMode(AUTOMATIC);
        }        
    }
    SetMotorControl();
}

/////////////////////////////////////////////////////////////

void SetMotorControl()
{
    if (MotorDirection == 1)            //Avanzar
    {
        digitalWrite(Control_fwd, LOW);
        digitalWrite(Control_back, HIGH);
    }
    else                                //Retroceder
    {
        digitalWrite(Control_fwd, HIGH);
        digitalWrite(Control_back, LOW);
    }
    
    ledcWrite(Control_v, MotorSpeed); // esp32
}

/* MQTT Callback */
void callback(char* topic, byte* payload, unsigned int length) {
    String mensaje;
    for (int i = 0; i < length; i++) {
        char nuevo = (char)payload[i];
        mensaje.concat(nuevo);
    }
    ////////////////////////////////
    // QUE HACE AL RECIBIR DATOS ///
    ////////////////////////////////
  
    if (String(topic) == "trenes/sync") {
        if (String(mensaje)=="True") {
            tiempo_inicial = millis();
            Serial.println("Sync Recibido");
            flag = false;
            myPID.SetMode(AUTOMATIC);            // prende el PID de distancia  
            myPID_v.SetMode(AUTOMATIC);          // prende el PID de velocidad   
        }
        else if (String(mensaje)=="False"){
            Serial.println("Detener");
            flag = true;
            start = false;
            MotorSpeed = 0;
            SetMotorControl();
            myPID.SetMode(MANUAL);            // apaga el PID de distancia  
            myPID_v.SetMode(MANUAL);          // apaga el PID de velocidad     
            u_distancia = 0;
            u_velocidad = 0; 
        }
    }
    if (strcmp(topic ,trenesp) == 0) {
        Kp = mensaje.toFloat();
    }
    if (strcmp(topic , trenesi) == 0) {
        Ki = mensaje.toFloat();
    }
    if (strcmp(topic , trenesd) == 0) {
        Kd = mensaje.toFloat();
    }
    if (strcmp(topic ,trenesp_v) == 0) {
        Kp_v = mensaje.toFloat();
    }
    if (strcmp(topic , trenesi_v) == 0) {
        Ki_v = mensaje.toFloat();
    }
    if (strcmp(topic , trenesd_v) == 0) {
        Kd_v = mensaje.toFloat();
    }
    if (strcmp(topic , "trenes/etha") == 0) {
        etha = mensaje.toFloat();
    }
    if (strcmp(topic, "trenes/alpha") == 0) {
    	alpha = mensaje.toFloat();
    }
    if(strcmp(topic, "trenes/start") == 0){
    	run_test = true;
    }
    if (strcmp(topic , "trenes/u_lim") == 0) {
        umax = mensaje.toInt();
        umin = -umax;
    }
    if (strcmp(topic , "trenes/carroL/v_lider") == 0) {
        v_lider = mensaje.toFloat();
    }
    if (strcmp(topic , "trenes/ref") == 0) {
        x_ref = mensaje.toFloat();
    }
    if (strcmp(topic , trenessampletime) == 0) {
        SampleTime = mensaje.toInt();
    }
    if (strcmp(topic , trenestransmision) == 0) {
        t_envio = mensaje.toInt();
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
    v_lider = rcv_data.velocity;
    uint32_t t_now = millis(); 
    //Serial.println("V_medida = " + String(v_medida) + ", V_lider = " + String(v_lider) + ", delta_t = " + String(t_now - rcv_time));
    rcv_time = t_now;
    // Definir que hacer al recibir
}
