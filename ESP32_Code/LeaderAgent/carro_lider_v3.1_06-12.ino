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

//filter1 filterPos(0.7265, 0.1367, 0.1367);
filterIIR filterPos(b_win, NULL, 5);
filterIIR filterVel(b_hamm, NULL, 20);

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
//const char* ssid = "VTR-6351300";
//const char* password= "zkd2bxhcHqHm";
const char* ssid = "MOVISTAR_7502";
const char* password = "X27JgSvWteS2US4";

//const char* mqtt_server = "10.1.28.117";  // IP de la Raspberry
//const char* mqtt_server = "192.168.137.1";
//const char* mqtt_server = "192.168.1.100";  // IP fvp
const char* mqtt_server = "192.168.1.114";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  
char msg[50];  

////////////////////////////////////////////
//      Configuracion ESP-NOW       ////////
////////////////////////////////////////////
uint8_t mac_leader[] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};		// custom MAC
uint8_t mac_A[] = {0xc8, 0x2b, 0x96, 0xb4, 0xe6, 0xcc};
uint8_t mac_addr_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct {
    uint32_t timestamp;
    double position;
    double velocity;
} ESPNOW_payload;

esp_now_peer_info_t peerInfo;   // Estructura para anadir peers a espnow


////////////////////////////////////////////////////
//          Configuración motor DC      ////////////
////////////////////////////////////////////////////
const int Control_fwd = 25;                //  Pin AIN1  [Control del sentido de rotaciÃ³n +]
const int Control_back = 26;            //  Pin AIN2   [Control del sentido de rotaciÃ³n -]
const int Control_v = 12;                 //  Pin PWMA    [Control de la velocidad de rotaciÃ³n]
int MotorSpeed = 0;                   // Velocidad del motor  0..1024
int MotorDirection = 1;               // Avanzar (1) o Retroceder (0)

////////////////////////////////////////////////
//         Variables para Sensor Distancia //////
////////////////////////////////////////////////
VL53L0X SensorToF;
double medi = 10;                
double temp_cal;     // medicion temporal distancia [cm]      
uint32_t old = 0;   // tiempo para mediciones 
uint32_t t_old;     // tiempo para envio
uint32_t t1;        // tiempo delay inicial
uint32_t t2;

//////////////////////////////////////////////
//          Variables sensor velocidad      //
/////////////////////////////////////////////
struct MD {              // Estructura para las mediciones de mouse_cam
    byte motion;
    char dx, dy;
    byte squal;
    word shutter;
    byte max_pix;
    int over;
};
double last_vel = 0;
double vel_lim = 50;
double scale;           // escalamiento de cuentas ADNS [cm/cuentas] 

////////////////////////////////////////////
//      Varaibles para código general //////
////////////////////////////////////////////
int dead = 1;     // flag buscar zona muerta
int deadband = 300;     //valor de deadband inicial en caso de no querer buscar deadband
int tiempo_inicial = 0;
bool flag = true; //flag para sincronizar
bool start = false; // variable para iniciar la prueba
bool run_test = false;

////////////////////////////////////////////
// Variables y parametros PID //////////////
////////////////////////////////////////////
double v_lider = 0, mierror, v_medida, v_min = 1000;
double u = 1;       // Actuacion (?)

//////////////////////////////////////////////////////
double READINGS[5];
double SUM = 0;
unsigned int INDEX = 0;
double VALUE = 0;


void loop() {

    //Verificación de conexion MQTT
   
    if(!flag){   // Envia desfase para sincronizar datos de la prueba
        String delta = String((millis()-tiempo_inicial)*0.001);
        delta.toCharArray(msg, delta.length() + 1);                                                                           
        client.publish("trenes/carroA/desfase", msg);
        flag = true;
        //start = true;
        Serial.println("Sync!");
    }  // Da comienzo a la prueba
	
    if(run_test){
    	client.disconnect();
    	start = true;
    	t1 = millis();
    	run_test = false;
    }
	
    if(!start){
        if (!client.connected()) 
        {
            esp_now_deinit();
            reconnect();
            esp_now_init();
        }
        client.loop();
        return;
    }
	
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
	medi = filterPos.filtering(medi);
    /*int WINDOW_SIZE = 5;
    SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
    VALUE = medi/4;        // Read the next sensor value
    READINGS[INDEX] = VALUE;           // Add the newest reading to the window
    SUM = SUM + VALUE;                 // Add the newest reading to the sum
    INDEX = (INDEX+1) % WINDOW_SIZE;   // Increment the index, and wrap to 0 if it exceeds the window size

    double AVERAGED = SUM / (float)WINDOW_SIZE;      // Divide the sum of the window by the window size for the resu
    medi = AVERAGED/10; // promediamos 4 mediciones del sensor de distancia. No se si sirve de algo*/

    Serial.println(medi);

    
	//double medi_y = filterVel.filtering(v_medida);   
	//Serial.println(String(medi) + " " + String(medi_y));
	//Serial.println(String(v_medida) + " " + String(medi_y));


    /* Envio por ESP-NOW */
    uint32_t time_now = millis() - t_old;
    if( time_now > 50 ){
        ESPNOW_payload.timestamp = time_now;
        ESPNOW_payload.velocity = v_medida;
        ESPNOW_payload.position = 20;
        esp_now_send(mac_addr_broadcast, (uint8_t*) &ESPNOW_payload, sizeof(ESPNOW_payload));         // 'True' broadcast, no hay ACK del receptor
        t_old = millis();
    }

    ////////////////////////////////////////////////////////
    //        Rutina del Carro                 /////////////
    ////////////////////////////////////////////////////////
	
	if(millis() - t1 < 3000)	return;     // retardo inicial al comienzo de la prueba para comenzar a capturar datos con el tren detenido
	
    if (abs(u) != 0 && medi > 12.0)
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
            u = 1;                            // Si queda en cero, al momento de reiniciar la prueba ingresara directamente a este bucle
            run_test = false;
            //reconnect();
        } 
    }
    SetMotorControl();
}

///////////////////////////////////////////////////////

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

    ledcWrite(Control_v, MotorSpeed); // esp32 (PIN, duty cycle PWM)
}



void callback(char* topic, byte* payload, unsigned int length) {
    String mensaje;
    for (int i = 0; i < length; i++) {
        char nuevo = (char)payload[i];
        mensaje.concat(nuevo);
    }
    Serial.println("Mensaje Recibido por MQTT");
    ////////////////////////////////
    // QUE HACE AL RECIBIR DATOS ///
    ////////////////////////////////
    if (String(topic) == "trenes/sync") {
        if (String(mensaje)=="True") {
            tiempo_inicial = millis();
            Serial.println("Sync Recibido");
            flag = false;
        }
        else if (String(mensaje)=="False"){
            Serial.println("Detener");
            flag = true;
            start = false;
            MotorSpeed = 0;
            SetMotorControl();      
        }
    }
    if (String(topic) == "trenes/start"){
    	if(String(mensaje) == "True"){
    		run_test = true;
    		Serial.println("Starting Test");
    	}
    }
    if (strcmp(topic , "trenes/carrol/u") == 0) {
        u = mensaje.toFloat();
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
