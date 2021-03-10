
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
#include <WiFiUdp.h>


//--------- Definiciones ESPNOW ---------------//
#define PAYLOAD_SIZE    32                  // Tamano en Bytes de los paquetes a enviar por ESP-NOW
#define DATARATE        WIFI_PHY_RATE_6M   // Para cambiar el bitrate de ESP-NOW a 24Mbps
#define CHANNEL         1                   // Canal WiFi
  
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


MD md;
int count_cam=0;


int yy=0;  // coordenadas obtenidas por sensor ADNS3080 (dead reckoning)
double temp_cal;
double scale; // escalamiento de cuentas ADNS a cm 
double medi = 10;
double xx = 10;


// filtro ToF
int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[10];
double AVERAGED = 0;

// filtro ADNS3080
int INDEXv = 0;
double VALUEv = 0;
double SUMv = 0;
double READINGSv[10];
double AVERAGEDv = 0;

//moving avg
int arrNumbers[3] = {0};

int pos = 0;
float newAvg = 0;
float oldAvg = 0;
int newSum = 0;
long sum = 0;
int len = sizeof(arrNumbers) / sizeof(int);
//int count = sizeof(sample) / sizeof(int);




///////////////////////////////////////
//     Configuración MQTT   ///////////
///////////////////////////////////////
//const char* ssid = "ac3e";  // Nombre WiFi a conectarse
//const char* password = "rac3e/07"; // Contraseña WiFi
const char* ssid = "DPTO. WIFI BERGEN";
const char* password = "110725011";

//const char* ssid = "VTR-6351300";
//const char* password = "zkd2bxhcHqHm";

//const char* mqtt_server = "10.1.28.117";  // IP de la Raspberry
//const char* mqtt_server = "10.1.30.82";
//const char* mqtt_server = "192.168.1.100";  // IP fvp
//const char* mqtt_server = "192.168.0.14";
const char* mqtt_server = "192.168.100.12";
const int udpPort = 5555;                   //puerto de envio UDP*
String estado;        //variables para los mensajes publicados en MQTT

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;  
char msg[50];
WiFiUDP udp;

////////////////////////////////////////////
//      Configuracion ESP-NOW       ////////
////////////////////////////////////////////
uint8_t mac_addr_A[] = {0xC8, 0x2B, 0x96, 0xB4, 0xE6, 0xCC};
uint8_t mac_addr_broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool received, resend;      // Variables de control
uint32_t rcv_time = 0;

typedef struct {
    uint8_t leader;    // == 0xA0 if leader
    uint32_t timestamp;
    double position;
    double velocity;
} ESPNOW_payload;

ESPNOW_payload monitor_data;
ESPNOW_payload rcv_data;

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
//         Variables para Sensor Distancia /////
////////////////////////////////////////////////
VL53L0X SensorToF;
double distancia = 25;
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

////////////////////////////////////////////
// Variables y parametros PIDs /////////////
////////////////////////////////////////////
double v_lider = 0, mierror;
float v_medida = 0;
float v_ref = 0;
double x_ref = 15;
double u_distancia;                         //Actuacion calculada por distancia                           
double u_velocidad;                         //Actuacion calculada por velocidad
double u;                                   //Actuacion ponderada
double u_send; //Actuacion para ser enviada sin zona muerta

int umin = -800, umax = 800;//C5
double Kp =51, Ki = 20, Kd = 0;             //Ganancias PID control de distancia
double Kp_v = 10, Ki_v = 1, Kd_v = 0;       //Ganancias PID control de velocidad   
int SampleTime = 50;                       //Tiempo de muestreo ambos controladores
double etha = 0.5;
int Frequency = 100;
int PreviousFrequency;
int deadband1 = 0;
double alpha = 0; // constante para el control con velocidad lider

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
    if (!client.connected()) {
        reconnect();
        estado = String(carro) + " Reconectando";
        estado.toCharArray(msg, estado.length() + 1);                                                                           
        client.publish(trenesestado, msg);
    }
    
    client.loop();
    if(flag==false){   // Envia desfase para sincronizar datos de la prueba
        String delta = String((millis()-tiempo_inicial)*0.001);
        delta.toCharArray(msg, delta.length() + 1);                                                                           
        client.publish(trenesdesfase, msg);
        flag = true;
        start = true;
        Serial.println("Sync!");
    }  // Da comienzo a la prueba
    
    else if (start)
    {   
        // Si recibe la señal de inicio  
        // Medicion  y Envio de Variables
        // Mediciones  
        xx=medi; // memoria para filtrar medi (medicion de distancia)
        medi=0;
      
        count_cam += (int8_t) md.dy;        
        old=micros();
        SUM=count_cam;
        //MD md;

        //int val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
        //delayMicroseconds(3500);
        for (int i = 0; i < 2 ; i++) {
            
                uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
                medi=medi+range;
         
          delay(21);  // retardo arbitrario. No cambia mucho la medición del adns3080. Si se lee el registro de movimiento muy seguido entrega 0s pq no ha detectado movimiento.
        }

        medi = medi/2;
        Serial.print(medi);
        newSum = movingSum(arrNumbers, &sum, pos, len, medi);
        newAvg = newSum/(float)len;
        Serial.print(" ,");
        Serial.println(0.7*newAvg+0.3*oldAvg);
        pos++;
        if (pos >= len){
          pos = 0;
        }
        oldAvg = newAvg;
        medi =(0.8*newAvg+0.2*oldAvg)/10;
        
        
        

        
 

        /*
        int WINDOW_SIZEv = 5;
        SUMv = SUMv - READINGSv[INDEX];       // Remove the oldest entry from the sum
        VALUEv = v_medida;        // Read the next sensor value
        READINGSv[INDEX] = VALUEv;           // Add the newest reading to the window
        SUMv = SUMv + VALUEv;                 // Add the newest reading to the sum
        INDEXv = (INDEXv+1) % WINDOW_SIZEv;   // Increment the index, and wrap to 0 if it exceeds the window size

        AVERAGEDv = SUMv / (float)WINDOW_SIZEv;       // Divide the sum of the window by the window size for the resu
        v_medida=AVERAGEDv;*/
        
        //Serial.print(v_medida);Serial.print(",");Serial.println(median);

        //Serial.print(millis()-old);
        
        //Serial.println(count_cam);
        //Serial.println(v_medida);
            
        error_distancia = (x_ref - medi);
       // error_velocidad = (0.7*newAvg+0.3*oldAvg - v_ref); 
    
        myPID.SetTunings(Kp, Ki, Kd);
        myPID.SetOutputLimits(umin, umax);
        myPID.Compute();
        myPID.SetSampleTime(SampleTime);
        /*
        myPID_v.SetTunings(Kp_v, Ki_v, Kd_v);
        myPID_v.SetOutputLimits(umin, umax);
        myPID_v.Compute();
        myPID_v.SetSampleTime(SampleTime);
        */
        ponderado = abs(v_medida);  // error ponderado para evitar oscilación cuando el error es pequenho
        
        u = (1 - etha) * u_distancia + etha * ( u_velocidad);                   // ponderacion de actuaciones 
        alpha = 0;
        u =  u_distancia + alpha * ( u_velocidad);                   // control nuevo alpha>0, debería mejorar alpha > alpha_min por determinar
        
        // Envio cada 50 ms
    
        /*if ((millis()-t_old) > t_envio){
            String envio = String(millis()-tiempo_inicial) + ", " + String(medi); //String(u)+ "; " + String(medi) + "; " + String(ponderado)+ "; " + String(millis()-tiempo_inicial);
            envio.toCharArray(msg, envio.length() + 1);                    // Datos enviados para analizar controlador
            client.publish(trenestest, msg);
            t_old = millis();
        }*/
        uint32_t time_now = millis() - t_old;
        t_old = millis();
        
               estado = String(time_now)+","+String(medi)+","+String(x_ref)+","+String(error_distancia)+","+String(Kp)+","+String(Ki)+","+String(Kd)+","+String(u);
                estado.toCharArray(msg, estado.length() + 1);                                                                           
                
                //Serial.println(estado);
                //client.publish("esp/datos", msg);
                //------------------------------------
                udp.beginPacket(mqtt_server,udpPort);
                udp.printf(msg);      //
                udp.endPacket();
        
/*
    else{
      delay(47-(millis()-t_old)); //49
      String envio = String(u)+ ", " + String(medi) + ", " + String(v_medida)+ ", " + String(millis()-tiempo_inicial);
      envio.toCharArray(msg, envio.length() + 1);                    // Datos enviados para analizar controlador
      client.publish(trenestest, msg);
      t_old = millis();
    }
    
  */
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
                MotorSpeed = 0;
                if (u > 0)  MotorDirection = 1;
                else  MotorDirection = 0;
            }
  
            if (((u >= -lim) && (u <= lim) && (abs(ponderado) <= 0.75))) // detener si estamos en reposo
            {
                MotorSpeed = 0;
                if (u < 0)  MotorDirection = 1;
                else  MotorDirection = 0;
            }
            if( (medi < 200)){ // condicion para revivir el PID
                myPID.SetMode(AUTOMATIC);
                myPID_v.SetMode(AUTOMATIC);
            }        
        }
        
        int test_sin_control=0;
        if (test_sin_control)
        {
          myPID.SetMode(MANUAL);            // apaga el PID de distancia
          myPID_v.SetMode(MANUAL);          // apaga el PID de velocidad
          MotorSpeed=0;
        }
        
        
        SetMotorControl();
    }
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
    if (strcmp(topic , "trenes/u_lim") == 0) {
        umax = mensaje.toInt();
        umin = -umax;
    }
    if (strcmp(topic , "trenes/carroL/v_lider") == 0) {
        v_lider = mensaje.toFloat();
    }
    if (strcmp(topic , "trenes/carroL/vref") == 0) {
        v_ref = mensaje.toFloat();
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

///////////////////////////////////////////////////////////////////
//void SensorToF() {
//
//  old_d = distancia;
//  VL53L0X_RangingMeasurementData_t measure;
//  lox.rangingTest(&measure, false);
//  
//  if (measure.RangeStatus != 4) 
//  {  // phase failures have incorrect data
//    distancia = measure.RangeMilliMeter*0.1;
//    distancia = distancia*0.1 + old_d*0.9;
//  }
//  mierror = x_ref - distancia;
//  String error = String(mierror, 2) + "," + String(u, 2) + "," + String(distancia) + "," + String(deadband);
//  error.toCharArray(msg, error.length() + 1);                                                                            // Datos enviados para analizar controlador
//  client.publish(treneserror, msg); 
//}

//////////////////////////////////////////

/* ESPNOW Sent Callback Function */
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if ( status == ESP_NOW_SEND_SUCCESS ) {
        //Serial.println("Delivery Success");
        //Serial.printf("sent = %u (CPU%d)\n",sent,xPortGetCoreID());
        //resend = false;
    }
    else {
        Serial.println("Delivery Fail");
        resend = true;
    }
}

/* ESPNOW Receive Callback Function */
void OnDataRecv(const uint8_t *mac, const uint8_t *Data, int len)
{
    /*Serial.printf("%b%b%b%b\n",Data[0],Data[1],Data[2],Data[3]);
    if(Data[0] != 1 && Data[2] != 1){   // chequea si el paquete proviene del lider, o es info para monitoreo
        return;
    }*/
    memcpy(&rcv_data, Data, len);
    if(rcv_data.leader != 0xA0){
        return;
    }
    v_lider = rcv_data.velocity;
    uint32_t t_now = millis(); 
    Serial.println("V_medida = " + String(v_medida) + ", V_lider = " + String(v_lider) + ", delta_t = " + String(t_now - rcv_time));
    rcv_time = t_now;
    // Definir que hacer al recibir
}

float movingSum(int *ptrArrNumbers, long *ptrSum, int pos, int len, int nextNum)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
  //Assign the nextNum to the position in the array
  ptrArrNumbers[pos] = nextNum;
  //return the Sum
  return *ptrSum;
}
