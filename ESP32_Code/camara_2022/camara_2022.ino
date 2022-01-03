#include "SPI.h"
#include <Wire.h>   // ???
#include <VL53L0X.h>  // Pololu 1.0.2
#include "KickFiltersRT.h"
#include <PID_v1.h>

////////////////////////////////////////////
//                                        //
//       DEFINICIONES SENSOR OPTICO       //
//                                        //
////////////////////////////////////////////

#define REFERENCE 8
#define KP 15
#define KI 50
#define KD 3
#define CUTFREQ .5
#define DELTAT 50

#define DELAY_CAM   50

#define PIN_SS       D8
#define PIN_MISO     19
#define PIN_MOSI     23
#define PIN_SCK      18

#define PIN_MOUSECAM_RESET     14
#define PIN_MOUSECAM_CS        16 //D3

#define ADNS3080_PIXELS_X                30
#define ADNS3080_PIXELS_Y                30

#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

#define ADNS3080_PRODUCT_ID_VAL        0x17

VL53L0X SensorToF;
struct MD               // Estructura para las mediciones de mouse_cam
{
	uint8_t motion;
	int8_t dx, dy;
	uint8_t squal; //surface quality
	uint16_t shutter;
	uint8_t max_pix;
	int over;
};

////////////////////////////////////////////////////
//          Configuración motor DC      ////////////
////////////////////////////////////////////////////
const int Control_fwd = 25;                //  Pin AIN1  [Control del sentido de rotaciÃ³n +]
const int Control_back = 26;            //  Pin AIN2   [Control del sentido de rotaciÃ³n -]
const int Control_v = 12;                 //  Pin PWMA    [Control de la velocidad de rotaciÃ³n]
int MotorSpeed = 0;                   // Velocidad del motor  0..1024
int MotorDirection = 0;               // Avanzar (1) o Retroceder (0)


void SetMotorControl() {
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
/*-------------------------------------------------------------------*/
void averageDistance(double* x_0, int N){
  for (int i = 0; i < N; i++) {
    uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
    *x_0 += (double) range;
    delay(100);
  }
  *x_0 = *x_0 / (10*N); //[cm]
}
//double scale = 0.0079;	// carro 1
double scale = 0.0124;		// carro 2

/*-------------------------------------------------------------------*/
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
/*-------------------------------------------------------------------*/

double mSpeed;
double velocity;
double reference;
double absvelocity;

double dead;

double Kp = KP, Ki = KI, Kd = KD;
PID myPID(&absvelocity, &mSpeed, &reference, Kp, Ki, Kd, DIRECT);
 
/*-------------------------------------------------------------------*/
void setup() {
  myPID.SetSampleTime(DELTAT);
	Serial.begin(115200);
	Serial.println("");
	/////////////////////////////////////
	///       Inicio Motor         //////
	/////////////////////////////////////
	pinMode(Control_fwd, OUTPUT);   // 1A - Definición Pin como salida
	pinMode(Control_back, OUTPUT);    // 2A - Definición Pin como salida
	pinMode(Control_v, OUTPUT);       // 1,2 EN - Definición Pin como salida
	digitalWrite(Control_v, HIGH);    // Motor off - O Volts (HIGH = 0, por algún motivo)
	ledcSetup(Control_v, 100, 10); //Freq a 100 Hz ESP32 resolución 10 bits
	ledcAttachPin( 12, Control_v);

	///////////////////////////////////////////
	//         Inicio de Sensores           ///
	///////////////////////////////////////////
	Wire.begin(); //Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called only once.
	SensorToF.init();
	SensorToF.setTimeout(500);
	SensorToF.setMeasurementTimingBudget(33000);
	SensorToF.startContinuous();

	pinMode(PIN_MISO,INPUT);
	pinMode(PIN_MOSI,OUTPUT);
	pinMode(PIN_SCK,OUTPUT);
	SPI.begin();
	SPI.setClockDivider(SPI_CLOCK_DIV8);
	SPI.setDataMode(SPI_MODE3);
	SPI.setBitOrder(MSBFIRST);
	Serial.println("Iniciando Cámara");
	if(mousecam_init()==-1)
	{
		Serial.println("Fallo en inicio Cámara");
		while(1);
	}
	Serial.println("Cámara Iniciada Correctamente");

	Serial.printf("Resolution in counts per inch %d\n", (mousecam_read_reg(ADNS3080_CONFIGURATION_BITS) & 0x10)>>4); //Resolution in counts per inch 0=400, 1=1600
  Serial.printf("0x%02x%02x\n" ,mousecam_read_reg(0x1a),mousecam_read_reg(0x19));
	delay(3000); //para que el carro se estabilice al presionar reset

	MD cam_med;

	///////////////////////////////////////////
	////    Motor Dead-Zone Routine	      /////
	///////////////////////////////////////////
	Serial.println("Starting Deadband Routine");

 /*---------------------------------------------------------------*/
 
	//calibracion();
  /*-------------------------------------------------------------------*/

  //mSpeed = 150;
  velocity = 0;
  absvelocity = abs(velocity);
  reference = REFERENCE;
  myPID.SetMode(AUTOMATIC);
}
 /*---------------------------------------------------*/


int cuentasTotal = 0;
uint32_t deltaT = 5;
int kdirection = 0;
uint32_t theTime= 10000;
float cutFreq = CUTFREQ;

KickFiltersRT<float> filtersRT;
butterworth filterButter(2*cutFreq*deltaT/1000);
//butterworth fBW(2*cutFreq*deltaT/1000);


/*------------------------------------------------------------------------------------------------------*/

void loop()
{
  // Test integracion camara

  MD camara;
  int count_cam = 0;
  uint32_t maxtime = (kdirection%2 == 0) ? (1.0)*theTime : theTime; //el motor avanza más en un sentido por el cable USB
  MotorDirection = kdirection%2 + 1; //retroceder o avanzar
  //MotorSpeed = mSpeed; //comentar para el control
  //SetMotorControl();

  /*rutina medición*/
  /*---------------------------------------------------*/
  uint32_t time0 = millis();
  while( millis() - time0 < maxtime){ 
      delay(deltaT);
      count_cam=0;
      do{
        //leer todos los datos del buffer
        mousecam_read_motion(&camara);
        count_cam += (int8_t)camara.dy;
        if(camara.motion & 0x10) //1 = Overflow has occurred
          Serial.println("Overflow");
      } while(camara.motion & 0x80); //1 = Motion occurred, data ready for reading in Delta_X and Delta_Y registers
      cuentasTotal += count_cam;

      /*------------PIDPIDPID--------------*/
      velocity = filtersRT.lowpass((double)count_cam*scale*1000./(double)deltaT, cutFreq, 1000./deltaT);
      //velocity = filterButter.butter_filtering((double)count_cam*scale*1000./(double)deltaT);
      absvelocity = abs(velocity);
      myPID.Compute();
      MotorSpeed = mSpeed;
      SetMotorControl();

      /*Serial.print((double)count_cam*scale*1000./(double)deltaT);
      Serial.print(" ");
      Serial.print(MotorSpeed*(MotorDirection > 0 ? 1:-1)/50.);
      Serial.print(" ");
      Serial.print((MotorDirection > 0 ? 1:-1)*reference);
      Serial.print(" ");
      Serial.println(velocity);*/
  }
  /*---------------------------------------------------*/
  //MotorSpeed = 0;
  //SetMotorControl();
  
  double refTemp = reference;
  reference = 0;
  myPID.Compute();
  //MotorSpeed = mSpeed;
  MotorSpeed = reference;
  SetMotorControl();

  
  /*rutina medición*/
  /*---------------------------------------------------*/
  time0 = millis();
  while( millis() - time0 < maxtime){ 
      //distancewFilterB[0] = filterButter.butter_filtering((double)cuentasTotal*scale);
      delay(deltaT);
      count_cam=0;
      do{
        //leer todos los datos del buffer
        mousecam_read_motion(&camara);
        count_cam += (int8_t)camara.dy;
        if(camara.motion & 0x10) //1 = Overflow has occurred
          Serial.println("Overflow");
      } while(camara.motion & 0x80); //1 = Motion occurred, data ready for reading in Delta_X and Delta_Y registers
      cuentasTotal += count_cam;
      //distancewFilterB[1] = filterButter.butter_filtering((double)cuentasTotal*scale);


      velocity = filtersRT.lowpass((double)count_cam*scale*1000./(double)deltaT, cutFreq, 1000./deltaT);
      //velocity = filterButter.butter_filtering((double)count_cam*scale*1000./(double)deltaT);
      absvelocity = abs(velocity);
      myPID.Compute();
      //MotorSpeed = mSpeed;
      MotorSpeed = reference;
      SetMotorControl();  

      /*Serial.print((double)count_cam*scale*1000./(double)deltaT);
      Serial.print(" ");
      Serial.print(MotorSpeed*(MotorDirection > 0 ? 1:-1)/50.);
      Serial.print(" ");
      Serial.print((MotorDirection > 0 ? 1:-1)*reference);
      Serial.print(" ");
      Serial.println(velocity);*/
  }
  /*---------------------------------------------------*/
  kdirection += 1;
  //reference = -refTemp;
  reference = refTemp;
}

void calibracion(){
	int N = 20; //número de mediciones para calcular promedio
  int deltaT = 9; //ms
 	// Medir distancia a objeto en reposo
  double x_0 = 0;
  uint16_t range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
  delay(200);

  /* Mide distancia inicial */
  averageDistance(&x_0, N);
  Serial.printf("Distancia Inicial = %.2f [cm]\n", x_0);

  int count_cam = 0;                      // variable para guardar las 'cuentas' entregadas por el sensor (yy1)
  MD camara;                              // estructura para las mediciones del mouse_cam

  MotorDirection = 1; //retroceder
  MotorSpeed = 200;
  SetMotorControl();

  uint32_t time0 = millis();
  uint32_t t_last = millis();

  while( millis() - time0 < 1400){ // tiempo total de recorrido son 1.4 segundos
    if(millis() - t_last >= deltaT){
      do{
        mousecam_read_motion(&camara);
        count_cam += (int8_t)camara.dy;
        if(camara.motion & 0x10) //1 = Overflow has occurred
          Serial.println("Overflow");
      } while(camara.motion & 0x80); //1 = Motion occurred, data ready for reading in Delta_X and Delta_Y registers
    }
  }

  MotorSpeed = 0;
  SetMotorControl();

  delay(500); //asegurar que se detuvo

  do{
    mousecam_read_motion(&camara);
    count_cam += (int8_t)camara.dy;
    if(camara.motion & 0x10)
      Serial.println("Overflow");
  } while(camara.motion & 0x80);

  double x_ff = 0;
  averageDistance(&x_ff, N);
  Serial.printf("Distancia Final = %.2f [cm]\n\n", x_ff);
  Serial.printf("Distancia Recorrida = %.2f [cm]\n", -(x_ff - x_0));
  Serial.printf("Cuentas Camara = %d\n", count_cam);

  scale = -(x_ff - x_0) / count_cam;
  Serial.printf("Scale = %.5f [cm/cuentas]\n", scale);
  delay(1000);
  Serial.println("------- Prueba Terminada ----------");
  /*---------------------------------------------------*/
  Serial.println("------- Comprobación Scale---------");
// Medir distancia a objeto en reposo
  x_0 = 0;
  range = SensorToF.readReg16Bit(SensorToF.RESULT_RANGE_STATUS + 10);
  delay(200);

  /* Mide distancia inicial */
  averageDistance(&x_0, N);
  Serial.printf("Distancia Inicial = %.2f [cm]\n", x_0);

  count_cam = 0;                  // variable para guardar las 'cuentas' entregadas por el sensor (yy1)
  camara;                         // estructura para las mediciones del mouse_cam

  MotorDirection = 0; //avanzar
  MotorSpeed = 200;
  SetMotorControl();

  time0 = millis();
  t_last = millis();

  while( millis() - time0 < 1200){
    if(millis() - t_last >= deltaT){
      do{
        mousecam_read_motion(&camara);
        count_cam += (int8_t)camara.dy;
        if(camara.motion & 0x10)
          Serial.println("Overflow");
      } while(camara.motion & 0x80);
    }
  }

  MotorSpeed = 0;
  SetMotorControl();

  delay(500); //asegurar que se detuvo

  do{
    mousecam_read_motion(&camara);
    count_cam += (int8_t)camara.dy;
    if(camara.motion & 0x10)
      Serial.println("Overflow");
  } while(camara.motion & 0x80);

  averageDistance(&x_ff, N);
  Serial.printf("Distancia Final = %.2f [cm]\n\n", x_ff);
  Serial.printf("Distancia Recorrida = %.2f [cm]\n", -(x_ff - x_0));
  Serial.printf("Distancia según cámara = %.2f [cm]\n", count_cam*scale);
  Serial.printf("Cuentas Camara = %d\n", count_cam);
  scale = -(x_ff - x_0) / count_cam;
}
