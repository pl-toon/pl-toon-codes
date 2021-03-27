// Test the ADNS3080 Optical Flow Sensor
// based on: Example of AP_OpticalFlow library by Randy Mackay. DIYDrones.com
//

#include "SPI.h"
// ADNS3080 hardware config
#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30
#define ADNS3080_CLOCK_SPEED        24000000

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
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

// Configuration Bits
#define ADNS3080_LED_MODE_ALWAYS_ON        0x00
#define ADNS3080_LED_MODE_WHEN_REQUIRED    0x01

#define ADNS3080_RESOLUTION_400     400
#define ADNS3080_RESOLUTION_1600    1600

// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF  0x02

#define ADNS3080_FRAME_RATE_MAX         6469
#define ADNS3080_FRAME_RATE_MIN         2000


#define AP_SPI_DATAIN          19  //MISO
#define AP_SPI_DATAOUT         23  //MOSI
#define AP_SPI_CLOCK           18  //SCK
#define ADNS3080_CHIP_SELECT   16  //SS
#define ADNS3080_RESET         14   //RESET

byte orig_spi_settings_spcr;
byte orig_spi_settings_spsr;
int _cs_pin=ADNS3080_CHIP_SELECT;
int _reset_pin=1; // set to 1 if you have reset connected
unsigned int last_update;
boolean _overflow=false;
boolean _motion=false;
int raw_dx;
int raw_dy;
unsigned int surface_quality;

void setup() 
{
  Serial.begin(115200);
  Serial.println("www.bot-thoughts.com\nOptical Flow test program V1.0");
  Serial.println("Based on APM/ArduCopter AP_OpticalFlow library");
  Serial.println("by Randy Mackay");
  delay(1000);
  
  // flowSensor initialization
  if( initOF() == false )
    Serial.println("Failed to initialise ADNS3080");

  delay(1000);
}

void loop() 
{
  int value;
  
  display_menu();

  // wait for user to enter something
  while( !Serial.available() ) {
    delay(20);
  }

  // get character from user
  value = Serial.read();
  
  switch( value ) {
  
  case 'c' :
    //display_config();
    break;
      
  case 'f' :
    //set_frame_rate();
    break;
      
  case 'i' :
    // display image
    display_image();
    break;
      
  case 'I' :
    display_image_continuously();
    break;
      
  case 'm' :
    display_motion();
    break;
      
  case 'r' :
    // set resolution
    //set_resolution();
    break;
      
  case 's' :
    //set_shutter_speed();
    break;
      
  case 'z' :
    //flowSensor.clear_motion();
    break;    
    
  case '\r' : // ignore return type characters
  case '\n' :
    break;
      
  default:
    Serial.println("unrecognised command");
    Serial.println();
    break;  
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DISPLAY FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Prints out a list of functions.
void display_menu()
{
    Serial.println();
    Serial.println("please choose from the following options:");
//  Serial.println("     c - display all config");
//  Serial.println("     f - set frame rate");
  Serial.println("     i - display image");
  Serial.println("     I - display image continuously");
  Serial.println("     m - display motion");  
//  Serial.println("     r - set resolution");
//  Serial.println("     s - set shutter speed");
//  Serial.println("     z - clear all motion");
//  Serial.println("     a - frame rate auto/manual");
  Serial.println();
}


// Captures and displays image from flowSensor
void display_image()
{ 
  Serial.println("image data --------------");
  print_pixel_data(&Serial);
  Serial.println("-------------------------");
}


// display_image - captures and displays image from flowSensor flowSensor
void display_image_continuously()
{ 
  int i;
  Serial.println("press any key to return to menu");

  Serial.flush();
  
  while( !Serial.available() ) {
  display_image();
  i=0;
    while( i<20 && !Serial.available() ) {
      delay(100);  // give the viewer a bit of time to catchup
      i++;
    }
  }
    
  Serial.flush();
}


// show x,y and squal values constantly until user presses a key
//
void display_motion()
{
  boolean first_time = true;
  Serial.flush();
  
  // display instructions on how to exit
  Serial.println("press x to return to menu..");
  delay(1000);
  
  while( !Serial.available() ) {
    updateOF();

    // check for errors
    if( _overflow )
      Serial.println("overflow!!");

    // x,y,squal
    Serial.print("dx: ");
    Serial.print(raw_dx,DEC);
    Serial.print("\tdy: ");
    Serial.print(raw_dy,DEC);
    Serial.print("\tsqual:");
    Serial.print(surface_quality,DEC);
    Serial.println();
    first_time = false;
    
    // short delay
    delay(100);
  }
  
  // flush the serial
  Serial.flush();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADNS3080 SPECIFIC FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// reset sensor by holding a pin high (or is it low?) for 10us.
void reset()
{
  // return immediately if the reset pin is not defined
  if( _reset_pin == 0)
    return;

  digitalWrite(_reset_pin,HIGH);                 // reset sensor
  delayMicroseconds(10);
  digitalWrite(_reset_pin,LOW);                  // return sensor to normal
}


// Read a register from the sensor
byte read_register(byte address)
{
  byte result = 0, junk = 0;

  //backup_spi_settings();

  // take the chip select low to select the device
  digitalWrite(_cs_pin, LOW);

  // send the device the register you want to read:
  junk = SPI.transfer(address);

  // small delay
  delayMicroseconds(50);

  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);

  // take the chip select high to de-select:
  digitalWrite(_cs_pin, HIGH);

  //restore_spi_settings();

  return result;
}


// init - initialise sensor
// initCommAPI parameter controls whether SPI interface is initialised (set to false if other devices are on the SPI bus and have already initialised the interface)
boolean initOF()
{
  int retry = 0;

  pinMode(AP_SPI_DATAOUT,OUTPUT);
  pinMode(AP_SPI_DATAIN,INPUT);
  pinMode(AP_SPI_CLOCK,OUTPUT);
  pinMode(_cs_pin,OUTPUT);
  if( _reset_pin != 0)
    pinMode(ADNS3080_RESET,OUTPUT);

  digitalWrite(_cs_pin,HIGH);                 // disable device (Chip select is active low)

  // reset the device
  reset();

  // start the SPI library:
  SPI.begin();

  // check the sensor is functioning
  if( retry < 3 ) {
    if( read_register(ADNS3080_PRODUCT_ID) == 0x17 )
      return true;
    retry++;
  }

  return false;
}


//
// backup_spi_settings - checks current SPI settings (clock speed, etc), sets values to what we need
//
//byte backup_spi_settings()
//{
//  // store current spi values
//  orig_spi_settings_spcr = SPCR & (DORD | CPOL | CPHA);
//  orig_spi_settings_spsr = SPSR & SPI2X;
//  
//  // set the values that we need
//  SPI.setBitOrder(MSBFIRST);
//  SPI.setDataMode(SPI_MODE3);
//  SPI.setClockDivider(SPI_CLOCK_DIV8);  // sensor running at 2Mhz.  this is it's maximum speed
//  
//  return orig_spi_settings_spcr;
//}


// restore_spi_settings - restores SPI settings (clock speed, etc) to what their values were before the sensor used the bus
//byte restore_spi_settings()
//{
//  byte temp;
//
//  // restore SPSR
//  temp = SPSR;
//  temp &= ~SPI2X;
//  temp |= orig_spi_settings_spsr;
//  SPSR = temp;
//
//  // restore SPCR
//  temp = SPCR;
//  temp &= ~(DORD | CPOL | CPHA);   // zero out the important bits
//  temp |= orig_spi_settings_spcr;  // restore important bits
//  SPCR = temp;
//
//  return temp;
//}


// write a value to one of the sensor's registers
void write_register(byte address, byte value)
{
  byte junk = 0;

 // backup_spi_settings();

  // take the chip select low to select the device
  digitalWrite(_cs_pin, LOW);

  // send register address
  junk = SPI.transfer(address | 0x80 );

  // small delay
  delayMicroseconds(50);

  // send data
  junk = SPI.transfer(value);

  // take the chip select high to de-select:
  digitalWrite(_cs_pin, HIGH);

 // restore_spi_settings();
}


// get_pixel_data - captures an image from the sensor and stores it to the pixe_data array
void print_pixel_data(Stream *serPort)
{
  int i,j;
  boolean isFirstPixel = true;
  byte regValue;
  byte pixelValue;

  // write to frame capture register to force capture of frame
  write_register(ADNS3080_FRAME_CAPTURE,0x83);

  // wait 3 frame periods + 10 nanoseconds for frame to be captured
  delayMicroseconds(1510);  // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

  // display the pixel data
  for( i=0; i<ADNS3080_PIXELS_Y; i++ ) {
    for( j=0; j<ADNS3080_PIXELS_X; j++ ) {
      regValue = read_register(ADNS3080_FRAME_CAPTURE);
      if( isFirstPixel && (regValue & 0x40) == 0 ) {
        serPort->println("failed to find first pixel");
      }
      isFirstPixel = false;
      pixelValue = ( regValue << 2);
      serPort->print(pixelValue,DEC);
      if( j!= ADNS3080_PIXELS_X-1 )
        serPort->print(",");
      delayMicroseconds(50);
    }
    serPort->println();
  }

  // hardware reset to restore sensor to normal operation
  reset();
}

bool updateOF()
{
  byte motion_reg;
  surface_quality = (unsigned int)read_register(ADNS3080_SQUAL);
  delayMicroseconds(50);  // small delay

  // check for movement, update x,y values
  motion_reg = read_register(ADNS3080_MOTION);
  _overflow = ((motion_reg & 0x10) != 0);  // check if we've had an overflow
  if( (motion_reg & 0x80) != 0 ) {
    raw_dx = ((char)read_register(ADNS3080_DELTA_X));
    delayMicroseconds(50);  // small delay
    raw_dy = ((char)read_register(ADNS3080_DELTA_Y));
    _motion = true;
  }else{
    raw_dx = 0;
    raw_dy = 0;
  }
  last_update = millis();

  return true;
}
