uint8_t burst;
void mousecam_reset()
{
	digitalWrite(PIN_MOUSECAM_RESET,HIGH);
	delayMicroseconds(10); // reset pulse >10us
	digitalWrite(PIN_MOUSECAM_RESET,LOW);
	delayMicroseconds(500); // 35ms from reset to functional
}


int mousecam_init()
{
	pinMode(PIN_MOUSECAM_RESET,OUTPUT);
	pinMode(PIN_MOUSECAM_CS,OUTPUT);
	
	digitalWrite(PIN_MOUSECAM_CS,HIGH);
	
	mousecam_reset();
	
	int pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
	if(pid != ADNS3080_PRODUCT_ID_VAL)
	return -1;
	
	// turn on sensitive mode
	mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
	
	return 0;
}

void mousecam_write_reg(int reg, int val)
{
	digitalWrite(PIN_MOUSECAM_CS, LOW);
	SPI.transfer(reg | 0x80);
	SPI.transfer(val);
	digitalWrite(PIN_MOUSECAM_CS,HIGH);
	delayMicroseconds(50);
}

int mousecam_read_reg(int reg)
{
	digitalWrite(PIN_MOUSECAM_CS, LOW);
	SPI.transfer(reg);
	delayMicroseconds(75);
	int ret = SPI.transfer(0xff);
	digitalWrite(PIN_MOUSECAM_CS,HIGH); 
	delayMicroseconds(1);
	return ret;
}


void mousecam_read_motion(struct MD *p)
{
	digitalWrite(PIN_MOUSECAM_CS, LOW);
	burst = SPI.transfer(ADNS3080_MOTION_BURST);
	delayMicroseconds(75);
	if (burst & 0x10)
		Serial.println("Overflow");
	p->motion =  SPI.transfer(0x00);
	p->dx =  SPI.transfer(0x00);
	p->dy =  SPI.transfer(0x00);
	p->squal =  SPI.transfer(0x00);
	p->shutter =  SPI.transfer(0x00)<<8;
	p->shutter |=  SPI.transfer(0x00);
	p->max_pix =  SPI.transfer(0x00);
	digitalWrite(PIN_MOUSECAM_CS,HIGH); 
	delayMicroseconds(5);
}

// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(uint8_t *pdata)
{
	mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
	
	digitalWrite(PIN_MOUSECAM_CS, LOW);
	
	SPI.transfer(ADNS3080_PIXEL_BURST);
	delayMicroseconds(50);
	
	uint8_t pix;
	byte started = 0;
	int count;
	int timeout = 0;
	int ret = 0;
	for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
	{
	pix = SPI.transfer(0xff);
	delayMicroseconds(10);
	if(started==0)
	{
	  if(pix&0x40)
	    started = 1;
	  else
	  {
	    timeout++;
	    if(timeout==100)
	    {
	      ret = -1;
	      break;
	    }
	  }
	}
	if(started==1)
	{
	  pdata[count++] = pix <<2; // scale to normal grayscale byte range
	}
	}
	
	digitalWrite(PIN_MOUSECAM_CS,HIGH); 
	delayMicroseconds(14);
	
	return ret;
}
