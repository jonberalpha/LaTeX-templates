#define SENS_DS18X20_MOD

/*_______________________________ INCLUDES ___________________________________*/
#include "SENS_DS18X20.h"

/*________________________________ EXTERN ____________________________________*/
extern uint8_t datareg[9];

/*___________________________ GLOBAL VARIABLES _______________________________*/
uint8_t datareg[9];

/*__________________________ PRIVATE PROTOTYPES ______________________________*/
static void init_ports(void);
static uint8_t send_mri(void);
static uint8_t read_bit(void);
static uint8_t read_byte(void);
static void write_bit(uint8_t val);
static void write_byte(uint8_t val);
static void skip_ROM(void);
static float conv_temp(void);

void DS18X20_init(void)
{
	init_ports();
	delay_init();
}

static void init_ports(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //enable CLK for GPIOB
	GPIOB->CRL &= ~0x0F;                //Konfig-Bits von PB0 loeschen
	GPIOB->CRL |= 0x07;                 //GPOOD...IR=7
}

static uint8_t send_mri(void)
{
	uint8_t rec;
	
  DQ_o = 0;							 // DQ to 0 for MRI
  delay_us(490);         // wait for >480 us
  DQ_o = 1;              // config the  DQ-IO as input (high-z -> pull up)
  delay_us(40);
  rec = DQ_i;            // read the level (if low, slave available)
  delay_us(450);         // wait for end of slot
  
	return rec;
}

static void write_bit(uint8_t val)
{
  DQ_o = 0;              // config the DQ-IO as output (-> low)
  if(val)                // if the bit to transfer is a "1"
  {
    delay_us(1);         // wait 1 us and..
    DQ_o = 1;            // ..config the DQ-IO as input (high-z -> pull up)
  }
  delay_us(100);         // wait for end of slot
  DQ_o = 1;              // config the DQ-IO as input (high-z -> pull up)
}

static void write_byte(uint8_t val)
{
  uint8_t i, mask = 1;
  
  // write the byte by sending eight bits (LSB first)
  for (i=0;i<8;i++)
  {
		write_bit(val & mask);
    mask = (mask << 1);
  }
}

static uint8_t read_bit (void)
{
  uint8_t rec;           
  
  DQ_o = 0;           // perform a very short low impuls
  DQ_o = 1;           // config the DQ-IO as input (high-z -> pull up)
  delay_us(15);
  rec = DQ_i;         // read the level on DQ (this is the read bit)
  delay_us(105);      // wait for end of slot
  
  return rec;
}

static uint8_t read_byte(void)
{
  uint8_t value = 0 , i;
	
  // read the byte by reading eight bits (LSB first)
  for(i=0;i<8;i++)
  {
		if(read_bit())
		{
			value |= 0x01 << i;
		}
  }
  return value;
}

static void skip_ROM(void)
{
  write_byte(SKIP_ROM);
}

static float conv_temp()
{
  float ftemp; // Temperatur als float-Var.
  uint8_t temp = 0;
	
	temp = (datareg[1]<<8)|datareg[0];
	ftemp = (float)temp/16 + 14; // +14 Korrekturfaktor
	
	return ftemp;
}

float get_temperature()
{
	int i=0;
	float temp=0;
	
	send_mri();       //send Master Reset Impulse; must return 0x00
	skip_ROM();       //simply calls all slaves instead of an specific address call
		
	write_byte(CONVERT_TEMP);	//Temperaturmessung ausfuehren
	delay_ms(750);            //Messdauer abwarten
	
	send_mri();
	skip_ROM();
	
	write_byte(READ_SCRATCHPAD); //Temperaturwert auslesen
	
	for(i=0;i<9;i++)    //Antwort einlesen: 9 Byte grosßse Scratch Pad-Inhalt einlesen
	{
		datareg[i] = read_byte();
	}
	temp = conv_temp();
	
	return temp;
}
