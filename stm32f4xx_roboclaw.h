
/*WARNING:1. USART2 AND TIMER 3 IS RESERVED*/

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>

#define rc_baud 115200
#define rc_baudrate (((F_CPU / (rc_baud * 16UL))) - 1)

#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg

#define MAXRETRY 2

int16_t data = 0;
uint16_t crc = 0, ccrc = 0;
int rcv_case = 0;

uint16_t data_array[10]={};
int timerFlag = 0,timer_count=0,timercount_2=0;

uint16_t buff_val = 0,motorStatus=0;

uint8_t *status = 0,bufferM1 = 0,bufferM2 = 0;
bool *valid = NULL;

void driveM1(uint8_t address, int sp_vect);
void driveM2(uint8_t address, int sp_vect);

void SpeedM1(uint8_t address, uint32_t speed);
void SpeedM2(uint8_t address, uint32_t speed);
void SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
void SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
void SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag);
void SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag);
void SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag);
void SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag);
void SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag);

void setAllM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
void setAllM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
void setAll(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);

int32_t readEnc1(uint8_t address);
int32_t readEnc2(uint8_t address);
void resetEnc(uint8_t address);

uint8_t readBufferM1(uint8_t address);
uint8_t readBufferM2(uint8_t address);
uint16_t ReadError(uint8_t address);

void setM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
void setM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);

void write_n(uint8_t cnt, ... );
void Read2(uint8_t address,uint8_t cmd, uint16_t *val);
void Read4_1(uint8_t address, uint8_t cmd, int32_t *val);

void clearCRC();
void updateCRC (uint8_t data);
uint16_t getCRC();

void rc_serialstart_3();
void transmit(uint8_t data);
void timer3_init();
void timer3_start();
void timer3_stop();
//void usart2_tx_init();
//void usart2_rx_init();
void _delay_ms_roboclaw(int time);
int16_t rcvTimeout();
void usart_flush();
long limit_var(long in_var, long l_limit, long h_limit);
void usart2_init();

/*void usart2_tx_init() {
  RCC->APB2ENR |= (1 << 14); //ENABLE CONFIGURATION
  RCC->AHB1ENR |= (1 << 3);  //ENABLE GPIOD

  GPIOD->MODER |= (1 << 13) | (1 << 11);                                                  //PD6 AND PD5 set as AF
  GPIOD->OSPEEDR|=(1<< 11) | (1<<10);
  GPIOD->PUPDR |= (1 << 10);                                                  
  GPIOD->AFR[0] |= (1 << 20) | (1 << 21) | (1 << 22) | (1 << 24) | (1 << 25) | (1 << 26); //SET AS AF7


  RCC->APB1ENR |= (1 << 17); //USART2 clock enable

  USART2->BRR |= 0x0045;    //FOR BAUDRATE 115200
  USART2->CR1 |= (1  << 3);//Transmitter is enabled
  USART2->CR1 |= (1 << 6);  //TC interrupt enable
  USART2->CR1 |= (1 << 13); //USART ENABLE
}

void usart2_rx_init()
{
  RCC->APB2ENR |= (1 << 14); //ENABLE CONFIGURATION
  RCC->AHB1ENR |= (1 << 3);  //ENABLE GPIOD

  GPIOD->MODER |= (1 << 13) | (1 << 11);                                                  //PD6 AND PD5 set as AF
  GPIOD->OSPEEDR|=(1<< 12) | (1<<13);
  GPIOD->PUPDR |= (1 << 12);                                                  
  GPIOD->AFR[0] |= (1 << 20) | (1 << 21) | (1 << 22) | (1 << 24) | (1 << 25) | (1 << 26); //SET AS AF7

  RCC->APB1ENR |= (1 << 17);         //USART2 clock enabled

  USART2->BRR |= 0x0045;             //BAUDRATE = 115200
  USART2->CR1 |= (1 << 2);          //RX ENABLE
  USART2->CR1 |= (1 << 5);          //RXNE ENABLE
  USART2->CR1 |= (1 << 13);         //USART enabled
}*/
void usart2_init()
{
  RCC->APB1ENR |= (1<<17);
  RCC->AHB1ENR |= (1<<0);
  GPIOA->MODER |= (1<<7) | (1<<5);//A2 & A3
  GPIOA->OSPEEDR |= (1<<7) | (1<<6) | (1<<5) | (1<<4);
  GPIOA->AFR[0] |= (1<<14) | (1<<13) | (1<<12) | (1<<10) | (1<<9) | (1<<8);
  USART2->CR1 |=(1<<13);//enable usart 
  USART2->CR2 &= ~(1<<13) & ~(1<<12);//stop 1
  USART2->GTPR |=(1<<0);//prescalar
  //USART2->BRR = 0x683;//9600
  USART2->BRR =0x008b ;//115200 WITHOUT PLL
  //USART2->BRR =0x16D ;//115200 WITH PLL
  //USART2->BRR = 0x045;//230400
  USART2->CR1 |= (1<<2) | (1<<3);//enable receive transmit
  USART2->CR1 |= (1 << 5);          //RXNE ENABLE
  USART2->CR1 |= (1 << 6);  //TC interrupt enable
}


void transmit(unsigned char data)
{
  usart2_init();
  USART2->CR1 |= (1 << 6);  //TC interrupt enable  
  USART2->DR = (data & (uint8_t)0xFFFF);
  while (!(USART2->SR & (1 << 6))); //Wait till transmission complete
  USART2->SR&=~(1<<6);
}

void transmit_1(unsigned char data)
{
  usart2_init();
  USART2->CR1 |= (1 << 6);  //TC interrupt enable  
  USART2->DR = (data & (uint8_t)0xFFFF);
  while (!(USART2->SR & (1 << 6))); //Wait till transmission complete
  USART2->SR&=~(1<<6);
}

int16_t rcvTimeout()
{
	timer3_start();
    usart2_init();
	while(!(USART2->SR & (1 << 5)))
	{
		if(timerFlag)
                {
                timer3_stop();
		return (-1);
                }
	}
	data = USART2->DR;
	return data;
}

void usart_flush()
{
        unsigned char dummy;
        usart2_init();
	while (USART2->SR & (1<<5))
	dummy = USART2->DR;
}

void driveM1(uint8_t address, int sp_vect)
{
	sp_vect = limit_var(sp_vect,-127,127);
	if(sp_vect == 0 || sp_vect > 0)
	{
		write_n(3,address,0,sp_vect);
	}
	else if(sp_vect < 0)
	{
		write_n(3,address,1,(-sp_vect));
	}
}

void driveM2(uint8_t address, int sp_vect)
{
	sp_vect = limit_var(sp_vect,-127,127);
	if(sp_vect == 0 || sp_vect > 0)
	{
		write_n(3,address,4,sp_vect);
	}
	else if(sp_vect < 0)
	{
		write_n(3,address,5,(-sp_vect));
	}
}

void SpeedM1(uint8_t address, uint32_t speed)
{
	write_n(6,address,35,SetDWORDval(speed));
}

void SpeedM2(uint8_t address, uint32_t speed)
{
	write_n(6,address,36,SetDWORDval(speed));
}

void SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed)
{
	write_n(10,address,38,SetDWORDval(accel),SetDWORDval(speed));
}

void SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed)
{
	write_n(10,address,39,SetDWORDval(accel),SetDWORDval(speed));
}

void SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
	write_n(11,address,41,SetDWORDval(speed),SetDWORDval(distance),flag);
}

void SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag)
{
	write_n(11,address,42,SetDWORDval(speed),SetDWORDval(distance),flag);
}

void SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	write_n(15,address,44,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

void SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	write_n(15,address,45,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag);
}

void SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag)
{
	write_n(23,address,46,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag);
}

void setAllM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag)
{
	write_n(19,address,65,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

void setAllM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag)
{
	write_n(19,address,66,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

void setAll(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag)
{
	write_n(35,address,67,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag);
}

int32_t readEnc1(uint8_t address)
{	
	int32_t	enc_val	=0;
	Read4_1(address,16,&enc_val);
	return enc_val;
}

int32_t readEnc2(uint8_t address)
{
	int32_t	enc_val	=0;
 	Read4_1(address,17,&enc_val);		
	return enc_val;
}

void resetEnc(uint8_t address)
{
	write_n(2,address,20);
}

uint8_t readBufferM1(uint8_t address)
{
	Read2(address,47,&buff_val);
	return buff_val>>8;
}

uint8_t readBufferM2(uint8_t address)
{
	Read2(address,47,&buff_val);
	return buff_val;
}
							
void setM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)
{
	uint32_t kp = kp_fp*1024;
	uint32_t ki = ki_fp*1024;
	uint32_t kd = kd_fp*1024;
	write_n(30,address,61,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

void setM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)
{
	uint32_t kp = kp_fp*1024;
	uint32_t ki = ki_fp*1024;
	uint32_t kd = kd_fp*1024;
	write_n(30,address,62,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max));
}

uint16_t ReadError(uint8_t address)
{
	Read2(address,90,&motorStatus);
	return motorStatus;
}

void write_n(uint8_t cnt, ... )
{
	uint8_t trys=MAXRETRY;
	do{
		clearCRC();
		va_list marker;
		va_start(marker,cnt);     
		for(uint8_t index=0;index<cnt;index++)
		{
			uint8_t data = va_arg(marker, int);
			updateCRC(data);
			transmit(data);
		}
		va_end(marker);              
		uint16_t crc = getCRC();
		transmit(crc>>8);
		transmit(crc);
		
	}while(trys--);
}

void Read2(uint8_t address, uint8_t cmd, uint16_t *val)
{
	uint16_t value = 0;

	if(rcv_case == 0)
	{
		ccrc = 0;

		usart_flush();
		clearCRC();
		transmit_1(address);
		updateCRC(address);
		transmit_1(cmd);
		updateCRC(cmd);
	}
	
	_delay_ms_roboclaw(1);

	if(rcv_case == 4)
	{
		updateCRC(data_array[0]);
		value = (uint16_t)data_array[0]<<8;

		updateCRC(data_array[1]);
		value |= (uint16_t)data_array[1];
	
		ccrc = data_array[2] << 8;			
		ccrc |= data_array[3];
	
		if(getCRC() == ccrc)
		{
			*val = value;
		}
		rcv_case = 0;
	}	
}

void Read4_1(uint8_t address, uint8_t cmd, int32_t *val)
{
	uint32_t value = 0;
	int16_t data = 0, statData = 0;

	usart_flush();

	clearCRC();
	transmit_1(address);
	updateCRC(address);
	transmit_1(cmd);
	updateCRC(cmd);

	data = rcvTimeout();		
	updateCRC(data);
	value = (uint32_t)data<<24;

	if(data != (-1))
	{
		data = rcvTimeout();
		updateCRC(data);
		value |= (uint32_t)data<<16;
	}

	if(data != (-1))
	{
		data = rcvTimeout();
		updateCRC(data);
	 	value |= (uint32_t)data<<8;
	}

	if(data != (-1))
	{
		data = rcvTimeout();
		updateCRC(data);
		value |= (uint32_t)data;
	}

	if(data != (-1))
	{
		data = rcvTimeout();
		updateCRC(data);
		statData = data;			
	}

	if(data != (-1))
	{
		uint16_t ccrc;
		data = rcvTimeout();
		
		if(data != (-1))
		{
			ccrc = data << 8;
			data = rcvTimeout();
			if(data != (-1))
			{
				ccrc |= data;
				if(getCRC() == ccrc)
				{
					if(!(statData && 0x02))
						*val = (-value);
					else
						*val = value;
				}
			}
		}
	}
}

void clearCRC()
{
	crc = 0;
}

void updateCRC (uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
		crc = (crc << 1) ^ 0x1021;
		else
		crc <<= 1;
	}
}

uint16_t getCRC()
{
	return crc;
}


long limit_var(long in_var, long l_limit, long h_limit)
{
	if (in_var>h_limit)
	{
		in_var=h_limit;
	}
	else if (in_var<l_limit)
	{
		in_var=l_limit;
	}
	return in_var;
}

void timer3_init() {
  RCC->APB1ENR |= (1 << 1);     //TIM3 clock enabled
    
  TIM3->PSC=83;   //PRESCALAR = 83
  TIM3->ARR=99;   //PERIOD = 9 
  TIM3->DIER|=(1<<0);   //Update interrupt enabled
  NVIC_EnableIRQ(TIM3_IRQn);
}

void timer3_start() {
  timer3_init();
  timerFlag=0;
  timercount_2=0;
  TIM3->CR1|=(1<<0);    //Counter enabled
}

void timer3_stop() {
  timercount_2=0;
  TIM3->CR1&=~(1<<0);    //Counter enabled
}

void TIM3_IRQHandler() {
  if(TIM3->SR & (1<<0))
  { 
    TIM3->SR &=~(1<<0);
    if(timer_count>=10)
    {
      timerFlag=1;
    }
    timer_count++;
  }
}

void _delay_ms_roboclaw(int time)
{
  timer3_start();
  while(!(timer_count==time));
  timer_count=0;
  timer3_stop();
}
