#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
#include <Metro.h> 	
#include "FastLED.h"//ws2812
#define NUM_LEDS 8
#define DATA_PIN 13
CRGB leds[NUM_LEDS];

#define END 255
#define STOP 0							//停止
#define FORWARD 1						//前进+200	速度200
#define BACKWARD 2						//后退+200	速度200
#define TURN_LEFT 3						//左转+200	速度200
#define TURN_RIGHT 4					//右转+200	速度200
#define ACC_FORWARD 5					//加速前进+200	速度200
#define DEC_FORWARD 6					//减速前进+200	速度200
#define ACC_BACKWARD 7					//加速后退+200	速度200
#define DEC_BACKWARD 8					//减速后退+200	速度200
#define ACC_LEFT 9						//加速左传+200	速度200
#define DEC_LEFT 10						//减速左转+200	速度200
#define ACC_RIGHT 11					//加速右转+200	速度200
#define DEC_RIGHT 12					//减速右转+200	速度200
#define LINE 13 						//巡线
#define SET_IR_THRESHOLD 14				//100

//MP3播放： 061 + 1  播放第一首歌
#define MP3_MUSIC 61
#define MP3_VALUE 62
//LED灯：

#define LEDCONTROL 71 
//(0xff代表控制所有8个灯)
#define LEDCOLOR 72
#define LEDTIME 73
//(00关,01开)

//小车定时：
#define TIMER_SET 90
//定时器报警音效：例子+01代表第一首歌
#define TIMER_SOUND 91
//取消定时
#define TIMER_CANCEL 92
//获取小车运动状态：
#define GET_STATE 100
//获取电池电量：
#define GET_ENERGY 101

//小车返回宏定义
#define CRASH_HIGH 1 			//碰撞传感器高
#define CRASH_LOW 2				//碰撞传感器低
#define IR1_HIGH 11				//巡线传感器1高
#define IR1_LOW 12				//巡线传感器1低
#define IR2_HIGH 13				//巡线传感器2高
#define IR2_LOW 14				//巡线传感器2低
#define IR3_HIGH 15				//巡线传感器3高
#define IR3_LOW 16				//巡线传感器3低
#define IR4_HIGH 17				//巡线传感器4高
#define IR4_LOW 18				//巡线传感器4低
#define IR5_HIGH 19				//巡线传感器5高
#define IR5_LOW 20				//巡线传感器5低
#define STATE_FORWARD  31		//运动状态 前进
#define STATE_BACKWARD 32		//运动状态 后退
#define STATE_LEFT 33			//运动状态 左转
#define STATE_RIGHT  34			//运动状态 右转
#define STATE_STOP 35			//运动状态 停止






Metro Cmd=Metro(20);			//接收线程
Metro Speed=Metro(5000);		//加速度线程
Metro Led=Metro(20);			//加速度线程

//电机引脚
#define ENL 5     				//左侧电机使能引脚pwm量
#define DIRL 12     			//左侧电机方向引脚
#define ENR 6    				//右侧电机使能引脚
#define DIRR 7//4   			//右侧电机方向引脚
//传感器位置宏定义  5个红外传感器
#define MID A2
#define LEFT1 A1
#define LEFT2 A0
#define RIGHT1 A3
#define RIGHT2 A4				//7
#define LIR 13					//9
#define RIR 8					//10
#define IR_IN  17				//IR receiver pin
//串口位置
#define tx 11
#define rx 2
SoftwareSerial mySerial(rx, tx);// RX, TX
//变量
int irCount=0;					//接近传感器


//闭环电机控制
//获取脉冲间隔
static unsigned long leftTime=micros(),leftTime_1;
void lSpeed()
{	
        static int numl=0;
        numl++;
        if(numl==2)
        {
      	  leftTime_1=micros()-leftTime;
          leftTime=micros();
          numl=0;
        }
	
}
double readLeftTime()
{
	static double readL;
	readL=micros()-leftTime;
	if(readL<leftTime_1)
	{
		return leftTime_1;
	}
	else{
		return readL;
	}
}
//PID控制
void moveLeft(double hopeTime)//左轮PID控制，参数为40000-20000us,脉冲间隔
{
	static double movel_i_scope=10000,
				 movel_i_scope_1=5,
				 movel_i=3000,
				 movel_p=1000,
				 movel_p_scope=40;
	static double enL;
	long pow;
		
	if(abs(readLeftTime()-hopeTime)>movel_i_scope)
		enL=enL+movel_i_scope_1*(readLeftTime()-hopeTime)/abs(readLeftTime()-hopeTime);
	else
		enL=enL+(readLeftTime()-hopeTime)/movel_i;
		
	digitalWrite(DIRL,LOW);
	
	if(enL>=255)
		enL=255;
	else if(enL<=0)
		enL=0;
		
	if(abs((readLeftTime()-hopeTime)/movel_p)>=movel_p_scope)
		pow=enL+movel_p_scope*((readLeftTime()-hopeTime)/abs(readLeftTime()-hopeTime));
	else
		pow=enL+(readLeftTime()-hopeTime)/movel_p;
	if(pow>=255)
		pow=255;
	else if(pow<=0)
		pow=0;  
	else 
		analogWrite(ENL,pow);
}
//闭环电机控制
//获取脉冲间隔
static unsigned long rightTime=micros(),rightTime_1;
void rSpeed()
{	
        static int numr=0;
        numr++;
        if(numr==2)
        {
      	  rightTime_1=micros()-rightTime;
          rightTime=micros();
          numr=0;
        }
}
double readRightTime()
{
	static double readR;
	readR=micros()-rightTime;
	if(readR<rightTime_1)
	{
		return rightTime_1;
	}
	else{
		return readR;
	}
}
//PID控制
void moveRight(double hopeTime)//右轮PID控制，参数为40000-20000us,脉冲间隔
{
	static double mover_i_scope=10000,
				 mover_i_scope_1=5,
				 mover_i=3000,
				 mover_p=1000,
				 mover_p_scope=40;
	static double enR;
	long pow;
		
	if(abs(readRightTime()-hopeTime)>mover_i_scope)
		enR=enR+mover_i_scope_1*(readRightTime()-hopeTime)/abs(readRightTime()-hopeTime);
	else
		enR=enR+(readRightTime()-hopeTime)/mover_i;
		
	digitalWrite(DIRR,LOW);
	
	if(enR>=255)
		enR=255;
	else if(enR<=0)
		enR=0;
		
	if(abs((readRightTime()-hopeTime)/mover_p)>=mover_p_scope)
		pow=enR+mover_p_scope*((readRightTime()-hopeTime)/abs(readRightTime()-hopeTime));
	else
		pow=enR+(readRightTime()-hopeTime)/mover_p;
	if(pow>=255)
		pow=255;
	else if(pow<=0)
		pow=0;  
	else 
		analogWrite(ENR,pow);
}









//PB0引脚变化中断
/*
void update() 
{
	irCount++;
	Serial.println("1");
}
*/

//引脚初始化
void pinInit()
{	
	mySerial.begin (9600);
	mp3_set_serial (mySerial);	                               //set softwareSerial for DFPlayer-mini mp3 module
	Serial.begin(9600);
	FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
	pinMode(DIRR,OUTPUT);//电机方向
	pinMode(DIRL,OUTPUT);
	pinMode(LIR,OUTPUT);//接近传感器灯
	pinMode(RIR,OUTPUT);
	pinMode(IR_IN,INPUT);//init the ir receiver pin
	digitalWrite(LIR,HIGH);
	digitalWrite(RIR,HIGH);
	PCICR=0x01;//开中断
	PCMSK0=0x01;   	
	delay(100);
	//attachInterrupt(0,update,CHANGE);//开中断	
	attachInterrupt(2,rSpeed,HIGH);//开编码器中断	
	attachInterrupt(3,lSpeed,HIGH);//开编码器中断	
}

//串口接收
void Command()
{
    if(Serial.available()==0) return;
    uint8_t cmd=Serial.read();
    switch(cmd)
    {
      case END:

      break;
      case STOP:
          
      break;
	  case FORWARD:
	  
      break;
      case BACKWARD:
          
      break;
	  case TURN_LEFT:
          
      break;
      case TURN_RIGHT:
          
      break;
	  case ACC_FORWARD:
          
      break;
      case DEC_FORWARD:
          
      break;
	  case ACC_BACKWARD:
          
      break;
      case DEC_BACKWARD:
          
      break;
	  case ACC_LEFT:
          
      break;
      case DEC_LEFT:
          
      break;
	  case ACC_RIGHT:
          
      break;
      case DEC_RIGHT:
          
      break;
	  case SET_IR_THRESHOLD:
          
      break;
      case MP3_MUSIC:
          
      break;
	  case MP3_VALUE:
          
      break;
      case LEDCONTROL:
          
      break;
	  case LEDCOLOR:
          
      break;
      case LEDTIME:
          
      break;
	  case TIMER_SET:
          
      break;
      case TIMER_SOUND:
          
      break;
	  case TIMER_CANCEL:
          
      break;
	  case GET_STATE:
          
      break;
      case GET_ENERGY:
          
      break;	  
	  default:break;
    }
}


//加速度计算，输入时间0-255*10，speed为八档左右轮速度，第八位第四位方向
float reallspeed=0,realrspeed=0;//实际速度
float g_lacc,g_racc;//加速度传递变量
void accCount(uint8_t speed,int acctime)
{

	int lspeed,rspeed;
	lspeed=((speed&0x70)>>4)*36;
	if(!(speed&0x80))
		lspeed*=-1;
	rspeed=(speed&0x07)*36;
	if(!(speed&0x08))
		rspeed*=-1;	
	if(acctime<=0)
	{
		g_lacc=0;
		g_racc=0;
	}
	else
	{
		g_lacc=abs(float((lspeed-reallspeed)*2)/float(acctime));
		g_racc=abs(float((rspeed-realrspeed)*2)/float(acctime));
	}
}
//速度计算输出
void Move(uint8_t speed,float lacc,float racc)
{	
	int lspeed,rspeed;
	lspeed=((speed&0x70)>>4)*36;
	if(!(speed&0x80))
		lspeed*=-1;
	rspeed=(speed&0x07)*36;
	if(!(speed&0x08))
		rspeed*=-1;
	if(lacc==0)
	{
		reallspeed=lspeed;
	}
	else
	{                       
		if(lspeed>reallspeed)
		{
			reallspeed=reallspeed+lacc;
			if(lspeed<reallspeed)
			reallspeed=lspeed;
		}
		if(lspeed<reallspeed)
		{
			reallspeed=reallspeed-lacc;
			if(lspeed>reallspeed)
			reallspeed=lspeed;
		}

	}
        if(racc==0)
        {
            realrspeed=rspeed;
        }
        else
        {
          	if(rspeed>realrspeed)
		{
			realrspeed=realrspeed+racc;
			if(rspeed<realrspeed)
			realrspeed=rspeed;
		}
		if(rspeed<realrspeed)
		{
			realrspeed=realrspeed-racc;
			if(rspeed>realrspeed)
			realrspeed=rspeed;
		}
        }
	if(reallspeed>=0)
	{
		digitalWrite(DIRL,LOW);
		analogWrite(ENL,abs(reallspeed));

	}
	else
	{
		digitalWrite(DIRL,HIGH);
		analogWrite(ENL,abs(reallspeed));
	}
	if(realrspeed>=0)
	{
		digitalWrite(DIRR,LOW);
		analogWrite(ENR,abs(realrspeed));

	}
	else
	{
		digitalWrite(DIRR,HIGH);

		analogWrite(ENR,abs(realrspeed));
	}	
}
//灯带加速度变换
float realled[8][2],ledacc[8][2];
void ledAccCount(uint8_t date,uint8_t lignt_color,uint8_t time)
{
	uint8_t light,color;
	light=(lignt_color>>4)*17;
	color=(lignt_color&0x0f)*17;
	for(int j=0;j<8;j++)
	{
		if(date&(1<<j))
		{
			if(time==0)
				ledacc[j][0]=0;
			else
				ledacc[j][0]=abs(float(light-realled[j][0]))*2/float(time);
			
		}
	}
	for(int j=0;j<8;j++)
	{
		if(date&(1<<j))
			if(time==0)
				ledacc[j][1]=0;
			else
			{
				if (abs(color-realled[j][1])<127)
					ledacc[j][1]=abs(float(color-realled[j][1]))*2/float(time);
				else 
					ledacc[j][1]=(255-abs(float(color-realled[j][1])))*2/float(time);
			}	
	}
}
void ledAction(uint8_t date,uint8_t lignt_color)
{
	uint8_t light,color;
	light=(lignt_color>>4)*17;
	color=(lignt_color&0x0f)*17;
	for(int i=0;i<8;i++)
	{
		if(date&(1<<i))
		{
			if(ledacc[i][0]==0)
				realled[i][0]=light;
			else//亮度平滑
			{
				if(realled[i][0]<light)
				{
					realled[i][0]+=ledacc[i][0];
					if(realled[i][0]>light)
						realled[i][0]=light;
				}
				if(realled[i][0]>light)
				{
					realled[i][0]-=ledacc[i][0];
					if(realled[i][0]<light)
						realled[i][0]=light;
				}
			}
			if(ledacc[i][1]==0)
				realled[i][1]=color;
			else//颜色平滑
			{
				if(realled[i][1]<color)
					if(color-realled[i][1]<127)
					{
						realled[i][1]+=ledacc[i][1];
						if(realled[i][1]>color)
							realled[i][1]=color;
					}
					else
					{
						realled[i][1]-=ledacc[i][1];
						if(realled[i][1]<=0)
						{
							realled[i][1]=255+realled[i][1];
							if(realled[i][1]<color)
								realled[i][1]=color;							
						}
					}
				else if(realled[i][1]>color)
					if(realled[i][1]-color<127)
					{
						realled[i][1]-=ledacc[i][1];
						if(realled[i][1]<color)
							realled[i][1]=color;
					}
					else
					{
						realled[i][1]+=ledacc[i][1];
						if(realled[i][1]>=255)
						{
							realled[i][1]=realled[i][1]-255;
							if(realled[i][1]>color)
								realled[i][1]=color;
						}
						
					}
			}
			
		}
		leds[i]=CHSV(int(realled[i][1]),255,int(realled[i][0]));
	}	
	FastLED.show(); 
}
ISR(PCINT0_vect)//
{
        irCount++;
}
//接近传感器
int getSwitch()
{
	irCount=0;
	for (unsigned char i=0;i<20;i++)
	{
		for (int i=0;i<24;i++)
		{
			digitalWrite(LIR,LOW) ;      //L_IR ON
			digitalWrite(RIR,LOW) ;    //R_IR ON
			delayMicroseconds(8);
			digitalWrite(LIR,HIGH);
			digitalWrite(RIR,HIGH);
			delayMicroseconds(8);
		}
		delayMicroseconds(200);
	}
	if (irCount>20)
	{
		return 1;
	}
	else return 0;
}
//巡线传感器
uint8_t readIR(uint8_t seek=0)
{	
	int differvalue=500;//差值阈值，可用作排除干扰
	static int white=0;
	static uint8_t form=0;
	if(seek==1)//重载空白值
		white=analogRead(MID);
	if(abs(white-analogRead(RIGHT2))>=differvalue)
		form|=0x01;
	else
		form&=~0x01;
	if(abs(white-analogRead(RIGHT1))>=differvalue)
		form|=0x02;
	else
		form&=~0x02;
	if(abs(white-analogRead(MID))>=differvalue)
		form|=0x04;
	else
		form&=~0x04;
	if(abs(white-analogRead(LEFT1))>=differvalue)
		form|=0x08;
	else
		form&=~0x08;
	if(abs(white-analogRead(LEFT2))>=differvalue)
		form|=0x10;
	else
		form&=~0x10;
	return form;
}

//mp3播放or停止
void mp3(int ranking,int val)
{
        mp3_set_volume (val);
        delay(100);
        if(ranking==0)
          mp3_stop();
        else
	  mp3_play(ranking);		
}
void setup() {
  // put your setup code here, to run once:
  
        pinInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!getSwitch())
  Move(255,0,0);
  else
  Move(0,0,0);
  delay(20);
}