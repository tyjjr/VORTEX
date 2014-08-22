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

Metro Cmd=Metro(20);//接收线程
Metro Speed=Metro(200);//加速度线程
Metro Led=Metro(20);//加速度线程
//电机引脚
#define ENL 6     //左侧电机使能引脚pwm量
#define DIRL 7     //左侧电机方向引脚
#define ENR 5     //右侧电机使能引脚
#define DIRR 4     //右侧电机方向引脚
//传感器位置宏定义  5个红外传感器
#define MID A2
#define LEFT1 A1
#define LEFT2 A0
#define RIGHT1 A3
#define RIGHT2 A7
#define LIR 9
#define RIR 10
//串口位置
#define tx 11
#define rx 2
SoftwareSerial mySerial(rx, tx);	// RX, TX
//变量
int irCount=0;//接近传感器
int blackVal=300;//黑色阈值


//引脚初始化

void update() //PB0引脚变化中断
{
	irCount++;
}
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
	//PCICR=0x01;//开中断
	//PCMSK0=0x01;   	
        delay(100);
	attachInterrupt(0,update,HIGH);//开中断	
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

//速度计算输出
void Move(int lspeed,int rspeed,int acc)
{
	static int reallspeed=0,realrspeed=0;
	if(acc==0)
	{
		reallspeed=lspeed;
		realrspeed=rspeed;
	}
	else
	{
		if(lspeed>reallspeed)
		{
			reallspeed=reallspeed+acc;
			if(lspeed<reallspeed)
			reallspeed=lspeed;
		}
		if(lspeed<reallspeed)
		{
			reallspeed=reallspeed-acc;
			if(lspeed>reallspeed)
			reallspeed=lspeed;
		}
		if(rspeed>realrspeed)
		{
			realrspeed=realrspeed+acc;
			if(rspeed<realrspeed)
			realrspeed=rspeed;
		}
		if(rspeed<realrspeed)
		{
			realrspeed=realrspeed-acc;
			if(rspeed>realrspeed)
			realrspeed=rspeed;
		}
	}
	if(reallspeed>=0)
	{
		digitalWrite(DIRL,HIGH);
		analogWrite(ENL,abs(reallspeed));

	}
	else
	{
		digitalWrite(DIRL,LOW);
		analogWrite(ENL,abs(reallspeed));
	}
	if(realrspeed>=0)
	{
		digitalWrite(DIRR,HIGH);
		analogWrite(ENR,abs(realrspeed));

	}
	else
	{
		digitalWrite(DIRR,LOW);
		analogWrite(ENR,abs(realrspeed));
	}	
}
//LED控制
void LED(uint8_t form,uint8_t color,uint8_t ledacc)
{
	static int ledlight[8];
	static int ledcolor=0;
	
	if((ledacc==0)||(ledcolor==color))
	{
		ledcolor=color;
	}
	else//颜色平滑
	{
		if(color>ledcolor)
		{
			if(color-ledcolor<128)
			{
				ledcolor+=ledacc;
				if(ledcolor>color)
					ledcolor=color;
			}
			else
			{
				ledcolor-=ledacc;
				if(ledcolor<=0)
				{
					ledcolor=255+ledcolor;
					if(ledcolor<color)
					{
						ledcolor=color;
					}
				}
			}
		}
		else
		{
			if(ledcolor-color<128)
			{
				ledcolor-=ledacc;
				if(ledcolor<color)
					ledcolor=color;
			}
			else
			{
				ledcolor+=ledacc;
				if(ledcolor>=255)
				{
					ledcolor=ledcolor-255;
					if(ledcolor>color)
					{
						ledcolor=color;
					}
				}
			}
		}

		
	}
	for(int i=0;i<8;i++)//亮度平滑
	{
		if(form&(1<<i))
		{	
			if(ledlight[i]<255)
			{	
				ledlight[i]+=ledacc;				
				if((ledlight[i]>255)||(ledacc==0))
				ledlight[i]=255;
			}
		}
		else
		{	
			if(ledlight[i]>0)
			{
				ledlight[i]-=ledacc;
				if((ledlight[i]<0)||(ledacc==0))
				ledlight[i]=0;	
			}			
		}
		leds[i]=CHSV(ledcolor,255,ledlight[i]);  
	}
    FastLED.show(); 
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
uint8_t getIR(int black)
{
	uint8_t form=0;
	if(analogRead(RIGHT2)>=blackVal)
		form|=0x01;
	else
		form&=~0x01;
	if(analogRead(RIGHT1)>=blackVal)
		form|=0x02;
	else
		form&=~0x02;
	if(analogRead(MID)>=blackVal)
		form|=0x04;
	else
		form&=~0x04;
	if(analogRead(LEFT1)>=blackVal)
		form|=0x08;
	else
		form&=~0x08;
	if(analogRead(LEFT2)>=blackVal)
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
  /*
   if(Cmd.check()==1)
    Command();
	mp3(1,15);
	delay(3000);
	mp3(0,15);
	delay(3000);	*/
        mp3(4,15);
        delay(3000);
        mp3(0,15);
	delay (3000);
}