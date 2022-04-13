#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "SysTick.h"
#include "interrupt.h"
#include "uart.h"
#include "tm4c1294ncpdt.h"
#include "pwm.h"
#include "eeprom.h"

//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR           0x22
#define PCA9557_I2CADDR           0x18

#define PCA9557_INPUT             0x00
#define PCA9557_OUTPUT            0x01
#define PCA9557_POLINVERT         0x02
#define PCA9557_CONFIG            0x03

#define TCA6424_CONFIG_PORT0      0x0c
#define TCA6424_CONFIG_PORT1      0x0d
#define TCA6424_CONFIG_PORT2      0x0e

#define TCA6424_INPUT_PORT0       0x00
#define TCA6424_INPUT_PORT1       0x01
#define TCA6424_INPUT_PORT2       0x02

#define TCA6424_OUTPUT_PORT0      0x04
#define TCA6424_OUTPUT_PORT1      0x05
#define TCA6424_OUTPUT_PORT2      0x06

#define SYSTICK_FREQUENCY  1000
#define BEEPPeriod   16000000

//system function definition
void    Delay(uint32_t value);
void    S800_GPIO_Init(void);
void    S800_I2C0_Init(void);
void    S800_CLOCK_Init(void);
void    S800_SysTick_Init(void);
void    S800_UART_Init(void);
void    S800_PWM_Init(uint16_t);
uint8_t   I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t   I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void UARTStringPut(const char *);

//clock function definition
void time_module(uint32_t *,uint8_t);
void date_module(uint32_t *,uint8_t);
void recorrect(uint32_t*);
void settime(uint32_t *);
void setdate(uint32_t *);
void setalarm(uint8_t);
void alarmclock(uint8_t);
void showchazhi(uint32_t *);

void daojishi();
void jieqi();
void reset();
//system parameter
uint32_t ui32SysClock;

volatile uint8_t result,gpio_status;
//clock parameter
uint32_t Time[]={0,12,00,00,0,6,16,2021};      //blank,hour,min,sec,sec01,month,date;
uint32_t alarm_hour[]={0,0,0},alarm_min[]={0,0,1};
volatile uint64_t now,datenow;
uint32_t i_t,i_d,i=0;
uint32_t xindex,yindex=0;
uint32_t xindex_step,yindex_step=0;

uint32_t pui32Data[8];
//function flags
bool flag_timehour = 0,flag_pm = 0;                           //0?24???,1?12???
bool timeup_index1=0,timeup_index2=0;                                     //????
bool alarm_flag[] = {0,0};                                    //????????
bool sleep_mode=0;

bool press_sw[]={0,0,0,0,0,0,0,0,0},press_pj1=0,press_pj0=0,press_flag=0;      //????
uint16_t index_module = 0;                                        //??    
uint8_t studentflag=0;
uint8_t remember_flag=0;

uint8_t backflag=0;
uint8_t timecompare=0;
uint8_t timeseted_flag=0,dateseted_flag=0,alarmseted_flag=0;  //??????
uint16_t set_flagtime=1,set_flagdate=5,set_flagalarm=1;
bool Flash_f=0;
uint32_t Temp[8];
uint8_t temp_k=0;
bool presshistory_sw=1;
int RxEndFlag = 0;
uint8_t flash_counter=0;
uint8_t num_t,num_d,num_a=0;
uint32_t k;
uint32_t temp_k_dj=0;
uint32_t dj_10=0;
uint32_t set_flagdj=2;
uint32_t ld_max=0;
uint32_t ld_index=0;
uint32_t djseted_flag=0;
uint8_t index_flash=0;//????
//useful constant
uint16_t clock[] = {24,12},Datetable[]={0,31,29,31,30,31,30,31,31,30,31,30,31};//2020
uint32_t flash[] = {0,1,2,4,8,16,32,64,128};
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71,0x5c};
uint16_t stepctl[] = {0x01,0x02,0x04,0x08};
uint32_t studentnum=20910924; //519020910924
uint16_t FLASH=0x00;
int Time_dj[]={0,0,1,0,0,0,0};
uint8_t backcounter,counter=0;

unsigned char RxBuf[256];
unsigned char SerialOutput[256];
unsigned char FINISH[]="Finished!";
unsigned char INVALID[]="Invalid Input!";
uint8_t PF,PF0,PF1,PF2,PF3;
uint16_t mm,dd,hh,min;
uint16_t motorangle;

uint8_t index_jieqi=0;
uint8_t dj_index=0;
bool pj0index=1;
//uint8_t seg_jieqi[] = {0x3e,0x3f,0x06};
uint8_t seg_jieqi[] = {0x6d,0x3e,0x06};
volatile uint16_t systick_5ms_counter,systick_10ms_counter,systick_150ms_counter,systick_1s_counter,systick_25ms_counter,systick_40ms_counter,systick_250ms_counter,systick_1ms_counter,systick_80ms_counter;
volatile uint8_t systick_5ms_status,systick_10ms_status,systick_150ms_status,systick_1s_status,systick_25ms_status,systick_40ms_status,systick_250ms_status,systick_1ms_status,systick_80ms_status;
  
int main(void)
{
//??? 
  S800_CLOCK_Init();
  S800_SysTick_Init();
  S800_I2C0_Init();
  S800_UART_Init();
  S800_GPIO_Init();
  UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
  IntEnable(INT_UART0); 
  IntMasterEnable();
  i_t=0;i_d=0;k=0;

  while (1)
  {
    while(studentflag == 0)                                                                //????
    {
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
      if(i == 0)//???
      {
        now=studentnum;
        i=10000000;
        k=1;
      }
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[now/i]);
      if(!index_flash){
      result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x00); //?
      GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x0F);
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,flash[k]);}//????
      else {
        I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);//????
        result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff); //?,??????
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x00);
        index_flash=0;}
      now=now%i;i/=10;k++;
      Delay(8000);
      if(systick_250ms_status)
      {
      systick_250ms_status = 0;
      counter++;
      }
      if(counter%4==0) index_flash=1;//?1s?1s
      if(counter==8) {studentflag=1;result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff); }//??,??
    }
  
     

    
    switch (index_module)
    {
      case 0:                                                                    //????
          {
              if(sleep_mode==1) I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
                else time_module(Time,0);
              break;
          }
      case 1:                                                                    //????
          {if(sleep_mode==1) I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
          else{
              date_module(Time,0);
              break;}
          }
      case 2:                                                                    //??1??
          {
              if(timeup_index1==1) alarmclock(1);
                else setalarm(1);
              break;
          }
      case 3:                                                                    //??2??
          {
              if(timeup_index2==1) alarmclock(2);
                else setalarm(2);
              break;
          }
      case 4:                                                                    //????
          {
              settime(Time);
              break;
          }
      case 5:                                                                    //????
          {
              setdate(Time);
              break;
          }
          case 6:                                                                    //?????
          {
              daojishi();
              break;
          }
          case 7:                                                                    //????
          {
              jieqi();
              break;
          }
          
    }
    
    if(systick_10ms_status)                                                                     //????
    {
      systick_10ms_status = 0;
       Time[4]++;
      xindex++;
      yindex++;
     
      if(xindex==3){//1?????,60s,6000?Time[4],6000/512=11.7?12,?12?????,???
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,1<<(3-(yindex/xindex-1)));
        xindex=0;//3,2,1,0??
      } 
      if(yindex==12)
      yindex=0;
      recorrect(Time);
    }
    
    if(presshistory_sw==true) {presshistory_sw=false;backcounter=0;}                            //???????????
    if(systick_1s_status)
    {
      systick_1s_status = 0;
     
      if(timeup_index1==0 && timeup_index2==0 ) backcounter++;//1s??
     
      if(Time[1]==alarm_hour[1] && Time[2]==alarm_min[1] && alarm_flag[0]==1) {index_module=2;timeup_index1=1;}
      if(Time[1]==alarm_hour[2] && Time[2]==alarm_min[2] && alarm_flag[1]==1) {index_module=3;timeup_index2=1;}
    }
    
    
    if(backcounter==15 && sleep_mode==0 ) {index_module=0;backcounter=0;}//?15s?????????
     if(systick_1ms_status)
      {
        systick_1ms_status = 0;
        ld_index++;
        if((ld_index-1)==ld_max)
          ld_index=0;}
    if(systick_80ms_status)                                                                     //????80ms
    {
        systick_80ms_status = 0;
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfc)reset();//???sw1,sw2???,??
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xf3){
          ld_max++;
          ld_index=0;
          if(ld_max==3)ld_max=0;
        };
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfe){press_sw[1]=true;presshistory_sw=true;}
          else{
                if(press_sw[1]==true) {temp_k_dj=0;index_module++;temp_k=0;remember_flag=0;alarmseted_flag = 0;backflag=0;timeseted_flag=0;dj_index=0;pj0index=1;dateseted_flag=0;press_sw[1]=false;}
                
            }
        if(index_module==8) index_module=0;
            
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xef){press_sw[5]=true;presshistory_sw=true;}    //sw5????1??
          else{
              if(press_sw[5]) alarm_flag[0]=!alarm_flag[0];
              press_sw[5]=false;
          }
        
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xdf){press_sw[6]=true;presshistory_sw=true;}    //sw6????2??
          else{
              if(press_sw[6]) alarm_flag[1]=!alarm_flag[1];
              press_sw[6]=false;
          }
          
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xbf){press_sw[7]=true;presshistory_sw=true;}    //sw7??????
          else{
              if(press_sw[7]) sleep_mode=!sleep_mode;
              press_sw[7]=false;
          }
          
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0x7f){press_sw[8]=true;presshistory_sw=true;}    //sw8?????
          else{
              if(press_sw[8]) flag_timehour=!flag_timehour;
              press_sw[8]=false;
          }
        
    }
  
    
    if(sleep_mode == 0)                                                                         //led??
    {
        
        I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff-flash[index_module+1]);              //??????
        if(alarm_flag[0]==1) PF0=2; else PF0=0;                                                 //??1??
        if(alarm_flag[1]==1) PF1=1; else PF1=0;                                                 //??2??
        PF=PF0+PF1;
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, PF);

        if(flag_timehour==0) GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,0x10);                     //?????M2(??24h) 
            else GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,0x00);
        if(flag_timehour==1 && Time[1]>12) I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x7f); //pm??
    }
    else
    {
        
        I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xff);
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, 0x00);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);
    }
    
    if(RxEndFlag==1)                                                                       //????,?uart??????1,???????????,???????
    {
      
      if(RxBuf[0]=='G' && RxBuf[1]=='E' && RxBuf[2]=='T'){
        if(RxBuf[3]=='D' && RxBuf[4]=='A' && RxBuf[5]=='T' && RxBuf[6]=='E') {SerialOutput[0]=(Time[5]/10)+'0';SerialOutput[1]=(Time[5]%10)+'0';SerialOutput[2]='.';SerialOutput[3]=(Time[6]/10)+'0';SerialOutput[4]=(Time[6]%10)+'0';UARTStringPut(SerialOutput);}
        else if(RxBuf[3]=='T' && RxBuf[4]=='I' && RxBuf[5]=='M' && RxBuf[6]=='E') {SerialOutput[0]=(Time[1]/10)+'0';SerialOutput[1]=(Time[1]%10)+'0';SerialOutput[2]=':';SerialOutput[3]=(Time[2]/10)+'0';SerialOutput[4]=(Time[2]%10)+'0';UARTStringPut(SerialOutput);}
        else UARTStringPut((uint8_t *)"Invalid Input!\r\n");
      }
      else if(RxBuf[0]=='S' && RxBuf[1]=='E' && RxBuf[2]=='T'){
              if(RxBuf[3]=='D' && RxBuf[4]=='A' && RxBuf[5]=='T' && RxBuf[6]=='E') {Time[5]=(RxBuf[7]-'0')*10+(RxBuf[8]-'0');Time[6]=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');UARTStringPut((uint8_t *)"Successed!\r\n");}
              else if(RxBuf[3]=='T' && RxBuf[4]=='I' && RxBuf[5]=='M' && RxBuf[6]=='E') {Time[1]=(RxBuf[7]-'0')*10+(RxBuf[8]-'0');Time[2]=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');UARTStringPut((uint8_t *)"Successed!!\r\n");}
              else if(RxBuf[3]=='A' && RxBuf[4]=='L' && RxBuf[5]=='M'){
                if(RxBuf[6]=='1') {alarm_hour[1]=(RxBuf[7]-'0')*10+(RxBuf[8]-'0');alarm_min[1]=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');alarm_flag[0]=1;UARTStringPut((uint8_t *)"Successed!\r\n");}
                else if(RxBuf[6]=='2') {alarm_hour[2]=(RxBuf[7]-'0')*10+(RxBuf[8]-'0');alarm_min[2]=(RxBuf[9]-'0')*10+(RxBuf[10]-'0');alarm_flag[1]=1;UARTStringPut((uint8_t *)"Successed!\r\n");}
                        else UARTStringPut((uint8_t *)"Invalid Input!\r\n");
              } 
              else UARTStringPut((uint8_t *)"Invalid Input!\r\n");
          }
      else if(RxBuf[0]=='R' && RxBuf[1]=='E' && RxBuf[2]=='S' && RxBuf[3]=='E' && RxBuf[4]=='T'){reset();UARTStringPut((uint8_t *)"Reseted!\r\n");}
      else UARTStringPut((uint8_t *)"Invalid Input!\r\n");
      RxEndFlag=0;//??
    }   
  }
}


void S800_CLOCK_Init(void)
{
  ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 16000000);
}

void Delay(uint32_t value)
{
  uint32_t ui32Loop;
  for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}
void S800_SysTick_Init(void)
{
  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);
  SysTickEnable();
  SysTickIntEnable();
}

void S800_GPIO_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);            //Enable PortF
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));     //Wait for the GPIO moduleF ready
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);            //Enable PortJ  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));     //Wait for the GPIO moduleJ ready 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);            //Enable PortN
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));     //Wait for the GPIO moduleN ready
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);            //Enable PortK
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));     //Wait for the GPIO moduleK ready
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);     //Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1);      //Set PN0 as Output pin
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
  GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5); 
  GPIOPinConfigure(GPIO_PK5_M0PWM7);
  GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5);
  GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);  
  GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);
}

void S800_I2C0_Init(void)
{
  uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);//???i2c??
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//??I2C??0,?????I2C0SCL--PB2?I2C0SDA--PB3
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);//??PB2?I2C0SCL
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);//??PB3?I2C0SDA
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);//I2C?GPIO_PIN_2??SCL
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);//I2C?GPIO_PIN_3??SDA

  I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);                    //config I2C0 400k
  I2CMasterEnable(I2C0_BASE); 

  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);    //config port 0 as input
  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);      //config port 1 as output
  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);      //config port 2 as output 

  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);         //config port as output
  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);        //turn off the LED1-8
}
void S800_PWM_Init(uint16_t BeepPeriod) 
{ 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
  PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);     
  PWMGenConfigure(PWM0_BASE,PWM_GEN_3, PWM_GEN_MODE_DOWN |PWM_GEN_MODE_NO_SYNC); 
  PWMGenPeriodSet(PWM0_BASE,PWM_GEN_3,BeepPeriod);
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,BeepPeriod/4); 
  PWMOutputState(PWM0_BASE,PWM_OUT_7_BIT,true); 
  PWMGenEnable(PWM0_BASE,PWM_GEN_3); 
}
void S800_UART_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);            //Enable PortA
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));     //Wait for the GPIO moduleA ready
  GPIOPinConfigure(GPIO_PA0_U0RX);                        // Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);          
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  // Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
  UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX4_8,UART_FIFO_RX6_8);
  UARTStringPut((uint8_t *)"\r\nHello!\r\n");
}


void UARTStringPut(const char *cMessage)
{
  while(*cMessage!='\0')
    UARTCharPut(UART0_BASE,*(cMessage++));
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
  uint8_t rop;
  while(I2CMasterBusy(I2C0_BASE)){};//??I2C0???,??
    //
  I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
    //????????????????false???????,true???????
    
  I2CMasterDataPut(I2C0_BASE, RegAddr);//??????????
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//????????
  while(I2CMasterBusy(I2C0_BASE)){};
    
  rop = (uint8_t)I2CMasterErr(I2C0_BASE);//???

  I2CMasterDataPut(I2C0_BASE, WriteData);
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//???????????
  while(I2CMasterBusy(I2C0_BASE)){};

  rop = (uint8_t)I2CMasterErr(I2C0_BASE);//???

  return rop;//??????,????0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
  uint8_t value,rop;
  while(I2CMasterBusy(I2C0_BASE)){};  
  I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
  I2CMasterDataPut(I2C0_BASE, RegAddr);
//  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);   
  I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//????????
  while(I2CMasterBusBusy(I2C0_BASE));
  rop = (uint8_t)I2CMasterErr(I2C0_BASE);
  Delay(1);
  //receive data
  I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//??????
  I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//???????
  while(I2CMasterBusBusy(I2C0_BASE));
  value=I2CMasterDataGet(I2C0_BASE);//???????
    Delay(1);
  return value;
}

void UART0_Handler(void)
{
    uint8_t cnt = 0;
    uint32_t ulStatus;
    ulStatus = UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE,ulStatus);
  
    while(UARTCharsAvail(UART0_BASE)){
        RxBuf[cnt++]=UARTCharGetNonBlocking(UART0_BASE);
    }
    RxBuf[cnt]= '\0';
    RxEndFlag=1;
}

void SysTick_Handler(void)
{
  if (systick_10ms_counter != 0) systick_10ms_counter--;
  else
  {
      systick_10ms_counter = 10;
      systick_10ms_status = 1;
  }
   if (systick_5ms_counter != 0) systick_5ms_counter--;
  else
  {
      systick_5ms_counter = 5;
      systick_5ms_status = 1;
  }
  if (systick_1s_counter != 0) systick_1s_counter--;
  else
  {
      systick_1s_counter = 1000;
      systick_1s_status = 1;
  }
  if (systick_150ms_counter != 0) systick_150ms_counter--;
  else
  {
      systick_150ms_counter = 150;
      systick_150ms_status = 1;
  }
  if (systick_25ms_counter != 0) systick_25ms_counter--;
  else
  {
      systick_25ms_counter = 25;
      systick_25ms_status = 1;
  }
  if (systick_80ms_counter != 0) systick_80ms_counter--;
  else
  {
      systick_80ms_counter = 40;
      systick_80ms_status = 1;
  }
  if (systick_40ms_counter != 0) systick_40ms_counter--;
  else
  {
      systick_40ms_counter = 40;
      systick_40ms_status = 1;
  }
  if (systick_250ms_counter != 0) systick_250ms_counter--;
  else
  {
      systick_250ms_counter = 250;
      systick_250ms_status = 1;
  }
  if (systick_1ms_counter != 0) systick_1ms_counter--;
  else
  {
      systick_1ms_counter = 1;
      systick_1ms_status = 1;
  }
}

//clock functions
void time_module (uint32_t *temp,uint8_t flash_flag)
{
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
      if(i_t == 0)//???
      {
        if(flag_timehour==1 && temp[1]>12) now= (temp[1]-12)*1000000+temp[2]*10000+temp[3]*100+temp[4];
          else now= temp[1]*1000000+temp[2]*10000+temp[3]*100+temp[4];
        i_t=10000000;
        num_t=1;
      }
      if(systick_150ms_status)
      {
        systick_150ms_status = 0;
        if((flash_flag==9 || flash_flag==10)&&flash_counter>=(11-flash_flag)) {Flash_f=!Flash_f;flash_counter=0;}//???????????????????
        else Flash_f=!Flash_f;
      }
      if(flash_flag==9 || flash_flag==10) FLASH=(Flash_f==0)?(flash[num_t]):(0x00);//??????????,??????
      else//????????,???????????????
      if((num_t+1)/2 == flash_flag)//??????,?????2
      {
        FLASH=(Flash_f==0)?(flash[num_t]):(0x00);
      }
      else FLASH=flash[num_t];//?????
      
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[now/i_t]);
      if(ld_index==ld_max)
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,FLASH);
      else I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
      now=now%i_t;i_t/=10;num_t++;    
}
void date_module(uint32_t *temp,uint8_t flash_flag)
{
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
      if(i_d == 0)
      {
        datenow= temp[7]*10000+temp[5]*100+temp[6];
        i_d=10000000;
        num_d=1;
      }
      if(systick_150ms_status)
      {
        systick_150ms_status = 0;
        Flash_f=!Flash_f;
      }
      if((num_d+5)/2 == flash_flag)//????Time?????????,????5
      {
        FLASH=(Flash_f==0)?(flash[num_d]):(0x00);
      }
      else FLASH=flash[num_d];
      
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[datenow/i_d]);
      if(ld_index==ld_max)
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,FLASH);
      else  I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
      datenow=datenow%i_d;i_d/=10;num_d++;
}
void settime(uint32_t *temp)
{
  
    uint8_t flash_f;
    while(temp_k==0)
    {
      Temp[1]=temp[1];Temp[2]=temp[2];Temp[3]=temp[3];Temp[4]=temp[4];
      Temp[5]=temp[5];Temp[6]=temp[6];Temp[7]=temp[7];    //????????,????????
      temp_k=1;
    }
  
    if(timeseted_flag == 0)
    {
      if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfd){press_sw[2]=true;presshistory_sw=true;}  //?????
        else{
            if(press_sw[2]) set_flagtime++;//????????
            press_sw[2]=false;
        }
        if(set_flagtime>=5) set_flagtime=1;
        
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfb){press_sw[3]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[3]) {Temp[set_flagtime]++;recorrect(Temp);}//??????
            press_sw[3]=false;
        }
         if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xf7){press_sw[4]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[4]) Temp[set_flagtime]--;//??????
            press_sw[4]=false;
        }
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){press_pj1=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj1) {Temp[1]=temp[1];Temp[2]=temp[2];Temp[3]=temp[3];Temp[4]=temp[4];}
            press_pj1=false;
        }
        
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj0) 
            { 
              
              timeseted_flag = 1;
            }
            press_pj0=false;
        }
      }
      if(systick_1s_status)
      {
          systick_1s_status = 0;
          if(press_sw[3]) Temp[set_flagtime]++;
      }
      recorrect(Temp);
      flash_f = set_flagtime;
      time_module(Temp,flash_f);
    }
    else
    {
      time_module(Temp,0);
      if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj0) 
            {showchazhi(Temp);
                Time[1]=Temp[1];Time[2]=Temp[2];Time[3]=Temp[3];Time[4]=Temp[4];
                index_module++;
                press_pj0=false;
            }
            
            }
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){press_pj1=true;presshistory_sw=true;}
        else{
            if(press_pj1) {Temp[1]=temp[1];Temp[2]=temp[2];Temp[3]=temp[3];Temp[4]=temp[4];timeseted_flag = 0;}
            press_pj1=false;
        }
      }
    
  }
}

void setdate(uint32_t *temp)
{
    uint8_t flash_f;
    while(temp_k==0)
    {
      Temp[1]=temp[1];Temp[2]=temp[2];Temp[3]=temp[3];Temp[4]=temp[4];
      Temp[5]=temp[5];Temp[6]=temp[6];Temp[7]=temp[7];    //????????
      temp_k=1;
    }
    
    if(dateseted_flag == 0)
    {
      if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfd){press_sw[2]=true;presshistory_sw=true;}  //?????
        else{
            if(press_sw[2]) set_flagdate++;//????????
            press_sw[2]=false;
        }
        if(set_flagdate>7) set_flagdate=5;
        
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfb){press_sw[3]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[3]) Temp[set_flagdate]++;//??????
            press_sw[3]=false;
        }
         if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xf7){press_sw[4]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[4]) Temp[set_flagdate]--;//??????
            press_sw[4]=false;
        }
        
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){press_pj1=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj1) {Temp[5]=temp[5];Temp[6]=temp[6];Temp[7]=temp[7];}
            press_pj1=false;
        }
        
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj0) dateseted_flag = 1;
            press_pj0=false;
        }
      }
      if(systick_1s_status)
      {
          systick_1s_status = 0;
          if(press_sw[3]) Temp[set_flagdate]++;
      }
      recorrect(Temp);
      flash_f = set_flagdate;
      date_module(Temp,flash_f);
    }
    else
    {
      date_module(Temp,0);
      if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj0) {Time[5]=Temp[5];Time[6]=Temp[6];Time[7]=Temp[7];
            press_pj0=false;
            dateseted_flag = 0;
            index_module++;}
        }
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){press_pj1=true;presshistory_sw=true;}
        else{
            if(press_pj1) {Temp[5]=temp[5];Temp[6]=temp[6];Temp[7]=temp[7];dateseted_flag = 0;}
            press_pj1=false;
        }
      }
    }
}

void recorrect(uint32_t *time)
{//????
    if(time[4]>=100) {time[4]=0;time[3]++;}
    if(time[3]>=60) {time[3]=0;time[2]++;}
    if(time[2]>=60) {time[2]=0;time[1]++;}
    if(time[1]>=24) {time[1]=0;time[6]++;}
    if(time[6]>Datetable[time[5]]) {time[6]=1;time[5]++;}
    if(time[5]>12) {time[5]=1;Time[7]++;}
}

void setalarm(uint8_t alarmnum)
{
    
    uint32_t temp[8]={0,0,0,0,0,0,0,0};
    uint8_t flash_f;

    temp[1]=alarm_hour[alarmnum];temp[2]=alarm_min[alarmnum];    //???????? 
    while(temp_k==0)
    {
      Temp[1]=temp[1];Temp[2]=temp[2];Temp[3]=temp[3];Temp[4]=temp[4];
      Temp[5]=temp[5];Temp[6]=temp[6];Temp[7]=temp[7];    //????????
      temp_k=1;
    }   
    
   
    if(alarmseted_flag == 0)
    {
      if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfd){press_sw[2]=true;presshistory_sw=true;}  //?????
        else{
            if(press_sw[2]) set_flagalarm++;
            press_sw[2]=false;
        }
        if(set_flagalarm>=3) set_flagalarm=1;
        
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfb){press_sw[3]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[3]) {Temp[set_flagalarm]++;recorrect(Temp);}
            press_sw[3]=false;
        }
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xf7){press_sw[4]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[4]) Temp[set_flagalarm]--;//??????
            press_sw[4]=false;
        }
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){press_pj1=true;}                  //??
        else{
            if(press_pj1) {Temp[1]=alarm_hour[alarmnum];Temp[2]=alarm_min[alarmnum];presshistory_sw=true;}
            press_pj1=false;
        }
        
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj0) 
            {
               
                alarmseted_flag = 1;
               
            }
            press_pj0=false;
        }
      }
      if(systick_1s_status)
      {
          systick_1s_status = 0;
          if(press_sw[3]) Temp[set_flagalarm]++;
      }
      recorrect(Temp);
      flash_f = set_flagalarm;
      time_module(Temp,flash_f);
    }
     else
    {
      time_module(Temp,0);
      if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
        else{
            if(press_pj0) 
            {
                alarm_hour[alarmnum]=Temp[1];alarm_min[alarmnum]=Temp[2];
                index_module++;
                alarmseted_flag = 0;
                press_pj0=false;
                temp_k=0;
            }
            
        }
        if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0){press_pj1=true;presshistory_sw=true;}
        else{
            if(press_pj1) {Temp[1]=alarm_hour[alarmnum];Temp[2]=alarm_min[alarmnum];alarmseted_flag = 0;}
            press_pj1=false;
        }
      }
    }
}
 

void alarmclock(uint8_t alarmnum)
{
    uint32_t temp[8]={0,0,0,0,0,0,0,0};
    temp[1]=alarm_hour[alarmnum];temp[2]=alarm_min[alarmnum];
    
      if(alarm_flag[alarmnum-1]==1)
      {
        if(systick_250ms_status)
        {
            systick_250ms_status = 0;
            flash_counter++;
            if(flash_counter>=(3-alarmnum)) {Flash_f=!Flash_f;flash_counter=0;}//??1??????1Hz??,???1Hz;??2?? ????2Hz??,???2Hz?
        }
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0|GPIO_PIN_1, Flash_f*(alarmnum==1?2:1));
        sleep_mode=0;//????
        S800_PWM_Init(BEEPPeriod/alarmnum);
        time_module(temp,8+alarmnum);

        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) ==0xf7) {press_flag=true;presshistory_sw=true;}
        else
        {
            if(press_flag) {alarm_flag[alarmnum-1]=0;PWMGenDisable(PWM0_BASE,PWM_GEN_3);index_module=0;
            press_flag=false;//??sw4????
            timeup_index1=0;
            timeup_index2=0;}

        } 
      }
      else PWMGenDisable(PWM0_BASE,PWM_GEN_3);
      
}
    

void jieqi(){//??su1?summer 1??
  if(systick_1ms_status)
      {
        systick_1ms_status = 0;
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);
        FLASH=flash[index_jieqi+1];
        I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg_jieqi[index_jieqi]);
  if(ld_index==ld_max)
      I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,FLASH); 
  else I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)0);   
      index_jieqi++;
      if(index_jieqi==3)index_jieqi=0;  
      }
}
void daojishi(){//???8?led????????,??????????????????
    uint32_t temp[8]={0,0,0,0,0,0,0,0};
    

    temp[1]=Time_dj[1];temp[2]=Time_dj[2];  temp[3]=Time_dj[4];temp[3]=Time_dj[4]; temp[4]=Time_dj[4];   //???????? 
    while(temp_k_dj==0)
    {
      Temp[1]=temp[1];Temp[2]=temp[2];Temp[3]=temp[3];Temp[4]=temp[4];
      Temp[5]=temp[5];Temp[6]=temp[6];Temp[7]=temp[7];    //????????
      temp_k_dj=1;
    }
  
        if(dj_index==0){//??????????
        
         time_module(Temp,set_flagdj);
       if(systick_25ms_status)
      {
        systick_25ms_status = 0;
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfd){press_sw[2]=true;presshistory_sw=true;}  //?????
        else{
            if(press_sw[2]) set_flagdj++;
            press_sw[2]=false;
        }
        if(set_flagdj>=5) set_flagdj=2;
        
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xfb){press_sw[3]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[3]) {Temp[set_flagdj]++;recorrect(Temp);}
            press_sw[3]=false;
        }
        if(I2C0_ReadByte(TCA6424_I2CADDR,TCA6424_INPUT_PORT0) == 0xf7){press_sw[4]=true;presshistory_sw=true;}  //?1
        else{
            if(press_sw[4]) {Temp[set_flagdj]--;recorrect(Temp);}//??????
            press_sw[4]=false;
        }                //??
          
              if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                  //??
          else{
              if(press_pj0) 
              {//??
                  recorrect(Temp);
                  Time_dj[1]=Temp[1];temp[2]=Time_dj[2]=Temp[2];  Time_dj[3]=Temp[3];Time_dj[4]=Temp[4]; 
                  dj_index=1;
                  temp_k_dj=0;
                  press_pj0=false;

              }
          
          }
          
          }
           if(systick_1s_status)
      {
          systick_1s_status = 0;
          if(press_sw[3]) Temp[set_flagdj]++;
      }
      recorrect(Temp);
          }
          

          else{//?????
          time_module(Time_dj,0);
                if(systick_40ms_status)
      {
        systick_40ms_status = 0;
        backcounter=0;//?????????
         if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0){press_pj0=true;presshistory_sw=true;}                //????????? 
          else{
              if(press_pj0) 
              {
                  press_pj0=false;
                  pj0index=!pj0index;
              }
          
          }
      }
              if(systick_5ms_status)                                                                     //????
                  {
                    systick_5ms_status = 0;
                    dj_10++;
                    if(dj_10==2){
                      dj_10=0;
                     Time_dj[4]=Time_dj[4]-pj0index;//??
                     if(Time_dj[4]<=-1)
                       {
                          if((Time_dj[3]!=0)||(Time_dj[2]!=0)||(Time_dj[1]!=0))
                            {Time_dj[4]=59;Time_dj[3]--;}
                          else Time_dj[4]=0;//?????
                            }
                     if(Time_dj[3]<=-1){Time_dj[3]=59;Time_dj[2]--;}
                     if(Time_dj[2]<=-1){Time_dj[2]=59;Time_dj[1]--;}//????
                  }
                  }
               } 
}

void showchazhi(uint32_t *TEMP)
{bool index;
    if((TEMP[3]*100+TEMP[4])>(Time[3]*100+Time[4]))
      {index=0;
      motorangle=(TEMP[3]*100+TEMP[4])-(Time[3]*100+Time[4]);
      motorangle=motorangle/12;}
    else{
      index=1;
      motorangle=(Time[3]*100+Time[4])-(TEMP[3]*100+TEMP[4]);
      motorangle=motorangle/12;
    }
    if(index==0)
      for(k=0;k<motorangle;k++)
      {
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08);
          Delay(10000);
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);
          Delay(10000);
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);
          Delay(10000);
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x01);
          Delay(10000);
      }
    else 
        for(k=0;k<motorangle;k++)
      {
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x01);
          Delay(10000);
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x02);
          Delay(10000);
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x04);
          Delay(10000);
          GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0x08);
          Delay(10000);
      }
    motorangle=0; 
    
}
void reset(){
press_sw[0]=0;
press_sw[1]=0;
press_sw[2]=0;
press_sw[3]=0;
press_sw[4]=0;
press_sw[5]=0;
press_sw[6]=0;
press_sw[7]=0;
press_pj1=0;
press_pj0=0;
press_flag=0; 
 index_module = 0;                                        //??    
 studentflag=0;
 remember_flag=0;

 backflag=0;
 timecompare=0;
 set_flagtime=1;
 set_flagdate=5;
 set_flagalarm=1;
 Flash_f=0;
 temp_k=0;
presshistory_sw=1;
 RxEndFlag = 0;
 flash_counter=0;
num_t=0;
num_d=0;
num_a=0;
dj_10=0;
ld_index=0;
ld_max=0;
index_flash=0;//????
Time_dj[0]=0;
Time_dj[1]=0;
Time_dj[2]=1;
Time_dj[3]=0;
Time_dj[4]=0;
Time_dj[5]=0;
counter=0;
index_jieqi=0;
dj_index=0;
set_flagdj=2;
pj0index=1;
alarm_hour[0]=0;
alarm_hour[1]=0;
alarm_hour[2]=0;
alarm_min[0]=0;
alarm_min[1]=0;
alarm_min[2]=1;
i_t=0;
i_d=0;
i=0;
 xindex=0;
 yindex=0;
 xindex_step;
 yindex_step=0;
flag_timehour = 0;
flag_pm = 0;                           //0?24???,1?12???
timeup_index1=0;
timeup_index2=0;                                     //????
alarm_flag[0] = 0; 
alarm_flag[1] = 0;                                         //????????
sleep_mode=0;


}

