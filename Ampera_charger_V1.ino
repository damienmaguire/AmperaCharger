/*Control Ampera charger.
 * Uses a repurposed Leaf vcu v2 as hardware. Will run on any arduino due (SAM3X8E) based board with a can transciever.
 * Connect at 115200 baud and send ? to bring up menu
 * Termination current option not enabled in this release.
 * Copyright 2021 D.Maguire
 * openinverter.org
 * evbmw.com
 * 
 * Based on the work of Tom deBree, Arber Kramer , Colin Kidder , EVTV and others
 */


#include <Metro.h>
#include <due_can.h>  //https://github.com/collin80/due_can
#include <due_wire.h> //https://github.com/collin80/due_wire
#include <DueTimer.h>  //https://github.com/collin80/DueTimer
#include <Wire_EEPROM.h> //https://github.com/collin80/Wire_EEPROM

#define EEPROM_VERSION      11



////////////////////////////////////////////////////////////////////////////////////////////////////
CAN_FRAME outFrame;  //A structured variable according to due_can library for transmitting CAN data.
CAN_FRAME inFrame;    //structure to keep inbound inFrames

 
Metro timer_diag = Metro(1100);
Metro timer_Frames200 = Metro(200);
Metro timer_Frames30 = Metro(30);
float HVcur=0;
float HVvol=0;
float ACcur=0;
float ACvol=0;
float LVcur=0;
float LVvol=0;
float Version=1.0;
uint16_t Vol_temp=0;
byte CHGCON=0x00;
bool chgdisp=false;

#define SerialDEBUG SerialUSB
 template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; } //Allow streaming

 typedef struct
{
uint8_t  version; //eeprom version stored
uint16_t Vol=0;
uint16_t Cur=0;
uint16_t Term=0;
}ControlParams;
     
ControlParams parameters;
 
void setup() {

   Can0.begin(CAN_BPS_500K);  //CAN bus
   Can1.begin(CAN_BPS_500K);  //CAN bus 
   Can1.watchFor();
   pinMode(13, OUTPUT);  //led
   Serial2.begin(19200); //setup serial 2 for wifi access

      Wire.begin();
  EEPROM.read(0, parameters);
  if (parameters.version != EEPROM_VERSION)
  {
    parameters.version = EEPROM_VERSION;
    parameters.Vol=0;
    parameters.Cur=0;
    parameters.Term=0;
    EEPROM.write(0, parameters);
  }
 
}






void handle_wifi(){
/*
 * 
 * Routine to send data to wifi on serial 2
The information will be provided over serial to the esp8266 at 19200 baud 8n1 in the form :
vxxx,ixxx,pxxx,mxxxx,nxxxx,oxxx,rxxx,qxxx* where :

v=pack voltage (0-700Volts)
i=current (0-1000Amps)
p=power (0-300kw)
m=half pack voltage (0-500volts)
n=Amp Hours (0-300Ah)
o=KiloWatt Hours (0-150kWh)
r=HV Box Temp (0-100C)
q=third pack Volts (0-500Volts)
*=end of string
xxx=three digit integer for each parameter eg p100 = 100kw.
updates will be every 1000ms approx.

v100,i200,p35,m3000,n4000,o20,r100,q50*
*/
  
Serial2.print("v100,i200,p35,m3000,n4000,o20,r30,q50*"); //test string

digitalWrite(13,!digitalRead(13));//blink led every time we fire this interrrupt.
if(chgdisp)
{
SerialDEBUG.println("//////////////////AMPERA CHARGER//////////////////////////////////////////////////////////");
SerialDEBUG.print("HV Current=");                                                                             
SerialDEBUG.print(HVcur,1);
SerialDEBUG.println("Amps");
SerialDEBUG.print("HV Voltage=");
SerialDEBUG.print(HVvol,1);
SerialDEBUG.println("Volts");
SerialDEBUG.print("AC current=");
SerialDEBUG.print(ACcur,1);
SerialDEBUG.println("Amps");
SerialDEBUG.print("AC Voltage=");
SerialDEBUG.print(ACvol,1);
SerialDEBUG.println("Volts");
SerialDEBUG.print("LV current=");
SerialDEBUG.print(LVcur,1);
SerialDEBUG.println("Amps");
SerialDEBUG.print("LV Voltage=");
SerialDEBUG.print(LVvol,1);
SerialDEBUG.println("Volts");
SerialDEBUG.print("Voltage Setpoint=");
SerialDEBUG.print(parameters.Vol);
SerialDEBUG.println("Volts");
SerialDEBUG.print("Current Setpoint=");
SerialDEBUG.print(parameters.Cur);
SerialDEBUG.println("Amps");
SerialDEBUG.print("Termination Current Setpoint=");
SerialDEBUG.print(parameters.Term);
SerialDEBUG.println("Amps");
SerialDEBUG.print("Charger Status=");
if(CHGCON==0x00) SerialDEBUG.println("Disabled");
if(CHGCON==0x03) SerialDEBUG.println("Enabled");
SerialDEBUG.println("//////////////////////////////////////////////////////////////////////////////////////////");
}
}

void printMenu()
{
   SerialDEBUG<<"\f\n=========== EVBMW Ampera Charger Controller "<<Version<<" ==============\n************ List of Available Commands ************\n\n";
   SerialDEBUG<<"  ?  - Print this menu\n ";
   SerialDEBUG<<"  d - Toggle charger data display\n";
   SerialDEBUG<<"  D - Disable Charger\n";
   SerialDEBUG<<"  e  - Enable charger\n ";
   SerialDEBUG<<"  g  - Set Charge Voltage. e.g. g250 sets 250volts charge voltage\n ";
   SerialDEBUG<<"  i  - Set charge Current. e.g. i5 sets 5amps charge current\n ";
   SerialDEBUG<<"  q  - Set Charge Termination Current. e.g. i=2 sets 2amps charge termination current\n ";
   SerialDEBUG<<"  z  - Save configuration data to EEPROM memory\n ";
   
   SerialDEBUG<<"**************************************************************\n==============================================================\n\n";
   
}
void checkforinput()
{ 
  //Checks for keyboard input from Native port 
   if (SerialDEBUG.available()) 
     {
      int inByte = SerialDEBUG.read();
      switch (inByte)
         {
          case 'z':            
          EEPROM.write(0, parameters);
           SerialDEBUG.println("Parameters stored to EEPROM");
            break;
  
          case 'd':    
            chgdisp=!chgdisp;
            break;
            
          case 'D':    
            CHGCON=0x00;
            SerialDEBUG.println("Charger Disabled");
            break;
        
          case 'e':    
            CHGCON=0x03;
            SerialDEBUG.println("Charger Enabled");
            break;
            
          case 'g':    
              SerialDEBUG.println("");
              SerialDEBUG.print("Configured charge voltage: ");
              if (SerialDEBUG.available()) parameters.Vol = SerialDEBUG.parseInt();
              if(parameters.Vol>420) parameters.Vol=420; //limit to max 420v
              if(parameters.Vol<200) parameters.Vol=200;//limit to 200v min
              SerialDEBUG.println(parameters.Vol); 
            break; 

          case 'i':    
              SerialDEBUG.println("");
              SerialDEBUG.print("Configured charge current: ");
              if (SerialDEBUG.available()) parameters.Cur = SerialDEBUG.parseInt();
              if(parameters.Cur>12) parameters.Cur=12; //limit to max 12A
              if(parameters.Cur<0) parameters.Cur=0;//limit to 0A min
              SerialDEBUG.println(parameters.Cur); 
            break; 

          case 'q':    
              SerialDEBUG.println("");
              SerialDEBUG.print("Configured charge termination current: ");
              if (SerialDEBUG.available()) parameters.Term = SerialDEBUG.parseInt();
              if(parameters.Term>12) parameters.Term=12; //limit to max 12A
              if(parameters.Term<1) parameters.Term=1;//limit to 1A min
              SerialDEBUG.println(parameters.Term); 

            break;             
           
          case '?':     //Print a menu describing these functions
              printMenu();
            break;
            
         
          }    
      }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
///////
//HV Current: first 13 bits (divide by 20 for Amps)
//HV Voltage: next 10 bits (divide by 2 for Volts)
//LV Current: next 8 bits (divide by 5 for Amps)
//LV Voltage: next 8 bits (divide by 10 for Volts)

//0x30A seems it could be related to the AC input:
//AC Current: first 12 bits (divide by 5 perhaps?)
//AC Voltage: next 8 bits (multiply by 2 seems logical)
////////////////////////////////////////////////////////////////////////////////////////////////////
void CheckCAN()
{
  if(Can1.available())
  {
    Can1.read(inFrame);
    if(inFrame.id == 0x212)
    {
          uint16_t HVcur_temp = (uint16_t)(inFrame.data.bytes[0]<<8 | inFrame.data.bytes[1]);
          HVcur = (float)(HVcur_temp>>3)*0.05;
          
         uint16_t HVvol_temp = (uint16_t)((((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[2]))>>1)&0x3ff);
           HVvol = (float)(HVvol_temp)*.5;
           
         uint16_t LVcur_temp = (uint16_t)(((inFrame.data.bytes[2]<<8 | inFrame.data.bytes[3])>>1)&0x00ff);
          LVcur = (float)(LVcur_temp)*.2;

          
          uint16_t LVvol_temp = (uint16_t)(((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[4])>>1)&0x00ff);
          LVvol = (float)(LVvol_temp)*.1;
          
    }
    if(inFrame.id == 0x30A)
    {
         uint16_t ACcur_temp = (uint16_t)((inFrame.data.bytes[0]<<8 | inFrame.data.bytes[1])>>4);
         ACcur = (float)(ACcur_temp)*0.2;
         
         uint16_t ACvol_temp = (uint16_t)(((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[2])>>4)&0x00ff);
         ACvol = (float)(ACvol_temp)*2;
    }

    }

}

void Frames30MS()
{
    if(timer_Frames30.check())
  {
          //Coda charger
          /*
        outFrame.id = 0x050;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=0x00;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[1]=0xDC;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[2]=0x0A;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[3]=0xF0;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[4]=0x00;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[5]=0x00;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[6]=0x96;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[7]=0x01;    //command lear charger on in charge and 12v aux mode.                                                        
        Can1.sendFrame(outFrame);
        */
        //Lear charger (Ampera)
        
        outFrame.id = 0x30E;            // Set our transmission address ID
        outFrame.length = 1;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=CHGCON;  //0x03;    //command lear charger on in charge and 12v aux mode.                                                 
        Can1.sendFrame(outFrame);

        
    
  }
 
    
}


void Frames200MS()
{
    if(timer_Frames200.check())
  {
          //Lear charger (Ampera)
        outFrame.id = 0x304;            // Set our transmission address ID
        outFrame.length = 4;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=0x40;    //static value
        outFrame.data.bytes[1]=parameters.Cur*20;//0x28;    //current commanded, convert to decimal and divide by 20.A0(hex) which is 160 decimal. Divided by 20 is 8 and that is the commanded current.
        Vol_temp=parameters.Vol*2;
        outFrame.data.bytes[2]=highByte (Vol_temp);//0x03;    //first 2 bits are MSB of the voltage command.
        outFrame.data.bytes[3]=lowByte (Vol_temp);//0x20;    //LSB of the voltage command. Then MSB LSB is divided by 2. 
                                                  
        Can1.sendFrame(outFrame);

        //Data2 is 03(hex) and Data3 is 20(hex) which is 0320(hex) equals 800(decimal) divided by 2 is 400vdc.
 
  }
 
    
}



void loop() {

  if(timer_diag.check())
  {
  handle_wifi();
 
  }

 CheckCAN();
 checkforinput();
Frames30MS(); //send can frames
Frames200MS();

}
