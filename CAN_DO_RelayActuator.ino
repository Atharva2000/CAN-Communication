#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2    // Set INT to pin 2
MCP_CAN CAN0(10);     // Set CS to pin 10

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];  // Array to store serial string
byte relay_stat[3]= {0x00, 0x00, 0x00};//status display
byte relayCANID=0x003;
byte controlCANID=0x004;
///////////////////////////
byte Rel1=A1;
byte Rel2=4;
byte Rel3=5;
byte Rel4=6;
byte Rel5=7;
byte Rel6=8;
byte Rel7=9;
byte RelA=A2;
byte RelB=A3;
unsigned int actuatordelay=500;
byte sensorPin1 = 3; // select the input pin for Pilot

void relaymatrix(byte Relval);
void TestMode();
void SerialRead();

byte Relval1=0b10000000;byte Relval2=0b01000000;byte Relval3=0b00100000;byte Relval4=0b00010000;
byte Relval5=0b00001000;byte Relval6=0b00000100;byte Relval7=0b00000010;byte Relval8=0b00000001;
unsigned int Relstat=0;

///////////////////////////
/////SETUP/////////////////
///////////////////////////
void setup() 
{
Serial.begin(9600);
 
pinMode(Rel1, OUTPUT); pinMode(Rel2, OUTPUT); pinMode(Rel3, OUTPUT); pinMode(Rel4, OUTPUT);
pinMode(Rel5, OUTPUT); pinMode(Rel6, OUTPUT); pinMode(Rel7, OUTPUT);//pinMode(Rel8, OUTPUT);
pinMode(RelA, OUTPUT); pinMode(RelB, OUTPUT);
pinMode(sensorPin1,INPUT);
if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully! Ready to receive ...");
else Serial.println("Error Initializing MCP2515...");
CAN0.setMode(MCP_NORMAL); 
  
relaymatrix(0);

}


///////////////////////////
/////////MAIN LOOP/////////
///////////////////////////
void loop() {
//************************  RECEIVE CAN **************************************************
    rxId=0xFFF;//initialise to unused ID
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
//************************  RECEIVE CAN **************************************************
    
    if(rxId == controlCANID)
    {    
      rxId=0xFFF;//initialise to unused ID
      
      Serial.print("Control Sig. received : 0x0"); Serial.println(rxId);
      for(byte i = 0; i<len; i++){sprintf(msgString, " 0x%.2X", rxBuf[i]);Serial.print(msgString);}
      //Set Proxy Resistance
      if(rxBuf[0]==0x01){ relaymatrix(rxBuf[1]); CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);}

      //Actuate Microbot A
      if(rxBuf[0]==0x02) ActuateA();
         
      //Actuate Microbot B
      if(rxBuf[0]==0x03) ActuateB();
      
      //Start Proxy Resistance Test Mode
      if(rxBuf[0]==0xFF)TestMode();
          
    }
  
  CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
  delay(100);
  SerialRead();
}

///////////////////////////
/////Proxy Relay Setting///
///////////////////////////

void relaymatrix(byte Relval)
{

 
  
 digitalWrite(Rel1,(Relval1&Relval));  digitalWrite(Rel2,(Relval2&Relval));  digitalWrite(Rel3,(Relval3&Relval));  digitalWrite(Rel4,(Relval4&Relval));
 digitalWrite(Rel5,(Relval5&Relval));  digitalWrite(Rel6,(Relval6&Relval));  digitalWrite(Rel7,(Relval7&Relval));// digitalWrite(Rel8,(Relval8&Relval));

Serial.println("");
 *(&Relstat + Relval1+1)=3400;//3.4K
 *(&Relstat + Relval2+1)=2700;//
 *(&Relstat + Relval3+1)=1500;//
 *(&Relstat + Relval4+1)=680;//
 *(&Relstat + Relval5+1)=470;//
 *(&Relstat + Relval6+1)=220;//
 *(&Relstat + Relval7+1)=150;//
if(Relval==0) {Serial.println("OpenLine");
}else {Serial.print( *(&Relstat+ Relval+1));Serial.println(" Ohm  ");}

//For CAN
 *(&Relstat + Relval1)=0x1;//3.4K
 *(&Relstat + Relval2)=0x2;//
 *(&Relstat + Relval3)=0x3;//
 *(&Relstat + Relval4)=0x4;//
 *(&Relstat + Relval5)=0x5;//
 *(&Relstat + Relval6)=0x6;//
 *(&Relstat + Relval7)=0x7;//
relay_stat[0]=*(&Relstat+ Relval);

}

///////////////////////////
/////Proxy Test Mode///////
///////////////////////////

void TestMode()
{ Serial.println(" ");
  Serial.println("Test mode starts in 5s - Check the resistance with multimeter :");
        int delayrelay = 5000;
        delay(delayrelay);
        relaymatrix(Relval1);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(Relval2);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(Relval3);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(Relval4);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(Relval5);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(Relval6);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(Relval7);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
        delay(delayrelay);
        relaymatrix(0);
        CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
}

///////////////////////////
/////Serial COM control////
///////////////////////////
void SerialRead(){
  int inByte = Serial.read(); // read the incoming data
  if (inByte == '0') 
      relaymatrix(0);
  if (inByte == '1') 
      relaymatrix(Relval1);
  if (inByte == '2') 
      relaymatrix(Relval2);
  if (inByte == '3') 
      relaymatrix(Relval3); 
  if (inByte == '4') 
      relaymatrix(Relval4);
  if (inByte == '5') 
      relaymatrix(Relval5);
  if (inByte == '6') 
      relaymatrix(Relval6);
  if (inByte == '7') 
      relaymatrix(Relval7);
  if (inByte == 't') 
      TestMode();
  if (inByte == 'A') 
      ActuateA();  
  if (inByte == 'B') 
      ActuateB();
    if(inByte == 'a') 
      Serial.println(analogRead(sensorPin1)); // read the value from the sensor
}

void ActuateA(){
  Serial.println("Trying to actuate Actuator A");
         digitalWrite(RelA,1);   
         relay_stat[1]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
         delay(actuatordelay/4);   
         relay_stat[1]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
         delay(actuatordelay/4);
         relay_stat[1]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
         delay(actuatordelay/4);
         relay_stat[1]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
         delay(actuatordelay/4);
         digitalWrite(RelA,0);
         relay_stat[1]=0x00;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
}

void ActuateB(){ 
     Serial.println("Trying to actuate Actuator B");
     digitalWrite(RelB,1);   
     relay_stat[2]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
     delay(actuatordelay/4);   
     relay_stat[2]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
     delay(actuatordelay/4); 
     relay_stat[2]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
     delay(actuatordelay/4); 
     relay_stat[2]=0x01;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
     delay(actuatordelay/4); 
     digitalWrite(RelB,0);
     relay_stat[2]=0x00;CAN0.sendMsgBuf(relayCANID, 0, 3, relay_stat);
     }
