#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define DO   3
#define CS   4
#define CLK  5
Adafruit_MAX31855 thermocouple(CLK, CS, DO);

#define HeaterPin 14
#define stirpin 21
#define OD_LED 19
#define LevelLEDPin 17
#define PumpWS 47 //Pump West South
#define PumpES 49 //Pump East South
#define PumpWN 51 //Pump West North
#define PumpEN 53 //Pump East North

#define ODSensorPin A15
#define LevelSensorPin A13
#define ODLEDSensorPin A12
#define LevelLEDSensorPin A14

//color 1&2 pins
#define ExciteLEDRefPin A6

//-------------------------------------------------------
float temperature_SetTo = 37;
int target_OD=215;//OD 0.33

int Fluorostat_target_channel=0;
int Fluorostat_target_gain= 46; //26,31,36,41,46,51,56
int Target_fluoro[2]={500,200};
//-------------------------------------------------------
const char PMTReadingPin[2] = {A3,A9};
const char ExciteLEDPin[2] = {A0,A7};
const char PMTSensitivityPin[2] = {A2,A5};
const int PMTGainPin[2] = {11,9};
const int LEDGainPin[2] = {12,8};

float thermocouple_celcius=0;
char OneWireAddress ='0';

int InitialPMTGain[2]={0,0}; //25-51 should be 0.5~1.1V    when set 100-204//   // value 0-255 as 0-5V // sensitive control
int InitialLEDGain[2]= {0,0};//value 0-255 as 0-5V

float SensitivitySetTo[2]={0,0};
int ExcitationSetTo[2]={0,0};
int SensitivityOutput[2];
int PMTRead[2];
int ExcitationOutput[2]; 
float ExcitationRFRead[2];
int PMTSwitch[2]={1,1};
float ODRead;
float ODLEDrf;
float LiquidLevel;
float LevelODLEDrf;

int LEDGain[2]={InitialLEDGain[0],InitialLEDGain[1]};
int PMTGain[2]={InitialPMTGain[0],InitialPMTGain[1]};
const int PMTGainUpperBoundLimit=6;
const int PMTGainLowerBoundLimit=0;
int BoundChangeByUp[2]={0,0};
int BoundChangeByLow[2]={0,0};
int PMTGainCurrent[2]={0,0};
int PMTGainLowerBound[2]={PMTGainLowerBoundLimit,PMTGainLowerBoundLimit};
int PMTGainUpperBound[2]={PMTGainUpperBoundLimit,PMTGainUpperBoundLimit};


int PMTGainInput[2]={0,0};
int LEDGainInput[2]={0,0};
int LastPMTRead[2]={0,0};
int FluoroTargetRead=0;
int LastPMTGain[2]={0,0};
int LastODRead=0;
int LastTempRead=0;
int LastLiquidLevel=0;

int ODLEDStatus=0;
int StirStatus=0;
int HeaterStatus=0;
int PumpWNStatus=0;
int PumpWSStatus=0;
int PumpENStatus=0;
int PumpESStatus=0;
int LevelLEDStatus=0;
double ChemicalFraction=0;

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(9600);  // initialize serial communication at 9600 bits per second:
  pinMode(LEDGainPin[0], OUTPUT);   // sets the pin as output
  pinMode(LEDGainPin[1], OUTPUT);   // sets the pin as output
  pinMode(PMTGainPin[0], OUTPUT);   // sets the pin as output
  pinMode(PMTGainPin[1], OUTPUT);   // sets the pin as output
  pinMode(HeaterPin, OUTPUT);
  pinMode(stirpin, OUTPUT);
  pinMode(PumpWS, OUTPUT);
  pinMode(PumpES, OUTPUT);
  pinMode(PumpWN, OUTPUT);
  pinMode(PumpEN, OUTPUT);
  pinMode(OD_LED, OUTPUT);
  pinMode(LevelLEDPin, OUTPUT);

  delay(50);
  
 
  ExciteLED_SET(0,InitialLEDGain[0]);
  PMT_SET(0,InitialPMTGain[0]);
  ExciteLED_SET(1,InitialLEDGain[1]);
  PMT_SET(1,InitialPMTGain[1]);
  
  stir(5);
  wait(5);
  
  //ExciteLED_SET(0,255);

  
}

//----------------------------------------------
void PMT_SET(int Which,int Gain){
  analogWrite(PMTGainPin[Which],Gain);
  SensitivitySetTo[Which] = float(5*Gain*4)/1023;
  send_data();
  delay(100);
}
void PMT_OFF(int Which){
  analogWrite(PMTGainPin[Which],0);
  delay(100);
  SensitivitySetTo[Which]=0;
  send_data();
}
void ExciteLED_SET(int Which, int Gain){
  analogWrite(LEDGainPin[Which], Gain);
  delay(100);
  ExcitationSetTo[Which] = Gain*4;
  send_data();
}
void ExciteLED_OFF(int Which){
  analogWrite(LEDGainPin[Which], 0);
  ExcitationSetTo[Which]=0;
  send_data();
  delay(100);
}

void ODLED_ON(){
  digitalWrite(OD_LED,HIGH);
  ODLEDStatus=1;
  send_data();
  delay(100);
}
void ODLED_OFF(){
  digitalWrite(OD_LED,LOW);
  ODLEDStatus=0;
  send_data();
  delay(100);
}
void Stir_ON(){
  analogWrite(stirpin, 230);
  StirStatus=1;
  send_data();
  delay(100);
}
void Stir_OFF(){
  analogWrite(stirpin, 0);
  StirStatus=0;
  send_data();
  delay(100);
}
void Heater_ON(){
  digitalWrite(HeaterPin, LOW);
  HeaterStatus=1;
  send_data();
  delay(100);
}
void Heater_OFF(){
  digitalWrite(HeaterPin,HIGH);
  HeaterStatus=0;
  send_data();
  delay(100);
}
void PumpMediaIn_ON(){
  digitalWrite(PumpWN, HIGH);
  PumpWNStatus=1;
  send_data();
  delay(100);
}
void PumpMediaIn_OFF(){
  digitalWrite(PumpWN, LOW);
  PumpWNStatus=0;
  send_data();
  delay(100);  
}
void PumpChemicalIn_ON(){
  digitalWrite(PumpWS, HIGH);
  PumpWSStatus=1;
  send_data();
  delay(100);
}
void PumpChemicalIn_OFF(){
  digitalWrite(PumpWS, LOW);
  PumpWSStatus=0;
  send_data();
  delay(100);  
}

void PumpOut_ON(){
  digitalWrite(PumpEN, HIGH);
  PumpENStatus=1;
  send_data();
  delay(100);
}
void PumpOut_OFF(){
  digitalWrite(PumpEN, LOW);
  PumpENStatus=0;
  send_data();
  delay(100);
}
void LevelLED_ON(){
  digitalWrite(LevelLEDPin, HIGH);
  LevelLEDStatus=1;
  send_data();
  delay(100);
}
void LevelLED_OFF(){
  digitalWrite(LevelLEDPin, LOW);
  LevelLEDStatus=0;
  send_data();
  delay(100);
}


//----------------------------------------------

void turbido_pump(int duration, int ODSet) {
  
    if(LastODRead < ODSet){
      stir(5);//5
      pump_out(duration); // in case the water level too high
     
      //Stir_ON();
      wait(1);
      
      PumpMediaIn_ON();
      wait(duration);
      PumpMediaIn_OFF();
      //wait(9-duration);
      
      //Stir_OFF();
      stir(10);//10
      wait(2);
      
      temperature_measurements_and_set_peltier();
      pump_out(duration*4); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
      pump_off();//make sure that the pump is off  
  }
    
    else{ /// just in case
      stir(5);
      pump_out(duration); 
      pump_off();
    }
}

//----------------------------------------------


void Fluoro_pump(int FluoroSet, int duration) {
  
    if(FluoroTargetRead > FluoroSet){
      stir(5);
      pump_out(duration); // in case the water level too high
     
      //Stir_ON();
      wait(1);
      
      PumpMediaIn_ON();
      wait(duration);
      PumpMediaIn_OFF();
      
      //wait(9-duration);
      
      //Stir_OFF();
      stir(10);
      
      wait(2);
      
      temperature_measurements_and_set_peltier();
      pump_out(duration*4); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
      pump_off();//make sure that the pump is off  
  }
    
    else{ /// just in case
      stir(5);
      pump_out(duration); 
      pump_off();
    }
}

void Metabo_pump(int duration,int ODSet, int FluoroSet) {

    if(LastODRead < ODSet){

      if(FluoroTargetRead > FluoroSet){ //expression too high
        ChemicalFraction=ChemicalFraction-0.03;
      }
      if(FluoroTargetRead < FluoroSet){
        ChemicalFraction=ChemicalFraction+0.03;
      }
      else {}

    if (ChemicalFraction>1){
          ChemicalFraction=1;
    }
    if (ChemicalFraction<0){
          ChemicalFraction=0;
    }
      
      stir(5);
      pump_out(duration); // in case the water level too high
     
      wait(1);
       
      PumpMediaIn_ON();
      delay((duration*(1-ChemicalFraction))*1000);
      PumpMediaIn_OFF();

      PumpChemicalIn_ON();
      delay(duration*ChemicalFraction*1000);
      PumpChemicalIn_OFF();
      
      stir(10);
      wait(2);
           
      temperature_measurements_and_set_peltier();
      pump_out(duration*4); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
      pump_off();//make sure that the pump is off  
  }
    
    else{ /// just in case
      stir(5);
      pump_out(duration); 
      pump_off();
    }
}


//----------------------------------------------

void pump_out(int duration) {
    PumpOut_ON();
    wait(duration);
    PumpOut_OFF();
}

void PumpMedia_in(int duration) {
    PumpMediaIn_ON();
    wait(duration);
    PumpMediaIn_OFF();
}

//----------------------------------------------
void pump_off() {
      PumpMediaIn_OFF();
      PumpChemicalIn_OFF();
      PumpOut_OFF();
}

//----------------------------------------------

void stir(int duration) {
  Stir_ON();
  wait(duration);
  Stir_OFF();
}

//----------------------------------------------
void wait(int duration){
  for (int i = 0; i <duration; i++){
    delay(1000);
  }
}
//----------------------------------------------
void temperature_measurements_and_set_peltier() { // N seconds
  /////////Temp Measurements
  thermocouple_celcius = thermocouple.readCelsius();
  LastTempRead=thermocouple_celcius;
  delay(50);
  /////////set temperature
  if (LastTempRead < temperature_SetTo){
    Heater_ON();  
  }
  else
    Heater_OFF(); 
}

//----------------------------------------------
void  PMT_rolling_measurements(int which){
  
  if (BoundChangeByUp !=0 and BoundChangeByLow[which] !=0 ){
    PMTGainLowerBound[which]=PMTGainLowerBound[which]+1;
  }
  BoundChangeByUp[which]=0;
  BoundChangeByLow[which]=0;

  
  PMTGainCurrent[which]=PMTGainLowerBound[which];
  
  while(PMTGainCurrent[which]<PMTGainUpperBound[which]+1 && BoundChangeByLow[which]==0 && BoundChangeByUp[which]==0){
    
    PMTGainCommand_converting(which);
    PMTSet_measurements(which,LEDGain[which],PMTGain[which]);
    set_PMT_from_internal(which);//rolling PMT
  }
}

void set_PMT_from_internal(int which){
//////////////////////////////////////////////////////////////////////Decide what to set  
  if (LastPMTRead[which]<900 && LastPMTRead[which]>10){ // when signal voltage is in good range
    PMTGainCurrent[which]++;      // increase PMT Gain
  }
  
  if (LastPMTRead[which]<10){ // when signal voltage is too low
    if (PMTGainUpperBound[which]<PMTGainUpperBoundLimit){ // if  the max PMT gain is not yet set to the cap
      PMTGainLowerBound[which]=PMTGainCurrent[which]+1;
      PMTGainUpperBound[which]=PMTGainUpperBound[which]+1;
      BoundChangeByLow[which]++;
      PMTGainCurrent[which]++;
  }
    else {
      PMTGainLowerBound[which]=PMTGainCurrent[which]+1;
      PMTGainUpperBound[which]=PMTGainUpperBoundLimit;
      BoundChangeByLow[which]++;
      PMTGainCurrent[which]++;
    }
  }
  
  if (LastPMTRead[which]>900){           //when signal voltage is to high   
    if (PMTGainUpperBound[which]>PMTGainLowerBoundLimit){
       PMTGainUpperBound[which]=PMTGainCurrent[which]-1;          // set the max PMT gain as 1 level lower than the current PMT gain

       if(PMTGainLowerBound[which]>PMTGainLowerBoundLimit ){          // if  the PMT gain is not yet set to 0, and 
          PMTGainLowerBound[which]=PMTGainLowerBound[which]-1;          // set the max PMT gain as 1 level lower than the current PMT gain
       }
      
       if(PMTGainLowerBound[which]==PMTGainLowerBoundLimit){          // if  the PMT gain is not yet set to 0, and 
          PMTGainLowerBound[which]=PMTGainLowerBoundLimit;          // set the max PMT gain as 1 level lower than the current PMT gain
       }
       BoundChangeByUp[which]++;
       PMTGainCurrent[which]++;
      }
    
    else{             // if  the max PMT gain has set to 0
      PMTSwitch[which]=0;    // stop using PMT
      //LEDGain[which] =LEDGain[which]-100//or try to lower LED and set gain cap back
      PMTGainCurrent[which]++;
      }
    } 
}
//----------------------------------------------

void PMTGainCommand_converting(int which){
  
  if (PMTSwitch[which]!=0){  // continue using PMT
    LEDGain[which] = 255;
    PMTGain[which] =int(26+5*float(PMTGainCurrent[which]));
  }
  else {              // stop using PMT
    LEDGain[which]=0;
    PMTGain[which]=0;
  }
}

//----------------------------------------------
void PMTSet_measurements(int Which, int LedG, int PmtG){
  ExciteLED_SET(Which,LedG);
  PMT_SET(Which,PmtG);
  delay(10500); // cant not lower than 10000. LED and PMT needs time to be stable when start from 0 
  temperature_measurements_and_set_peltier();
  //Heater_OFF();//////turn off the heater when measure
  //delay(2000);

  float sen=0;
  float PMTr=0;
  float excit=0;
  float LEDr=0;
  int i=0;
  //int NumOfPoints=20;
  int NumOfPoints=100;

  for (i=0; i<NumOfPoints; i++){ /// start measuring
    sen=sen+analogRead(PMTSensitivityPin[Which]);
    PMTr=PMTr+analogRead(PMTReadingPin[Which]);
    excit=excit+analogRead(ExciteLEDPin[Which]);
    LEDr=LEDr+analogRead(ExciteLEDRefPin);
    delay(50);
  }

  PMTRead[Which] =float(float(PMTr)/float(NumOfPoints)); /// calculate average
  SensitivityOutput[Which]=float(float(sen)/float(NumOfPoints));
  ExcitationOutput[Which]=int(float(excit)/float(NumOfPoints)); 
  ExcitationRFRead[Which]=int(float(LEDr)/float(NumOfPoints)); 
  LastPMTRead[Which]=PMTRead[Which]; ////////////////////////////////////////////////
  if (PmtG==Fluorostat_target_gain && Which ==Fluorostat_target_channel){
    FluoroTargetRead=PMTRead[Which]; ////////////////////////////////////////////////
  }
  send_data();
  delay(50);
  
  ExciteLED_OFF(Which);
  PMT_OFF(Which);  
  temperature_measurements_and_set_peltier();
  delay(2000);//wait for voltage stable

}

//----------------------------------------------
void OD_signal_read(){
  float ODsensor=0;
  float ODLEDsensor=0;
  int i=0;
  int NumOfPoints=20;
  Heater_OFF(); // for LED stablize
  delay(500);
  ODLED_ON();
  delay(1000);// wait 1s for Led stablize

  for (i=0; i<NumOfPoints; i++){
    delay(500);
    ODLEDsensor=ODLEDsensor+analogRead(ODLEDSensorPin);
    delay(500);
    ODsensor=ODsensor+analogRead(ODSensorPin);
    Serial.println(analogRead(ODSensorPin));
  }
  ODLEDrf=float(ODLEDsensor)/float(NumOfPoints); 
  ODRead =float(ODsensor)/float(NumOfPoints); 
  LastODRead=ODRead;
  send_data();
  delay(50);
  ODLED_OFF();//turn off ODLED
  delay(50);
  temperature_measurements_and_set_peltier();
}

void LevelSensing(){
  
  float Levelsensor=0;
  float LevelLEDsensor=0;
  int i=0;
  //int NumOfPoints=50;
  int NumOfPoints=10;
  Heater_OFF(); // for LED stablize
  delay(500);
  LevelLED_ON();
  delay(1000);
  
  for (i=0; i<NumOfPoints; i++){
    delay(100);
    Levelsensor=Levelsensor+analogRead(LevelSensorPin);
    LevelLEDsensor=LevelLEDsensor+analogRead(LevelLEDSensorPin);
  }
  LevelODLEDrf=float(LevelLEDsensor)/float(NumOfPoints); 
  LiquidLevel=float(Levelsensor)/float(NumOfPoints); 
  LastLiquidLevel=LiquidLevel;
  send_data();
  delay(50);
  LevelLED_OFF();//turn off ODLED
  delay(50);

}

//----------------------------------------------
void CleanData(){
PMTRead[0]= double(0);
PMTRead[1]= double(0);
ODRead=0;
SensitivityOutput[0]= double(0);
ExcitationOutput[0]= double(0); 
ExcitationRFRead[0]= double(0);
SensitivityOutput[1]= double(0);
ExcitationOutput[1]= double(0); 
ExcitationRFRead[1]= double(0);
ODLEDrf=0;
thermocouple_celcius=0;
LiquidLevel=0;
LevelODLEDrf=0;
}

//----------------------------------------------
void Printdata(){
  //signal scale conversion
  float  SenV0 = SensitivityOutput[0] * (5.0 / 1023.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float  SenV1 = SensitivityOutput[1] * (5.0 / 1023.0); 

  /////////print measurements
  
  double data[]={SensitivitySetTo[0],SenV0,ExcitationSetTo[0],ExcitationOutput[0],ExcitationRFRead[0],Target_fluoro[0],PMTRead[0],
                 SensitivitySetTo[1],SenV1,ExcitationSetTo[1],ExcitationOutput[1],ExcitationRFRead[1],Target_fluoro[1],PMTRead[1],
                 target_OD,
                 ODRead,ODLEDrf,ODLEDStatus,
                 LiquidLevel,LevelODLEDrf,LevelLEDStatus,
                 temperature_SetTo,thermocouple_celcius,
                 StirStatus,HeaterStatus,
                 PumpWNStatus,PumpWSStatus,PumpENStatus,PumpESStatus,
                 ChemicalFraction};
                   
  int data_length=0;
  data_length=sizeof(data)/sizeof(data[0]);

  for (int j=0;j<data_length-1;j++){  
    Serial.print(data[j]);
    Serial.print(" ");
  }
  Serial.println(data[data_length-1]);
}

void send_data(){
  Printdata();
  CleanData();
  delay(100);
}

//.--------------------------------------------------------------algorithms--------------------------------------------------------------


void Pump_for_Exp_Start(){
  
  pump_out(8); // motors pump fluid out (for tudbidostat),4 times PumpOut time than PumpIn time
  
  Stir_ON();
  wait(1);
      
  PumpMediaIn_ON();
  wait(2);
  PumpMediaIn_OFF();
  wait(7);
      
  Stir_OFF();
  wait(2);
  
  temperature_measurements_and_set_peltier();
  pump_out(2);
  pump_off();//make sure that the pump is off  

}


void Thorlab(){
  //temperature_measurements_and_set_peltier();
  ExcitationRFRead[0]=int(analogRead(ExciteLEDRefPin));
  send_data();
}

void turbidostat_twocolors(int targetOD){
  target_OD=targetOD;
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;

  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
  
  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(1);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
  
  stir(10);

////////for turbido
  for(int i=0;i<3;i++){ //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    turbido_pump(2,targetOD); // 2 secs pump in, in case of overflow
    LevelSensing();
  }
}

void turbidostat_green(int targetOD){
  target_OD=targetOD;
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;

  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();  
  stir(10);

  for(int i=0;i<3;i++){ //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    turbido_pump(2,targetOD); // 2 secs pump in, in case of overflow
    LevelSensing();
  }
}


void calibration(){
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;
  
  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();  
  stir(10);
}

void system_test(){
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;
  PMT_rolling_measurements(0);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(1);
  temperature_measurements_and_set_peltier();
  OD_signal_read();  
  LevelSensing();
}

void fluorostat(int channel, int target){

  target_OD=0;
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;
  Target_fluoro[channel]=target;
  
  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
  
  //stir(10);
  //temperature_measurements_and_set_peltier();
  //PMT_rolling_measurements(1);
  //stir(10);
  //temperature_measurements_and_set_peltier();
  //OD_signal_read();
  
  stir(10);

////////for fluorostat
  for(int i=0;i<3;i++){ //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    Fluoro_pump(target, 2); //(channel, target reading, secs pump in)
    LevelSensing();
  }
}


void batch(){
  ////batch 
  stir(5);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
  
  stir(5);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(1);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
}

void rolling_measure(){

  target_OD=0;
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;
  
  stir(10);
  PMT_rolling_measurements(0);
  stir(10);
  OD_signal_read();
}


void Add_Sample(){
stir(10);
temperature_measurements_and_set_peltier();
}


void Calibration_exp(){

  //while (LastTempRead < temperature_SetTo){ 
  //temperature_measurements_and_set_peltier();
  //stir(5);  
  //}
  stir(10);

  for (int i=0;i<5;i++){
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(5);
  temperature_measurements_and_set_peltier();
  OD_signal_read();
  }
  
  for (int i=0;i<3000;i++){ 
  temperature_measurements_and_set_peltier();
  stir(5);  
  }
}

void Metabostat(int targetOD, int channel, int target){

  target_OD=targetOD;
  Target_fluoro[0]=1000;
  Target_fluoro[1]=1000;
  Target_fluoro[channel]=target;
  
  stir(10);
  temperature_measurements_and_set_peltier();
  PMT_rolling_measurements(0);
  stir(10);
  temperature_measurements_and_set_peltier();
  OD_signal_read();

  stir(10);

////////for Metabostat
  for(int i=0;i<3;i++){ //3 times make sure dilute enough
    temperature_measurements_and_set_peltier();
    wait(1);
    Metabo_pump(2,targetOD,target); // c secs pump in, in case of overflow
    LevelSensing();
  }
}
  


//--------------------------------------------------------------modes--------------------------------------------------------------


// the loop routine runs over and over again forever:
void loop() {

//Pump_for_Exp_Start();

//PumpChemicalIn_OFF();
//Add_Sample();
//Calibration_exp();

//turbidostat_twocolors(target_OD);
//turbidostat_green(target_OD);
//fluorostat(Fluorostat_target_channel,Target_fluoro[Fluorostat_target_channel]);//(channel (0 or 1), gain, target reading)

Metabostat(target_OD,Fluorostat_target_channel,Target_fluoro[Fluorostat_target_channel]);//(OD,channel (0 or 1), gain, target reading)

//rolling_measure();
//Thorlab();
//system_test();

}
