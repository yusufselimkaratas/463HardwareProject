#include <PID_v1.h>

const int GATE=3;  //GATE PIN
const int EMF=0;   //EMF PIN
const int INT=2;   //INTERRUPT PIN



double Setpoint=1;
double Measured_Voltage;
double Measured_Value;
double Real_Voltage;
double Reference_Voltage;
double PID_OUT;
double DUTY;
double Reference_Speed;


double Error;
double Kp=10,Ki=100,Kd=0;
PID pid(&Error,&PID_OUT,&Setpoint,Kp,Ki,Kd,DIRECT);


double ResistanceRatio=100;  //It is expected as 2 MOhm-200 KOhm
double VoltageSpeed_Ratio=0.189; //It is written according to Simulation

void setup() {
  attachInterrupt(INT,setRef_Speed,RISING);
  pinMode(GATE,OUTPUT);
  pinMode(EMF,INPUT);
  pid.SetMode(AUTOMATIC);
  pid.SetTunings(Kp,Ki,Kd);
  Serial.begin(9600);
  

}

void setRef_Speed()
{
  Serial.write("Please enter desired REFERENCE SPEED as RPM in 0-1500 interval");
  Setpoint=Serial.parseInt();
}

void loop() {
  // READ DIGITAL VALUE OVER VOLTAGE DIVIDER
  Measured_Value=analogRead(EMF);
  // Calculation of Measured Voltage
  Measured_Voltage=(Measured_Value/1024)*5;
  // Calculation of Real EMF Voltage
  Real_Voltage=Measured_Voltage*(1+ResistanceRatio);
  // Reference Speed to Reference Voltage
  Reference_Voltage=Reference_Speed*VoltageSpeed_Ratio;
  // Error Calculation
  Error=Reference_Voltage-Real_Voltage;
  // PID Calculation
  pid.Compute();

  // CHECK PID_OUT for LIMITED INTERVAL
  if(PID_OUT<0)
  {
    PID_OUT=0;
    Serial.write("PID is under Limit");  
  }
  else if(PID_OUT>255)
  {
    PID_OUT=255;  
    Serial.write("PID is over Limit");
  }
  

  // SET DUTY_CYCLE
  DUTY=255-PID_OUT;
  
  //Drive the Transistor
  analogWrite(GATE,DUTY);
 }
