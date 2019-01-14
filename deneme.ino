#include <PID_v1.h>
//------------------------------------------PIN NUMBER ASSIGNMENT--------------------------------------------//
const byte BUTON1=2;  //REFERENCE SPEED BUTTON
const byte BUTON2=3;  //ROTATION CHANGE BUTTON

const int EMF=A5;    //VOLTAGE SENSE PIN

const int PWM0=5;    //BUCK CONVERTOR PWM
const int PWM1=6;    //H-BRIDGE PWM-1
const int PWM2=9;    //H-BRIDGE PWM-2
const int PWM3=10;   //H-BRIDGE PWM-3
const int PWM4=11;   //H-BRIDGE PWM-4

//------------------------------------------VARIABLE ASSIGNMENT---------------------------------------------//
long sayac=0;

int rot_sayac=0;
bool clockwise_rotation= true;
const int rot_speed[1][7]={{500,600,750,1000,1200,1400,1500}};
double ref_speed=500;
double measured_speed=0;
double measured_value=0;
double measured_voltage=0;
double volt_ratio=0.008;          //UPDATE
double volt_rpm_ratio=50;         //UPDATE
double real_voltage;
double DUTY;
double PID_OUT;
double Error;
double ref;
double mea;


double Kp=100,Ki=0.1,Kd=0;
PID pid(&Error,&PID_OUT,&ref,Kp,Ki,Kd,DIRECT);

//------------------------------------------SETUP-----------------------------------------------------------//
void setup() {
  // put your setup code here, to run once:
  
  pinMode(BUTON1,INPUT_PULLUP);
  pinMode(BUTON2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTON1),change_ref_speed,RISING);
  attachInterrupt(digitalPinToInterrupt(BUTON2),change_rotation,RISING);
  
  pinMode(EMF,INPUT);
  pinMode(PWM0,OUTPUT);
  //62 kHz   https://playground.arduino.cc/Main/TimerPWMCheatsheet
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetTunings(Kp,Ki,Kd);
  pid.SetOutputLimits(0,255);


  
  Serial.begin(9600);


  

}

void change_ref_speed() //int *rotsayac,int *rot_speed[1][7]
{
  rot_sayac=rot_sayac%6;
  rot_sayac=rot_sayac+1;
  ref_speed=rot_speed[0][rot_sayac];
  
}
void change_rotation()
{
  ref_speed=0;    //STOP ROTATION
  while(real_voltage>10)
  {
    Serial.println("HIZIN AZALMASI BEKLENIYOR...");
    delay(100);
  }
  clockwise_rotation= true or clockwise_rotation;  //TOGGLE
  ref_speed=rot_speed[0][rot_sayac];
  
}

double meas_speed()
{
   measured_value=analogRead(EMF);
   measured_voltage=(measured_value/1024)*5;
   real_voltage=measured_voltage*volt_ratio;
   measured_speed=real_voltage*volt_rpm_ratio;
   return measured_speed; 
  
}

void measure_frequency()
{
 Serial.println("BASLADI");
 sayac=0;
  while(sayac<1000000)
  {
  digitalWrite(13,100);
  delayMicroseconds(0.01);

  sayac=sayac+1;
 
  }
 Serial.println("1000 oldu");
  sayac=0; 
}
//------------------------------------------LOOP----------------------------------------------------------//
void loop() {

  



  if(clockwise_rotation)
  {
    digitalWrite(PWM1,HIGH);
    digitalWrite(PWM4,HIGH);
    digitalWrite(PWM2,LOW);
    digitalWrite(PWM3,LOW);    
  }
  else if(!clockwise_rotation)
  {
    digitalWrite(PWM1,LOW);
    digitalWrite(PWM4,LOW);
    digitalWrite(PWM2,HIGH);
    digitalWrite(PWM3,HIGH);    
  }

  else
  {
    Serial.println("Boolean ERROR");
  }
ref=3;
  
  mea=analogRead(A5);
  mea=(mea/1024)*5;
  Error=ref-mea;
  Serial.print("DUTY:");
  Serial.print(DUTY);
  Serial.print("/MEA");
  Serial.print(mea);
  Serial.print("/PID_OUT");
  Serial.print(PID_OUT);
  Serial.print("/ERROR");
  Serial.print(Error);
  Serial.println();
  delay(1000);
  
  pid.Compute();
  DUTY=255-PID_OUT;
  analogWrite(PWM0,DUTY);

  Serial.println("........................................");
  Serial.println(ref_speed);
  

  

  
 
 
  

}
