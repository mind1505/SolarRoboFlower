/// 2009 by Daniel Schatzmayr (dirty-coding)
/// This an example code to show how few code is needed to operate the flower!
/// It contains no calibration and offsets for servo and sensors (probably you want to hard code them)!
/// The Pulse with should also be set correctly (I cheat them now and drive the servo up to 190Degree of 180)

#include <SoftwareServo.h>

int ServoPin1 = 10;
int ServoPin2 = 9;
int ServoPin3 = 8;

                      
long prv2 = 0;   

SoftwareServo  servo1;
SoftwareServo  servo2;
SoftwareServo  servo3;
//SoftwareServo  servo4;
 
void setup()                    
{   
  // Attach Servos
  servo1.attach(ServoPin1);
  servo2.attach(ServoPin2);
  servo3.attach(ServoPin3);
 // servo4.attach(ServoPin4);  
 // servo1.setMaximumPulse(2200);
}
int ServoPos1 = 90;
int ServoPos2 = 90;
int ServoPos3 = 90;

void loop()                     
{     
    if (millis() - prv2 > 40) 
    {
       prv2 = millis();   
	   
       int val1 = analogRead(0);    // read the input pin
      
       int val2 = analogRead(1);    // read the input pin
      
       int val3 = analogRead(2);    // read the input pin
       
       int val4 = analogRead(3);    // read the input pin
      
       int val5 = analogRead(4);    // read the input pin
     
       
       if(val1>val2) ServoPos2+= (val1-val2);
       if(val3>val2) ServoPos2-= (val3-val2);
       if(val1>val2) ServoPos3+= (val1-val2);
       if(val3>val2) ServoPos3-= (val3-val2);
       
	   // Is the whole thing straight? (switch bottom servo dirrection)
       if(val4>val2){ ServoPos1-= ServoPos2>98?-(val1-val2):(val1-val2);}
       if(val5>val2) ServoPos1+= ServoPos2>98?-(val3-val2):(val3-val2);
              
       if(ServoPos1<-3){ServoPos1=188;ServoPos2 = 190 - ServoPos2;ServoPos3 = 190 - ServoPos3;}
       if(ServoPos1>193){ServoPos1=2;ServoPos2 = 190 - ServoPos2;ServoPos3 = 190 - ServoPos3;}
       
       if(ServoPos1<0)ServoPos1=0;
       if(ServoPos1>190)ServoPos1=190;
       
       if(ServoPos2<0)ServoPos2=0;
       if(ServoPos2>190)ServoPos2=190;
       
       if(ServoPos3<0)ServoPos3=0;
       if(ServoPos3>190)ServoPos3=190;
       
       if(ServoPos3<25&&ServoPos2<35)ServoPos2=35;
       
       servo1.write(ServoPos1);
       servo2.write(ServoPos2);                    
       servo3.write(ServoPos3);      
    }
    
  // Refresh Server timing
  SoftwareServo::refresh();
}
