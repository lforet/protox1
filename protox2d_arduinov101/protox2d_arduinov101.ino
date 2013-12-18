
#define Rdirpin  8    // Right motor direction control pin
#define Rpwmpin 10    // Right motor pulse width modulation pin
//#define Ldirpin  7    // Left  motor direction control pin
//#define Lpwmpin  9    // Left  motor pulse width modulation pin

#define Battery  7    // Battery voltage monitor pin (analog input)

#define SERVO_PIN 12  
//#include <Servo.h>

// Motor control
const int DesiredRPM=300;  // Setting Desired RPM Here.
const int MotorPWMPin=10;
unsigned char SpeedRPHhighbyte;
unsigned char SpeedRPHLowbyte;
int SpeedRPH = 0;
const unsigned char PWM4dutyMax=255;
const unsigned char PWM4dutyMin=200;
unsigned char PWM4duty=255; // have to set a default value make motor start spining

int lenMicroSecondsOfPeriod = 50 * 1000; // 25 milliseconds (ms)
int lenMicroSecondsOfPulse = 1 * 1000; // 1 ms is 0 degrees
int increment = 50;
int servo_pos = 1500;
int clockwise = 0;
int revolution_count = 0;
int servo_max = 1900;
int servo_min = 800;
//Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
int pos = 0;    // variable to store the servo position 

unsigned int distance[4];
unsigned int quality[4];
unsigned int checksum;

byte distanceLow=0;
byte distanceHigh=0;
byte flag1=0;
byte flag2=0;
byte qualityLowByte = 0;
byte qualityHighByte = 0;

byte Data_status=0;
byte Data_4deg_index=0;
byte Data_loop_index=0;

byte Data_Package[22];
unsigned int Data_Word_Package[10];
byte checksum_highByte;
byte checksum_lowByte;

int inByte = 0;         // incoming serial byte


void setup()
{

  Serial.begin(115200); // initialize serial interface   
  pinMode(SERVO_PIN, OUTPUT); //initialize the servo pin
  pinMode(Rdirpin,OUTPUT);  // set left  direction pin as an output pin
  digitalWrite(Rdirpin, HIGH);
  analogWrite(MotorPWMPin, PWM4duty ); 
  //digitalWrite(MotorPWMPin, HIGH);
  //Serial.println("STARTING.......");
  SpeedControl ( DesiredRPM );
  //move_servo();
  //delay(1000);
  Data_loop_index=1;
  //myservo.attach(12);
  digitalWrite(13, HIGH);
}

void loop()
{
  //Servo_Sweep();
  //delay(250);
  if (Serial.available() > 0) {
    inByte = Serial.read();
    //Serial.println(inByte);
    Data_Package[Data_loop_index] = inByte; 
    decodeData(inByte);
    //digitalWrite(13, HIGH);
    //LED_Blink();
    
  }
}

void decodeData(unsigned char inByte){
  
  switch (Data_status){
    
    case 0: // no header
      if (inByte==0xFA){
        Data_status=1;
        Data_loop_index=1;
        Data_Package[0] = inByte;
      }
      break;
  
    case 1: // Find 2nd FA
      if (Data_loop_index==22){
        if (inByte==0xFA){
          Data_status=2;
          Data_loop_index=1;
        }else // if not FA search again
         Data_status=0;
      }else{
        Data_loop_index++;
      }
      break;
    
    case 2: // Read data out   
      if (Data_loop_index==22){
        if (inByte==0xFA){
          Data_loop_index=1;
        }
        else // if not FA search again
          Data_status=0;
      }else{
        readData(inByte);
        Data_loop_index++;
      }
      break;
  }
  
}

void readData(unsigned char inByte){
  int angleIndex = 0;
  int dataIndex;
  int i = 0;
  flag1 = 0;
  flag2 = 0;
  
  switch (Data_loop_index){

    case 1: // 4 degree index
      Data_4deg_index = inByte-0xA0;
      break;
      
    case 2: // Speed in RPH low byte
      SpeedRPHLowbyte=inByte;
      break;
      
    case 3: // Speed in RPH high byte
      SpeedRPHhighbyte=inByte;
      SpeedRPH=(SpeedRPHhighbyte<<8)|SpeedRPHLowbyte;
      SpeedControl ( DesiredRPM ) ; //
      break;
      
    case 4:    
      distanceLow=inByte;
      break; 
      
    case 5:
      distanceHigh=inByte & B00111111; //mask 14 and 15 bits 
      distance[0] = 0;  
      //flag1=inByte & B10000000;
      //flag2=inByte & B01000000;
      //if (flag1 == 0 && flag2 == 0 ){
        distance[0] = (distanceHigh<<8)|distanceLow; 
      //}
      break;
      
    case 6:   
       qualityLowByte = inByte;
       break;
       
    case 7:   
      qualityHighByte = inByte;
      quality[0] = 0;
      quality[0] =  (qualityHighByte<< 8) | qualityLowByte; // quality is on 16 bits  
      break;

    case 8:   
      distanceLow=inByte;
      break; 
      
    case 9:
      distanceHigh=inByte & B00111111; //mask 14 and 15 bits 
      distance[1]= 0;  
      //flag1=inByte & B10000000;
      //flag2=inByte & B01000000;
      //if (flag1 == 0 && flag2 == 0 ){
        distance[1] = (distanceHigh<<8)|distanceLow; 
      //}
      break;
      
    case 10:
       qualityLowByte = inByte;
       break;
       
    case 11:    
      qualityHighByte = inByte;
      quality[1] = 0;
      quality[1]  =  (qualityHighByte<< 8) | qualityLowByte; // quality is on 16 bits      
      break;
      
    case 12:   
      distanceLow=inByte;
      break; 
      
    case 13:
      distanceHigh=inByte & B00111111; //mask 14 and 15 bits 
      distance[2]= 0;  
      //flag1=inByte & B10000000;
      //flag2=inByte & B01000000;
      //if (flag1 == 0 && flag2 == 0){
        distance[2]= (distanceHigh<<8)|distanceLow; 
      //}
      break;
      
    case 14:   
      qualityLowByte = inByte;
      break;
       
    case 15:   
      qualityHighByte = inByte;
      quality[2] = 0;
      quality[2] =  (qualityHighByte<< 8) | qualityLowByte; // quality is on 16 bits        
      break;
      
    case 16:   
      distanceLow=inByte;
      break; 
      
    case 17:
      distanceHigh = inByte & B00111111; //mask 14 and 15 bits 
      distance[3]= 0;  
      //flag1=inByte & B10000000;
      //flag2=inByte & B01000000;
      //if (flag1 == 0 && flag2 == 0){
        distance[3] = (distanceHigh<<8)|distanceLow; 
      //}
      break;
      
    case 18:  
       qualityLowByte = inByte;
       break;
       
    case 19:    
      qualityHighByte = inByte;
      quality[3] = 0;
      quality[3] =  (qualityHighByte<< 8) | qualityLowByte; // quality is on 16 bits         
      break;
     
    case 20:   
       checksum_lowByte = inByte;
       break;
       
    case 21:       
        checksum_highByte = inByte;
        checksum = int(checksum_lowByte) | int((checksum_highByte << 8));
           
        if ( checksum == calculate_checksum()){
        
            //Serial.print("Checksums Match!");
            //Serial.println(""); 
            
            for(i = 0; i  < 4; i+=1){
                angleIndex = Data_4deg_index*4 + i;
                //if (distance[i] > 0){
                  //Serial.print("  angle:");
                  Serial.print("A1");
                  Serial.print(",");
                  Serial.print("0");
                  Serial.print(",");
                  Serial.print(angleIndex);
                  Serial.print(",");
                  //Serial.print("  Distance:");    
                  Serial.print(distance[i], DEC);
                  Serial.print(",");
                  Serial.print(quality[i], DEC);
                  Serial.print(",");
                  Serial.print(SpeedRPH/60);
                  Serial.println("");
                  //Serial.print(",");
                  //Serial.print (" ");
                  //Serial.print("mm ");
                  //Serial.print("  Quality: ");
                  //Serial.println(quality[i], DEC);
                  //Serial.print("freeMemory()=");
                  //Serial.println(freeRam());
                //}
             //LED_Blink();
            }
            /*
            if (revolution_count < 90){
                revolution_count  = revolution_count +1;
                //myservo.write(revolution_count);    // tell servo to go to position in variable 'angle'
                //delay(20);
            }
            if (revolution_count  >= 90){
                revolution_count  = 0;
                update_servo_pos();
                move_servo();  
            } 
            */            
        } 
         
       break;   
     
     
    default: // others do checksum
        break;
  }

}

int calculate_checksum(){
        unsigned long chk32 = 0;
        int i = 0;
        unsigned int calculated_checksum;
        
        //for(i = 0; i  < 22; i+=1){
        //   Serial.print(Data_Package[i], HEX);
        //   Serial.print(" "); 
        //}     
        //Serial.println(" "); 
        for(i = 0; i  < 10; i+=1){
           Data_Word_Package[i] = Data_Package[2*i] + (Data_Package[2*i+1]<<8);
        }
        //for(i = 0; i  < 10; i+=1){
        //   Serial.print(Data_Word_Package[i], HEX);
        //   Serial.print(" "); 
        //}
        //Serial.println(" "); 
       
        chk32 = 0;
        for(i = 0; i  < 10; i+=1){
           chk32 = (chk32 << 1) + Data_Word_Package[i];
        }  
        
       calculated_checksum = (chk32 & 0x7FFF) + (chk32  >> 15);    
       calculated_checksum = calculated_checksum & 0x7FFF;       
 
       //Serial.print("  checksum:");
       //Serial.print(checksum, DEC);
       //Serial.print("  Calculated_checksum:");
       //Serial.println(calculated_checksum, DEC);
       return calculated_checksum;
}  

// Very simple speed control
void SpeedControl ( int RPMinput)
{
 //if (Data_4deg_index%30==0) { // I only do 3 updat I feel it is good enough for now
 //if (Data_4deg_index%40==0) { // I only do 3 updat I feel it is good enough for now
    if ((SpeedRPH/60) < RPMinput){
       if (PWM4duty < PWM4dutyMax){ 
         PWM4duty++; // limit the max PWM make sure it don't overflow and make LDS stop working
        // Serial.print("increasing speed");
         //Serial.println(PWM4duty);
       }
    }
    if ((SpeedRPH/60) > RPMinput){
       if(PWM4duty>PWM4dutyMin){
         PWM4duty--; //Have to limit the lowest pwm keep motor running
        // Serial.print("decreasing speed");
        // Serial.println(PWM4duty);
       }
    }
  //}
  //Serial.print("PWM4duty:");
  //Serial.println(PWM4duty);
  analogWrite(MotorPWMPin, PWM4duty ); // update value
  //digitalWrite(MotorPWMPin, HIGH ); // update value
}

void move_servo(){
  int i;
  //Serial.println(" ");
  //Serial.print("servo:");
  //Serial.print(servo_pos);
  //delay(20);
  for (i=0; i<5; i++){
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(servo_pos);
    digitalWrite(SERVO_PIN, LOW);
    //delayMicroseconds(20);
    //delayMicroseconds(lenMicroSecondsOfPeriod - servo_pos);
    delay(10);
  }
}  


void update_servo_pos(){
  
  if (clockwise == 1 && servo_pos >= servo_max){
      clockwise = 0;
  }
  if (clockwise == 1 && servo_pos <= servo_max){
      servo_pos = servo_pos + increment;
  }
  
  if (clockwise == 0 && servo_pos <= servo_min){
      clockwise = 1;
  }
  if (clockwise == 0 && servo_pos >= servo_min){
      servo_pos = servo_pos - increment;
  }
   

}  


void Servo_Sweep()
{
    int current = 0;
    for(current = servo_min; current < servo_max; current+=increment){
         // Servos work by sending a 25 ms pulse.  
         // 0.7 ms at the start of the pulse will turn the servo to the 0 degree position
         // 2.2 ms at the start of the pulse will turn the servo to the 90 degree position 
         // 3.7 ms at the start of the pulse will turn the servo to the 180 degree position 
         // Turn voltage high to start the period and pulse
         digitalWrite(SERVO_PIN, HIGH);

         // Delay for the length of the pulse
         delayMicroseconds(current);

         // Turn the voltage low for the remainder of the pulse
         digitalWrite(SERVO_PIN, LOW);

         // Delay this loop for the remainder of the period so we don't
         // send the next signal too soon or too late
         delayMicroseconds(lenMicroSecondsOfPeriod - current); 
         //Serial.print("current: ");
         //Serial.println(current);
         delay(20);
    }
    for(current = servo_max; current > servo_min; current-=increment){
         // Servos work by sending a 20 ms pulse.
         // 0.7 ms at the start of the pulse will turn the servo to the 0 degree position
         // 2.2 ms at the start of the pulse will turn the servo to the 90 degree position
         // 3.7 ms at the start of the pulse will turn the servo to the 180 degree position
         // Turn voltage high to start the period and pulse
         digitalWrite(SERVO_PIN, HIGH);

         // Delay for the length of the pulse
         delayMicroseconds(current);

         // Turn the voltage low for the remainder of the pulse
         digitalWrite(SERVO_PIN, LOW);

         // Delay this loop for the remainder of the period so we don't
         // send the next signal too soon or too late
         delayMicroseconds(lenMicroSecondsOfPeriod - current);
         //Serial.print("current: ");
         //Serial.println(current);
         delay(20);
    }
}


/*

void drawMap(){
  
  for (int i=0;i<360;i++){
    
    int distMax=6000;
    int d=distance_array[i];
    distance_array[i]=0;
    if (d==0 || d>distMax)continue;

    int r2=map(d, 0, distMax, 0, 115);
    
    //Serial.print(i);
    //Serial.print(" D:");
    //Serial.println(r2);
 
    //to radians
    double rad_angle= i * 3.1415 / 180;
 
   int distX=r2*cos(rad_angle);
   int distY=r2*sin(rad_angle);
   
   x2 = cX + distX;
   y2 = cY + distY;
 
  }
  
  delay(1);
  
}
*/



//void sensorMap(int degreeIndex, int degree,int distance){
        
   //int r2=map(distance, 0, 7000, 0, 115);
   //int r2=map(distance, 0, 10000, rMin, r);
   
   //degree=constrain(degree,0,89);
   //int angle=degree*degreeIndex;
   //int angle=degree*4;
   
    //to radians
    //double rad_angle= angle * 3.1415 / 180;
    
   //Serial.println(distance);
// Serial.print(" ");
// Serial.println(rad_angle);

   
   //int distX=r2*cos(rad_angle);
   //int distY=r2*sin(rad_angle);
   
   //x2 = cX + distX;
   //y2 = cY + distY;
   
   //myGLCD.fillCircle(x2,y2,2);
   //myGLCD.drawPixel(x2,y2);
 
 /*
x2 = cX + r*cos(rad_angle);
y2 = cY + r*sin(rad_angle);
*/
   //myGLCD.drawLine(cX,cY,x2,y2);
   
/*
Serial.print(distX);
Serial.print(",");
Serial.println(distY);
*/
  
//}


void Motor_Forward(int spd)
{
  digitalWrite(Rdirpin, HIGH);
  analogWrite(Rpwmpin,spd); 
  //Serial.print("motor spd:");
  //Serial.println(spd);
}

void LED_Blink()
{
  for(pos = 0; pos < 4; pos += 1) 
  {                                  
    digitalWrite(13, HIGH);
    delay(20);
    digitalWrite(13, LOW);
    delay(20);
  } 
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
