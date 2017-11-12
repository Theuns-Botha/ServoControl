
//int servo_max = 920;
//int servo_min = 160;
int servo_max = 880;
int servo_min = 200;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int forward_pin1 = 0;
int backward_pin1 = 0;
// motor two
int enB = 9;
int in3 = 8;
int in4 = 7;
int DesiredPosition = 0;
int ActualPosition = 500;
int sensorValue = 0;        // value read from the pot

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 9;
int in1 = 5;
int in2 = 6;

int PWMOutput;
int Error[10];
long Accumulator;
long PID;
long PTerm;
int ITerm;
int DTerm;
int Divider;

uint16_t currentMicros;
uint16_t milliTime;
uint16_t milliStart;
byte readValue;

byte incomingByte;

boolean system_cycle = false;

uint16_t microTime[100];
byte motorPosition[100];

void setup()
{
/*
  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts*/
  
 // set all the motor control pins to outputs
 pinMode(enA, OUTPUT);
 pinMode(enB, OUTPUT);
 pinMode(in1, OUTPUT);
 pinMode(in2, OUTPUT);
 pinMode(in3, OUTPUT);
 pinMode(in4, OUTPUT);
 Serial.begin(9600);

  servoDirectionTest();
  servoTest();
  
}
  
void loop()
{

  if (Serial.available() > 0) {   // something came across serial
    DesiredPosition = 0;         // throw away previous integerValue
    while(1) {            // force into a loop until 'n' is received
      incomingByte = Serial.read();
      if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
      if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
      DesiredPosition *= 10;  // shift left 1 decimal place
      // convert ASCII to integer, add, and shift left 1 decimal place
      DesiredPosition = ((incomingByte - 48) + DesiredPosition);
    }
    //Serial.println(DesiredPosition);   // Do something with the value
  }

  

  if(DesiredPosition != 0){

    GetError();
    CalculatePID();
    
    //Serial.print("PWMOutput value = ");
    //Serial.println(PWMOutput);

    if(PWMOutput < 0){
      //mediumBackward();
      moveBackward(-1 *( (int) PWMOutput));

    } else {
           moveForward((int) PWMOutput);

      //mediumForward();
    }
  } else {
    stopServo();
  }

  sensorValue = analogRead(analogInPin);
  Serial.print("sensor value = ");
  Serial.println(sensorValue);  


 delay(1000);
}

void moveForward(int value){
  digitalWrite(forward_pin1, HIGH);
  digitalWrite(backward_pin1, LOW);
  analogWrite(enB, value);
}

void moveBackward(int value){
  digitalWrite(forward_pin1, LOW);
  digitalWrite(backward_pin1, HIGH);
  analogWrite(enB, value);
}

void servoTest(){
    Serial.println("Testing servo.");
    boolean fwd = false;
    boolean rwd = false;

    /*
    sensorValue = analogRead(analogInPin);
    //Serial.print("sensor value = ");
    //Serial.println(sensorValue);

    //Serial.println("Moving forward");
    mediumForward();
    
    while(sensorValue < servo_max){
      sensorValue = analogRead(analogInPin);
      //Serial.print("sensor value = ");
      //Serial.println(sensorValue);
    }

    //Serial.println("Servo stopped");
    stopServo();

      sensorValue = analogRead(analogInPin);
      //Serial.print("sensor value = ");
      //Serial.println(sensorValue);

    //Serial.println("Moving backward");
    mediumBackward();
    
    while(sensorValue > servo_min){
      sensorValue = analogRead(analogInPin);
      //Serial.print("sensor value = ");
      //Serial.println(sensorValue);
    }

    stopServo();
    delay(2000);*/

    sensorValue = analogRead(analogInPin);
    //Serial.print("sensor value = ");
    //Serial.println(sensorValue);


    boolean continuePulse = true;

    //mediumForward();
    //moveForward(254);
    Serial.println("Testing params");
    milliStart = micros();
    //delayMicroseconds(10);
    
    int index = 0;
    while (continuePulse == true){

      currentMicros = micros();
      readValue = analogRead(analogInPin);
      
      milliTime = currentMicros - milliStart;
      
      motorPosition[index] = index;
      microTime[index] = index;
           
      if(milliTime > 20000){
        continuePulse = false;
        stopServo();
      }
      index++;
    }
    

    //delay(20);
    Serial.println("Params done");

    int i;
    for(i=0;i<50;i++){ 
      Serial.print(microTime[index]);
      Serial.print(",");   
      Serial.println(motorPosition[index]);
    }  

    Serial.println("Testing servo done.");
    
}

void mediumForward(){
  digitalWrite(forward_pin1, HIGH);
  digitalWrite(backward_pin1, LOW);
  analogWrite(enB, 40);
}

void mediumBackward(){
  digitalWrite(forward_pin1, LOW);
  digitalWrite(backward_pin1, HIGH);
  analogWrite(enB, 40);
}

void stopServo(){
  digitalWrite(in4, LOW);
  digitalWrite(in3, LOW);
  analogWrite(enB, 0);
}

void servoDirectionTest(){
    //erial.println("Direction has not been set up. Starting the calibration.");
    
    int sensor_start = analogRead(analogInPin);
    //Serial.print("starting sensor value = ");
    //Serial.println(sensor_start);

    // this function will run the motors in both directions at a fixed speed
    // turn on motor A
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    
    analogWrite(enB, 50);
    delay(100);
    analogWrite(enB, 0);

    
    int sensor_end = analogRead(analogInPin);
    //Serial.print("ending sensor value = ");
    //Serial.println(sensor_end);

    if(sensor_start > sensor_end){
      forward_pin1 = in3;
      backward_pin1 = in4;
    } else {
      forward_pin1 = in4;
      backward_pin1 = in3;
    }

    //Serial.println("The directions of the motor has been succesfully calibrated.");

    //Serial.println("Testing servos.");

 }

/* GetError():
Read the analog values, shift the Error array down 
one spot, and load the new error value into the
top of array.
*/
void GetError(void)
{
  byte i = 0;
  ActualPosition = analogRead(analogInPin);

  //shift error values
  for(i=9;i>0;i--)
    Error[i] = Error[i-1];
  // load new error into top array spot  

  //Serial.print("desired position = ");
  //Serial.println(DesiredPosition);

  //Serial.print("actual position = ");
  //Serial.println(ActualPosition);

  Error[0] =  (long)DesiredPosition - (long)ActualPosition;

  //Serial.print("error = ");
  //Serial.println(Error[0]);
}

/* CalculatePID():
Error[0] is used for latest error, Error[9] with the DTERM
*/
void CalculatePID(void)
{
// Set constants here
  PTerm = 200;
  ITerm = 0;
  DTerm = 0;
  Divider = 1000;

// Calculate the PID  
  PID = Error[0]*PTerm;     // start with proportional gain
  // comment out to speed up PID loop  
  //Serial.print("PID= ");
  //Serial.println(PID);
  
  //Accumulator += Error[0];  // accumulator is sum of errors
  //PID += ITerm*Accumulator; // add integral gain and error accumulation
  PID += DTerm*(Error[0]-Error[9]); // differential gain comes next
  PID = PID/Divider; // scale PID down with divider

  // comment out to speed up PID loop  
  //Serial.print("PID= ");
  //Serial.println(PID);

// limit the PID to the resolution we have for the PWM variable

  if(PID>=254)
    PID = 254;
  if(PID<=-254)
    PID = -254;

//PWM output should be between 1 and 254 so we add to the PID    
  PWMOutput = PID;

// comment out to speed up PID loop
//  Serial.print("PWMOutput= ");
//  Serial.println(PWMOutput,DEC);

}


//ISR(TIMER1_COMPA_vect)//timer1 interrupt 1Hz
//{
  //system_cycle = true;
//}
