
int servo_max = 930;
int servo_min = 150;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int sensorValue = 0;        // value read from the pot

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 9;
int in1 = 5;
int in2 = 6;
// motor two
int enB = 9;
int in3 = 8;
int in4 = 7;

boolean setup_completed = false;
boolean servo_test_completed = false;
boolean device_on = false;

int forward_pin1 = 0;
int backward_pin1 = 0;
void setup()
{
 // set all the motor control pins to outputs
 pinMode(enA, OUTPUT);
 pinMode(enB, OUTPUT);
 pinMode(in1, OUTPUT);
 pinMode(in2, OUTPUT);
 pinMode(in3, OUTPUT);
 pinMode(in4, OUTPUT);
 Serial.begin(9600);
}
  
void loop()
{
  if (!setup_completed && device_on){
    delay (1000);
    servoDirectionTest();
  }

  if (!servo_test_completed && device_on){
    delay (1000);
    servoTest();
  }

  sensorValue = analogRead(analogInPin);
  if (sensorValue > 20){
    device_on = true;
  }


}

void servoTest(){
    Serial.println("Testing servo.");
    boolean fwd = false;
    boolean rwd = false;
    
    sensorValue = analogRead(analogInPin);
    Serial.print("sensor value = ");
    Serial.println(sensorValue);

    Serial.println("Moving forward");
    mediumForward();
    
    while(sensorValue < servo_max){
      sensorValue = analogRead(analogInPin);
      Serial.print("sensor value = ");
      Serial.println(sensorValue);
    }

    Serial.println("Servo stopped");
    stopServo();

      sensorValue = analogRead(analogInPin);
      Serial.print("sensor value = ");
      Serial.println(sensorValue);

    Serial.println("Moving backward");
    mediumBackward();
    
    while(sensorValue > servo_min){
      sensorValue = analogRead(analogInPin);
      Serial.print("sensor value = ");
      Serial.println(sensorValue);
    }

    stopServo();

    sensorValue = analogRead(analogInPin);
    Serial.print("sensor value = ");
    Serial.println(sensorValue);

    mediumForward();
    delay(100);
    stopServo();

    Serial.println("Testing servo done.");

    servo_test_completed = true;
    
}

void mediumForward(){
  digitalWrite(forward_pin1, HIGH);
  digitalWrite(backward_pin1, LOW);
  analogWrite(enB, 25);
}

void mediumBackward(){
  digitalWrite(forward_pin1, LOW);
  digitalWrite(backward_pin1, HIGH);
  analogWrite(enB, 25);
}

void stopServo(){
  digitalWrite(in4, LOW);
  digitalWrite(in3, LOW);
  analogWrite(enB, 0);
}

void servoDirectionTest(){
    Serial.println("Direction has not been set up. Starting the calibration.");
    
    int sensor_start = analogRead(analogInPin);
    Serial.print("starting sensor value = ");
    Serial.println(sensor_start);

    // this function will run the motors in both directions at a fixed speed
    // turn on motor A
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    
    analogWrite(enB, 50);
    delay(100);
    analogWrite(enB, 0);

    
    int sensor_end = analogRead(analogInPin);
    Serial.print("ending sensor value = ");
    Serial.println(sensor_end);

    if(sensor_start > sensor_end){
      forward_pin1 = in3;
      backward_pin1 = in4;
    } else {
      forward_pin1 = in4;
      backward_pin1 = in3;
    }

    Serial.println("The directions of the motor has been succesfully calibrated.");

    Serial.println("Testing servos.");

    setup_completed = true;
 }


byte PWMOutput;
long Error[10];
long Accumulator;
long PID;
int PTerm;
int ITerm;
int DTerm;
byte Divider;



/* GetError():
Read the analog values, shift the Error array down 
one spot, and load the new error value into the
top of array.
*/
void GetError(void)
{
  byte i = 0;
  // read analogs
  word ActualPosition = analogRead(ActPos);  
// comment out to speed up PID loop
//  Serial.print("ActPos= ");
//  Serial.println(ActualPosition,DEC);

  word DesiredPosition = analogRead(DesPos);
// comment out to speed up PID loop
//  Serial.print("DesPos= ");
//  Serial.println(DesiredPosition,DEC);

  // shift error values
  for(i=9;i>0;i--)
    Error[i] = Error[i-1];
  // load new error into top array spot  
  Error[0] = (long)DesiredPosition-(long)ActualPosition;
// comment out to speed up PID loop
//  Serial.print("Error= ");
//  Serial.println(Error[0],DEC);

}

/* CalculatePID():
Error[0] is used for latest error, Error[9] with the DTERM
*/
void CalculatePID(void)
{
// Set constants here
  PTerm = 2000;
  ITerm = 25;
  DTerm = 0;
  Divider = 10;

// Calculate the PID  
  PID = Error[0]*PTerm;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  PID += ITerm*Accumulator; // add integral gain and error accumulation
  PID += DTerm*(Error[0]-Error[9]); // differential gain comes next
  PID = PID>>Divider; // scale PID down with divider

// comment out to speed up PID loop  
//Serial.print("PID= ");
//  Serial.println(PID,DEC);

// limit the PID to the resolution we have for the PWM variable

  if(PID>=127)
    PID = 127;
  if(PID<=-126)
    PID = -126;

//PWM output should be between 1 and 254 so we add to the PID    
  PWMOutput = PID + 127;

// comment out to speed up PID loop
//  Serial.print("PWMOutput= ");
//  Serial.println(PWMOutput,DEC);

}

