#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro
#include <RXInterrupt.h>

#define PINLSTEP 5
#define PINLDIR 4
#define PINRSTEP 7
#define PINRDIR 6
#define ENABLE_MOTORS 8

// Pins for LEDs
#define LED_LowVoltage 10
#define LED_Fallen 9
#define LED_Late 11

#define ILIMIT 400

int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = -400;                            //Enter the accelerometer calibration value

//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller
float pid_i_gain = 0.8;                                      //Gain setting for the I-controller
float pid_d_gain = 25;                                       //Gain setting for the D-controller
float max_target_speed = 250;                                //Max target speed
float angle_returning_speed = 0.1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float left_motor_speed, right_motor_speed;
float left_motor_timeperiod, right_motor_timeperiod;

//set reference voltage for Lipo measurement
float refVcc = 5;

//Loop counter
int loop_counter = 0;

// RC receiver output
short RCOutput[2];

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(500000);

  Wire.begin();                                                             //Start the I2C bus as master
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.

  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  // Set up LEDs and outputs
  pinMode(PINLSTEP, OUTPUT);
  pinMode(PINLDIR, OUTPUT);
  pinMode(PINRSTEP, OUTPUT);
  pinMode(PINRDIR, OUTPUT);
  pinMode(13, OUTPUT);  //onboard LED
  pinMode(LED_Fallen, OUTPUT);
  pinMode(LED_Late, OUTPUT);
  pinMode(LED_LowVoltage, OUTPUT);
  pinMode(ENABLE_MOTORS, OUTPUT);

  // interupts only on digital pins 2 & 3
  int pins[2] = {2, 3};
  initChannels(pins, 2);

  //Test LEDs
  digitalWrite(LED_Fallen, HIGH);
  digitalWrite(LED_Late, HIGH);
  digitalWrite(LED_LowVoltage, HIGH);
  delay(1000);
  digitalWrite(LED_Fallen, LOW);
  digitalWrite(LED_Late, LOW);
  digitalWrite(LED_LowVoltage, LOW);

  //Calibration
  for (receive_counter = 0; receive_counter < 500; receive_counter++)
  {
    if (receive_counter % 15 == 0)
      digitalWrite(13, !digitalRead(13));       //Change the state of the LED every 15 loops to make the LED blink fast

    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x45);                                                       //Start reading at register 45 (was 43) GYRO{XYZ}OUT
    Wire.endTransmission();                                                 //End the transmission

    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();         //Combine the two bytes to make one integer
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();           //Combine the two bytes to make one integer

    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }

  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset

  Serial.print(" PitchCal:");
  Serial.print(gyro_pitch_calibration_value);
  Serial.print(" YawCal:");
  Serial.print(gyro_yaw_calibration_value);
  Serial.println();

  loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time

  //Enable motors
  digitalWrite(ENABLE_MOTORS, HIGH);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() //Runs at 4mS
{

  updateChannels(RCOutput, 2);

  float FB = (float)(map(RCOutput[0], 1000, 2000, -200, 200)) / 1000;
  float LR = map(RCOutput[1], 1000, 2000, -70, 70);


  //Check Lipo voltage
  int VoltageSensorValue = analogRead(A1);
  double LipoVoltage = (VoltageSensorValue * refVcc / 1023) * 3;  // x3 bec of voltage divider

  if (LipoVoltage < 9.5 && LipoVoltage > 0.1) //specify lower limit so program runs if no Lipo
  {
    digitalWrite(LED_LowVoltage, HIGH);
    low_bat = 1;
  }
  else
  {
    low_bat = 0;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Get accel info from MPU
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3B);                                                         //Start reading at register 3B (was 3F) ACCEL{XYZ}OUT
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();                  //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value

  //TODO: Should 8200 be 8192 (being the limit of the mpu)?
  if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;        //Calculate the current angle according to the accelerometer and convert from rad to deg

  if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5)                    //If the angle is almost 0 and not started
  {
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }

  //Get gyro info from MPU
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x45);                                                         //Start reading at register 45 (was 43) GYRO{XYZ}OUT
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();                     //Combine the two bytes to make one integer
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();                       //Combine the two bytes to make one integer

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value

  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board.
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  //Uncomment the following line to make the compensation active
  angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  //Complentary filter
  double driftComp = 0.99999;
  angle_gyro = (angle_gyro * driftComp) + (angle_acc * (1 - driftComp));                    //Correct the drift of the gyro angle with the accelerometer angle

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.

  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;

  if (pid_output > 10 || pid_output < -10)
    pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable

  if (abs(pid_i_mem) > ILIMIT)
    start = 0;

  //Calculate the PID output value
  pid_output = (pid_p_gain * pid_error_temp) + pid_i_mem + (pid_d_gain * (pid_error_temp - pid_last_d_error));

  if (pid_output > 400)
    pid_output = 400;                                    //Limit the PI-controller to the maximum controller output
  else if (pid_output < -400)
    pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if (abs(pid_output) < 5)
  {
    pid_output = 0;                     //Create a dead-band to stop the motors when the robot is balanced
  }

  if (abs(angle_gyro) > 30 || start == 0 || low_bat == 1)  //If the robot tips over or the start variable is zero or the battery is empty
  {
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
    digitalWrite(LED_Fallen, HIGH);
  }
  else
    digitalWrite(LED_Fallen, LOW);


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  left_motor_speed = pid_output;                                             //Copy the controller output to the left_motor_speed variable for the left motor
  right_motor_speed = pid_output;                                            //Copy the controller output to the right_motor_speed variable for the right motor

  // Left and right code
  if (LR > 5 || LR < -5)
  {
    if (FB < 0)     // If it's is going backwards
    {
      left_motor_speed += LR;
      right_motor_speed -= LR;
    }
    else            // If it's going forwards or is stationary
    {
      left_motor_speed -= LR;
      right_motor_speed += LR;
    }
  }

  // Forwards and backwards code
  if (FB > 0.01)
  {
    if (pid_setpoint > -2.5)
      pid_setpoint -= 0.1;                         //Slowly change the setpoint angle so the robot starts leaning forwards
    if (pid_output > max_target_speed * -1)
      pid_setpoint -= 0.005;           //Slowly change the setpoint angle so the robot starts leaning forwards
  }
  else if (FB < -0.01)
  {
    if (pid_setpoint < 2.5)
      pid_setpoint += 0.1;                          //Slowly change the setpoint angle so the robot starts leaning backwards
    if (pid_output < max_target_speed)
      pid_setpoint += 0.005;                //Slowly change the setpoint angle so the robot starts leaning backwards
  }
  else
  {
    if (pid_setpoint > 0.5)
      pid_setpoint -= angle_returning_speed;                            //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if (pid_setpoint < -0.5)
      pid_setpoint += angle_returning_speed;                      //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else
      pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (pid_setpoint == 0) //If the setpoint is zero degrees
  {
    if (pid_output < 0)
      self_balance_pid_setpoint += 0.0015;                 //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if (pid_output > 0)
      self_balance_pid_setpoint -= 0.0015;                 //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if (left_motor_speed > 0)
    left_motor_timeperiod = 405 - (1 / (left_motor_speed + 9)) * 5500;
  else if (left_motor_speed < 0)
    left_motor_timeperiod = -405 - (1 / (left_motor_speed - 9)) * 5500;
  else
    left_motor_timeperiod = 0;

  if (right_motor_speed > 0)
    right_motor_timeperiod = 405 - (1 / (right_motor_speed + 9)) * 5500;
  else if (right_motor_speed < 0)
    right_motor_timeperiod = -405 - (1 / (right_motor_speed - 9)) * 5500;
  else
    right_motor_timeperiod = 0;

  int iConverge = 400; //was 400

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if (left_motor_timeperiod > 0)
    left_motor_speed = iConverge - left_motor_timeperiod;
  else if (left_motor_timeperiod < 0)
    left_motor_speed = -iConverge - left_motor_timeperiod;
  else
    left_motor_speed = 0;

  if (right_motor_timeperiod > 0)
    right_motor_speed = iConverge - right_motor_timeperiod;
  else if (right_motor_timeperiod < 0)
    right_motor_speed = -iConverge - right_motor_timeperiod;
  else
    right_motor_speed = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor_speed;
  throttle_right_motor = right_motor_speed;

  if (loop_counter == -1)
  {
    Serial.print("  angle_gyro:");
    Serial.print(angle_gyro);
    Serial.print("\tSetpoint_S_B:");
    Serial.print(self_balance_pid_setpoint);
    Serial.print("\tSetpoint_PID:");
    Serial.print(pid_setpoint);
    Serial.print("\tI Memory:");
    Serial.print(pid_i_mem);
    Serial.print("\tPID o/p:");
    Serial.print(pid_output);
    Serial.print("\tRC receiver output:");
    Serial.print(LR);
    Serial.print(" ");
    Serial.print(FB);

    Serial.println();

    loop_counter = 0;
  }

  loop_counter++;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.

  if (loop_timer < micros())
  {
    digitalWrite(LED_Late, HIGH);
  }
  else
  {
    digitalWrite(LED_Late, LOW);
  }

  while (loop_timer > micros()); //wait until loop execution has taken 4mS
  loop_timer += 4000;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect)  //this function runs ever 20us, ie 50kHz
{
  //Left motor pulse calculations
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttle_counter_left_motor > throttle_left_motor_memory)
  {
    //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if (throttle_left_motor_memory < 0) //If the throttle_left_motor_memory is negative
    {
      PORTD &= 0b11101111;                                                  //Set output 4 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else
    {
      PORTD |= 0b00010000;                                               //Set output 4 high for a forward direction of the stepper motor
    }
  }
  else if (throttle_counter_left_motor == 1)
  {
    PORTD |= 0b00100000;             //Set output 5 high to create a pulse for the stepper controller
  }
  else if (throttle_counter_left_motor == 2)
  {
    PORTD &= 0b11011111;             //Set output 5 low because the pulse only has to last for 20us
  }

  //Right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttle_counter_right_motor > throttle_right_motor_memory) //If the number of loops is larger then the throttle_right_motor_memory variable
  {
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if (throttle_right_motor_memory < 0)  //If the throttle_right_motor_memory is negative
    {
      PORTD &= 0b10111111;                                               //Set output 6 low for a forward direction of the stepper motor
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else
    {
      PORTD |= 0b01000000;                                                  //Set output 6 high to reverse the direction of the stepper controller
    }
  }
  else if (throttle_counter_right_motor == 1)
  {
    PORTD |= 0b10000000;            //Set output 7 high to create a pulse for the stepper controller
  }
  else if (throttle_counter_right_motor == 2)
  {
    PORTD &= 0b01111111;            //Set output 7 low because the pulse only has to last for 20us
  }
}
