
//Includes
#include <Wire.h>
#include <Servo.h>

Servo pitch;
Servo roll;

//Gyro Variables
float elapsedTime, time, timePrev;            //Variables for time control
int gyro_error=0;                             //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;           //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;             //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y;     //Here we store the initial gyro data error

//Acc Variables
int acc_error=0;                              //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;           //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;           //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;               //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y;   //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;           //Here we store the final total angle

//More variables for the code...
int i;
int mot_activated=0;
long activate_count=0;
long des_activate_count=0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=10; //4;//3.55
double roll_ki=0.0001; //0.1;//0.003
double roll_kd=3; //0.1;//2.05
float roll_desired_angle =0;     //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=10;//3.55
double pitch_ki=0.0001;//0.003
double pitch_kd=3;//2.05
float pitch_desired_angle =0;     //This is the angle in which we want the gimbal to stay (for now it will be 0) Joystick for future versions

float PWM_pitch, PWM_roll=90;

/*//// Kalman Variables
float previous_estimate = 0;
float error_estimate = 4;
float previous_error = 4;

float kalman_gain;
float measured_value;
float error_measure = 4;
float present_estimate ;*/

void setup() {
  pitch.attach(5); //servo motor for pitch
  roll.attach(3);  //servo motor for roll

  
  Wire.begin();   
  //begin the wire comunication
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  //Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.write(0x00);                       //set bits to 0x00 for 250dps
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  //Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.write(0x00);                         //for +/-2g
  Wire.endTransmission(true); 

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor  
  time = millis();                        //Start counting time in milliseconds
}//end of setup void






void loop() {
    
  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000;     
 
 //////////////////////////////////////Gyro read/////////////////////////////////////
  Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
  Wire.write(0x43);                        //First adress of the Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers        
  Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();

   Gyr_rawX = (Gyr_rawX/131);          //divide by 131 for 250dps range
 
   Gyr_rawY = (Gyr_rawY/131);     //divide by 131 for 250dps range

  Gyro_angle_x = Gyr_rawX*elapsedTime;

  Gyro_angle_y = Gyr_rawY*elapsedTime;


    
  
  //////////////////////////////////////Acc read/////////////////////////////////////
  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
    
  Acc_rawX=(Wire.read()<<8|Wire.read())/16384.0 ; //each value needs two registres
  Acc_rawY=(Wire.read()<<8|Wire.read())/16384.0 ;
  Acc_rawZ=(Wire.read()<<8|Wire.read())/16384.0 ;

 Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) ;

 Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) ;   


 //////////////////////////////////////Total angle and filter/////////////////////////////////////
 
 Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
 
 Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;


/*   kalman_gain = error_estimate/(error_estimate + error_measure);
    present_estimate = previous_estimate + (kalman_gain * (Total_angle_x - previous_estimate));
    error_estimate = (1 - kalman_gain) * previous_error;

    previous_error = error_estimate;
    previous_estimate = present_estimate;
   // Serial.println(present_estimate);

    Total_angle_x=present_estimate;*/
    

    
    Serial.print("GyroX angle: ");
    Serial.print(Total_angle_x);
    Serial.print("   |   ");
   // Serial.print("GyroY angle: ");
    //Serial.println(Total_angle_y);
    


    /*///////////////////////////P I D///////////////////////////////////*/
    roll_desired_angle = 0;   //The angle we want the gimbal to stay is 0 and 0 for both axis for now...
    pitch_desired_angle = 0;
    

    roll_error = Total_angle_y - roll_desired_angle;
    pitch_error = Total_angle_x - pitch_desired_angle;    

    roll_pid_p = roll_kp*roll_error;
    pitch_pid_p = pitch_kp*pitch_error;

    if(-3 < roll_error <3)
    {
      roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
    }
    if(-3 < pitch_error < 3)
    {
      pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
    }

    roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
    pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);

    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d ;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d ;

    if(roll_PID < -90){roll_PID = -90;}
    if(roll_PID > 90) {roll_PID = 90; }
    if(pitch_PID < -90){pitch_PID = -90;}
    if(pitch_PID > 90) {pitch_PID = 90;}
    
    roll_previous_error = roll_error;     //Remember to store the previous error.
    pitch_previous_error = pitch_error;   //Remember to store the previous error.

              //Angle for each motor is 90 plus/minus the PID value
   /* PWM_pitch = 90 + pitch_PID;
    PWM_roll = 90 - roll_PID;*/
    PWM_pitch = 90+pitch_PID;
    PWM_roll = 90-roll_PID;

    
    Serial.println(PWM_pitch);
    //Serial.println(PWM_roll);*/
    
    pitch.write(PWM_pitch);               //Finally we write the angle to the servos
    roll.write(PWM_roll);
}//end of void loop
