//(x-pitch,y-roll,z-yaw)

#include <math.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

const uint16_t MPU_addr=0x68;

               

//Gyro Variables
float elapsedTime, timee, timePrev;                           //Variables for time control
float gyro_pitch_x,gyro_roll_y,gyro_yaw_z=0;                                             //We use this variable to only calculate once the gyro data error
int16_t Gyr_rawX, Gyr_rawY, Gyr_rawZ=0;                           //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y,Gyro_angle_z=0;                //Here we store the angle value obtained with Gyro data(x-pitch,y-roll,z-yaw)
float Gyro_raw_error_x, Gyro_raw_error_y,Gyro_raw_error_z=0;    //Here we store the initial gyro data error



//Acc Variables
int acc_error=0;                                              //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;                           //This value is for pasing from radians to degrees values
int16_t Acc_rawX, Acc_rawY, Acc_rawZ=0;                           //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y,Acc_angle_z=0;                   //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y,Acc_angle_error_z=0; //Here we store the initial Acc data error
float acc_pitch_x ,acc_roll_y ,acc_yaw_z=0;
float Acc_angle_error_pitch_x, Acc_angle_error_roll_y,Acc_angle_error_yaw_z=0;


//final angles
float Total_angle_x, Total_angle_y,Total_angle_z=0;


//altitude variables
float reference_pressure,prev_pressure,altitude,c=0;
Adafruit_BMP085 bmp;

float a[]={0,0,0,0,0,0};
float filter_coeffs[] = {0.6429 ,   0.4027   , 0.1915   , 0.0318  , -0.0658,   -0.1035};
int b =(sizeof(a)/sizeof(a[0]));

void setup() 
{   
  Serial.begin(115200);                     //Remember to set this same baud rate to the serial monitor  
  timee = millis();                        //Start counting time in milliseconds
  mpuerror();
  
  
  
}//end of setup void


void loop() {
  
   gyroangles(); 
   accangles();
  
   
//Total angle and filter

    timePrev = timee;                               // the previous time is stored before the actual time read
    timee = millis();                               // actual time read
    elapsedTime = (timee - timePrev)/1000;          //divide by 1000 in order to obtain seconds

    /*---X----pitch*/
    Total_angle_x =  (Total_angle_x + gyro_pitch_x*elapsedTime)*.9 + Acc_angle_x * .1 ;
    /*---Y---roll*/
    Total_angle_y =  (Total_angle_y + gyro_roll_y*elapsedTime)*.9 + Acc_angle_y * .1;
    /*---Z---yaw*/
    Total_angle_z =  (Total_angle_z + gyro_yaw_z*elapsedTime)  ;

//filter
   
    for(int i=0;i<b-1;i++)
    {
      a[b-1-i] = a[b-i-2];
      
    }

    a[0] = bmp.readAltitude(reference_pressure);

    for(int j=0;j<b;j++)
    {
      altitude += filter_coeffs[j]*a[j];
    }
    
    
Serial.print(a[0]*1000);
 Serial.print("        ");
 Serial.println(altitude*1000);
 delay(10);
 
 
 
}



















void mpuerror()
{
  Wire.begin();                           //begin the wire comunication  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00000000 (+/- 2g full scale range)
  Wire.endTransmission(true); 

  


/*Here we calculate the acc data error before we start the loop
 * I make the mean of 200 values, that should be enough*/

  
  
    for(int a=0; a<2000; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                                   //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_rawX=(Wire.read()<<8|Wire.read()) ;      //each value needs two registres
      Acc_rawY=(Wire.read()<<8|Wire.read()) ;
      Acc_rawZ=(Wire.read()<<8|Wire.read()) ;

      Wire.endTransmission(true);
      /*---X---*/
        //Acc_angle_error_x = Acc_angle_error_x + (float)Acc_rawX/4096; //because of unnecessary calculation i commented it out
      /*---Y---*/
        //Acc_angle_error_y = Acc_angle_error_y + (float)Acc_rawY/4096;

        //Acc_angle_error_z = Acc_angle_error_z + (float)Acc_rawZ/4096;


        //test-successfull than the above method
        Acc_angle_error_pitch_x+= atan2((float)Acc_rawY/4096,(float)Acc_rawZ/4096)*rad_to_deg;
        Acc_angle_error_roll_y += atan2((float)Acc_rawX/4096,(float)Acc_rawZ/4096)*rad_to_deg;
        
    }
    
    //Acc_angle_error_x = Acc_angle_error_x/2000;
    //Acc_angle_error_y = Acc_angle_error_y/2000;
    //Acc_angle_error_z = Acc_angle_error_z/2000;   

      Acc_angle_error_pitch_x=Acc_angle_error_pitch_x/2000;
      Acc_angle_error_roll_y =Acc_angle_error_roll_y/2000;
  //end of acc error calculation   


/*Here we calculate the gyro data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  
    for(int i=0; i<2000; i++)
    {
      Wire.beginTransmission(0x68);                       //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                                   //First adress of the Gyro data
      Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true);         //We ask for just 4 registers 
         
      Gyr_rawX=(Wire.read()<<8|Wire.read());               //Once again we shif and sum
      Gyr_rawY=(Wire.read()<<8|Wire.read());
      Gyr_rawZ=(Wire.read()<<8|Wire.read());
      Wire.endTransmission(true);
      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + ((float)Gyr_rawX)/32.8; 
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + ((float)Gyr_rawY)/32.8;
  /*---Z---*/
      Gyro_raw_error_z = Gyro_raw_error_z + ((float)Gyr_rawZ)/32.8;
    }
    Gyro_raw_error_x = Gyro_raw_error_x/2000;
    Gyro_raw_error_y = Gyro_raw_error_y/2000;
    Gyro_raw_error_z = Gyro_raw_error_z/2000;
  //end of gyro error calculation  

   bmp.begin();
   for(int r=0;r<100;r++)
   {
      reference_pressure += bmp.readPressure();
   }
  reference_pressure=reference_pressure/100;
}













void gyroangles()
{
  
  //////////////////////////////////////Gyro read/////////////////////////////////////

    Wire.beginTransmission(0x68);                 //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                             //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);   //We ask for just 6 registers
        
    Gyr_rawX=(Wire.read()<<8|Wire.read());           //Once again we shif and sum
    Gyr_rawY=(Wire.read()<<8|Wire.read());
    Gyr_rawZ=(Wire.read()<<8|Wire.read());
    Wire.endTransmission(true);
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/

    /*---X---*/
    gyro_pitch_x = ((float)Gyr_rawX)/32.8 - Gyro_raw_error_x; 
    /*---Y---*/
    gyro_roll_y = ((float)Gyr_rawY)/32.8 - Gyro_raw_error_y;
    /*---Z---*/
    gyro_yaw_z = ((float)Gyr_rawZ)/32.8 - Gyro_raw_error_z;

    
    
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
    


    /*---X---*/
    //Gyro_angle_x =  Gyro_angle_x + gyro_pitch_x*elapsedTime;
    /*---Y---*/
   // Gyro_angle_y =  Gyro_angle_y + gyro_roll_y*elapsedTime;
    /*---Z---*/
    //Gyro_angle_z =  Gyro_angle_z + gyro_yaw_z*elapsedTime;



}
















void accangles()
{
  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);               //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                           //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);                //keep the transmission and next
  Wire.requestFrom(0x68,6,true);         //We ask for next 6 registers starting withj the 3B 

 
/*We have asked for the 0x3B register. The IMU will send a brust of register.
* The amount of register to read is specify in the requestFrom function.
* In this case we request 6 registers. Each value of acceleration is made out of
* two 8bits registers, low values and high values. For that we request the 6 of them  
* and just make then sum of each pair. For that we shift to the left the high values 
* register (<<) and make an or (|) operation to add the low values.
If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/  

  
  Acc_rawX=(Wire.read()<<8|Wire.read()) ;      //each value needs two registres
  Acc_rawY=(Wire.read()<<8|Wire.read()) ;
  Acc_rawZ=(Wire.read()<<8|Wire.read()) ; 
  Wire.endTransmission(true);
  
  //acc_pitch_x=((float)Acc_rawX) /4096 - Acc_angle_error_x;  not used in alculation
  //acc_roll_y =((float)Acc_rawY) /4096 - Acc_angle_error_y;
  //acc_yaw_z  =((float)Acc_rawZ) /4096 - Acc_angle_error_z;
  
/*Now in order to obtain the Acc angles we use euler formula with acceleration values
after that we substract the error value found before*/  
 /*---X---*/
 /*Acc_angle_x = atan2(acc_roll_y/9.8,acc_yaw_z/9.8)*rad_to_deg;
 /*---Y---*/
 /*Acc_angle_y = atan2(acc_pitch_x/9.8,acc_yaw_z/9.8)*rad_to_deg;*/ 

Acc_angle_x =Acc_angle_x * .7 + .3 * (atan2((float)Acc_rawY,(float)Acc_rawZ)*rad_to_deg-Acc_angle_error_pitch_x);
 /*---Y---*/
 Acc_angle_y = Acc_angle_y * .7 + .3 * -(atan2((float)Acc_rawX,(float)Acc_rawZ)*rad_to_deg-Acc_angle_error_roll_y); 


 
}
