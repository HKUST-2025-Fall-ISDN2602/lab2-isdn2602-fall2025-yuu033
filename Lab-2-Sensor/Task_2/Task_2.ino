#include "IMU.h"

/*Define Pinout*/
#define IMU_SDA 17
#define IMU_SCL 18 

/*Define constant */
const float LowPassFilterAlpha = 0.2f ; //*** Set filter constant for accelerometer which Alpha + Beta = 1.0f ***
const float LowPassFilterBeta =  0.8f ;  

const float ComplementaryFilterALPHA = 0.98f;  
const float dt =  0.01f;    //*** Set the loop time as 10 ms***

/*------------ DO NOT Change the code BELOW ------------*/
// Initialize variables for low-pass filtering
float filteredAccX = 0.0f;
float filteredAccY = 0.0f;
float filteredAccZ = 0.0f;
float filteredGyroX = 0.0f;
float filteredGyroY = 0.0f;
float filteredGyroZ = 0.0f;

// Initialize global variables to store orientation
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

/*------------ DO NOT Change the code ABOVE ------------*/
/*Enable Filter*/
bool Filter = true;           //True:  Enable
                              //False: Disale

/*Enable Serial Plot*/
bool SerialPlotGrapgh = false; //True:  Enable
                              //False: Disale

/*------------ DO NOT Change the code BELOW ------------*/
// an ICM42688 object with the ICM42688 sensor on I2C bus with address 0x68 using SDA pin 17 and SCL pin 18
ICM42688 IMU(Wire, 0x68, IMU_SDA, IMU_SCL);

void setup() {
	// serial to display data
	Serial.begin(115200);
	while (!Serial) {}

	// start communication with IMU
	int status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.println("Check IMU wiring or try cycling power");
		Serial.print("Status: ");
		Serial.println(status);
		while (1) {}
	}

	// setting the accelerometer full scale range to +/-8G
	IMU.setAccelFS(ICM42688::gpm8);
	// setting the gyroscope full scale range to +/-500 deg/s
	IMU.setGyroFS(ICM42688::dps500);

	// set output data rate to 12.5 Hz
	IMU.setAccelODR(ICM42688::odr12_5);
	IMU.setGyroODR(ICM42688::odr12_5);

	Serial.println("---IMU Initialized---");
}
/*------------ DO NOT Change the code ABOVE ------------*/


/**/


void loop() {
	// read the sensor
	IMU.getAGT();
  /*For tuning the filter, please change the varibles but not the code below*/
  /*------------ DO NOT Change the code BELOW ------------*/
  /*Applying Filter*/
  filteredAccX = LowPassFilterAlpha * (IMU.accX()) + (1 - LowPassFilterAlpha) * filteredAccX;
  filteredAccY = LowPassFilterAlpha * (IMU.accY()) + (1 - LowPassFilterAlpha) * filteredAccY;
  filteredAccZ = LowPassFilterAlpha * (IMU.accZ()) + (1 - LowPassFilterAlpha) * filteredAccZ;

  // Apply low-pass filter to gyr)oscope readings
  filteredGyroX = LowPassFilterBeta * (IMU.gyrX()) * DEG_TO_RAD + (1 - LowPassFilterBeta) * filteredGyroX;
  filteredGyroY = LowPassFilterBeta * (IMU.gyrY()) * DEG_TO_RAD + (1 - LowPassFilterBeta) * filteredGyroY;
  filteredGyroZ = LowPassFilterBeta * (IMU.gyrZ()) * DEG_TO_RAD + (1 - LowPassFilterBeta) * filteredGyroZ;

  // Calculate roll and pitch from accelerometer data
  float acc_roll = atan2(filteredAccY, sqrt(filteredAccX * filteredAccX + filteredAccZ * filteredAccZ));
  float acc_pitch = atan2(-filteredAccX, sqrt(filteredAccY * filteredAccY + filteredAccZ * filteredAccZ));

  // Integrate gyroscope data to get roll, pitch, and yaw
  float gyro_roll = roll + filteredGyroX * dt;
  float gyro_pitch = pitch + filteredGyroY * dt;
  float gyro_yaw = yaw + filteredGyroZ * dt;

  // Apply complementary filter
  roll = ComplementaryFilterALPHA * gyro_roll + (1 - ComplementaryFilterALPHA) * acc_roll;
  pitch = ComplementaryFilterALPHA * gyro_pitch + (1 - ComplementaryFilterALPHA) * acc_pitch;
  yaw = gyro_yaw;  // Yaw is not corrected by accelerometer

  /*------------ DO NOT Change the code ABOVE ------------*/

  if(SerialPlotGrapgh && !Filter){
  Serial.print("accX:");
  Serial.print(IMU.accX(), 3);
	Serial.print(",");
  Serial.print("accY:");
	Serial.print(IMU.accY(), 3);
	Serial.print(",");
  Serial.print("accZ:");
	Serial.print(IMU.accZ(), 3);
	Serial.print(",");
  Serial.print("gyrX:");
	Serial.print(IMU.gyrX(), 3);
	Serial.print(",");
  Serial.print("gyrY:");
	Serial.print(IMU.gyrY(), 3);
	Serial.print(",");
  Serial.print("gyrZ:");
	Serial.print(IMU.gyrZ(), 3);
	Serial.println("\t");   
  }

  else if(SerialPlotGrapgh && Filter){
  Serial.print("accX:");
  Serial.print(IMU.accX(), 3);
	Serial.print(",");
  Serial.print("accY:");
	Serial.print(IMU.accY(), 3);
	Serial.print(",");
  Serial.print("accZ:");
	Serial.print(IMU.accZ(), 3);
	Serial.print(",");
  Serial.print("gyrX:");
	Serial.print(IMU.gyrX(), 3);
	Serial.print(",");
  Serial.print("gyrY:");
	Serial.print(IMU.gyrY(), 3);
	Serial.print(",");
  Serial.print("gyrZ:");
	Serial.print(IMU.gyrZ(), 3);
	Serial.println("\t");  
  }
  else if(!SerialPlotGrapgh && !Filter){
	// display the data
	Serial.print(IMU.accX(), 3);
	Serial.print(",");
	Serial.print(IMU.accY(), 3);
	Serial.print(",");
	Serial.print(IMU.accZ(), 3);
	Serial.print(",");
	Serial.print(IMU.gyrX(), 3);
	Serial.print(",");
	Serial.print(IMU.gyrY(), 3);
	Serial.print(",");
	Serial.print(IMU.gyrZ(), 3);
	Serial.println("\t");
  }
  else if(!SerialPlotGrapgh && Filter){
  	// display the data
	Serial.print(filteredAccX, 3);
	Serial.print(",");
	Serial.print(filteredAccY, 3);
	Serial.print(",");
	Serial.print(filteredAccZ, 3);
	Serial.print(",");
	Serial.print(roll, 3);
	Serial.print(",");
	Serial.print(pitch, 3);
	Serial.print(",");
	Serial.print(yaw, 3);
	Serial.println("\t");  
  
  }
  delay(10);
  }
	

