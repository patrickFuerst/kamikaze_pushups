
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>


/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
   2015/NOV/13  - Added calibration save and restore
   */

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (15)

#define VBATPIN A7

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int status = WL_IDLE_STATUS;  
char ssid[] = "chruthli"; //  your network SSID (name)
char pass[] = "pfumpfli2";    // your network password (use for WPA, or use as key for WEP)

WiFiUDP Udp;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}


/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void initialiseSensor()
{
  
    /* Initialise the sensor */
    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }


    sensors_event_t event;
//    bno.getEvent(&event);
//    if (foundCalib){
//        Serial.println("Move sensor slightly to calibrate magnetometers");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
//    }
//    else
//    {
//        Serial.println("Please Calibrate Sensor: ");
//        while (!bno.isFullyCalibrated())
//        {
//            bno.getEvent(&event);
//
//            Serial.print("X: ");
//            Serial.print(event.orientation.x, 4);
//            Serial.print("\tY: ");
//            Serial.print(event.orientation.y, 4);
//            Serial.print("\tZ: ");
//            Serial.print(event.orientation.z, 4);
//
//            /* Optional: Display calibration status */
//            displayCalStatus();
//
//            /* New line for the next sample */
//            Serial.println("");
//
//            /* Wait the specified delay before requesting new data */
//            delay(BNO055_SAMPLERATE_DELAY_MS);
//        }
  //  }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n--------------------------------\n");
    delay(500);
}

void sendSensorData()
{
  /* Get a new sensor event */
    
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::adafruit_vector_type_t::VECTOR_LINEARACCEL);

    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    bool calibrated = bno.isFullyCalibrated();

    union{
      double num;
      byte num_bytes[sizeof(double)];
    } double_data;

    union{
      float num;
      byte num_bytes[sizeof(float)];
    } float_data;
    
    /* Optional: Display calibration status */
   // displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();


    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage

    Udp.beginPacket("192.168.0.100", 7775);
    
    Udp.write('o');   // header character
   
    double_data.num = quat.x();
    Udp.write(double_data.num_bytes , sizeof(double)); // data

    double_data.num = quat.y();
    Udp.write(double_data.num_bytes , sizeof(double)); // data    
        
    double_data.num = quat.z();
    Udp.write(double_data.num_bytes , sizeof(double)); // data    
        
    double_data.num = quat.w();
    Udp.write(double_data.num_bytes , sizeof(double)); // data 
        
    double_data.num = acc.x();
    Udp.write(double_data.num_bytes , sizeof(double)); // data  
        
    double_data.num = acc.y();
    Udp.write(double_data.num_bytes , sizeof(double)); // data 
        
    double_data.num = acc.z();
    Udp.write(double_data.num_bytes , sizeof(double)); // data  
        
    Udp.write( system ); // data
    Udp.write( calibrated ); // data
      
    float_data.num = measuredvbat;
    Udp.write(float_data.num_bytes , sizeof(float)); // data
   
    Udp.endPacket();

}

void setup() {

  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);  
  
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}

  initialiseSensor();


  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(1000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Udp.begin(7776);

}

void loop() {

  if(WiFi.status() != WL_CONNECTED){
    // attempt to connect to Wifi network:
    status = WiFi.status();
    while (status != WL_CONNECTED) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid, pass);
  
      // wait 10 seconds for connection:
      delay(1000);
    }
    Serial.println("Connected to wifi");
    printWifiStatus();
  }

  sendSensorData();
  Serial.println("Sending data");


  /* Wait the specified delay befo
  re requesting new data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}




