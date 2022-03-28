/****************************************************************
 * Example6_DMP_Quat9_Orientation.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, April 25th, 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * ** This example is based on InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
 * ** We are grateful to InvenSense for sharing this with us.
 * 
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in:
 * ** Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/

//#include <math.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.                
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                       // the ADR jumper is closed the value becomes 0

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// -----------------------------------------------------------------------------------

class Quaternion
{
    public:
        Quaternion(const double q0, const double q1, const double q2, const double q3):
            _q0(q0), _q1(q1), _q2(q2), _q3(q3) 
        {
            //q0 = q0; 
        }

        // ----------------------------------------------------------------------------

        double q0() {return _q0;}   // W
        double q1() {return _q1;}   // X
        double q2() {return _q2;}   // Y
        double q3() {return _q3;}   // Z

        // ----------------------------------------------------------------------------

        bool IsValid()
        {
            bool isvalid = true;

            // Test the value range of each quaternion field.
            isvalid &= (abs(_q0) <= 1.0000000);
            isvalid &= (abs(_q1) <= 1.0000000);
            isvalid &= (abs(_q2) <= 1.0000000);
            isvalid &= (abs(_q3) <= 1.0000000);
            
            return isvalid;
        }

        // ----------------------------------------------------------------------------

        double wrapAngle(double angle)
        {
            double twoPi = TWO_PI;
            return angle - twoPi * floor(angle / twoPi);
        }
        
        // ----------------------------------------------------------------------------

        double yaw(bool degrees = false)
        {
            double siny_cosp =       2.0 * ((_q0 * _q3) + (_q1 * _q2));
            double cosy_cosp = 1.0 - 2.0 * ((_q2 * _q2) + (_q3 * _q3));
            double result = atan2(siny_cosp, cosy_cosp);
            result = wrapAngle(result);
            if (degrees)
            {
               result = result * rad2deg;
            }
            return result;
        }

        // ----------------------------------------------------------------------------

    private:
        const double _q0;
        const double _q1;
        const double _q2;
        const double _q3;

        const double rad2deg = RAD_TO_DEG;
        const double deg2rad = DEG_TO_RAD;
                
};

// -----------------------------------------------------------------------------------

void log(const char* text)
{
    SERIAL_PORT.print("log: ");
    SERIAL_PORT.println(text);
}

// -----------------------------------------------------------------------------------

void write_quat(Quaternion &quat)
{
    // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
    // Output as JSON

    size_t resolution = 9;
  
    SERIAL_PORT.print(F("{\"quat_w\":"));
    SERIAL_PORT.print(quat.q0(), resolution);
        
    SERIAL_PORT.print(F(", \"quat_x\":"));
    SERIAL_PORT.print(quat.q1(), resolution);
        
    SERIAL_PORT.print(F(", \"quat_y\":"));
    SERIAL_PORT.print(quat.q2(), resolution);
        
    SERIAL_PORT.print(F(", \"quat_z\":"));
    SERIAL_PORT.print(quat.q3(), resolution);

    SERIAL_PORT.print(F(", \"yaw\":"));
    SERIAL_PORT.print(quat.yaw(true), resolution);

    SERIAL_PORT.println(F("}"));

}

// -----------------------------------------------------------------------------------

void setup()
{
    SERIAL_PORT.begin(115200); // Start the serial console
    delay(100);

    while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
        SERIAL_PORT.read();

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    myICM.enableDebugging(SERIAL_PORT); // Uncomment this line to enable helpful debug messages on Serial

    bool initialized = false;
    while (!initialized)
    {
        // Initialize the ICM-20948
        // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
        myICM.begin(WIRE_PORT, AD0_VAL);

        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            SERIAL_PORT.println(F("Trying again..."));
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    SERIAL_PORT.println(F("Device connected!"));

    bool success = true; // Use success to show if the DMP configuration was successful
    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP orientation sensor
    // TEST  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    // ====

    
    // Enable any additional sensors / features
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

            
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok); // Set to the maximum
    
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
        SERIAL_PORT.println(F("DMP enabled!"));
    }
    else
    {
        SERIAL_PORT.println(F("Enable DMP failed!"));
        SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
            ; // Do nothing more
    }
}

// -----------------------------------------------------------------------------------




  Quaternion *current_quaternion = nullptr;

void loop()
{
    static uint32_t imu_write_time = millis();

    /*
        Read any DMP data waiting in the FIFO
        Note:
            readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
            If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
            readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
            readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
            readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    */


    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);

    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        //if (data.header == DMP_header_bitmap_Quat9) // We have asked for orientation data so we should receive Quat9
        if (data.header == DMP_header_bitmap_Quat6) // We have asked for orientation data so we should receive Quat9
        {
            // Scale to +/- 1

            //double q1 = (int32_t)data.Quat9.Data.Q1 / 1073741824.0; // Convert to double. Divide by 2^30
            //double q2 = (int32_t)data.Quat9.Data.Q2 / 1073741824.0; // Convert to double. Divide by 2^30
            //double q3 = (int32_t)data.Quat9.Data.Q3 / 1073741824.0; // Convert to double. Divide by 2^30

            double q1 = (int32_t)data.Quat6.Data.Q1 / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = (int32_t)data.Quat6.Data.Q2 / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = (int32_t)data.Quat6.Data.Q3 / 1073741824.0; // Convert to double. Divide by 2^30

            //int16_t accuracy = data.Quat9.Data.Accuracy;

            double q0_squared = 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3));
            if (q0_squared > 0.0)
            {
                // q0_squared MUST be positive.
                double q0 = sqrt(q0_squared);
                
                Quaternion quaternion(q0, q1, q2, q3);
              
                if (quaternion.IsValid())
                {
                    write_quat(quaternion);
                    imu_write_time = millis();
                }
                else
                {
                    log("IMU firmware error. Invalid quaternion data.");
                }

                /*
                
                SERIAL_PORT.print(F("Yaw:   "));
                SERIAL_PORT.print(quaternion.yaw(false), 3);
                
                SERIAL_PORT.print("     ");
                SERIAL_PORT.print(quaternion.yaw(true), 3);
                
                //SERIAL_PORT.print("     ");
                //SERIAL_PORT.print(accuracy);

                //SERIAL_PORT.print("  ERROR:   ");
                //SERIAL_PORT.print(quaternion.yaw(false) - quat2.yaw(false), 3);
                
                SERIAL_PORT.println("");
                */
            } 
            else    
            {
                log("Error Calculating Q0");
            }

            
        }
    } 

    uint32_t timeout = millis() - imu_write_time;
    
    if (timeout >= 1000)
    {
        log("IMU firmware error. Read FIFO timeout error."); 
        imu_write_time = millis();
        delay(100);
    }

    if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
        // Delay while waiting for data on the fifo buffer. 
        delay(5);  // 5 milliseconds.
    }

}
