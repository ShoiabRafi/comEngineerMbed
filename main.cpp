#include "mbed.h"
#include "MPU6050.h"

float sum = 0;
float accelTime = 0.0f; 
float dv = 0.0f;
float v = 0.0f;
float v_old = 0.0f; 
float accelTimeLast = 0.0f;
uint32_t sumCount = 0;

MPU6050 mpu6050;
void init_MPPU6050(void);
void calibrateMPU6050();
void beep(int beepTimes, int tone, float delay);
void read_MPU6050(void);
void calc_accel_gyro(void);
   
Timer t;

Serial pc(USBTX, USBRX); // tx, rx
Serial BT(D1, D0); // tx, rx
        
int main()
{
    pc.baud(9600);  
    BT.baud(9600);
    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C   
    t.start();  

    AnalogIn PulseSensor(A6); //setting the pin for analog input
    float PulseData;

    beep(1, 10, 1); // Power up beep
    init_MPPU6050();

    // tone - put down on a flat serface 
    beep(10, 1, 1); // beepTimes, tone, delay    pin A4  
    wait(1);
    beep(10, 2, 1);  
    wait(1); 
    beep(1, 1, 40); 

    calibrateMPU6050();

    // tone - calibration successful
    beep(2, 3, 1);  
    beep(1, 2, 10); 



    while(1) // super loop
    { 
    
        read_MPU6050();
        PulseData = 100 * PulseSensor.read();



        //pc.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \r\n", ax, ay, az, yaw, pitch, roll, temperature);       
        BT.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f \r\n",PulseData, ax, ay, az, yaw, pitch, roll, temperature);       
        myled= !myled;
        sum = 0;
        sumCount = 0; 
    }
}
 
 


void init_MPPU6050()
{
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    BT.printf("I AM 0x%x\n\r", whoami); pc.printf("I SHOULD BE 0x68\n\r");
    
    if (whoami == 0x68) // WHO_AM_I should always be 0x68
    {  
        pc.printf("MPU6050 is online...");
        wait(1);
            
        mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
        // pc.printf("x-axis self test: acceleration trim within : "); pc.printf("%f", SelfTest[0]); pc.printf("% of factory value \n\r");
        // pc.printf("y-axis self test: acceleration trim within : "); pc.printf("%f", SelfTest[1]); pc.printf("% of factory value \n\r");
        // pc.printf("z-axis self test: acceleration trim within : "); pc.printf("%f", SelfTest[2]); pc.printf("% of factory value \n\r");
        // pc.printf("x-axis self test: gyration trim within : "); pc.printf("%f", SelfTest[3]); pc.printf("% of factory value \n\r");
        // pc.printf("y-axis self test: gyration trim within : "); pc.printf("%f", SelfTest[4]); pc.printf("% of factory value \n\r");
        // pc.printf("z-axis self test: gyration trim within : "); pc.printf("%f", SelfTest[5]); pc.printf("% of factory value \n\r");
        wait(1);

        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
        {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
            mpu6050.initMPU6050(); pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
            wait(2);
        }
        else
        {
            pc.printf("Device did not the pass self-test!\n\r");          
        }
    }
    else
    {
        pc.printf("Could not connect to MPU6050: \n\r");
        pc.printf("%#x \n",  whoami);
            
        while(1) ; // Loop forever if communication doesn't happen
    }
}

void read_MPU6050()
{
    // If data ready bit set, all data registers have new data
    if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)  // check if data ready interrupt
    { 
        mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
        mpu6050.getAres();
        
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes - accelBias[1];   
        az = (float)accelCount[2]*aRes - accelBias[2];  
    
        mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
        mpu6050.getGres();
    
        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
        gz = (float)gyroCount[2]*gRes; // - gyroBias[2];   

        tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
        temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
    }  
   
    Now = t.read_us();
    deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
    lastUpdate = Now;
    
    sum += deltat;
    sumCount++;
    
    if(lastUpdate - firstUpdate > 10000000.0f)
    {
        beta = 0.04;  // decrease filter gain after stabilized
        zeta = 0.015; // increasey bias drift gain after stabilized
    }
    
   // Pass gyro rate as rad/s
    mpu6050.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

    // convert the g value to m/s
    float axm = ax * 9.81; 
    float aym = ay * 9.81;
    float azm = az * 9.81; 
    

    
    // get the time 
    accelTime = t.read_ms()/1000000; 
    dv = axm * (accelTime - accelTimeLast);
    v = v_old + dv; 
    accelTimeLast = accelTime; 
    
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
}
 
void calibrateMPU6050()
{
    BT.printf("\r\nPut the device on a flat plain serface\r\n");
    BT.printf("Device calibrating...\r\n");
    set_time(0);  // Set RTC time to Jan 1 1970 00:00:00
    // Get the current time as a time_t value
    time_t seconds = time(NULL);

    // Convert the time_t value to a tm structure
    struct tm* timeinfo = localtime(&seconds);

    // Extract the seconds component
    int seconds_value = timeinfo->tm_sec;
    int secondBeep;

    while (seconds_value < 32)
    {
        // Get the current time as a time_t value
        seconds = time(NULL);
        // Convert the time_t value to a tm structure
        struct tm* timeinfo = localtime(&seconds);
        // Extract the seconds component
        secondBeep = timeinfo->tm_sec;
        if (!(secondBeep == seconds_value))
        {
            beep(1, 10, 0.3);  
        }
        seconds_value = secondBeep;
        
        read_MPU6050();
    }

    BT.printf("Calibration Successful in %d seconds\r\n", seconds_value);
    wait(0.5);
}

void beep (int beepTimes, int tone, float delay)
{
    DigitalOut Buzzer (D10); // Buzzer

    for (int i = 0; i < beepTimes; i++)
    {
        for (int k = 0; k < (30/tone)*delay; k++)
        {
            Buzzer = 1;
            wait_ms(tone);
            Buzzer = 0;
            wait_ms(tone);
        }
        for (int k = 0; k < (30/tone)*delay; k++)
        {
            Buzzer = 0;
            wait_ms(tone);
        }
    }
}
