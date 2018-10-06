//LIDAR
//LIDAR values

#include <SoftwareSerial.h>

#define T0  1000000000
#define T1  1000000000
#define TO  5000000
#define BUFSIZE 50
#define ACC_RANGE 16384

SoftwareSerial mySerial(10,11);  //RX,TX

//MPU code
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double myYaw = 0;
double myPitch = 0;
double myRoll = 0;
double vx = 0;
double vy = 0;
double vz = 0;

//math code
struct vector {
    double x;
    double y;
    double z;
};

double d = 0;
double d_offset = 0.03; //offset of 30mm => the wall between mic and rest of the board

void vectPrint(vector v) {
    Serial.print("Vector: ");
    Serial.print("x = ");
    Serial.print(v.x,4);
    Serial.print(", y = ");
    Serial.print(v.y,4);
    Serial.print(", z = ");
    Serial.println(v.z,4);
}

double vectDP(vector a, vector b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

double vectMag(vector v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

void vectGet(vector *v) {
    (*v).x = vx;
    (*v).y = vy;
    (*v).z = vz;
}

//Make some globals 
vector vc = {0,0,0}; //current measurement vector
vector vp = {0,0,0}; //previous measurement vector
vector v = {0,0,0};  //current mpu vector
double dc = 0;
double dp = 0;
double cosC = 0;

char cmd = 'I';
char buf[BUFSIZE] = {0};
String sbuf;
char *dbuf;

// ================================================================
// ===               Functions                                  ===
// ================================================================

void help() {
    Serial.println(F("*****************Help*****************"));
    Serial.println(F("Commands:"));
    Serial.println("O | turn on laser");
    Serial.println("C | turn off laser");
    Serial.println("S | read state of laser");
    Serial.println("D | measure d via laser");
    Serial.println("M | slow measure");
    Serial.println("F | fast measure");
    Serial.println("P | set position for measurement");
    Serial.println("G | get distance / accumulated distance");
    Serial.println("V | set vector 1");
    Serial.println("U | set vector 2");
    Serial.println("K | kalculate distance");
    Serial.println("I | default state");
    Serial.println("A | get angle");
    Serial.println("T | change measurement speed");
}

int lasGetB( unsigned int to, char *buff, int buff_size ) {
    int charc = 0;
    int count = 0;

    //Serial.println("Getting laser data");
    while( mySerial.available() == 0) {
    }
    
    while( count < to && charc < buff_size - 1 ) {
        if( mySerial.available() ) {
             buff[charc] = mySerial.read();
             charc++;
             count = 0;
        }
//        Serial.println(count);
        count++;
    }
    buff[charc] = '\0';
    return 0;
}

char *findDigit(char *str) {
    while(*str != '\0' && (*str > '9' || *str < '0')) {
        str++;
    }
    return str;
}



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    //Init serial communication
 
//    Serial.begin(19200);     // communication with the host computer
    
    // Start the software serial for communication with the ESP8266
    mySerial.begin(19200);  
 
    Serial.println("");
    Serial.println("3117 YEEEEEEEEEEEEEEET\n");
    Serial.println("***********************************************\n");
    Serial.println("");    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // listen for user input 
        if ( Serial.available() ) 
        {
            cmd = Serial.read();
        }
        
        switch(cmd) {
            case 'O' : 
                mySerial.write(cmd);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'C' : 
                mySerial.write(cmd);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'S' : 
                mySerial.write(cmd);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'D' : 
                mySerial.write(cmd);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'M' :
                mySerial.write(cmd);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'F' :
                mySerial.write(cmd);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'P' :
                vectGet(&vc);
                vectPrint(vc);
                mySerial.write('D');
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
            break;
            case 'G' :
                
            break;
            case 'I' :
                
            break;
            case 'T' :
                
            break;
            case 'H' :
            help();
            break;
            case 'U' :
                vectGet(&vp);
                vectPrint(vp);
                mySerial.write('D');
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
                lasGetB( TO, buf, BUFSIZE );
                sbuf = String(findDigit(buf));
                dp = sbuf.toDouble() + d_offset;
                Serial.print(" Distance is ");
                Serial.println(dc,4);
                Serial.println(sbuf);
                mySerial.write('O');
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);

            break;
            case 'V' :
                vectGet(&vc);
                vectPrint(vc);
                mySerial.write('D');
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);
                lasGetB( TO, buf, BUFSIZE );
                sbuf = String(findDigit(buf));
                dc = sbuf.toDouble() + d_offset;
                Serial.print(" Distance is ");
                Serial.println(dc,4);
                Serial.println(sbuf);
                mySerial.write('O');
                lasGetB( TO, buf, BUFSIZE );
                Serial.print(buf);

            break;
            case 'A' :
                vectGet(&v);
                vectPrint(v);        
            break;
            case 'K' :
                //calculate cos theta
                cosC = vectDP(vc, vp) / (vectMag(vc) * vectMag(vp));
                //use cosine rule
                d = sqrt(dc*dc + dp*dp - 2*dc*dp*cosC);
                Serial.print("Distance between two points is ");
                Serial.print(d,4);
                Serial.println("m");
            break;
    
        }
        cmd = 'I';
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            myYaw = ypr[0];
            myPitch = ypr[1];
            myRoll = ypr[2];
            vx = cos(myYaw)*cos(myPitch);
            vy = sin(myYaw)*cos(myPitch);
            vz = sin(myPitch);

//            Serial.print("xyz\t");
//            Serial.print(vx);
//            Serial.print("\t");
//            Serial.print(vy);
//            Serial.print("\t");
//            Serial.println(vz);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
