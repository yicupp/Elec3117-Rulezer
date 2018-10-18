//LCD touchscreen
// Paint example specifically for the TFTLCD breakout board.
// If using the Arduino shield, use the tftpaint_shield.pde sketch instead!
// DOES NOT CURRENTLY WORK ON ARDUINO LEONARDO

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_TFTLCD.h> // Hardware-specific library
#include <TouchScreen.h>

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET 13

// Assign human-readable names to some common 16-bit color values:
#define    BLACK   0x0000
#define RED    0x001F
#define BLUE     0xF800
#define GREEN   0x07E0
#define YELLOW    0x07FF
#define MAGENTA 0xF81F
#define CYAN  0xFFE0
#define WHITE   0xFFFF
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;

//LIDAR
//LIDAR values

#include <SoftwareSerial.h>

#define T0  1000000000
#define T1  1000000000
#define TO  5000000
#define BUFSIZE 50
#define ACC_RANGE 16384
//#define DEBUG_MPU
#define DEBUG_FAST_START

SoftwareSerial mySerial(10,11);  //RX,TX

//MPU code
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#ifdef DEBUG_FAST_START
#define MPU_STAB_TIME 500 //stabilissation time for mpu in ms
#else 
#define MPU_STAB_TIME 10000
#endif

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//keypad 
#define KP_SDOPIN 9
#define KP_SCLPIN 8
int keyNum = 0;
int keyPress = 0;
int showVect = -1;
int showFifoReset = -1;
unsigned long runtimeSec = 0;
unsigned long runtimeStart = 0;


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
double d_offset = 0.070; //offset of 30mm => the wall between mic and rest of the board

#define lcdVectx1   20
#define lcdVecty1   180
#define lcdVectx2   200
#define lcdVecty2   190
#define lcdVectlx   lcdVectx2-lcdVectx1
#define lcdVectly   lcdVecty2-lcdVecty1
#define lcdVectcw   WHITE
#define lcdVectcb   BLACK

char vectPrintBuf[50];
char vxStr[10]={'\0'};
char vyStr[10]={'\0'};
char vzStr[10]={'\0'};

void vectPrint(vector v) {
    tft.setCursor(lcdVectx1,lcdVecty1);

    tft.setTextSize(1);

    dtostrf(v.x,5,4,vxStr);
    dtostrf(v.y,5,4,vyStr);
    dtostrf(v.z,5,4,vzStr);

    tft.setTextColor(lcdVectcw,lcdVectcb);
    sprintf(vectPrintBuf,"x=%s y=%s z=%s   ",vxStr,vyStr,vzStr);
    tft.print(vectPrintBuf);
/*    tft.print("x=");
    tft.print(v.x,4);
    tft.print(" y=");
    tft.print(v.y,4);
    tft.print(" z=");
    tft.println(v.z,4);*/
    
    /*Serial.print("Vector: ");
    Serial.print("x = ");
    Serial.print(v.x,4);
    Serial.print(", y = ");
    Serial.print(v.y,4);
    Serial.print(", z = ");
    Serial.println(v.z,4);
    
    Serial.println(vectPrintBuf);
    */
}
#define MINPRESSURE 10
#define MAXPRESSURE 1000

TSPoint p;



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



void mpuGetData() {
// get current FIFO count
#ifdef DEBUG_MPU
    Serial.println("get data start");
#endif
    
    fifoCount = mpu.getFIFOCount();
//    mpu.resetFIFO();
//    delay(5);
    // clean buffer
    if (fifoCount > packetSize) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        delay(25);
    }
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
#ifdef DEBUG_MPU
        Serial.println("Reading data packet");
#endif
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        //fifoCount -= packetSize; 
               
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
        //digitalWrite(LED_PIN, blinkState);    
#ifdef MPU_DEBUG
        Serial.println("Get data stop");
#endif
}

// LCD and touch functions
// 
//

struct button {
    int c;
    int x1;
    int x2;
    int y1;
    int y2;
    int lx;
    int ly;
};

//button startBut, setBut, resetBut;

#define     startBut_x1   320-BOXSIZE*5/2
#define     startBut_y1   0
#define     startBut_x2   320
#define     startBut_y2   0+BOXSIZE*3/2
#define     startBut_lx   BOXSIZE*5/2
#define     startBut_ly   BOXSIZE*3/2
#define     startBut_c    GREEN
#define     setBut_x1     320-BOXSIZE*5/2
#define     setBut_y1     BOXSIZE*3/2
#define     setBut_x2     320
#define     setBut_y2     BOXSIZE*6/2
#define     setBut_lx     BOXSIZE*5/2
#define     setBut_ly     BOXSIZE*3/2
#define     setBut_c      BLUE
#define     resetBut_x1   320-BOXSIZE*5/2
#define     resetBut_y1   BOXSIZE*3
#define     resetBut_x2   320
#define     resetBut_y2   BOXSIZE*9/2
#define     resetBut_lx   BOXSIZE*5/2
#define     resetBut_ly   BOXSIZE*3/2
#define     resetBut_c    RED
#define     unitBut_x1    320-BOXSIZE*5/2
#define     unitBut_y1    BOXSIZE*9/2
#define     unitBut_x2    320
#define     unitBut_y2    0+BOXSIZE*12/2
#define     unitBut_lx    BOXSIZE*5/2
#define     unitBut_ly    BOXSIZE*3/2
#define     unitBut_c     YELLOW
#define     unitBut_div   unitBut_x1+unitBut_lx/2-1

#define     disturbStr_x  BOXSIZE/8
#define     disturbStr_y  BOXSIZE+BOXSIZE*1/6
#define     disturbBar_x  BOXSIZE*3/2
#define     disturbBar_y  BOXSIZE
#define     disturbBar_lx BOXSIZE*7/2
#define     disturbBar_ly BOXSIZE/2


#define     out_tot_x     BOXSIZE/8
#define     out_tot_y     60
#define     out_tot_ux    200
#define     out_tot_uy    60

#define     out_p2p_x     BOXSIZE/8
#define     out_p2p_y     90
#define     out_p2p_ux    200
#define     out_p2p_uy    90
   
#define     out_l2p_x     BOXSIZE/8
#define     out_l2p_y     120
#define     out_l2p_ux    200
#define     out_l2p_uy    120

#define     out_ang_x     BOXSIZE/8
#define     out_ang_y     150
#define     out_ang_ux    200
#define     out_ang_uy    150

#define     out_str_y     120

void lcdScan() {
    
    pinMode(XM, INPUT);   //change mode for reading
    pinMode(YP, INPUT);
    TSPoint tmpp = ts.getPoint();

    //for some reason, the x and y values are swapped
    p.y = 240-map(tmpp.x, TS_MINX, TS_MAXX, tft.height(), 0);
    p.x = 320-map(tmpp.y, TS_MINY, TS_MAXY, tft.width(), 0);
    p.z = tmpp.z;
    
    pinMode(XM, OUTPUT);   //restore mode
    pinMode(YP, OUTPUT);
    tft.setCursor(lcdVectx1,lcdVecty1-20);
    tft.setTextSize(1);
    tft.setTextColor(lcdVectcw,lcdVectcb);
    //sprintf(vectPrintBuf,"P: x=%d y=%d z=%d",p.x,p.y,p.z);
    //if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
        tft.print("("); tft.print(p.x);
        tft.print(", "); tft.print(p.y);
        tft.print(", "); tft.print(p.z);
        tft.println(")            ");
        //Serial.print("("); Serial.print(p.x);
        //Serial.print(", "); Serial.print(p.y);
        //Serial.println(")");
    //}
    //tft.print(vectPrintBuf);
}

#define NONE 0
#define START 1
#define LASER 2
#define RESET 3
#define UNIT_LEN  4
#define UNIT_ANG  5

int lcdCmd = NONE;
int lcdPrevCmd = NONE;
unsigned long softDbounceT = 0;
#define SOFT_DBOUNCE_TIME 1000
bool lcdDB = true;
bool lcdValidCmd = true;


void lcdGetCmd() {
    lcdDB = millis()-softDbounceT>SOFT_DBOUNCE_TIME;
    lcdValidCmd = false;
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
        if(p.x>=startBut_x1 && p.x<=startBut_x2 && p.y>=startBut_y1 && p.y <= startBut_y2) {
            if(lcdCmd != START) {
                lcdCmd=START;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                lcdCmd=START;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
        }
        else if(p.x>=setBut_x1 && p.x<=setBut_x2 && p.y>=setBut_y1 && p.y <= setBut_y2){
            if(lcdCmd != LASER) {
                lcdCmd=LASER;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                lcdCmd=LASER;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
        }
        else if(p.x>=resetBut_x1 && p.x<=resetBut_x2 && p.y>=resetBut_y1 && p.y <= resetBut_y2){
            if(lcdCmd != RESET) {
                lcdCmd=RESET;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                lcdCmd=RESET;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
        }
        else if(p.x>=unitBut_x1 && p.x<=unitBut_x2 && p.y>=unitBut_y1 && p.y <= unitBut_y2){
            if((lcdCmd != UNIT_ANG || lcdCmd != UNIT_LEN)) {
                if(p.x<unitBut_div) lcdCmd=UNIT_LEN;
                else lcdCmd=UNIT_ANG;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                if(p.x<unitBut_div) lcdCmd=UNIT_LEN;
                else lcdCmd=UNIT_ANG;
                softDbounceT=millis();
                lcdValidCmd = true;
            }
        }
    }
    else {
        if(lcdDB) {
            lcdCmd=NONE;
        }
    }
    
}

void lcd_init() {
    Serial.println(F("Initialising LCD FUNCTION"));    
    tft.begin(0x8230);

    tft.fillScreen(BLACK);
    tft.setRotation(1);
    tft.setTextColor(GREEN);
    tft.setTextSize(3);
    tft.println("Rulezer!");

    tft.fillRect(startBut_x1,startBut_y1,startBut_lx ,startBut_ly , startBut_c);
    tft.fillRect(setBut_x1  ,setBut_y1  ,setBut_lx ,setBut_ly , setBut_c);
    tft.fillRect(resetBut_x1,resetBut_y1,resetBut_lx ,resetBut_ly , resetBut_c);
    tft.fillRect(unitBut_x1,unitBut_y1,unitBut_lx ,unitBut_ly , unitBut_c);

    tft.setCursor(325-BOXSIZE*2,BOXSIZE*9/4);
    tft.setCursor(320-BOXSIZE*2,BOXSIZE*8/4);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Laser");
    
    tft.setCursor(320-BOXSIZE*2,BOXSIZE*14/4);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Reset");

    tft.setCursor(310-BOXSIZE*2,BOXSIZE*2/4);
    tft.setTextColor(BLACK);
    tft.setTextSize(2);
    tft.print("Measure");
    
    tft.setCursor(325-BOXSIZE*2,BOXSIZE*19/4);
//    tft.setTextColor(BLACK);
//    tft.setTextSize(2);
    tft.print("Unit");
    tft.setTextSize(1);
    tft.setCursor(320-BOXSIZE*2,BOXSIZE*43/8);
    tft.print("Len");
    tft.setCursor(326-BOXSIZE,BOXSIZE*43/8);
    tft.print("Ang");
    tft.fillRect(unitBut_x1+unitBut_lx/2-1,unitBut_y1+unitBut_ly/2,2,unitBut_ly/2,BLACK);

    tft.setTextColor(WHITE);
    tft.setCursor(disturbStr_x,disturbStr_y);
    tft.println("Movement");
    tft.fillRect(disturbBar_x,disturbBar_y,disturbBar_lx,disturbBar_ly,RED);

    tft.setTextSize(2);
    tft.setCursor(out_tot_x,out_tot_y);
    tft.print("TOT D:");
    tft.setCursor(out_p2p_x,out_p2p_y);
    tft.print("P2P D:");
    tft.setCursor(out_l2p_x,out_l2p_y);
    tft.print("L2P D:");
    tft.setCursor(out_ang_x,out_ang_y);
    tft.print("ANGLE:");
    
  //Make ui
     
  //currentcolor = RED;
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
    Serial.begin(19200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Initialising LCD"));
    lcd_init();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
/*    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

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
    //pinMode(LED_PIN, OUTPUT);

    //Init serial communication
 
//    Serial.begin(19200);     // communication with the host computer
    
    // Start the software serial for communication with the ESP8266
    mySerial.begin(19200);  
 
    Serial.println("");
    Serial.println(F("3117  YEEEEEEEEEEEEEEET\n"));
    Serial.println(F("***********************************************\n"));
    Serial.println("");    
    
    unsigned long stab_start_t = millis();
    Serial.print(F("Waiting for MPU to stabilise for "));
    Serial.print(MPU_STAB_TIME);
    Serial.println(F(" ms"));
    while(millis() - stab_start_t < MPU_STAB_TIME) {
        mpu.resetFIFO();
        mpuGetData();
        vectGet(&v);
    }
    Serial.println(F("Start YEEEEEEEEEEEEEEET\n"));
    Serial.println(F("***********************************************\n"));
    runtimeStart = millis();
    runtimeSec = millis();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

#define DEG 1
#define RAD 2
#define MM  1
#define CM  2
#define ME  3
#define INC 4
#define FT  5

bool laserON = false;
vector *vpp=&vp;
vector *vpc=&vc;
bool firstCalc = true;
long radToDeg = 360/(2*PI);
char lenMode = MM;
char angMode = DEG;
unsigned long tot = 0;

void loop() {    
    if(millis() - runtimeSec > 10) {
        runtimeSec = millis();
        mpuGetData();
        vectGet(&v);
        vectPrint(v); 
    }
    lcdScan();
    lcdGetCmd();
    if(lcdCmd==START && lcdValidCmd) {
        lcdCmd=START;
        Serial.println("MEASURE");

        vector *vtmp;
        mySerial.write('D');
        lasGetB( TO, buf, BUFSIZE );
        vectGet(vpc);
        vectPrint(*vpc);

        /*
         * tft.setTextSize(2);
    tft.setCursor(out_tot_x,out_tot_y);
    tft.print("TOT D:");
    tft.setCursor(out_p2p_x,out_p2p_y);
    tft.print("P2P D:");
    tft.setCursor(out_l2p_x,out_l2p_y);
    tft.print("L2P D:");
    tft.setCursor(out_ang_x,out_ang_y);
    tft.print("ANGLE:");
         */
        
        Serial.print(buf);
        lasGetB( TO, buf, BUFSIZE );
        sbuf = String(findDigit(buf));
        dc = sbuf.toDouble() + d_offset;

        Serial.println("Point set");
        Serial.print(" Distance is ");
        Serial.println(dc,4);
        Serial.println(sbuf);
        if(laserON) {
            mySerial.write('O');
        }

        //calculate required values
        //calculate cos theta
        cosC = vectDP(vc, vp) / (vectMag(vc) * vectMag(vp));
        //use cosine rule
        d = sqrt(dc*dc + dp*dp - 2*dc*dp*cosC);
        Serial.print("Distance between two points is ");
        Serial.print(d,4);
        Serial.println("m");
        Serial.print("Angle between two points is ");
        Serial.print(acos(cosC)*radToDeg,8);
        Serial.println(" degrees");

        Serial.println("Complete");

        //if error from laser
        if(dc==0) {
            
        }
        else {
            //display l2p and ang
            
            
            //maybe display p2p and tot
            if(!firstCalc) {
                tot+=d;
                
            }

            firstCalc = false;
            //swap the vectors
            vtmp = vpp;
            vpp = vpc;
            vpc = vtmp;
            dp = dc;
        }           
    }
    else if(lcdCmd==LASER && lcdValidCmd) {
        lcdCmd==LASER;
        Serial.println("LASER");

        if(laserON) {
            mySerial.write('C');
            laserON=false;
            lasGetB( TO, buf, BUFSIZE );  
            Serial.print(buf);
        }
        else {
            mySerial.write('O');
            laserON=true;
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        }
    }
    else if(lcdCmd==RESET && lcdValidCmd) {
        lcdCmd=RESET;
        Serial.println("RESET");

        firstCalc = false;
        tot=0;
    }
    else if(lcdCmd==UNIT_ANG && lcdValidCmd) {
        lcdCmd=UNIT_ANG;
        Serial.println("UNIT_ANG");

    }
    else if(lcdCmd==UNIT_LEN && lcdValidCmd) {
        lcdCmd=UNIT_LEN;
        Serial.println("UNIT_LEN");
    }
    else {
        lcdCmd=NONE;
        //Serial.println("NONE");
    }
    
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
        case 'M' :
            mySerial.write(cmd);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'G' :
            showFifoReset *= -1;
            Serial.println("Toggled Fifo reset display");                
        break;
        case 'I' :
#ifdef DEBUG_MPU
            Serial.println("Command I");
#endif
            //mpu.resetFIFO();delay(5);;
            mpuGetData();
            vectGet(&v);
            if(showVect == 1) vectPrint(v);                
        break;
    }
    cmd = 'I';
//  cmd = 'A'; 
}
