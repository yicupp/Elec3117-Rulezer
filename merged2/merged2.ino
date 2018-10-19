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
    double x=0;
    double y=0;
    double z=0;
};

double d = 0;
double d_offset = 0.065; //offset of 30mm => the wall between mic and rest of the board

#define lcdVectx1   5
#define lcdVecty1   220
#define lcdVectx2   220
#define lcdVecty2   230
#define lcdVectlx   lcdVectx2-lcdVectx1
#define lcdVectly   lcdVecty2-lcdVecty1
#define lcdVectcw   WHITE
#define lcdVectcb   BLACK

char vectPrintBuf[50];
char vxStr[10]={'\0'};
char vyStr[10]={'\0'};
char vzStr[10]={'\0'};

//LCD defines
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
#define     disturbStr_y  BOXSIZE+BOXSIZE*1/6-15
#define     disturbBar_x  BOXSIZE*3/2
#define     disturbBar_y  BOXSIZE-15
#define     disturbBar_lx BOXSIZE*7/2
#define     disturbBar_ly BOXSIZE/2


#define     out_tot_x     BOXSIZE/8
#define     out_tot_y     75
#define     out_tot_ux    90
#define     out_tot_uy    75

#define     out_p2p_x     BOXSIZE/8
#define     out_p2p_y     105
#define     out_p2p_ux    90
#define     out_p2p_uy    105
   
#define     out_l2p_x     BOXSIZE/8
#define     out_l2p_y     135
#define     out_l2p_ux    90
#define     out_l2p_uy    135

#define     out_ang_x     BOXSIZE/8
#define     out_ang_y     165
#define     out_ang_ux    90
#define     out_ang_uy    165

#define     out_str_x     90

#define     status_x      BOXSIZE/8
#define     status_y      195

#define     VLEN          20
int vi = 0;
//vector vv[VLEN+1];



void statusPrint(char* myStr,int MYCOLOUR) {
    tft.setTextSize(3);
    tft.setCursor(status_x,status_y);
    tft.setTextColor(MYCOLOUR,BLACK);
    tft.print(myStr);
}

void vectPrint(vector v) {
    tft.setCursor(lcdVectx1,lcdVecty1);

    tft.setTextSize(1);

    dtostrf(v.x,5,4,vxStr);
    dtostrf(v.y,5,4,vyStr);
    dtostrf(v.z,5,4,vzStr);

    tft.setTextColor(lcdVectcw,lcdVectcb);
    sprintf(vectPrintBuf,"x=%s y=%s z=%s   ",vxStr,vyStr,vzStr);
    tft.print(vectPrintBuf);
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

//for average finding
double vxDiff=0;
double vyDiff=0;
double vzDiff=0;
double vvDiff=0;

//Make some globals 
vector vc; //current measurement vector
vector vp; //previous measurement vector
vector v;  //current mpu vector
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

double vv[VLEN+1][3]; 

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int xAng, yAng, zAng;
int minVal=265;
int maxVal=402;

<<<<<<< HEAD
void mpuGetData() {
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_addr,14,true);
        AcX=Wire.read()<<8|Wire.read();
        AcY=Wire.read()<<8|Wire.read();
        AcZ=Wire.read()<<8|Wire.read();
        xAng = map(AcX,minVal,maxVal,-90,90);
        yAng = map(AcY,minVal,maxVal,-90,90);
        zAng = map(AcZ,minVal,maxVal,-90,90);

        delay(5);
        vx=AcX;
        vy=AcY;
        vz=AcZ;

/*Serial.println("acc");
Serial.println(vx);
Serial.println(vy);
Serial.println(vz);*/

=======
>>>>>>> parent of c4bbdfc... removed comments
        //get the average
        //vv[VLEN].x+=(vx-vv[vi].x)/VLEN;
        //vv[VLEN].y+=(vx-vv[vi].y)/VLEN;
        //vv[VLEN].z+=(vz-vv[vi].z)/VLEN;
        //add item into array
        vv[vi][0]=vx;
        vv[vi][1]=vy;
        vv[vi][2]=vz;
        vi++;
        vi%=VLEN;
        //array sorted

        //now find sum of |v-v_avg|        
        //now find the average differences
/*        vxDiff=0;vyDiff=0;vzDiff=0;
        for(int x=0;x<VLEN;x++) {
            vxDiff+=fabs(vv[vi].x-vv[VLEN].x);
            vyDiff+=fabs(vv[vi].y-vv[VLEN].y);
            vzDiff+=fabs(vv[vi].z-vv[VLEN].z);
        }
        //Get root mean of the differences
        vvDiff=pow(pow(vxDiff,2)+pow(vyDiff,2)+pow(vzDiff,2),0.5);
        vvDiff=vxDiff+vyDiff+vzDiff;
        int i = vvDiff*100;
        tft.fillRect(disturbBar_x,disturbBar_y,disturbBar_lx,disturbBar_ly,BLACK);
        tft.fillRect(disturbBar_x,disturbBar_y,vvDiff,disturbBar_ly,RED);
        Serial.println(i);*/

        /*double vmin=vv[0].x,vmax=vv[0].x;
        for(int i=1;i<VLEN;i++) {
            if(vv[i].x>vmax) vmax=vv[i].x;
            if(vv[i].x<vmin) vmin=vv[i].x;
        }
        vv[VLEN].x=vmax-vmin;
        
        vmin=vv[0].y,vmax=vv[0].y;
        for(int i=1;i<VLEN;i++) {
            if(vv[i].y>vmax) vmax=vv[i].y;
            if(vv[i].y<vmin) vmin=vv[i].y;
        }
        vv[VLEN].y=vmax-vmin;
        
        vmin=vv[0].z,vmax=vv[0].z;
        for(int i=1;i<VLEN;i++) {
            if(vv[i].z>vmax) vmax=vv[i].z;
            if(vv[i].z<vmin) vmin=vv[i].z;
        }
        vv[VLEN].z=vmax-vmin;*/

        double  vminX=vv[0][0],vmaxX=vv[0][0],
                vminY=vv[0][1],vmaxY=vv[0][1],
                vminZ=vv[0][2],vmaxZ=vv[0][2];

        for(int i=1;i<VLEN;i++) {
            if(vv[i][0]>vmaxX) vmaxX=vv[i][0];
            if(vv[i][0]<vminX) vminX=vv[i][0];
            if(vv[i][1]>vmaxY) vmaxY=vv[i][1];
            if(vv[i][1]<vminY) vminY=vv[i][1];
            if(vv[i][2]>vmaxZ) vmaxZ=vv[i][2];
            if(vv[i][2]<vminZ) vminZ=vv[i][2]; 
        }

       // vvDiff=pow(pow(vxDiff,2)+pow(vyDiff,2)+pow(vzDiff,2),0.5);
        //vvDiff=vxDiff+vyDiff+vzDiff;
        //int i = 5000*(fabs(vv[VLEN].x)+fabs(vv[VLEN].y)+fabs(vv[VLEN].z));
        int i = sqrt(pow(vmaxZ-vminZ,2)+pow(vmaxY-vminY,2)+pow(vmaxX-vminX,2))/60;
        if(i>disturbBar_lx)i=disturbBar_lx;
        
        //i = log(i+1);
        int barCol = GREEN;
        if(i>75) {
            barCol=RED;
        }
        else if(i>25) {
            barCol=YELLOW;
        }
        tft.fillRect(disturbBar_x,disturbBar_y,disturbBar_lx,disturbBar_ly,BLACK);
        tft.fillRect(disturbBar_x,disturbBar_y,i,disturbBar_ly,barCol);
        Serial.println(i);
        
//            Serial.print("xyz\t");
//            Serial.print(vx);
//            Serial.print("\t");
//            Serial.print(vy);
//            Serial.print("\t");
//            Serial.println(vz);
        
        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);    
#ifdef MPU_DEBUG
        //Serial.println("Get data stop");
#endif
}

// LCD and touch functions
// 
//

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
        /*tft.print("("); tft.print(p.x);
        tft.print(", "); tft.print(p.y);
        tft.print(", "); tft.print(p.z);
        tft.println(")   ");*/
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



void lcdGetCmd() {
    //lcdDB = millis()-softDbounceT>SOFT_DBOUNCE_TIME;
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
        if(p.x>=startBut_x1 && p.x<=startBut_x2 && p.y>=startBut_y1 && p.y <= startBut_y2) {
            /*if(lcdCmd != START) {
                lcdCmd=START;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                lcdCmd=START;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }*/
            lcdCmd=START;
        }
        else if(p.x>=setBut_x1 && p.x<=setBut_x2 && p.y>=setBut_y1 && p.y <= setBut_y2){
            /*if(lcdCmd != LASER) {
                lcdCmd=LASER;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                lcdCmd=LASER;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }*/
            lcdCmd=LASER;
        }
        else if(p.x>=resetBut_x1 && p.x<=resetBut_x2 && p.y>=resetBut_y1 && p.y <= resetBut_y2){
            /*if(lcdCmd != RESET) {
                lcdCmd=RESET;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                lcdCmd=RESET;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }*/
            lcdCmd=RESET;
        }
        else if(p.x>=unitBut_x1 && p.x<=unitBut_x2 && p.y>=unitBut_y1 && p.y <= unitBut_y2){
            /*if((lcdCmd != UNIT_ANG || lcdCmd != UNIT_LEN)) {
                if(p.x<unitBut_div) lcdCmd=UNIT_LEN;
                else lcdCmd=UNIT_ANG;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }
            else if(lcdDB) {
                if(p.x<unitBut_div) lcdCmd=UNIT_LEN;
                else lcdCmd=UNIT_ANG;
                //softDbounceT=millis();
                lcdValidCmd = true;
            }*/
            if(p.x<unitBut_div) lcdCmd=UNIT_LEN;
            else lcdCmd=UNIT_ANG;
        }
    }
    /*else {
        if(lcdDB) {
            lcdCmd=NONE;
        }
    }*/
}

void lcd_init() {
    //Serial.println(F("Initialising LCD FUNCTION"));    
    tft.begin(0x8230);

    tft.fillScreen(BLACK);
    tft.setRotation(1);
    tft.setTextColor(GREEN);
    tft.setTextSize(3);
    tft.println("Rulezer!");

    statusPrint("YEETING   ",YELLOW);

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
    //tft.fillRect(disturbBar_x,disturbBar_y,disturbBar_lx,disturbBar_ly,RED);

    tft.setTextSize(2);
    tft.setCursor(out_tot_x,out_tot_y);
    tft.print("Total :");
    tft.setCursor(out_p2p_x,out_p2p_y);
    tft.print("P to P:");
    tft.setCursor(out_l2p_x,out_l2p_y);
    tft.print("L to P:");
    tft.setCursor(out_ang_x,out_ang_y);
    tft.print("ANGLE :");

    statusPrint("READY    ",GREEN);
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

    // initialize serial communication
    Serial.begin(19200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    //Init MPU6050
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    //Serial.println(F("Initialising LCD"));
    lcd_init();

<<<<<<< HEAD
=======
    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    mpu.testConnection();
    // wait for ready
/*    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus);
//        Serial.println(F(")"));
    }

>>>>>>> parent of c4bbdfc... removed comments
    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);

    //Init serial communication
 
//    Serial.begin(19200);     // communication with the host computer
    
    // Start the software serial for communication with the ESP8266
    mySerial.begin(19200);  

    mySerial.write('C');
    lasGetB( TO, buf, BUFSIZE ); 
    delay(500); 
    statusPrint("READY    ",GREEN);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

#define DEG 0
#define RAD 1
#define ME  0
#define CM  1
#define MM  2
#define INC 3
#define FT  4

#define DEG_SCALE 180/PI
#define RAD_SCALE 1

bool laserON = false;
vector *vpp=&vp;
vector *vpc=&vc;
int numCalc = 0;
long radToDeg = 360/(2*PI);
char lenMode = MM;
char angMode = DEG;
double angScale = DEG_SCALE;
double angVal = 0;
//radToDeg
double tot = 0;
double dScale = 1;
int dMode = ME;

#define M2CM  0.1
#define M2MM  0.01
#define M2INC  39.37007874
#define M2FT  M2INC/12
bool resetFlag = true;

char dModeStr[5][3] = {"M", "CM", "MM", "IN", "FT"};
char angModeStr[2][4] = {"DEG","RAD"};

int printLengths(int colour) {
    if(resetFlag)return 1;
    tft.setTextSize(2);
    tft.fillRect(out_tot_x+out_str_x,out_tot_y,120,75 , BLACK);
    tft.setTextColor(colour,BLACK);
    tft.setCursor(out_l2p_x+out_str_x,out_l2p_y);
    tft.print(dc*dScale,3);
    tft.setCursor(out_l2p_ux+out_str_x,out_l2p_uy);
    tft.print(dModeStr[dMode]);
            
    //maybe display p2p, tot 
    if(numCalc>1) {
        tft.setCursor(out_tot_x+out_str_x,out_tot_y);
        tft.print(tot*dScale,3);
        tft.setCursor(out_tot_ux+out_str_x,out_tot_uy);
        tft.print(dModeStr[dMode]);
        
        tft.setCursor(out_p2p_x+out_str_x,out_p2p_y);
        tft.print(d*dScale,3);
        tft.setCursor(out_p2p_ux+out_str_x,out_p2p_uy);
        tft.print(dModeStr[dMode]);
    }
    return 0;
}

void printAngle(int colour) {
    if(numCalc>1) {
        tft.setTextSize(2);
        tft.setTextColor(colour,BLACK);
        tft.fillRect(out_ang_x+out_str_x,out_ang_y,120,16,BLACK);
        tft.setCursor(out_ang_x+out_str_x,out_ang_y);
        tft.print(angVal*angScale,3);  
        tft.setCursor(out_ang_ux+out_str_x,out_ang_uy);
        tft.print(angModeStr[angMode]); 
    }
}

void loop() {    
    /*if(millis() - runtimeSec > 10) {
        runtimeSec = millis();
        mpuGetData();
        vectGet(&v);
        vectPrint(v); 
    }*/
    lcdScan();
    lcdGetCmd();
    if(lcdCmd==START) {
        lcdCmd=START;
//        Serial.println("MEASURE");
        statusPrint("MEASURING  ",YELLOW);

        vector *vtmp;
        mySerial.write('D');
        lasGetB( TO, buf, BUFSIZE );
        vectGet(vpc);
        vectPrint(*vpc);

        
//        Serial.print(buf);
        lasGetB( TO, buf, BUFSIZE );
        sbuf = String(findDigit(buf));
        dc = sbuf.toDouble() + d_offset;

//        Serial.println("Point set");
//        Serial.print(" Distance is ");
//        Serial.println(dc,4);
//        Serial.println(sbuf);
        if(laserON) {
            mySerial.write('O');
        }

        //calculate required values
        //calculate cos theta
        cosC = vectDP(vc, vp) / (vectMag(vc) * vectMag(vp));
        //use cosine rule
        d = sqrt(dc*dc + dp*dp - 2*dc*dp*cosC);
//        Serial.print("Distance between two points is ");
//        Serial.print(d,4);
//        Serial.println("m");
//        Serial.print("Angle between two points is ");
//        Serial.print(acos(cosC)*radToDeg,8);
//        Serial.println(" degrees");

//        Serial.println("Complete");

        //if error from laser
        if(dc==0) {
            statusPrint("LASER ERROR!  ",RED);
        }
        else {      
            resetFlag=false;
            numCalc++;
                        
            //maybe display p2p, tot and ang
            if(numCalc>1) {
                angVal = acos(cosC);
                printAngle(WHITE);
                tot=tot+d;
            }
            printLengths(WHITE); 
            //swap the vectors
            vtmp = vpp;
            vpp = vpc;
            vpc = vtmp;
            dp = dc;
            statusPrint("MEASURED  ",GREEN);
        }           
    }
    else if(lcdCmd==LASER) {
        lcdCmd==LASER;
        //Serial.println("LASER");

        if(laserON) {
            mySerial.write('C');
            laserON=false;
            lasGetB( TO, buf, BUFSIZE );  
            //Serial.print(buf);
            statusPrint("LASER OFF ",BLUE);
            delay(500);
        }
        else {
            mySerial.write('O');
            laserON=true;
            lasGetB( TO, buf, BUFSIZE );
            //Serial.print(buf);
            statusPrint("LASER ON  ",BLUE);
            delay(300);
        }
    }
    else if(lcdCmd==RESET) {
        lcdCmd=RESET;
        //Serial.println("RESET");

        statusPrint("RESET!    ",BLUE);
        resetFlag=true;
        numCalc=0;
        tot=0;
        angVal=0;
        dc=0;
        tft.setTextSize(2);
        tft.setTextColor(WHITE,BLACK);
        /*tft.setCursor(out_l2p_x+out_str_x,out_l2p_y);
        tft.print("          ");
        tft.setCursor(out_tot_x+out_str_x,out_tot_y);
        tft.print("          ");
        tft.setCursor(out_p2p_x+out_str_x,out_p2p_y);
        tft.print("          ");
        tft.setCursor(out_ang_x+out_str_x,out_ang_y);
        tft.print("          ");*/
        tft.fillRect(out_tot_x+out_str_x,out_tot_y,120,105,BLACK);
    }
    else if(lcdCmd==UNIT_ANG) {
        lcdCmd=UNIT_ANG;
        //Serial.println("UNIT_ANG");

        if(angMode==DEG) {
            angMode = RAD;
            statusPrint("USING RAD",CYAN);
            angScale = 1;
        }
        else {
            angMode = DEG;
            statusPrint("USING DEG",CYAN);
            angScale = radToDeg;
        }
        printAngle(CYAN);    
        delay(300);
    }
    else if(lcdCmd==UNIT_LEN) {
        lcdCmd=UNIT_LEN;
        //Serial.println("UNIT_LEN");
        //get new mode
        dMode++;
        dMode%=5;
        switch(dMode) {
            
            case ME:
            statusPrint("USING M  ",BLUE);
            dScale=1;
            break;
            
            case CM:
            dScale=M2CM;
            statusPrint("USING CM  ",BLUE);
            break;
            
            case MM:

            dScale=M2MM;
            statusPrint("USING MM  ",BLUE);
            break;
            
            case INC:

            dScale=M2INC;
            statusPrint("USING INCH",BLUE);
            break;
            
            case FT:

            dScale=M2FT;
            statusPrint("USING FEET ",BLUE);
            break;
        }
        
        printLengths(BLUE);
        delay(300);
    }
    else {
        lcdCmd=NONE;
        //Serial.println("NONE");
    }
    
    
    mpuGetData();
    vectGet(&v);
}
