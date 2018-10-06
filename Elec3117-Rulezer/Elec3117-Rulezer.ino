//LIDAR values

#include <SoftwareSerial.h>

#define T0  1000000000
#define T1  1000000000
#define TO  5000000
#define BUFSIZE 50
#define ACC_RANGE 16384

SoftwareSerial mySerial(10,11);  //RX,TX
int LEDPIN = 13;

//MPU
//Written by Ahmet Burkay KIRNIK
//TR_CapaFenLisesi
//Measure Angle with a MPU-6050(GY-521)

#include<Wire.h>

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int xAng, yAng, zAng;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;

struct vector {
    double x;
    double y;
    double z;
};

double d = 0;
double d_offset = 0.00; //offset of 10mm => the circular hole on the top of the lens/laser cylinders

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

//Make some globals 
vector vc = {0,0,0}; //current measurement vector
vector vp = {0,0,0}; //previous measurement vector
double dc = 0;
double dp = 0;
double cosC = 0;

char cmd = 'I';
char buf[BUFSIZE] = {0};
String sbuf;
char *dbuf;

/*
void myprint(const char* str) {
    Serial.print(F(str));
}

void myprintln(const char* str) {
    Serial.print(F(str));
    Serial.println(F("\n"));
}*/

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

void mpuGet(vector *v)
{
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

    x = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

    (*v).x = (double)AcX/ACC_RANGE;
    (*v).y = (double)AcY/ACC_RANGE;
    (*v).z = (double)AcZ/ACC_RANGE;
}

void mpuInit()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);    
}

void mpuPrint()
{
    Serial.println("-----------------------------------------");
    Serial.println("MPU accelerometer data");
    Serial.print("AngleX= ");
    Serial.println(x);

    Serial.print("AngleY= ");
    Serial.println(y);

    Serial.print("AngleZ= ");
    Serial.println(z);
    Serial.println("-----------------------------------------");
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

int lasGet( unsigned int to_start, unsigned int to_finish, char *buff, int buff_size ) {
    int count = 0;
    int charc = 0;

    while( count < to_start && mySerial.available() == 0) {
        count++;
    }
    if(count == to_start) {
        Serial.println("Msg Receive timed out");
        return 1;
    }
    
    while( count < to_finish && charc < buff_size - 1 ) {
        if( mySerial.available() ) {
             //mySerial.read() ;
             buff[charc] = mySerial.read();
             count = 0;
             charc++;
        }
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

void setup() {    
    //Init serial communication
    pinMode(LEDPIN, OUTPUT);
 
    Serial.begin(19200);     // communication with the host computer
    
    // Start the software serial for communication with the ESP8266
    mySerial.begin(19200);  
 
    Serial.println("");
    Serial.println("3117 YEEEEEEEEEEEEEEET\n");
    Serial.println("***********************************************\n");
    Serial.println("");    

    //Init MPU6050
    mpuInit();
}

void loop() 
{
 
    // listen for user input 
    if ( Serial.available() ) 
    {
        cmd = Serial.read();
    }

    switch(cmd) {
        case 'O' : 
            mySerial.write(cmd);
//            lasGet( T0, T1, buf, BUFSIZE );
//            lasGetB( TO, buf, BUFSIZE );
//            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'C' : 
            mySerial.write(cmd);
//            lasGet( T0, T1, buf, BUFSIZE );
//            lasGetB( TO, buf, BUFSIZE );
//            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'S' : 
            mySerial.write(cmd);
//            lasGet( T0, T1, buf, BUFSIZE );
//            lasGetB( TO, buf, BUFSIZE );
//            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'D' : 
            mySerial.write(cmd);
//            lasGet( T0, T1, buf, BUFSIZE );
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'M' :
            mySerial.write(cmd);
//            lasGet( T0, T1, buf, BUFSIZE );
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'F' :
            mySerial.write(cmd);
//            lasGet( T0, T1, buf, BUFSIZE );
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
            lasGetB( TO, buf, BUFSIZE );
            Serial.print(buf);
        break;
        case 'P' :
            mpuGet(&vc);
            mpuPrint();
            mySerial.write('D');
//            lasGet( T0, T1, buf, BUFSIZE );
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
            mpuGet(&vp);
            //mpuPrint();
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
        break;
        case 'V' :
            mpuGet(&vc);
            //mpuPrint();
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
        break;
        case 'A' :
            mpuGet(&vc);
            mpuPrint();
            vectPrint(vc);        
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
 
