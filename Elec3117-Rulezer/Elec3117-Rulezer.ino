//LIDAR values

#include <SoftwareSerial.h>

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


//Make some globals 
vector v = {0,0,0};
vector u = {0,0,0};

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
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

int prev = 0;

void loop() 
{
    
    // listen for communication from the ESP8266 and then write it to the serial monitor
    if ( mySerial.available() )   
    { 
        prev = 0;
        Serial.write( mySerial.read() );  
    }
    else 
    {
        if(prev > 1000) 
        {
            Serial.println("");
            prev = -1;
        }
        if(prev != -1) {
            prev ++;
        }
    }
 
    // listen for user input and send it to the ESP8266
    if ( Serial.available() )       {  mySerial.write( Serial.read() );  }

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

    x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
    Serial.print("AngleX= ");
    Serial.println(x);

    Serial.print("AngleY= ");
    Serial.println(y);

    Serial.print("AngleZ= ");
    Serial.println(z);
    Serial.println("-----------------------------------------");
}
 
