/*
* Neato 2D Object Tracker Code
* Last edited: 4/26/2013
* Vijay Govindarajan and Alfredo Muniz
* Description: This code allows the Neato to be controlled by user.
*/

#include "mbed.h"
#include "MRF24J40.h"

#include <string>

#define MODSERIAL_DEFAULT_RX_BUFFER_SIZE 1024   //One of these is not needed I think
#define MODSERIAL_DEFAULT_TX_BUFFER_SIZE 1024
#include "MODSERIAL.h"  //Normal Serial is limited to 16 byte buffers

MODSERIAL lidar(p9, p10);  // tx, rx
MODSERIAL pc(USBTX, USBRX); // tx, rx

/******************* Neato Communication *********************/
char c;
char s[5000];
int on;
int count;
int num1;
int num2;
int num3;
int num4;
int x1[359];
int x2[359];
int y1[359];
int y2[359];
int j;
int xball1;
int xball2;
int yball1;
int yball2;
int xsum;
int ysum;
int points;
int target;

// RF tranceiver to link with handheld.
MRF24J40 mrf(p11, p12, p13, p14, p21);

// LEDs you can treat these as variables (led2 = 1 will turn led2 on!)
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

// Timer
Timer timer;

// Used for sending and receiving
char txBuffer[128];
char rxBuffer[128];
int rxLen;

//***************** Do not change these methods (please) *****************//

/*
 Receive data from the MRF24J40.

 @param data A pointer to a char array to hold the data
 @param maxLength The max amount of data to read.
*/

int rf_receive(char *data, uint8_t maxLength)
{
    uint8_t len = mrf.Receive((uint8_t *)data, maxLength);
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};

    if(len > 10) {
        //Remove the header and footer of the message
        for(uint8_t i = 0; i < len-2; i++) {
            if(i<8) {
                //Make sure our header is valid first
                if(data[i] != header[i])
                    return 0;
            } else {
                data[i-8] = data[i];
            }
        }

        //pc.printf("Received: %s length:%d\r\n", data, ((int)len)-10);
    }
    return ((int)len)-10;
}

/**
 Send data to another MRF24J40.

 @param data The string to send
 @param maxLength The length of the data to send.
                  If you are sending a null-terminated string you can pass strlen(data)+1
*/

void rf_send(char *data, uint8_t len)
{
    //We need to prepend the message with a valid ZigBee header
    uint8_t header[8]= {1, 8, 0, 0xA1, 0xB2, 0xC3, 0xD4, 0x00};
    uint8_t *send_buf = (uint8_t *) malloc( sizeof(uint8_t) * (len+8) );

    for(uint8_t i = 0; i < len+8; i++) {
        //prepend the 8-byte header
        send_buf[i] = (i<8) ? header[i] : data[i-8];
    }
    //pc.printf("Sent: %s\r\n", send_buf+8);

    mrf.Send(send_buf, len+8);
    free(send_buf);
}

void getldsscan(void)
{
    /******** LIDAR DATA MANIPULATION ********/
    lidar.printf("getldsscan\n");
    //pc.printf("start\r\n");
    for (int i = 0; i < 363; i++) {
        lidar.scanf("%[^\n]%*c",s);
        count = sscanf(s,"%d,%d,%d,%d", &num1, &num2, &num3, &num4);
        if(num2 >= 4000) {
            num2 = 0;
        }
        if(num1 >= 0 && j == 5 && i >= 4) {
            // pc.printf("%d %s\r\n", i, s);
            //pc.printf("%d %d\r\n",num1,num2);
            x1[num1] = num2*cos((double)num1*3.14159/180+3.14159);
            y1[num1] = num2*sin((double)num1*3.14159/180+3.14159);
            //pc.printf("%d %d\r\n", x1[num1], y1[num1]);
        }
        if(num1 >= 0 && j == 6 && i >= 4) {
            // pc.printf("%d %s\r\n", i, s);
            //pc.printf("%d %d\r\n",num1,num2);
            x2[num1] = num2*cos((double)num1*3.14159/180+3.14159);
            y2[num1] = num2*sin((double)num1*3.14159/180+3.14159);
        }
    }

    //if (j <= 0) {
        j++;
    //} else {
      //  j--;
    //}
}

int main (void)
{
    uint8_t channel = 2;

    lidar.baud(115200);
    pc.baud(115200);

    on = 1;
    led4 = 1;
    j = 0;
    points = 0;
    xsum = 0;
    ysum = 0;
    points = 0;
    target = 0;
    xball1 = 0;
    yball1 = 0;
    xball2 = 0;
    yball2 = 0;

    wait(1);
    lidar.printf("testmode on\n");
    pc.printf("testmode on\r\n");
    wait(1);
    lidar.printf("setldsrotation on\n");
    pc.printf("setldsrotation on\r\n");

    //Set the Channel. 0 is default, 15 is max
    mrf.SetChannel(channel);

    //Start the timer
    timer.start();


    while(true) {
        if(on) {
            //Try to receive some data
            rxLen = rf_receive(rxBuffer, 128);
            if(rxLen > 0) {
                //Toggle the Led
                led1 = led1^1;
                pc.printf("Received: %s\r\n", rxBuffer);
                lidar.printf("%s\n", rxBuffer);
            }

            //Send some data every half second
            if(timer.read_ms() >= 500) {
                //Reset the timer to 0
                timer.reset();
                // Toggle LED 2.
                led2 = led2^1;
            }

        }
    }
}



/****** Data to Interpret ******
AngleInDegrees,DistInMM,Intensity,ErrorCodeHEX
0,221,1400,0
1,223,1396,0
2,228,1273,0
(. . .)
359,220,1421,0
*/
