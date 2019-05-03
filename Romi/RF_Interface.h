#ifndef _RF_Interface_h
#define _RF_Interface_h

#include "RFID.h"
#include <DigitalIO.h>
 
RFID rfid(SS_PIN, RST_PIN); //create an instance rfid for the class RFID
const int KEY_SIZE = 6;
unsigned char Key[KEY_SIZE];
unsigned long RFID;

/* This is where we define the RFID card lookup table
 *  Make sure to update the values here to match the setup of the map
 */
#define NUM_CARDS 3
float x_poses[NUM_CARDS] = {100, 150, 960};
float y_poses[NUM_CARDS] = {300, 450, 100};
float bearing[NUM_CARDS] = {PI, PI/4, -PI};
unsigned char serials[NUM_CARDS] =  {137, 169, 224};

void setupRFID()
{
  
    rfid.init();

    for (int i=0;i<KEY_SIZE;i++)
    {
        Key[i] = 0xFF;
    }

}

bool checkForRFID()
{

    bool found=false;
    //Serial.println(F("Checking"));
    if (rfid.isCard())
    {
      
        //Serial.println(F("Got a card"));
    
        if (rfid.readCardSerial())
        {
            for (int i=0; i<=4; i++)//card value: "xyz xyz xyz xyz xyz" (15 digits maximum; 5 pairs of xyz)hence 0<=i<=4 //
            {
                //Serial.print(rfid.serNum[i]);
            }
      
            found=true;
            //Serial.println("");
        }
    }

    return found;

}

float serialToXPos(char * serNum)
{
    for (int i=0;i<NUM_CARDS;i++)
    {
      if(serNum[0] == serials[i])
      {
        return x_poses[i];
      }
    }

    Serial.println(F("Error: Card not found"));
    
    return -1;
}

float serialToYPos(char * serNum)
{
    for (int i=0;i<NUM_CARDS;i++)
    {
        if(serNum[0] == serials[i])
        {
          return y_poses[i];
        }
    }

    Serial.println(F("Error: Card not found"));
    
    return -1;
}
float serialToBearing(char * serNum)
{
    for (int i=0;i<NUM_CARDS;i++)
    {
        if(serNum[0] == serials[i])
        {
          return y_poses[i];
        }
    }

    Serial.println(F("Error: Card not found"));
    
    return -1;
}


#endif
