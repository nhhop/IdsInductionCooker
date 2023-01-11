#include "IdsCooker.h"

IdsCooker Ids(IdsType::IDS1);

void setup()
{
    // starting serial communication with the baud rate 9600 bits/second
    Serial.begin(9600);

    //init induction cooker
    Ids.Init();
    
    Serial.println("Hello!");
}

void loop() 
{
    while (Serial.available() > 0) 
    {
        // look for the next valid integer in the incoming serial stream:
        int power = Serial.parseInt();
     
        Ids.Update(power);

        // look for the newline. That's the end of your sentence:
        if (Serial.read() == '\n') return;

    }
}