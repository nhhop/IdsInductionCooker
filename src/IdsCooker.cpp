#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...)                                                   \
    DEBUG_ESP_PORT.printf("%s ", timeClient.getFormattedTime().c_str()); \
    DEBUG_ESP_PORT.printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

#include "IdsCooker.h"


void IdsCooker::millis2wait(const int &value)
{
  unsigned long pause = millis();
  while (millis() < pause + value)
  {
    yield(); //wait approx. [period] ms
  }
}

unsigned long IdsCooker::BtoI(int start, int numofbits)
{    //binary array to integer conversion
  unsigned long integer=0;
  unsigned long mask=1;
  for (int i = numofbits+start-1; i >= start; i--)
  {
    if (this->inputBuffer[i]) integer |= mask;
    mask = mask << 1;
  }
  return integer;
} 

IdsCooker *IdsCooker::staticInduction;

IdsCooker::IdsCooker(IdsType type)
{
    staticInduction = this;
    this->IDS_TYPE = type;
}

IdsCooker::IdsCooker(IdsType type, uint8_t white, uint8_t yellow, uint8_t interrupt)
{
    staticInduction = this;
    this->IDS_TYPE      = type;
    this->PIN_WHITE     = white;
    this->PIN_YELLOW    = yellow;
    this->PIN_INTERRUPT = interrupt;
}

void IdsCooker::Init()
{
  // Relais
  pinMode(PIN_WHITE, OUTPUT);
  digitalWrite(PIN_WHITE, LOW);

  // Rückmeldung der Platte
  pinMode(PIN_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), readInputStatic, CHANGE);

  // Meldung an Platte
  pinMode(PIN_YELLOW, OUTPUT);
  digitalWrite(PIN_YELLOW, HIGH);

  this->setupCommands();
}

void IdsCooker::setupCommands()
{
    for (int i = 0; i < 33; i++)
    {
        for (int j = 0; j < 11; j++)
        {
          if (CMD[j][i] == 1)
          {
              CMD[j][i] = SIGNAL_HIGH;
          }
          else
          {
              CMD[j][i] = SIGNAL_LOW;
          }
        }
    }
}

void IdsCooker::Update(const int setpower)
{
  if (updateError())  
    return;
  
  this->newPower = setpower;

  this->updatePower();

  this->isRelayon = updateRelay();

  this->updateCommand();
}

bool IdsCooker::updateRelay()
{
    if (this->isInduon == true && this->isRelayon == false)
    { /* Relais einschalten */
        digitalWrite(this->PIN_WHITE, HIGH);
        return true;
    }

    if (this->isInduon == false && this->isRelayon == true)
    { /* Relais ausschalten */
        if (millis() > this->timeTurnedoff + this->delayAfteroff)
        {
            digitalWrite(this->PIN_WHITE, LOW);
            return false;
        }
    }

    if (this->isInduon == false && this->isRelayon == false)
    { /* Ist aus, bleibt aus. */
        return false;
    }

    return true; /* Ist an, bleibt an. */
}

  // Test 20220903
void IdsCooker::updatePower()
{
    if (this->power != this->newPower) // Neuer Befehl empfangen
    {
        if (this->newPower > 100)
        {
            this->newPower = 100; // Nicht > 100
        }
        if (this->newPower < 0)
        {
            this->newPower = 0; // Nicht < 0
        }
        this->power = this->newPower;

        this->timeTurnedoff = 0;
        this->isInduon = true;

        if (this->power == 0)
        {
            this->CMD_CUR = 0;
            this->timeTurnedoff = millis();
            this->isInduon = false;
            /* Wie lange "HIGH" oder "LOW" */
            this->powerHigh = this->powerSampletime;
            this->powerLow = 0;
        }
        else
        {
            for (int i = this->IDS_TYPE; i < 11; i+= this->IDS_TYPE)
            {
                if (this->power <= this->PWR_STEPS[i])
                {
                    this->CMD_CUR = i/this->IDS_TYPE;
                    /* Wie lange "HIGH" oder "LOW" */
                    this->powerLow = this->powerSampletime * (this->PWR_STEPS[this->CMD_CUR]*this->IDS_TYPE - this->power) / 20L;
                    this->powerHigh = this->powerSampletime - this->powerLow;
                    
                    DEBUG_MSG("P_set: %i% IDS%i -> Stufe:%i (%i%), on:%i/off:%i\n", power, IDS_TYPE, CMD_CUR, PWR_STEPS[CMD_CUR]*IDS_TYPE, powerHigh, powerLow);
                    
                    return;
                }
            }
        }
    }
}

void IdsCooker::updateCommand()
{
  // if (isRelayon == true && isInduon == true) {

  //   unsigned long timeNow = millis();                 /* aktuelle millis() festhalten */
    
  //   if (isPower == true) 
  //   {                            /* Aktuell Hohe Stufe */                         
  //     if (timeNow > powerLast + powerHigh) 
  //     {          /* Prüfen, ob Zeit für Hohe Stufe vorbei */
  //       isPower = false;                              /* Wenn ja, Niedrige Stufe. */
  //       powerLast = millis();                         /* Zeit festhalten. */
  //     } 
  //     else 
  //     {  
  //       sendCommand(CMD[CMD_CUR]);                    /* Befel "Hohe Stufe" sennden. */ 
  //     }
  //   } 
  //   else 
  //   {                                          /* Aktuell niedrige Stufe. */
  //     if (timeNow > powerLast + powerLow) 
  //     {           /* Prüfen, ob Zeit für niedrige Stufe vorbei */
  //       isPower = true;                               /* Wenn ja, hohe Stufe. */
  //       powerLast = millis();                         /* Zeit festhalten. */
  //     } 
  //     else 
  //     {
  //       sendCommand(CMD[(CMD_CUR - 1)]);              /* Befel "Niedrige Stufe" sennden. */     
  //     }
  //   }     
  // } 
  // else 
  // {                                            
  //    isPower = false;
  //    powerLast = 0;
  //    sendCommand(CMD[0]);                             /* Befel "Niedrige Stufe" sennden. */
  // }

    if (this->isInduon && this->power > 0)
    {
        if (millis() > this->powerLast + this->powerSampletime)
        {
            this->powerLast = millis();
        }
        if (millis() > this->powerLast + this->powerHigh)
        {
            this->sendCommand(CMD[CMD_CUR - 1]);
            this->isPower = false;
        }
        else
        {
            this->sendCommand(CMD[CMD_CUR]);
            this->isPower = true;
        }
    }
    else if (this->isRelayon)
    {
        this->sendCommand(CMD[0]);
    }
}

void IdsCooker::sendCommand(int command[33])
{
    digitalWrite(this->PIN_YELLOW, HIGH);
    this->millis2wait(SIGNAL_START);
    digitalWrite(this->PIN_YELLOW, LOW);
    this->millis2wait(SIGNAL_WAIT);

    for (int i = 0; i < 33; i++)
    {
        digitalWrite(this->PIN_YELLOW, HIGH);
        delayMicroseconds(command[i]);
        digitalWrite(this->PIN_YELLOW, LOW);
        delayMicroseconds(SIGNAL_LOW);
    }   
}

#ifdef ESP8266
void ICACHE_RAM_ATTR IdsCooker::readInputStatic()
#else
void IRAM_ATTR IdsCooker::readInputStatic()
#endif
{
    staticInduction->readInput();
}

void IdsCooker::readInput()
{
    // Variablen sichern
    bool ishigh = digitalRead(this->PIN_INTERRUPT);
    unsigned long newInterrupt = micros();
    long signalTime = newInterrupt - this->lastInterrupt;

    // Glitch rausfiltern
    if (signalTime > 10)
    {
        if (ishigh)
        {
            this->lastInterrupt = newInterrupt; // PIN ist auf Rising, Bit senden hat gestartet :)
        }
        else
        { // Bit ist auf Falling, Bit Übertragung fertig. Auswerten.

            if (!this->inputStarted)
            { // suche noch nach StartBit.
                if (signalTime < 35000L && signalTime > 15000L)
                {
                    this->inputStarted = true;
                    this->inputCurrent = 0;
                }
            }
            else
            { // Hat Begonnen. Nehme auf.
                if (this->inputCurrent < 34)
                { // nur bis 33 aufnehmen.
                    if (signalTime < (SIGNAL_HIGH + SIGNAL_HIGH_TOL) && signalTime > (SIGNAL_HIGH - SIGNAL_HIGH_TOL))
                    {
                        // HIGH BIT erkannt
                        this->inputBuffer[this->inputCurrent] = 1;
                        this->inputCurrent += 1;
                    }
                    if (signalTime < (SIGNAL_LOW + SIGNAL_LOW_TOL) && signalTime > (SIGNAL_LOW - SIGNAL_LOW_TOL))
                    {
                        // LOW BIT erkannt
                        this->inputBuffer[this->inputCurrent] = 0;
                        this->inputCurrent += 1;
                    }
                }
                else
                { 
                  // Aufnahme vorbei.

                  /* Auswerten */
                  newError = BtoI(13,4);          // Fehlercode auslesen.
                  DEBUG_MSG("Error: %i:", newError);
                  for(int i=0;i<33; i++)
                  {
                      // char c = char(inputBuffer[i]);
                       DEBUG_MSG(inputBuffer[i]);
                  }
                  DEBUG_MSG("!\n");
                  
                  /* von Vorne */   
                  this->inputCurrent = 0;
                  this->inputStarted = false;
                }
            }
        }
    }
}

bool IdsCooker::updateError()
{
  bool returnValue = false;
  if (newError != errorCode) 
  {        // Hat sich geändert?!
      
    errorCode = newError;
    returnValue = true;                     // Ja, hat sich geändert. 
    /* Fehlermeldung Setzen */
      switch (errorCode) 
      {
        case -2:
          errorMessage = errorMessages[12];   // Kein Induktionskochfeld
          break; 
        case -1:
          errorMessage = errorMessages[11];    // Serielle Kommunikation gestört
          break;
        case 0:
          errorMessage = errorMessages[0];    // Kein Fehler
          returnValue = false;
          break;
        case 2:
          errorMessage = errorMessages[1];    // Kein Topf
          break;
        default:
          errorMessage = "Fehler: " + errorCode;          // Unbekannt
       }

  }
  else 
  {
    if (errorCode != 0) 
    {
      returnValue = true;
    }
    else { returnValue = false; }
  }
  return returnValue;
}

int IdsCooker::getErrorCode() const {
    return errorCode;
}

const char* IdsCooker::getError() const {
    return errorCode != 0 ? errorMessage.c_str() : nullptr;
}