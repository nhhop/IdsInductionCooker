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


/*  Binaere Signale fuer Induktionsplatte. 1 = langer Puls, 0 = kurzer;
    sendCommand() setzt das in SIGNAL_HIGH/SIGNAL_LOW um. */
const int IdsCooker::CMD[11][33] = {
{1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    // Aus    (IDS1 und IDS2)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},    // P1     (IDS1 und IDS2)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},    // P2     (IDS1 und IDS2)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},    // P3     (IDS1 und IDS2)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},    // P4     (IDS1 und IDS2)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0},    // P5     (IDS1 und IDS2)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0},    // P6     (IDS1)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0},    // P7     (IDS1)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},    // P8     (IDS1)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0},    // P9     (IDS1)
{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0}};    // P10    (IDS1)

IdsCooker::IdsCooker(IdsType type)
{
    this->IDS_TYPE = type;
}

IdsCooker::IdsCooker(IdsType type, uint8_t white, uint8_t yellow, uint8_t interrupt)
{
    this->IDS_TYPE      = type;
    this->PIN_WHITE     = white;
    this->PIN_YELLOW    = yellow;
    this->PIN_INTERRUPT = interrupt;
}

IdsCooker::~IdsCooker()
{
    // Order matters. The interrupt has to go first: it carries this
    // instance as its argument, so an edge arriving after the object is
    // gone would write into freed memory.
    detachInterrupt(digitalPinToInterrupt(this->PIN_INTERRUPT));
#ifdef IDS_USE_RMT
    if (this->rmtTx != nullptr)
    {
        rmtDeinit(this->rmtTx);
        this->rmtTx = nullptr;
    }
#endif
    // PIN_YELLOW is deliberately left alone: after rmtDeinit() it stays LOW,
    // which is the resting level the cooker expects.
}

void IdsCooker::Init()
{
  // Relais
  pinMode(PIN_WHITE, OUTPUT);
  digitalWrite(PIN_WHITE, LOW);

  // Rückmeldung der Platte
  pinMode(PIN_INTERRUPT, INPUT_PULLUP);
  /* Mit Instanz-Argument statt ueber einen globalen Zeiger: den hat
     frueher jeder Konstruktor ueberschrieben, sodass eine zweite Instanz
     der ersten die Interrupt-Zustellung stahl - still, denn nur die
     zuletzt angelegte bekam noch Rueckmeldungen. */
  attachInterruptArg(digitalPinToInterrupt(PIN_INTERRUPT), readInputStatic,
                     this, CHANGE);

  // Meldung an Platte
  pinMode(PIN_YELLOW, OUTPUT);
#ifdef IDS_USE_RMT
  this->rmtTx = rmtInit(PIN_YELLOW, RMT_TX_MODE, RMT_MEM_64);
  if (this->rmtTx != nullptr)
  {
    // Mandatory, and mandatory *after* rmtInit(): that leaves clk_div at 1
    // (12.5 ns per tick), at which 25 ms does not fit the 15-bit duration
    // field. One tick per microsecond makes the CMD table usable as-is.
    rmtSetTick(this->rmtTx, 1000.0f);
    // No digitalWrite(HIGH) here: the peripheral drives the pin now and
    // idles it LOW (idle_level = RMT_IDLE_LEVEL_LOW, idle_output_en = true),
    // which is what the protocol wants. readInput() hunts for the start bit
    // on a RISING edge, so a HIGH idle level has no edge to trigger on and
    // would merge with the 25 ms preamble of the first frame.
  }
  else
  {
    /* Kein freier RMT-Kanal - zurueck auf den Software-Pfad, der den
       Aufrufer ~139 ms je Frame blockiert. Das Kanalbudget ist
       board-abhaengig (ESP32 8, ESP32-S2 4, ESP32-S3 4 sendefaehige) und
       begrenzt damit die Zahl gleichzeitiger Platten haerter als der
       Interrupt. Die ueberzaehlige Instanz faellt hier nicht auf, sie
       verlangsamt den ganzen loop(). */
    digitalWrite(PIN_YELLOW, HIGH);
  }
#else
  digitalWrite(PIN_YELLOW, HIGH);
#endif
}

void IdsCooker::Update(const int setpower)
{
  /* updateError() haelt errorCode und errorMessage fuer getError()
     aktuell - mehr nicht. Frueher stand hier ein "if (updateError())
     return;": ab dem ersten Fehlercode wurde der uebergebene Sollwert
     verworfen, Relais und Stufe froren ein, und es ging kein einziger
     Frame mehr raus, auch kein "Aus". Damit erreichte selbst ein
     Not-Aus die Platte nicht mehr, solange ein Fehler anstand.
     SensActCtrls Actuator-Contract verlangt das Gegenteil: wer ueber
     ein Protokoll spricht, muss aktiv Null kommandieren, weil
     Schweigen die Gegenseite weiterlaufen laesst. Ein erzwungenes
     Abschalten gehoert eine Schicht hoeher, wo es konfigurierbar ist. */
  updateError();

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
                    /* Stufenbreite aus der Tabelle, nicht die feste 20:
                       die war IDS2s 20-%-Raster. IDS1 hat 10-%-Stufen und
                       bekam dadurch nur die halbe Low-Zeit - bei 45 %
                       kamen 47,5 % heraus (am Geraet gemessen 2026-09-24:
                       75 % P5 / 25 % P4 statt 50/50). CMD_CUR >= 1 ist hier
                       sicher, die Schleife startet bei i = IDS_TYPE. */
                    const long stepSpan = (long)(this->PWR_STEPS[this->CMD_CUR] -
                                                 this->PWR_STEPS[this->CMD_CUR - 1]) *
                                          this->IDS_TYPE;
                    this->powerLow = this->powerSampletime * (this->PWR_STEPS[this->CMD_CUR]*this->IDS_TYPE - this->power) / stepSpan;
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

void IdsCooker::sendCommand(const int *command)
{
#ifdef IDS_USE_RMT
    if (this->rmtTx != nullptr)
    {
        // A frame is still being clocked out. Two reasons to drop this one
        // rather than queue it: rmt_write_items() takes the channel semaphore
        // with portMAX_DELAY, so a second call would block the caller for the
        // rest of the transmission - exactly what this change removes - and it
        // does not copy the item buffer, so refilling it now would corrupt the
        // frame in flight. At a 500 ms cadence and ~146 ms per frame this
        // cannot trigger; it costs one comparison to keep it that way.
        if ((long)(millis() - this->txEndMs) < 0)
        {
            return;
        }

        this->rmtItems[0].level0    = 1;
        this->rmtItems[0].duration0 = SIGNAL_START * 1000;
        this->rmtItems[0].level1    = 0;
        this->rmtItems[0].duration1 = SIGNAL_WAIT * 1000;

        unsigned long totalUs = (unsigned long)(SIGNAL_START + SIGNAL_WAIT) * 1000;
        for (int i = 0; i < 33; i++)
        {
            const int bitUs = command[i] ? SIGNAL_HIGH : SIGNAL_LOW;
            this->rmtItems[i + 1].level0    = 1;
            this->rmtItems[i + 1].duration0 = bitUs;
            this->rmtItems[i + 1].level1    = 0;
            this->rmtItems[i + 1].duration1 = SIGNAL_LOW;
            totalUs += (unsigned long)bitUs + SIGNAL_LOW;
        }

        this->txEndMs = millis() + (totalUs / 1000) + 1;
        rmtWrite(this->rmtTx, this->rmtItems, 34);
        return;
    }
#endif
    digitalWrite(this->PIN_YELLOW, HIGH);
    this->millis2wait(SIGNAL_START);
    digitalWrite(this->PIN_YELLOW, LOW);
    this->millis2wait(SIGNAL_WAIT);

    for (int i = 0; i < 33; i++)
    {
        digitalWrite(this->PIN_YELLOW, HIGH);
        delayMicroseconds(command[i] ? SIGNAL_HIGH : SIGNAL_LOW);
        digitalWrite(this->PIN_YELLOW, LOW);
        delayMicroseconds(SIGNAL_LOW);
    }   
}

#ifdef ESP8266
void ICACHE_RAM_ATTR IdsCooker::readInputStatic(void *arg)
#else
void IRAM_ATTR IdsCooker::readInputStatic(void *arg)
#endif
{
    static_cast<IdsCooker *>(arg)->readInput();
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
                /* 45 ms statt 35: die Platte sendet Startpulse bis 34,1 ms (am
                   Geraet gemessen 2026-09-23), das liess nur 0,9 ms Reserve. Ihre
                   Frames kommen lueckenlos, es gibt also kein HIGH-Intervall
                   zwischen 35 und 45 ms, das faelschlich als Start durchginge.
                   Verlorene Frames waeren unsichtbar - errorCode bliebe 0. */
                if (signalTime < 45000L && signalTime > 15000L)
                {
                    this->inputStarted = true;
                    this->inputCurrent = 0;
                }
            }
            else
            { // Hat Begonnen. Nehme auf.
                if (signalTime < (SIGNAL_HIGH + SIGNAL_HIGH_TOL) && signalTime > (SIGNAL_HIGH - SIGNAL_HIGH_TOL))
                {
                    // HIGH BIT erkannt
                    this->inputBuffer[this->inputCurrent] = 1;
                    this->inputCurrent += 1;
                }
                else if (signalTime < (SIGNAL_LOW + SIGNAL_LOW_TOL) && signalTime > (SIGNAL_LOW - SIGNAL_LOW_TOL))
                {
                    // LOW BIT erkannt
                    this->inputBuffer[this->inputCurrent] = 0;
                    this->inputCurrent += 1;
                }
                else
                { /* Weder HIGH noch LOW: der Frame ist aus dem Tritt (Stoerung,
                     oder ein Startpuls). Verwerfen und neu synchronisieren. Ohne
                     diesen Zweig bliebe inputStarted fuer immer true, sobald ein
                     einziger Puls danebenliegt - es kaeme nie wieder eine
                     Rueckmeldung an. */
                  this->inputCurrent = 0;
                  this->inputStarted = false;
                  return;
                }

                if (this->inputCurrent >= 33)
                { /* 33 Bits vollstaendig. Hier stand "< 34", womit erst ein 34.
                     Bit die Auswertung ausloeste - das die Platte nie sendet. Der
                     Fehlercode kam dadurch einen Frame zu spaet, und
                     inputBuffer[33] wurde ein Byte hinter dem Array beschrieben. */

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

void IdsCooker::updateError()
{
  if (newError != errorCode) 
  {        // Hat sich geändert?!
      
    errorCode = newError;
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
          break;
        case 2:
          errorMessage = errorMessages[1];    // Kein Topf
          break;
        default:
          /* String("...") + int, nicht "..." + int: letzteres ist
             Zeigerarithmetik auf dem Literal, keine Konkatenation. Bei
             errorCode 1 erschien dadurch "ehler: " (am Geraet gesehen
             2026-09-23), ab 9 zeigte der Zeiger hinter das Literal. */
          errorMessage = String("Fehler ") + errorCode;    // Unbekannt
       }

  }
}

int IdsCooker::getErrorCode() const {
    return errorCode;
}

const String& IdsCooker::getError() const {
    return errorMessage;
}