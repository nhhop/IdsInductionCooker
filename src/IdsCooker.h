#ifndef IdsCooker_h
#define IdsCooker_h

#include <Arduino.h>

// sendCommand() hands the whole frame to the RMT peripheral and returns,
// instead of clocking 33 bits out with delayMicroseconds(). The software
// path blocks the caller for ~139 ms per frame, twice a second.
//
// This uses the arduino-esp32 2.x RMT HAL. Core 3 ships an incompatible API
// and the ESP8266 has no RMT at all; both keep the software timing, which
// stays in place unchanged as the fallback.
#if defined(ARDUINO_ARCH_ESP32) && defined(ESP_ARDUINO_VERSION_MAJOR) && \
    ESP_ARDUINO_VERSION_MAJOR < 3
#define IDS_USE_RMT 1
#include <esp32-hal-rmt.h>
#endif

enum IdsType
{
    IDS1 = 1,
    IDS2 = 2
};

class IdsCooker
{
    private:
        static IdsCooker *staticInduction;

        unsigned long timeTurnedoff;
        unsigned long delayAfteroff = 120000;
        unsigned long lastInterrupt;

        bool inputStarted = false;
        unsigned char inputCurrent = 0;
        unsigned char inputBuffer[33];

        long powerSampletime = 20000;
        unsigned long powerLast;
        long powerHigh = powerSampletime; // Dauer des "HIGH"-Anteils im Schaltzyklus
        long powerLow = 0;
        
        int power = 0;
        int newPower = 0;

        // bool isError = false;
        // unsigned char error = 0;        
        // int powerLevelOnError = 100;   // 100% schaltet das Event handling für Induktion aus
        // int powerLevelBeforeError = 0; // in error event save last power state

        unsigned char CMD_CUR = 0; // Aktueller Befehl

#ifdef IDS_USE_RMT
        // 34 items: [0] carries the 25 ms start pulse plus the 10 ms wait,
        // [1+i] carries bit i plus its trailing 1280 us gap. This buffer must
        // outlive the call and must not be rewritten while a frame is on the
        // wire - rmt_write_items() points at this memory instead of copying it.
        rmt_obj_t *rmtTx = nullptr;
        rmt_data_t rmtItems[34];
        unsigned long txEndMs = 0;
#endif

        bool isRelayon = false; // Systemstatus: ist das Relais in der Platte an?
        bool isInduon = false;  // Systemstatus: ist Power > 0?
        bool isPower = false;
   
        bool isError = false;          // Systemstatus: Fehlermeldung von der Platte?
        int errorCode = 0;                // Ziffer der Fehlermeldung
        int newError = 0;                 // Empfangene Fehlermeldung
        String  errorMessage = "";        // Fehlermeldung String
        String errorMessages[13] = {
        "                ",
        "E0: Kein Topf   ",
        "E1: Stromkreisf.",
        "E2: ??          ",
        "E3: Überhitzung ",
        "E4: Temp.Sens.Fe",
        "E5: ??          ",
        "E6: ??          ",
        "E7: Niederspann.",
        "E8: Überspannung",
        "EC: Komm.Fehler ",
        "ES: Ser. Fehler ",
        "EI: Kein Kochf. "
        };
        // Induktion Signallaufzeiten
        const int SIGNAL_HIGH = 5120;
        const int SIGNAL_HIGH_TOL = 1500;
        const int SIGNAL_LOW = 1280;
        const int SIGNAL_LOW_TOL = 500;
        const int SIGNAL_START = 25;
        const int SIGNAL_START_TOL = 10;
        const int SIGNAL_WAIT = 10;
        const int SIGNAL_WAIT_TOL = 5;

        /*  Binäre Signale für Induktionsplatte */
        int CMD[11][33] = {
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
        {1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0}};   // P10    (IDS1)

        unsigned char PWR_STEPS[11] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};   // Prozentuale Abstufung zwischen den Stufen

        void setupCommands(); 

        static void readInputStatic();
        void readInput();
        
        bool updateRelay();
        void updatePower();
        void updateCommand();
        void sendCommand(int command[33]);

        void millis2wait(const int &value);
        unsigned long BtoI(int start, int numofbits);

        bool updateError();

    public:
        IdsType IDS_TYPE = IdsType::IDS2;

        unsigned char PIN_WHITE = 14;     // NodeMCU D5 = GPIO14 (Relais)
        unsigned char PIN_YELLOW = 12;    // NodeMCU D6 = GPIO12 (Ausgabe an Platte)
        unsigned char PIN_INTERRUPT = 13; // NodeMCU D7 = GPIO13 (Eingabe von Platte)

        IdsCooker(IdsType type);
        IdsCooker(IdsType type, uint8_t white, uint8_t yellow, uint8_t interrupt);
        void Update(const int setpower);
        void Init();

        int         getErrorCode() const;
        const String& getError()   const;  // empty string when no error; valid until next Update() call
};
#endif