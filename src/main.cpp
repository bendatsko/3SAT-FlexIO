#include <Arduino.h>
#include <FlexIO_t4.h>
#include <FlexIOSPI.h>

// Define FlexIO scan chain parameters
#define SCAN_CHAIN_LENGTH 220
#define BUS_WIDTH 3
#define SCAN_IN_PIN 4
#define SCAN_OUT_PIN 5
#define SCAN_ENABLE_PIN 3
#define CLOCK_PIN 2
#define PHASE_CLOCK_PIN 6
#define RESET_PIN 7

// Known pattern for testing (single pattern for demonstration)
const bool testPattern[SCAN_CHAIN_LENGTH][BUS_WIDTH] = {
    {1, 0, 1}, {0, 1, 0}, {1, 1, 1}, {0, 0, 0}, {1, 0, 0}, {0, 1, 1}, {1, 0, 1}, {0, 0, 1},
    /* Fill remaining 212 elements to complete the pattern */
};
bool scanChain[SCAN_CHAIN_LENGTH][BUS_WIDTH] = {{0, 0, 0}};

// FlexIOSPI object (using MISO as SCAN_IN, SCK as CLOCK)
FlexIOSPI flexIO(CLOCK_PIN, SCAN_IN_PIN, SCAN_OUT_PIN, -1);

// Function to shift and read data simultaneously
void shiftAndRead(bool input[BUS_WIDTH], bool output[BUS_WIDTH])
{
    // Shift data
    for (int i = SCAN_CHAIN_LENGTH - 1; i > 0; i--)
    {
        for (int j = 0; j < BUS_WIDTH; j++)
        {
            scanChain[i][j] = scanChain[i - 1][j];
        }
    }
    for (int j = 0; j < BUS_WIDTH; j++)
    {
        scanChain[0][j] = input[j];
        output[j] = scanChain[SCAN_CHAIN_LENGTH - 1][j];
    }
}

// Function to capture combinatorial test results
void captureTestResults()
{
    for (int i = 0; i < SCAN_CHAIN_LENGTH; i++)
    {
        for (int j = 0; j < BUS_WIDTH; j++)
        {
            scanChain[i][j] = !scanChain[i][j];
        }
    }
}

// Update the scan chain status
void updateScanChain()
{
    Serial.print("Scan Chain Status: ");
    for (int i = 0; i < SCAN_CHAIN_LENGTH; i++)
    {
        for (int j = 0; j < BUS_WIDTH; j++)
        {
            Serial.print(scanChain[i][j] ? "1" : "0");
        }
        Serial.print(" ");
    }
    Serial.println();
}

// Generate the 100 MHz clock signal using FlexIO timer
void initializeClock()
{
    FlexIOHandler *flexIOHandler = &FlexIOHandler::flexIO0;
    flexIOHandler->setClockSettings(3, 1, 0); // Clock settings
    flexIOHandler->setClockDivider(1);       // Divide by 1

    FlexIO_t4::pinConfig(CLOCK_PIN, kFlexIO_PinConfigOutput);
    flexIOHandler->hardware()->TIMCMP[0] = 1; // Set timer comparison value for 100 MHz clock
    flexIOHandler->hardware()->TIMCFG[0] = FLEXIO_TIMCFG_TIMOUT(0) | FLEXIO_TIMCFG_TIMDEC(0) | FLEXIO_TIMCFG_TIMDIS(1) | FLEXIO_TIMCFG_TIMENA(1);
    flexIOHandler->hardware()->TIMCTL[0] = FLEXIO_TIMCTL_TIMOD(1) | FLEXIO_TIMCTL_PINSEL(2) | FLEXIO_TIMCTL_PINCFG(3) | FLEXIO_TIMCTL_TRGSRC(1);

    flexIOHandler->hardware()->SHIFTCFG[0] = FLEXIO_SHIFTCFG_INSRC(0) | FLEXIO_SHIFTCFG_PWIDTH(0);
    flexIOHandler->hardware()->SHIFTCTL[0] = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_TIMPOL(0) | FLEXIO_SHIFTCTL_PINCFG(3);

    flexIOHandler->hardware()->TIMSTAT = 0x01; // Clear timer status
    flexIOHandler->hardware()->SHIFTBUF[0] = 0; // Clear shift buffer

    flexIOHandler->enable();
}

// Synchronize enable signal timing with the clock
void setEnableSignal(bool state)
{
    digitalWrite(SCAN_ENABLE_PIN, state ? HIGH : LOW);
}

// Edge detector function for reading data with phase-shifted clock
bool detectEdge(bool previousState, bool currentState)
{
    return !previousState && currentState;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000)
        ;
    delay(500);

    // Initialize FlexIOSPI
    if (!flexIO.begin())
    {
        Serial.println("FlexIO Begin Failed");
    }

    pinMode(SCAN_ENABLE_PIN, OUTPUT);
    digitalWrite(SCAN_ENABLE_PIN, HIGH); // Enable scan mode

    pinMode(SCAN_OUT_PIN, OUTPUT);
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW); // Reset initially

    pinMode(PHASE_CLOCK_PIN, INPUT);

    // Initialize 100 MHz clock signal
    initializeClock();

    Serial.printf("Flex IO speed: %u\n", flexIO.flexIOHandler()->computeClockRate());
    Serial.println("End Setup");
}

void loop()
{
    static int patternIndex = 0;
    static bool previousPhaseClock = false;
    bool currentPhaseClock = digitalRead(PHASE_CLOCK_PIN);
    bool output[BUS_WIDTH] = {0};

    // Detect phase-shifted clock edge
    if (detectEdge(previousPhaseClock, currentPhaseClock))
    {
        // Set FlexIOSPI speed to 10 MHz
        flexIO.beginTransaction(FlexIOSPISettings(10000000, MSBFIRST, SPI_MODE0));

        // Shift and read pattern through the scan chain
        if (patternIndex < SCAN_CHAIN_LENGTH)
        {
            shiftAndRead(testPattern[patternIndex], output);
            patternIndex++;
        }
        else
        {
            captureTestResults();
            patternIndex = 0; // Restart pattern
        }

        updateScanChain();
        flexIO.endTransaction();
    }

    previousPhaseClock = currentPhaseClock;
    delay(10); // Adjust delay as needed
}
