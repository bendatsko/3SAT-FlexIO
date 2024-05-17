//======= DOCUMENTATION =======
//
//# FlexIO Scan Chain Example
//
//## Overview
//
//        This Arduino sketch demonstrates how to use the FlexIO peripheral on supported microcontrollers to implement a scan chain. A scan chain is a shift register where each bit can be individually set or toggled. This example shows how to shift data through the chain, capture test results, and update the chain status.
//
//## Key Features
//
//- **Modular Design**: The code is modular, allowing easy addition of more parallel busses.
//- **Configurable Speed and Data**: Speed and test patterns can be easily modified.
//- **Readable and Maintainable**: Clear, well-commented, and structured code.
//
//## Components
//
//- **FlexIOSPI**: Used to handle the FlexIO interface.
//- **Scan Chain**: An array representing the scan chain bits.
//- **Test Pattern**: A default pattern for testing the scan chain.
//
//## How to Use
//
//1. **Initialization**:
//- Set up the scan chain parameters and initialize the FlexIOSPI object.
//- Configure the necessary pins.
//
//2. **Running the Test Pattern**:
//- The `runTestPattern` function is used to shift a pattern through the scan chain and capture test results.
//- This function can be called in the `loop` with the desired speed and pattern.
//
//3. **Adding More Parallel Busses**:
//- Define additional FlexIOSPI objects for each new bus.
//- Modify the `runTestPattern` function to handle multiple busses.
//
//## Example Setup
//
//- **SCAN_CHAIN_LENGTH**: 8
//- **IMF_SCAN_IN0**: 10
//- **IMF_SCAN_OUT0**: 35
//- **IMF_SCAN_WRITE_EN_DIE0**: 16
//- **IMF_SCAN_CLK_OUT**: 34
//- **DEFAULT_SPEED**: 1 MHz
//
//## Functions
//
//- `void shiftScanChain(bool scanIn)`: Shifts data into the scan chain.
//- `void captureTestResults()`: Captures the combinatorial test results.
//- `void updateScanChain()`: Updates the scan chain status and prints it to Serial.
//- `void runTestPattern(const bool* pattern, int length, uint32_t speed)`: Runs the test pattern through the scan chain.
//
//======= END DOCUMENTATION =======

#include <Arduino.h>
#include <FlexIO_t4.h>
#include <FlexIOSPI.h>

// Scan Chain Configuration
const int SCAN_CHAIN_LENGTH = 8;
const int SCAN_IN_PIN = 10;
const int SCAN_OUT_PIN = 35;
const int SCAN_ENABLE_PIN = 16;
const int CLOCK_PIN = 34;
const uint32_t DEFAULT_SPEED = 1000000; // 1 MHz

// FlexIOSPI object (using MISO as SCAN_IN, SCK as CLOCK)
FlexIOSPI flexIO(CLOCK_PIN, SCAN_IN_PIN, SCAN_OUT_PIN, -1);

// Scan chain and test pattern
bool scanChain[SCAN_CHAIN_LENGTH] = {0};
const bool defaultTestPattern[SCAN_CHAIN_LENGTH] = {0, 0, 0, 0, 1, 1, 1, 0};

// Function prototypes
void shiftScanChain(bool scanIn);
void captureTestResults();
void updateScanChain();
void runTestPattern(const bool* pattern, int length, uint32_t speed);

// Function to shift data into the scan chain
void shiftScanChain(bool scanIn) {
    for (int i = SCAN_CHAIN_LENGTH - 1; i > 0; i--) {
        scanChain[i] = scanChain[i - 1];
    }
    scanChain[0] = scanIn;
}

// Function to capture the combinatorial test results
void captureTestResults() {
    for (int i = 0; i < SCAN_CHAIN_LENGTH; i++) {
        scanChain[i] = !scanChain[i];
    }
}

// Update the scan chain status
void updateScanChain() {
    digitalWrite(SCAN_OUT_PIN, scanChain[SCAN_CHAIN_LENGTH - 1]);
    Serial.print("Scan Chain Status: ");
    for (int i = 0; i < SCAN_CHAIN_LENGTH; i++) {
        Serial.print(scanChain[i] ? "1" : "0");
    }
    Serial.println();
}

// Run test pattern
void runTestPattern(const bool* pattern, int length, uint32_t speed) {
    flexIO.beginTransaction(FlexIOSPISettings(speed, MSBFIRST, SPI_MODE0));
    static int patternIndex = 0;

    if (patternIndex < length) {
        shiftScanChain(pattern[patternIndex]);
        patternIndex++;
    } else {
        captureTestResults();
        patternIndex = 0;
    }

    updateScanChain();
    flexIO.endTransaction();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);
    delay(500);

    if (!flexIO.begin()) {
        Serial.println("FlexIO Begin Failed");
        while (1);
    }

    pinMode(SCAN_ENABLE_PIN, OUTPUT);
    digitalWrite(SCAN_ENABLE_PIN, HIGH); // Enable scan mode
    pinMode(SCAN_OUT_PIN, OUTPUT);

    Serial.println("Setup Complete");
}

void loop() {
    runTestPattern(defaultTestPattern, SCAN_CHAIN_LENGTH, DEFAULT_SPEED);
    delay(500); // Adjust delay as needed
}
