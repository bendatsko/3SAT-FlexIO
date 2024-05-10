#include <Arduino.h>
#include <FlexIO_t4.h>
#include <FlexIOSPI.h>

// Define FlexIO scan chain parameters
#define SCAN_CHAIN_LENGTH 8
#define SCAN_IN_PIN 4
#define SCAN_OUT_PIN 5
#define SCAN_ENABLE_PIN 3
#define CLOCK_PIN 2

// Known pattern for testing
const bool testPattern[SCAN_CHAIN_LENGTH] = {0, 0, 0, 0, 1, 1, 1, 0};
bool scanChain[SCAN_CHAIN_LENGTH] = {0};

// FlexIOSPI object (using MISO as SCAN_IN, SCK as CLOCK)
FlexIOSPI flexIO(CLOCK_PIN, SCAN_IN_PIN, SCAN_OUT_PIN, -1);

// Function to shift data into the scan chain
void shiftScanChain(bool scanIn)
{
  for (int i = SCAN_CHAIN_LENGTH - 1; i > 0; i--)
  {
    scanChain[i] = scanChain[i - 1];
  }
  scanChain[0] = scanIn;
}

// Function to capture the combinatorial test results
void captureTestResults()
{
  // Simulate by toggling the state of the scan chain
  for (int i = 0; i < SCAN_CHAIN_LENGTH; i++)
  {
    scanChain[i] = !scanChain[i];
  }
}

// Update the scan chain status
void updateScanChain()
{
  digitalWrite(SCAN_OUT_PIN, scanChain[SCAN_CHAIN_LENGTH - 1]);
  Serial.print("Scan Chain Status: ");
  for (int i = 0; i < SCAN_CHAIN_LENGTH; i++)
  {
    Serial.print(scanChain[i] ? "1" : "0");
  }
  Serial.println();
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

  Serial.printf("Updated Flex IO speed: %u\n", flexIO.flexIOHandler()->computeClockRate());
  Serial.println("End Setup");
}

void loop()
{
  static int patternIndex = 0;

  // Set FlexIOSPI speed to 10 MHz
  flexIO.beginTransaction(FlexIOSPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Shift pattern through the scan chain
  if (patternIndex < SCAN_CHAIN_LENGTH)
  {
    shiftScanChain(testPattern[patternIndex]);
    patternIndex++;
  }
  else
  {
    captureTestResults();
    patternIndex = 0; // Restart pattern
  }

  // Output status update
  updateScanChain();

  flexIO.endTransaction();
  delay(500); // Adjust delay as needed
}
