#include <SPI.h>              // Include SPI Bus library
#include <Wire.h>             // Include Wire library for SPI
#include <Adafruit_ILI9341.h> // Include library for ILI9341 LCD controller
#include <Adafruit_GFX.h>     // Include graphics drwaing library

#define LCD_WIDTH 320         // LCD display width
#define LCD_HEIGHT 240        // LCD display height
#define LCD_DC 9              // LCD register / data selection signal pin
#define LCD_CS 10             // LCD SPI chip select pin (low level enable)  
#define LCD_MOSI 11           // LCD SPI bus write data signal pin
#define LCD_SCK 13            // LCD SPI bus clock signal pin
#define BUILTIN_LED 13        // Built-in LED pin
#define ADC_PIN 0             // ADC pin
#define ADC_BUFFER_SIZE 20000 // ADC sampling buffer size (enough for recording 1 sec datas w/ 20kHz sampling rate)
#define WINDOW_WIDTH 260      // Display waveform window width
#define WINDOW_HEIGHT 200     // Display waveform window height

Adafruit_ILI9341 lcd = Adafruit_ILI9341(LCD_CS, LCD_DC);   // Create lcd object
IntervalTimer adcTriggerTimer;                             // Create timer object for timer triggered ADC sampling

// Set your oscillscope parameters here!!
const unsigned int windowTime = 1000;        // Single waveform window time in milliseconds, do not set below 5ms or above 1000ms
const float triggerLevel = 1.35;            // Trigger voltage, set between 0.00V ~ 3.30V
const int triggerMode = 1;                  // 0: None(Freerun), 1: Rising edge trigger, 2: Falling edge trigger

unsigned int sampleInterval = windowTime * 1000 / WINDOW_WIDTH;   // Note that single analogRead tooks around 17us so interval should be greater than 17us.
unsigned int waveBuffer[ADC_BUFFER_SIZE] = {0};
unsigned int bufferIndex = 0;
unsigned int waveform[WINDOW_WIDTH] = {0};
int triggerFlag = 0;                        //0: Not ready, 1: Ready to be triggered, 2: Triggered, 3: Done sampling
float wavFreq = 0;
float wavAvg = 0;
float wavVpp = 0;

// put your setup code here, to run once:
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);                   // Set Built-in LED pin to output mode
  lcd.begin();                                    // Initialize LCD panel
  lcd.setRotation(3);                             // Set screen orientation
  lcd.fillScreen(ILI9341_BLACK);                  // Fill screen with color black
  adcTriggerTimer.begin(adcTimerInterrupt, sampleInterval);  // Set ADC trigger timer interval
}

// put your main code here, to run repeatedly:
void loop() {
  digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));   // Toggle LED
  drawBackGround();                                       // Draw backgounds

  if (triggerMode) {                                      // If trigger mode set..
    while (triggerFlag != 3) delay(1);                    //    Wait until sampling is done
    triggerFlag = 0;                                      //    Clear trigger flag
    getTriggerWaveform();                                 //    Produce waveform from buffer
    analyzeWaveform();                                    //    Analyze waveform
    plotInformations();                                   //    Plot informations
    plotWaveform();                                       //    Plot waveform on screen
    while (triggerFlag != 3) delay(1);                    //    Again, wait until sampling is done
    fastClearScreen();                                    //    Clear screen
  } else {                                                // Else.. (Freerun mode)
    getFreerunWaveform();                                 //    Produce waveform from buffer
    analyzeWaveform();                                    //    Analyze waveform
    plotInformations();                                   //    Plot informations
    plotWaveform();                                       //    Plot waveform on screen
    delay(1000 / 24);                                     //    Delay (24FPS)
    fastClearScreen();                                    //    Clear screen
  }

}

void adcTimerInterrupt() {
  unsigned int adcValue = analogRead(ADC_PIN);        // Read analog input
  float adcVoltage = adcValue / 1023.0f * 3.3f;       // Scale ADC raw input to voltages (10bits ADC input)
  switch (triggerMode) {
    case 0: // Freerun Mode
      waveBuffer[bufferIndex++] = adcValue;
      bufferIndex = bufferIndex >= ADC_BUFFER_SIZE ? 0 : bufferIndex;     // loop back index to reuse memory
      break;
    case 1: // Rising edge trigger mode
      if (triggerFlag == 0) {
        if (adcVoltage < triggerLevel)  triggerFlag++;
      } else if (triggerFlag == 1) {
        if (adcVoltage >= triggerLevel) {
          bufferIndex = 0;
          waveBuffer[bufferIndex++] = adcValue;
          triggerFlag++;
        }
      } else if (triggerFlag == 2) {
        waveBuffer[bufferIndex++] = adcValue;
        if (bufferIndex >= WINDOW_WIDTH) triggerFlag++;
      }
      break;
    case 2: // Falling edge trigger mode
      if (triggerFlag == 0) {
        if (adcVoltage > triggerLevel)  triggerFlag++;
      } else if (triggerFlag == 1) {
        if (adcVoltage <= triggerLevel) {
          bufferIndex = 0;
          waveBuffer[bufferIndex++] = adcValue;
          triggerFlag++;
        }
      } else if (triggerFlag == 2) {
        waveBuffer[bufferIndex++] = adcValue;
        if (bufferIndex >= WINDOW_WIDTH) triggerFlag++;
      }
      break;
  }
}

void getTriggerWaveform() {
  memcpy(waveform, waveBuffer, WINDOW_WIDTH * sizeof(unsigned int));
}

void getFreerunWaveform() {
  if (bufferIndex >= WINDOW_WIDTH) {  // Simply extract last WINDOW_WIDTH of datas from buffer
    memcpy(waveform, &waveBuffer[bufferIndex - WINDOW_WIDTH], WINDOW_WIDTH * sizeof(unsigned int));
  } else {  // If buffer index loops back, concat new ones at the beginning and old ones at the tail
    int residue = WINDOW_WIDTH - bufferIndex;
    memcpy(waveform, &waveBuffer[ADC_BUFFER_SIZE - residue], residue * sizeof(unsigned int));
    memcpy(&waveform[residue], waveBuffer, bufferIndex * sizeof(unsigned int));
  }
}

void analyzeWaveform() {
  int edges = 1;
  float mean = waveform[0];
  float vMax = waveform[0];
  float vMin = waveform[0];
  for (int i = 0; i < WINDOW_WIDTH - 1; i++) {
    float v = waveform[i] / 1023.0f * 3.3f;
    float v1 = waveform[i + 1] / 1023.0f * 3.3f;
    if (triggerMode == 1) {
      if (v < triggerLevel && v1 >= triggerLevel) edges++;
    } else if (triggerMode == 2) {
      if (v > triggerLevel && v1 <= triggerLevel) edges++;
    } else {
      if (v < triggerLevel && v1 >= triggerLevel) edges++;
      if (v > triggerLevel && v1 <= triggerLevel) edges++;
    }
    mean += waveform[i + 1];
    if (waveform[i + 1] > vMax) vMax = waveform[i + 1];
    if (waveform[i + 1] < vMin) vMin = waveform[i + 1];
  }
  if (triggerMode == 0) {
    edges /= 2;
  }
  mean /= WINDOW_WIDTH;
  wavFreq = edges / (windowTime / 1000.0f);
  wavAvg = mean / 1023.0f * 3.3f;
  wavVpp = (vMax - vMin) / 1023.0f * 3.3f;
}

void drawBackGround() {
  int triggerYCursor = 230 - map(triggerLevel, 0, 3.3, 0, 200);

  lcd.startWrite();                                                 // Start LCD wirte
  lcd.writeFastVLine(60, 20, 220, ILI9341_WHITE);                   // Draw y axis
  lcd.writeFillRect(51, 29, 18, 3, ILI9341_WHITE);                  // Draw 3.30V cursor
  lcd.writeFillRect(55, 129, 10, 3, ILI9341_WHITE);                 // Draw 1.65V cursor
  lcd.writeFillRect(51, 229, 18, 3, ILI9341_WHITE);                 // Draw 0.00V cursor
  lcd.writeFillRect(55, triggerYCursor - 1, 10, 3, ILI9341_RED);    // Draw tirgger level cursor

  // Draw horizontal dotted lines
  for (int x = 60; x <= LCD_WIDTH; x += 10) {
    lcd.writeFastHLine(x, 30, 5, ILI9341_WHITE);
    lcd.writeFastHLine(x, 130, 5, ILI9341_WHITE);
    lcd.writeFastHLine(x, 230, 5, ILI9341_WHITE);
    lcd.writeFastHLine(x, triggerYCursor, 5, ILI9341_RED);
  }

  // Draw vertical dotted lines
  for (int x = 112; x <= LCD_WIDTH; x += 52) {
    for (int y = 20; y <= LCD_HEIGHT; y += 10) {
      lcd.writeFastVLine(x, y, 5, ILI9341_WHITE);
    }
  }
  lcd.endWrite();    // End LCD wirte

  lcd.setTextSize(1);                 // Set text size
  lcd.setTextColor(ILI9341_WHITE);    // Set text color
  lcd.setCursor(7, 28);               // Set cursor position
  lcd.print(F("3.30 V"));
  lcd.setCursor(7, 128);
  lcd.print(F("1.65 V"));
  lcd.setCursor(7, 228);
  lcd.print(F("0.00 V"));
  lcd.setTextColor(ILI9341_RED);
  lcd.setCursor(7, triggerYCursor - 2);
  lcd.print(String(triggerLevel, 2) + F(" V"));
}

void plotInformations() {
  lcd.setTextSize(2);                 // Set text size
  lcd.setTextColor(ILI9341_WHITE);    // Set text color
  lcd.setCursor(5, 5);
  lcd.print(String(windowTime) + F("ms "));

  lcd.setCursor(100, 8);
  lcd.setTextSize(1);                 // Set text size
  lcd.print(F("Freq:") + String(wavFreq, 0) + F("Hz "));
  lcd.print(F("Avg:") + String(wavAvg, 2) + F("V "));
  lcd.print(F("Vpp:") + String(wavVpp, 2) + F("V"));
}

void plotWaveform() {
  //Plot waveform (connect sample points w/ lines)
  for (int i = 0; i < WINDOW_WIDTH - 1; i++) {
    int x1 = i + 60;
    int y1 = 230 - map(waveform[i], 0, 1023, 0, 200);
    int x2 = i + 60;
    int y2 = 230 - map(waveform[i + 1], 0, 1023, 0, 200);
    lcd.drawLine(x1, y1, x2, y2, ILI9341_GREEN);
  }
}

void fastClearScreen() {
  // Draw the same waveform with color black to clear screen
  for (int i = 0; i < WINDOW_WIDTH - 1; i++) {
    int x1 = i + 60;
    int y1 = 230 - map(waveform[i], 0, 1023, 0, 200);
    int x2 = i + 60;
    int y2 = 230 - map(waveform[i + 1], 0, 1023, 0, 200);
    lcd.drawLine(x1, y1, x2, y2, ILI9341_BLACK);
  }

  lcd.fillRect(0, 0, 320, 20, ILI9341_BLACK);
}
