#include <OneButton.h>
#include <U8g2lib.h>

#define dwrlogo_width 60
#define dwrlogo_height 30
static const unsigned char dwrlogo_bits[] PROGMEM = {
   0x00, 0xf0, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00,
   0x07, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x80, 0x0f, 0x00, 0x00, 0x00,
   0x00, 0xf0, 0x03, 0xe0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0xf0,
   0x7f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0xf8, 0xfd, 0x00, 0x00, 0x00,
   0x00, 0xf0, 0x03, 0xf8, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x30,
   0x60, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xc0, 0xf7, 0x03, 0x00, 0x02, 0x00, 0x80, 0x0f, 0xf0, 0xff, 0xe3, 0x07,
   0x07, 0x3f, 0xcf, 0x0f, 0xf8, 0xff, 0xe3, 0x87, 0x0f, 0x3f, 0xef, 0x0f,
   0xfc, 0xff, 0xc3, 0x87, 0x0f, 0x1f, 0xff, 0x0f, 0xfe, 0xff, 0xc3, 0xcf,
   0x9f, 0x1f, 0xff, 0x07, 0xfe, 0xff, 0xc3, 0xef, 0xbf, 0x1f, 0xff, 0x03,
   0x7f, 0xf0, 0x83, 0xef, 0xbf, 0x0f, 0x7f, 0x00, 0x3f, 0xe0, 0x83, 0xff,
   0xff, 0x0f, 0x3f, 0x00, 0x3f, 0xe0, 0x83, 0xff, 0xff, 0x0f, 0x3f, 0x00,
   0x3f, 0xe0, 0x03, 0xff, 0xff, 0x07, 0x3f, 0x00, 0x3f, 0xe0, 0x03, 0xff,
   0xfd, 0x07, 0x3f, 0x00, 0x7e, 0xf0, 0x03, 0xff, 0xf8, 0x07, 0x3f, 0x00,
   0xfe, 0xff, 0x03, 0x7e, 0xf0, 0x03, 0x3f, 0x00, 0xfc, 0xff, 0x03, 0x7e,
   0xf0, 0x03, 0x3f, 0x00, 0xfc, 0xff, 0x03, 0x3c, 0xe0, 0x01, 0x3f, 0x00,
   0xf8, 0xbf, 0x03, 0x1c, 0xc0, 0x01, 0x3f, 0x00, 0xe0, 0x8f, 0x03, 0x1c,
   0xc0, 0x01, 0x3f, 0x00, 0x80, 0x07, 0x00, 0x08, 0x80, 0x00, 0x3f, 0x00
};

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define pinSensor 3
#define pinButtonSTB 4
#define pinButtonSWT 5
#define pinSwitchSTB 6
#define pinSwitchSWT 7
#define pinSwitchUP 8
#define pinSwitchDWN 9

const byte pseudoClickDelay = 25; const byte triggerNumber = 4; // Indicates the number of sensor triggers per rotation.
boolean debugSerial = true; // Determines if output written to serial (mostly if not exclusively for debugging purposes).
boolean tachometerOnly = false; // A setting of "true" disables pseudoClickButton() and switchRotationSpeed() for the code to work as tachometer only.
volatile unsigned long timeHallNew[triggerNumber], timeHallPrevious[triggerNumber]; // Variables pairs to hold new and old times for sensor triggers.
boolean activeSpin = false, justStarted = true, modeAutomatic = true; // Variables to indicate whether (1) actively spinning, (2) just started spinning (legacy purpose) and (3) status of automatic correction. 
long countSpin = 0; int countTempSpin = 0; // Variables to hold (1) number of full spins since start and (2) intermediary spin portions between full spins based on number of sensor triggers.
int spinSpeed = 0; // Variable to hold speed of spinning, where 0 = 33.33 and 1 = 45.00.
int countMessageDisplaySpins = 1; int waitingCycle = 0; long stoppingTime = 2500000; // Indicates (1) the number of rotations a display message is active before being replaced with RPM, (2) represents intermediary counter and (3) determines the amount of microseconds elapsed after which the platter is considered stopped.
int correctionSpin = 4; int correctionSpinShort = 2; long correctionSpinCount = 0; int correctionMovement = 5; // Indicates (1) number of rotations to wait for applying correction with small corrections, (2) number of rotations to wait for applying correction with large corrections, (3) temporary spin count and (4) maximum amount of steps to correct per cycle.
float correctionQ = 0.01; // Indicates the difference quotient from standard speed to activate automatic correction.
float derivedQ = 0.01; // Indicates the correction in RPM derived from a single 0.01Hz step (single "click").

OneButton buttonSTB(pinButtonSTB, true);
OneButton buttonSWT(pinButtonSWT, true);

void setup()
{
  // Correct spin counting variables when using more than one trigger per spin cycle.
  countMessageDisplaySpins = countMessageDisplaySpins * triggerNumber;

  // Initialize buttons, switches and displays.
  initializeButtonsSwitchesSensors();
  u8g2.begin();
  drawLogo();

  if (debugSerial) { Serial.begin(9600); }
}

void loop()
{
  buttonSTB.tick();
  buttonSWT.tick(); 
  if (activeSpin)
  {
    unsigned long currentMicros = micros();
    if (justStarted)
    {
      writeToDisplay(3);
      justStarted = false;
    }
    if (timeHallNew[countTempSpin] > timeHallPrevious[countTempSpin]) {
      if (countSpin > 1)
      {
        showRPM();
      }
    }
    else if (currentMicros - timeHallNew[countTempSpin] > stoppingTime) // Show waiting message after 2.5 seconds of no new pulses, but only if pulses were detected.
    {
      // Default to 33.33 RPM to allow for smoother start-up.
      spinSpeed = 0;
      digitalWrite(pinSwitchSWT, LOW);

      // Reset active rotation variables.
      activeSpin = false;
      justStarted = true;
      countSpin = 0;
      countTempSpin = 0;
      correctionSpinCount = 0;
      drawLogo();
    }
    timeHallPrevious[countTempSpin] = timeHallNew[countTempSpin];
    delay(pseudoClickDelay);
  }
}

void initializeButtonsSwitchesSensors()
{
  // Initialize output pins for analog switch.
  if (!tachometerOnly)
  {
    for (int p = 6; p <= 9; p++)
    {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
    // Attach subroutines to button events.
    buttonSTB.attachClick(startOperation);
    buttonSWT.attachClick(switchRotationSpeed);
    buttonSWT.attachLongPressStart(switchAutomaticMode);  
  }
  
  // Initialize sensor pin and attach interrupt routines.
  pinMode(pinSensor, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinSensor), triggerSensor, FALLING);
}

void startOperation()
{
  pseudoClickButton(pinSwitchSTB);
}

void switchRotationSpeed()
{
  if (tachometerOnly) { return; }
  if (!activeSpin) { return; }
  spinSpeed++;
  if (spinSpeed > 1) { spinSpeed = 0; digitalWrite(pinSwitchSWT, LOW); }
  else { digitalWrite(pinSwitchSWT, HIGH); }
  correctionSpinCount = 0;
  writeToDisplay(2);
}

void switchAutomaticMode()
{
  if (tachometerOnly) { return; }
  if (!activeSpin) { return; }
  modeAutomatic = !modeAutomatic;
  writeToDisplay(1);
}

// Triggers the sensor routine.
void triggerSensor() {
  countTempSpin++;
  if (countTempSpin > triggerNumber)
  {
    countSpin++;
    correctionSpinCount++;
    activeSpin = true;
    countTempSpin = 1;
  }
  timeHallNew[countTempSpin] = micros();
}

// Calculates current RPM and prompts a printout.
void showRPM() {
    if ((timeHallNew[countTempSpin] == 0) || (timeHallPrevious[countTempSpin] == 0)) { return; }
    float currentRPM = 60000000.0 / (timeHallNew[countTempSpin] - timeHallPrevious[countTempSpin]);
    if (debugSerial) { Serial.print(currentRPM, 4); }
    if (!tachometerOnly)
    {
      if (correctionSpinCount > correctionSpin) // Rest time has completed, and platter has spun more cycles than correctionSpin.
      {
        if (modeAutomatic)
        {
          float DQ = calculateDifferenceQ(currentRPM);
          if (abs(DQ) > correctionQ)
          {
            int correctX = floor(abs(DQ) / derivedQ); int correctDirection = 0; // Indicates direction for correction, where 0 = UP and 1 = DOWN.
            if (DQ < 0) { correctDirection = 1; }
            int stepsX = (correctX > correctionMovement ? correctionMovement : correctX);
            correctionSpinCount = ((correctX > correctionMovement * 2) ? (correctionSpin - correctionSpinShort) : 0); // Waits for correctionMovement rotations before applying new correction, or correctionSpinShort
            timeHallPrevious[countTempSpin] = timeHallNew[countTempSpin];
            for (int x = 0; x < stepsX; x++)
            {
              pseudoClickButton(8 + correctDirection);
              delay(pseudoClickDelay * 2);
            }
          }
        }
      }
    }
    writeToDisplay(currentRPM);
}

float calculateDifferenceQ(float rpmQ)
{
  float speedBase = 33.3333;
  if (spinSpeed == 1) { speedBase = 45.0000; }
  return speedBase - rpmQ;
}

// "Clicks" a button connected to the analog switch utilizing a pretedermined delay for debouncing purposes.
void pseudoClickButton(int pinButton)
{
  if (tachometerOnly) { return; }
  digitalWrite(pinButton, HIGH);
  delay(pseudoClickDelay);
  digitalWrite(pinButton, LOW);
  delay(pseudoClickDelay);
}

// Draws the logo using U8G2 paging.
void drawLogo(void) {
  u8g2.firstPage();
  do
  {
    u8g2.drawXBMP( 32, 1, dwrlogo_width, dwrlogo_height, dwrlogo_bits);
  }
  while(u8g2.nextPage());
}

// Writes (1) RPM to the display, (2) correction mode message and (3) speed change message. TTW > 10 gets written as RPM. For values below that threshold, 1 = "Manual/Automatic Correction Mode Change", 2 = "33/45 Speed Change" and 3 = "Starting Up". 
void writeToDisplay(float TTW)
{
  if (waitingCycle > 0) { waitingCycle--; return; } // Exit procedure if number of spins to show message has not expired.
  u8g2.firstPage();
  do
  {
    if (TTW > 10)
    {
      u8g2.setFont(u8g2_font_logisoso32_tr);
      u8g2.setCursor(0, 32);
      u8g2.print(TTW, 2);
      if (modeAutomatic)
      {
        u8g2.setFont(u8g2_font_unifont_t_symbols);
        u8g2.drawGlyph(108, 16, 0x23f6);
        u8g2.drawGlyph(108, 28, 0x23f7);
      }
    }
    else
    {
      u8g2.setFont(u8g2_font_logisoso16_tr);
      u8g2.setCursor(0, 16);
      if (TTW == 1)
      {
        u8g2.print(modeAutomatic ? "AUTOMATIC" : "MANUAL");
        u8g2.setCursor(0, 32);
        u8g2.print("correction");
      }
      else if (TTW == 2)
      {
        u8g2.print(spinSpeed == 0 ? "33.33 rpm" : "45.00 rpm");
        u8g2.setCursor(0, 32);
        u8g2.print("movement");
      }
      else if (TTW == 3)
      {
        u8g2.print("STARTUP");
        u8g2.setCursor(0, 32);
        u8g2.print("calculations");
      }
      waitingCycle = countMessageDisplaySpins;
    }
  }
  while( u8g2.nextPage() );
}
