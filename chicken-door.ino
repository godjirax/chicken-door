/*
 * Sketch for testing sleep mode with wake up on WDT.
 * Donal Morrissey - 2011.
 *
 */
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define LED_PIN (13)

volatile int f_wdt = 1;




// Custom Vars
#define LDR A1  // composante photorésistance sur la pin A1

const int boutonToOpenPin = 2;
const int boutonToClosePin = 3;

const int motorPin1 = 6;
const int motorPin2 = 9;

const int limitSwitchClose = 8;  // the number of the pushbutton pin
const int limitSwitchOpen = 7;   // the number of the LED pin

int doorState = 0;  // 0: closed, 1: opened
int forceOpen = 0;
int forceClose = 0;
int luminosityValue;

const int dark = 20;
const int hysterisis = 5;

const long MAX_TIME_DOOR_MOVE = 60000;


/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect) {
  if (f_wdt == 0) {
    f_wdt = 1;
  } else {
    // Serial.println("WDT Overrun!!!");
  }
}


/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void) {
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();

  /* Now enter sleep mode. */
  sleep_mode();

  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */

  /* Re-enable the peripherals. */
  power_all_enable();
}



/***************************************************
 *  Name:        setup
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Setup for the serial comms and the
 *                Watch dog timeout. 
 *
 ***************************************************/
void setup() {
  Serial.begin(9600);
  Serial.println("Initialising...");
  delay(100);  //Allow for serial print to complete.

  pinMode(LED_PIN, OUTPUT);

  /*** Setup the WDT ***/

  /* Clear the reset flag. */
  MCUSR &= ~(1 << WDRF);

  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /* set new watchdog timeout prescaler value */
  WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */

  /* Enable the WD interrupt (note no reset). */
  WDTCSR |= _BV(WDIE);

  // Custom setup
  pinMode(LDR, INPUT);
  pinMode(limitSwitchOpen, INPUT_PULLUP);
  pinMode(limitSwitchClose, INPUT_PULLUP);
  pinMode(boutonToOpenPin, INPUT_PULLUP);
  pinMode(boutonToClosePin, INPUT_PULLUP);

  // pinMode(reedsSwitch, INPUT_PULLUP);


  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(boutonToOpenPin), forceOpenDoor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(boutonToClosePin), forceCloseDoor, CHANGE);



  Serial.println("Initialisation complete.");
  delay(100);  //Allow for serial print to complete.
}



/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Main application loop.
 *
 ***************************************************/
void loop() {
  if (f_wdt == 1) {
    myLoop();

    /* Don't forget to clear the flag. */
    f_wdt = 0;

    /* Re-enter sleep mode. */
    enterSleep();
  }
}

void myLoop() {
  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);

  digitalWrite(LED_BUILTIN, LOW);
  luminosityValue = analogRead(LDR);

  Serial.print("luminosityValue: ");
  Serial.println(luminosityValue);

  // Serial.print("digitalRead(limitSwitchOpen): ");
  // Serial.println(digitalRead(limitSwitchOpen));

  // Serial.print(" digitalRead(limitSwitchClose): ");
  // Serial.println( digitalRead(limitSwitchClose));

  //closeChickenDoor();
  //openChickenDoor();

  if (isLight(luminosityValue) || forceOpen) {
    // Serial.println("open ?");
    openChickenDoor();
  } else if (isDark(luminosityValue) || forceClose) {
    // Serial.println("close ?");
    closeChickenDoor();
  }
}

void forceOpenDoor() {
  Serial.println("Force open");
  forceOpen = 1;
}

void forceCloseDoor() {
  Serial.println("Force close");
  forceClose = 1;
}

bool isDark(int photoSensorValue) {
  return photoSensorValue < (dark - hysterisis);
}

bool isLight(int photoSensorValue) {
  return photoSensorValue > (dark + hysterisis);
}

bool isOpenedDoorButtonOpened() {
  return digitalRead(limitSwitchOpen) == HIGH;
}

bool isClosedDoorButtonPushed() {
  return digitalRead(limitSwitchClose) == HIGH;
}

void stopChickenDoor() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void moveChickenDoorToClose() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
}

void moveChickenDoorToOpen() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}

void openChickenDoor() {
  if (doorState == 0 || forceOpen == 1) {
    Serial.println("openChickenDoor");
    doorState = 1;
    forceOpen = 0;

    moveChickenDoorToOpen();

    bool motorTurns = true;

    int buttonState = 0;

    long timeStart = millis();
    long currentTime = millis();

    while (motorTurns) {

      currentTime = millis();

      // if time excedeed (5s ?) => motorTurns false
      if (abs(currentTime - timeStart) > MAX_TIME_DOOR_MOVE) {
        Serial.println("stop open by timer");
        motorTurns = false;
        //break;
      }

      if (isOpenedDoorButtonOpened()) {
        Serial.println("stop open by switch");
        // Serial.println("OPEN > stopChickenDoor");

        // stopChickenDoor();
        motorTurns = false;
      }

      delay(250);
    }

    // Move a little door in opposite direction.
    moveChickenDoorToClose();
    delay(2000);

    stopChickenDoor();
  }
}

void closeChickenDoor() {
  if (doorState == 1 || forceClose == 1) {
    Serial.println("closeChickenDoor");
    doorState = 0;
    forceClose = 0;

    moveChickenDoorToClose();

    bool motorTurns = true;

    int buttonState = 0;

    long timeStart = millis();
    long currentTime = millis();

    while (motorTurns) {

      currentTime = millis();
      
      // if time excedeed (5s ?) => motorTurns false
      if (abs(currentTime - timeStart) > MAX_TIME_DOOR_MOVE) {

        Serial.println("STOP CLOSE by timeout");
        motorTurns = false;
        //break;
      }

      if (isClosedDoorButtonPushed()) {
        Serial.println("STOP CLOSE by switch");
        // stopChickenDoor();
        motorTurns = false;
      }

      delay(250);
    }

    // Move a little door in opposite direction.
    moveChickenDoorToOpen();
    delay(2000);

    stopChickenDoor();
  }
}