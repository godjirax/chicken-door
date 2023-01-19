/*
 * Sketch for testing sleep mode with wake up on WDT.
 * Donal Morrissey - 2011.
 *
 */
// #include <avr/sleep.h>
// #include <avr/power.h>
// #include <avr/wdt.h>

// #define LED_PIN (13)

// volatile int f_wdt = 1;


#include <LowPower.h>

// Custom Vars
#define LDR A1  // composante photorésistance sur la pin A1

const int boutonToOpenPin = 2;
const int boutonToClosePin = 3;

const int transistorPin = 4;

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

void setup() {
  Serial.begin(9600);
  Serial.println("Initialising...");
  delay(100);  //Allow for serial print to complete.

  // pinMode(LED_PIN, OUTPUT);

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
  pinMode(transistorPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(boutonToOpenPin), forceOpenDoor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(boutonToClosePin), forceCloseDoor, CHANGE);



  Serial.println("Initialisation complete.");
  delay(100);  //Allow for serial print to complete.
}

// void loop() {
void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);

  

  // delay(1000);

  // digitalWrite(LED_BUILTIN, LOW);
  luminosityValue = analogRead(LDR);

Serial.println("");
Serial.println("");
Serial.println("");
Serial.println("");

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

  lowPowerSleep(8);
}

void lowPowerSleep(long seconds)
{
  // int seconds = minutes * 60;
  int sleeps = seconds / 8;
  for (int i = 0 ; i < sleeps ; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
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
  digitalWrite(transistorPin,LOW);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void moveChickenDoorToClose() {
  digitalWrite(transistorPin,HIGH);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
}

void moveChickenDoorToOpen() {
  digitalWrite(transistorPin,HIGH);

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
