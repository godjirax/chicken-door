/*
0.1 ampère * 24 => 
5 volts
24 heures
=> 5 * 0.1 * 24 = 12 watts

Il faut donc un panneau de :
3.6 watts

Consommation journalière 3.6W x 1,2 (marge de sécurité) = 4.32W / 5V = 0.86 Ah x 3 jours = 2.59 Ah

https://fiz-ix.com/2012/11/low-power-arduino-using-the-watchdog-timer/
*/

#include <avr/sleep.h>

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

const int dark = 50;
const int hysterisis = 5;

const long MAX_TIME_DOOR_MOVE = 60000;
// MAx time for door to move

//char state[16];

// This will run only one time.
void setup() {
  Serial.begin(9600);

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
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000);

  digitalWrite(LED_BUILTIN, LOW);
  luminosityValue = analogRead(LDR);

  // Serial.print("luminosityValue: ");
  // Serial.println(luminosityValue);

  // Serial.print("digitalRead(limitSwitchOpen): ");
  // Serial.println(digitalRead(limitSwitchOpen));

  // Serial.print(" digitalRead(limitSwitchClose): ");
  // Serial.println( digitalRead(limitSwitchClose));

 //closeChickenDoor();
 //openChickenDoor();

  if (isLight(luminosityValue) || forceOpen) {
    // Serial.println("open ?");
    openChickenDoor();
  } else  if (isDark(luminosityValue) || forceClose) {
    // Serial.println("close ?");
    closeChickenDoor();
  }

  delay(2000);
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
    Serial.println("OPEN > doorstate = 1");
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
      } else {
        Serial.println("continue open by timer");
      }


      if (isOpenedDoorButtonOpened()) {
        Serial.println("stop open by switch");
        Serial.println("OPEN > stopChickenDoor");

        // stopChickenDoor();
        motorTurns = false;
      } else {
        Serial.println("continue open by switch");
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
  // Serial.println("enter in close");
  // Serial.println(doorState);

  if (doorState == 1 || forceClose == 1) {
    Serial.println("CLOSE > doorstate = 0");
    doorState = 0;
    forceClose = 0;
    
    moveChickenDoorToClose();

    bool motorTurns = true;

    int buttonState = 0;

    long timeStart = millis();
    long currentTime = millis();

    while (motorTurns) {

      currentTime = millis();
      // Serial.println("(currentTime - timeStart) = %d");


      Serial.print("timeStart=");
      Serial.print(timeStart);
      Serial.print(", currentTime=");
      Serial.println(currentTime);

      // Serial.println("MAX_TIME_DOOR_MOVE");
      // Serial.println("3000");

      // Serial.println("currentTime - timeStart");
      // Serial.println(currentTime - timeStart);

      Serial.print("currentTime: ");
      Serial.println(currentTime);

      Serial.print("timeStart: ");
      Serial.println(timeStart);

      Serial.print("currentTime - timeStart: ");
      Serial.println(currentTime - timeStart);

      Serial.print("MAX_TIME_DOOR_MOVE: ");
      Serial.println(MAX_TIME_DOOR_MOVE);

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

void stopChickenDoor() {

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}