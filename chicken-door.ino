// /* Photocell reading program */
// // Constants
// #define DELAY 500 // Delay between two measurements in ms
// #define VIN 5 // V power voltage
// #define R 10000 //ohm resistance value

// // Parameters
// const int sensorPin = A7; // Pin connected to sensor

// //Variables
// int sensorVal; // Analog value from the sensor
// int lux; //Lux value

// void setup(void) {
//   Serial.begin(9600);
// }

// void loop(void) {
//   sensorVal = analogRead(sensorPin);
//   lux=sensorRawToPhys(sensorVal);
//   Serial.print(F("Raw value from sensor= "));
//   Serial.println(sensorVal); // the analog reading
//   Serial.print(F("Physical value from sensor = "));
//   Serial.print(lux); // the analog reading
//   Serial.println(F(" lumen")); // the analog reading
//   delay(DELAY);
// }

// int sensorRawToPhys(int raw){
//   // Conversion rule
//   float Vout = float(raw) * (VIN / float(1024));// Conversion analog to voltage
//   float RLDR = (R * (VIN - Vout))/Vout; // Conversion voltage to resistance
//   int phys=500/(RLDR/1000); // Conversion resitance to lumen
//   return phys;
// }

// /*
//  * Sketch for testing sleep mode with wake up on WDT.
//  * Donal Morrissey - 2011.
//  *
//  */
// // #include <avr/sleep.h>
// // #include <avr/power.h>
// // #include <avr/wdt.h>

// // #define LED_PIN (13)

// // volatile int f_wdt = 1;


// /*
// http://www.ohm-easy.com/blog/sources-alimentation/20140321-calculer-taille-dun-panneau-solaire-petite-installation/
// 10 mA.h x 5 volts x 24 heures = 0.01 x 5 x 24  = 1.2W

// Il faiut un panneau de :
// 1.2 x 1.5 / 5 = 0.36 w

// Batterie :
// 1.2W x 1.2 = 1.44 / 5 volts = 0.29 x 3 jours = 0.90 Ah

// */


#include <LowPower.h>

// Custom Vars
#define LDR A1  // composante photorésistance sur la pin A1

// Not implemented
// const int boutonToOpenPin = 2;
// const int boutonToClosePin = 3;

const int activatePontHPin = 4;

const int motorPin1 = 6;
const int motorPin2 = 9;

const int limitSwitchClose = 8;  // the number of the pushbutton pin
const int limitSwitchOpen = 7;   // the number of the LED pin

int doorState = 0;  // 0: closed, 1: opened
int forceOpen = 0;
int forceClose = 0;

const int dark = 20;
const int hysterisis = 5;

const long MAX_TIME_DOOR_MOVE = 60000;

void setup() {
  Serial.begin(9600);
  Serial.println("Initialising...");
  delay(100);  //Allow for serial print to complete.

  // Custom setup
  pinMode(LDR, INPUT);

  // TO REMOVE
  pinMode(A2, INPUT);

  pinMode(limitSwitchOpen, INPUT_PULLUP);
  pinMode(limitSwitchClose, INPUT_PULLUP);

  // Not implemented
  // pinMode(boutonToOpenPin, INPUT_PULLUP);
  // pinMode(boutonToClosePin, INPUT_PULLUP);


  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(activatePontHPin, OUTPUT);
  //pinMode(transistorPhotoResistancePin, OUTPUT);

  // Not implemented
  // attachInterrupt(digitalPinToInterrupt(boutonToOpenPin), forceOpenDoor, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(boutonToClosePin), forceCloseDoor, CHANGE);

  Serial.println("Initialisation complete.");
  delay(100);  //Allow for serial print to complete.
}

// void loop() {
void loop() {
  // int luminosityValue = readLuminosity();

  // Serial.println("");

  // TO REMOVE
  // digitalWrite(activatePontHPin,LOW);

// moveChickenDoorToClose();
// moveChickenDoorToOpen();



  // Serial.print("luminosityValue: ");
  // Serial.println(luminosityValue);





  // Serial.print("analogRead(LDR): ");
  // Serial.println(analogRead(LDR));

   Serial.print("analogRead(A2): ");
  Serial.println(analogRead(A2));
  delay(500);





  // if (isLight(luminosityValue) || forceOpen || true) {
  //   // Serial.println("open ?");
  //   openChickenDoor();

  //   // TO REMOVE
  //   // closeChickenDoor();
  // } else if (isDark(luminosityValue) || forceClose) {
  //   // Serial.println("close ?");
  //   closeChickenDoor();
  // }

  

  // lowPowerSleep(8);
  // LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

int readLuminosity() {
  // digitalWrite(transistorPhotoResistancePin,LOW);

  int luminosityValue = analogRead(LDR);

  // digitalWrite(transistorPhotoResistancePin,LOW);

  return luminosityValue;
}

void lowPowerSleep(long seconds)
{
  // int seconds = minutes * 60;
  int sleeps = seconds / 8;
  for (int i = 0 ; i < sleeps ; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

// Not implemented
// void forceOpenDoor() {
//   Serial.println("Force open");
//   forceOpen = 1;
// }

// Not implemented
// void forceCloseDoor() {
//   Serial.println("Force close");
//   forceClose = 1;
// }

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
  digitalWrite(activatePontHPin,LOW);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
}

void moveChickenDoorToClose() {
  digitalWrite(activatePontHPin,HIGH);

  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
}

void moveChickenDoorToOpen() {
  digitalWrite(activatePontHPin,HIGH);

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
      // Serial.println("motorTurns");
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
