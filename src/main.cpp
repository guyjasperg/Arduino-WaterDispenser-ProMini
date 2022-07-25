#include <Arduino.h>

// LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// RFID / RC522
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 2
#define RELAY_PUMP 5
#define RELAY_SOLENOID 1 // TX PIN
#define WATERFLOW_PIN 2

// Define Slave I2C Address
#define SLAVE_ADDR 9

// Define Slave answer size
#define ANSWERSIZE 10

enum STATESM
{
  STATE_INIT,
  STATE_INIT2,
  STATE_WAIT_NEW_CARD,
  STATE_CARD_DETECTED,
  STATE_CARD_VALIDATING,
  STATE_CARD_VALIDATING_WAIT,
  STATE_CARD_VALIDATED,
  STATE_DISPENSING,
  STATE_DISPENSING_DONE,
  STATE_MAX
};
STATESM currentState = STATE_INIT;

#define CMD_PURCHASE "1"
#define CMD_RELOAD "2"

/* RC522
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    NodeMcu          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    ESP8266          Pro Mini
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         D3/GPIO0
 * SPI SS      SDA(SS)      10            53        D10        D4/GPIO2         10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        D7/GPIO13        11
 * SPI MISO    MISO         12 / ICSP-1   50        D12        D6/GPIO12        12
 * SPI SCK     SCK          13 / ICSP-3   52        D13        D5/GPIO14        13
 *
 * More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
 */

/* LCD I2C
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *              LCD           Arduino       NodeMcu          Arduino
 *              Reader/PCD    Uno/101       ESP8266          Pro Mini
 * Signal       Pin           Pin           Pin              Pin
 * -----------------------------------------------------------------------------------------
 *              SDA                         D2               A4
 *              SCL                         D1               A5
 *              VCC                         VU
 */

/* Other Connections
  RX - WaterFlow Sensor / Interrupt
  TX - Relay / Solenoid Valve
  D0 - Relay / Pump

  Note: Need to call pinMode(x, FUNTION_3) for both TX/RX pins so that they function as regular GPIO pins
        call pinMode(x,FUNCTION_0) to restore.
*/

LiquidCrystal_I2C lcd(0x27, 16, 2);

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;
// Init array that will store new NUID
byte nuidPICC[4];
String csCardID = "";

bool bCardDetected = false;
bool bCardDataSent = false;
bool bCardValidated = false;
bool bStateInitialized = false;

volatile int flow_frequency;
float water_vol = 0, l_minute;
unsigned long currentTime;
unsigned long cloopTime;

// I2C Communication
// volatile char receivedData[40];
char receivedData[40];
volatile byte newMessage = 0;
volatile byte newRequest = 0;

void printData(const char *str)
{
  // Serial.print(str);
  lcd.print(str);
}

void printDataLn(const char *str)
{
  // Serial.println(str);
  lcd.print(str);
}

void lcdClearLine(unsigned int line)
{
  lcd.setCursor(0, line - 1);
  lcd.print("                ");
  lcd.setCursor(0, line - 1);
}

// interrupt routine
void ISR_WaterFlow()
{
  flow_frequency++;
}

void printHex(byte *buffer, byte bufferSize)
{
  char s[5];
  for (byte i = 0; i < bufferSize; i++)
  {
    // Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    // Serial.print(buffer[i], HEX);

    sprintf(s, " %02x", buffer[i]);
    lcd.print(s);
  }
}

String GetCardID(byte *buffer, byte bufferSize)
{
  char temp[(bufferSize * 2) + 1];
  if (bufferSize == 4)
  {
    sprintf(temp, "%02X%02X%02X%02X", buffer[0], buffer[1], buffer[2], buffer[3]);
    String csID(temp);
    return csID;
  }
  else
  {
    return "";
  }
}

void WaitCardRemoval()
{
  byte control;
  bool bPrompt = false;

  while (true)
  {
    control = 0;

    for (int i = 0; i < 3; i++)
    {
      if (!rfid.PICC_IsNewCardPresent())
      {
        if (rfid.PICC_ReadCardSerial())
        {
          control |= 0x16;
        }
        if (rfid.PICC_ReadCardSerial())
        {
          control |= 0x16;
        }
        control += 0x1;
      }
      control += 0x4;
    }

    if (control == 13 || control == 14)
    {
      // display once
      if (!bPrompt)
      {
        bPrompt = true;
        lcd.setCursor(0, 1);
        lcd.print("Pls remove card");
      }
    }
    else
    {
      break;
    }
    delay(500);
  }
}

void DispenseWaterLiters(unsigned char liters)
{
  bool bDone = false;

  lcd.setCursor(0, 0);
  lcd.print("*Dispensing H2O");
  lcdClearLine(2);
  lcd.print("0.00 L");

  // make sure to reset counter
  flow_frequency = 0;
  l_minute = 0;
  water_vol = 0;

  // Turn On pump/solenoid valve
  digitalWrite(RELAY_PUMP, LOW);
  // slight delay to make sure there is water pressure near valve
  delay(1500);

  cloopTime = millis();
  digitalWrite(RELAY_SOLENOID, LOW);

  while (true)
  {
    currentTime = millis();
    // check every interval 500 == half second
    if (currentTime >= (cloopTime + 500))
    {
      cloopTime = currentTime;
      if (flow_frequency != 0)
      {
        // there is water flow, need to update
        // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
        l_minute = flow_frequency / 7.5;
        // Get Liters dispensed
        l_minute = l_minute / 60;
        water_vol += l_minute;

        if (water_vol >= liters)
        {
          // already dispensed required amount
          digitalWrite(RELAY_SOLENOID, HIGH);
          digitalWrite(RELAY_PUMP, HIGH);

          bDone = true;
        }

        // reset interrupt counter
        flow_frequency = 0;

        // update display
        lcd.setCursor(0, 1);
        lcd.print(water_vol);
        lcd.print(" L    ");

        if (bDone)
          break;
      }
    }
  }

  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("Done...       ");
}

bool IsCardValid(String csID)
{
  lcd.setCursor(0, 1);
  if (csID == "7B1CB128")
  {
    return true;
  }
  else
  {
    lcd.print("Invalid card   ");
    return false;
  }
}

bool CardDetected()
{
  if (rfid.PICC_IsNewCardPresent())
  {
    if (rfid.PICC_ReadCardSerial())
    {
      // card was detected
      // get card type
      MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
      Serial.println(rfid.PICC_GetTypeName(piccType));

      // Store NUID into nuidPICC array
      for (byte i = 0; i < 4; i++)
      {
        nuidPICC[i] = rfid.uid.uidByte[i];
      }
      csCardID = GetCardID(rfid.uid.uidByte, rfid.uid.size);
      return true;
    }
  }
  return false;
}

// received data from NodeMcu
void ISR_I2C_receiveEvent(int howMany)
{
  int i;
  for (i = 0; i < howMany; i++)
  {
    receivedData[i] = Wire.read();
  }
  receivedData[i] = '\0';
  newMessage = 1;
}

// NodeMcu requests data
void ISR_I2C_requestEvent()
{
  delay(20);

  switch (currentState)
  {
  case STATE_INIT:
    // received I2C message from NodeMCU, move to next state
    Wire.write("X");
    currentState = STATE_INIT2;
    break;
  case STATE_INIT2:
  case STATE_WAIT_NEW_CARD:
  case STATE_CARD_DETECTED:
    // send dummy data
    Wire.write("X");
    break;
  case STATE_CARD_VALIDATING:
    if (bCardDetected && !bCardDataSent)
    {
      // send card data + CMD + Amount
      int size = csCardID.length();
      char resp[size + 4];
      int i = 0;
      for (i = 0; i < size; i++)
      {
        resp[i] = csCardID.charAt(i);
      }
      resp[size] = '|';

      // CMD: 1 -> Purchase, 2 -> Reload
      resp[size + 1] = '1';

      // Amount: if Purchase, number of liters, if Reload, amount to reload
      resp[size + 2] = '5';

      // NULL terminator
      resp[size + 3] = '\0';

      Serial.println(resp);

      Wire.write(resp, sizeof(resp));
      bCardDataSent = true;
    }
    break;
  case STATE_CARD_VALIDATING_WAIT:
    Wire.write("X");
    break;
  case STATE_DISPENSING:
    Wire.write("X2");
    break;
  default:
    // send dummy data
    Wire.write("X");
    break;
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  lcd.init();
  lcd.backlight();
  lcd.print("Init...");

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  SPI.begin();     // Initiate  SPI bus
  rfid.PCD_Init(); // Init MFRC522

  for (byte i = 0; i < 6; i++)
  {
    key.keyByte[i] = 0xFF;
  }

  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);

  // Function to run when data requested from master
  Wire.onRequest(ISR_I2C_requestEvent);

  // Function to run when data received from master
  Wire.onReceive(ISR_I2C_receiveEvent);

  attachInterrupt(digitalPinToInterrupt(WATERFLOW_PIN), ISR_WaterFlow, RISING);

  delay(1000);
  lcdClearLine(1);
  lcd.print("Synching...");
  // currentState = STATE_INIT2;
}

void loop()
{
  switch (currentState)
  {
  case STATE_INIT:
    // wait here until NodeMcu has initialized
    // if (newMessage == 1)
    // {
    //   newMessage = 0;
    //   String data(receivedData);
    //   if (receivedData == "READY")
    //   {
    //     currentState = STATE_INIT2;
    //   }
    // }
    break;
  case STATE_INIT2:
    bCardDetected = false;
    bCardDataSent = false;

    // set initial display
    lcd.setCursor(0, 0);
    printDataLn("Scan RFID Card");
    lcd.setCursor(0, 1);

    // move to next state
    currentState = STATE_WAIT_NEW_CARD;
    break;
  case STATE_WAIT_NEW_CARD:
    if (CardDetected())
    {
      // Display card ID
      lcdClearLine(2);
      lcd.print(csCardID);

      // go to next state
      currentState = STATE_CARD_DETECTED;
    }
    break;
  case STATE_CARD_DETECTED:
    // wait for card to be removed
    delay(500);

    // wait for card to be removed
    WaitCardRemoval();

    // Halt PICC
    rfid.PICC_HaltA();
    // Stop encryption on PCD
    rfid.PCD_StopCrypto1();

    // need to send card data to NodeMcu
    bCardDataSent = false;
    bCardDetected = true;
    currentState = STATE_CARD_VALIDATING;
    break;
  case STATE_CARD_VALIDATING:
    // wait for card to be sent (through interrupt)
    if (bCardDataSent)
    {
      // card data sent to NodeMcu
      lcdClearLine(2);
      lcd.print("Validating...  ");

      currentState = STATE_CARD_VALIDATING_WAIT;
    }
    break;
  case STATE_CARD_VALIDATING_WAIT:
    // wait for response from NodeMcu for card validation
    if (newMessage)
    {
      String data(receivedData);
      if (data == "OK")
      {
        lcdClearLine(2);
        lcd.print("Card is valid");
        delay(1000);
        bStateInitialized = false;
        currentState = STATE_DISPENSING;
      }
      else
      {
        lcdClearLine(2);
        lcd.print("Invalid card!");

        delay(2000);
        lcdClearLine(2);
        currentState = STATE_WAIT_NEW_CARD;
      }
      newMessage = 0;
    }
    delay(100);
    break;
  case STATE_DISPENSING:
    if (!bStateInitialized)
    {
      bStateInitialized = true;
      lcd.setCursor(0, 0);
      lcd.print("Dispensing...  ");
      lcd.setCursor(0, 1);
      lcd.print("0.00 L         ");
    }

    break;
  default:
    break;
  }
  delay(100);
}