// ARC ARIES Remote Controller
#include <nRF24L01.h>
#include <RF24.h>
#include <LCD_I2C.h>

#define DEBUG

// Radio Addresses
#define ARIES_ADDRESS {0x41, 0x52, 0x49, 0x45, 0x53} // "ARIES"
#define CRIS_ADDRESS {0x43, 0x52, 0x49, 0x53, 0x31} // "CRIS1"

// Pin assignments
#define CSN 8
#define CE 9
#define MOSI 11
#define MISO 12
#define SCK 13

#define SDA A4
#define SCL A5

#define LCD_ADDRESS 0x27

#define BUTTON1 5
#define BUTTON2 4
#define BUTTON3 3 // No longer used
#define ARM_SWITCH 2

#define LED1 A0
#define LED2 A1
#define LED3 A2 // No longer used
#define LED_ARM A3


typedef enum{
  NO_COMMAND,
  IGNITE,
  ABORT
} pyro_command_t;

typedef enum{
  PYRO_OFF,
  PYRO_ON
} pyro_state_t;

typedef enum {
  STATUS_NORMAL,
  STATUS_ARMED,
  STATUS_ERROR
} status_t;

typedef struct {
  pyro_command_t pyro_1;
  pyro_command_t pyro_2;
  status_t ptx_status; // 0 = normal, 1 = armed, 2 = error
} arc_data_t;

typedef struct {
  int32_t resistance_1; // in milliohms
  int32_t resistance_2; // in milliohms
  int32_t sensor_data;
  pyro_state_t pyro_1_feedback;
  pyro_state_t pyro_2_feedback;
  status_t prx_status; // 0 = normal, 1 = armed, 2 = error
} aries_data_t;

typedef struct {
  bool button1;
  bool button2;
  bool arm;
} button_data_t;

// NRFL
RF24 radio(CE, CSN);
const uint8_t aries_address[5] = ARIES_ADDRESS;
const uint8_t cris_address[5] = CRIS_ADDRESS;

unsigned long last_recieve_time = 0;

//LCD
LCD_I2C lcd(LCD_ADDRESS, 16, 2);
bool lcdReset = false;

// Initialize data structs
arc_data_t arc_data = {NO_COMMAND, NO_COMMAND, STATUS_NORMAL};

aries_data_t aries_data = {-1, -1, -1, PYRO_OFF, PYRO_OFF, STATUS_NORMAL};

button_data_t buttons = {true, true, true};

button_data_t buttons_last = {true, true, true};



pyro_command_t pyroConfirm(int pyroNum, int buttonNum) {
  lcd.clear();
  lcd.setCursor(3, 0);
  lcd.print("WAITING...");
  delay(1000);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("CONFIRM  PYRO");
  lcd.setCursor(14, 0);
  lcd.print(pyroNum);
  lcd.setCursor(4, 1);
  lcd.print("IGNITION");
  unsigned long timeoutTime = millis() + 10000; // 10s Timeout
  while(millis() < timeoutTime) {
    if(digitalRead(buttonNum) == LOW && digitalRead(ARM_SWITCH) == LOW) {
      lcd.clear();
      lcd.setCursor(4, 0);
      lcd.print("IGNITING");
      lcd.setCursor(5, 1);
      lcd.print("PYRO");
      lcd.setCursor(10, 1);
      lcd.print(pyroNum);
      delay(1000);
      return IGNITE;
    }
    if(digitalRead(ARM_SWITCH) == HIGH) {
      break;
    }
  }
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("ABORTED");
  lcd.setCursor(4, 1);
  lcd.print("PYRO");
  lcd.setCursor(10, 1);
  lcd.print(pyroNum);
  delay(1000);
  return NO_COMMAND;  
}



void setup() {
  Serial.begin(9600);

  // Pin Modes
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(ARM_SWITCH, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED_ARM, OUTPUT);

  // NRFL
  if(!radio.begin()) {
    Serial.println(F("\nRadio hardware not responding!"));
    while (true) {} // hold program in infinite loop to prevent subsequent errors
  }
  else {
    Serial.println("\nNRFL init Success!\n");
  }
  radio.openWritingPipe(aries_address);
  radio.openReadingPipe(1, cris_address);
  radio.setPALevel(RF24_PA_MIN);
  radio.enableDynamicPayloads();
  //radio.disableDynamicPayloads();
  //radio.setPayloadSize(32);
  radio.setChannel(120);
  Serial.println(radio.getChannel());
  radio.startListening();
  radio.flush_rx();
  radio.flush_tx();

  if(!radio.isChipConnected()) {
    Serial.println("\nRadio Not Connected!");
    while(true) {}
  }

  // LCD
  lcd.begin();
  lcd.backlight();

  // Check Arm Switch
  if(digitalRead(ARM_SWITCH) == LOW) {
    lcd.setCursor(2, 0);
    lcd.print("ARM LOCKOUT");
    lcd.setCursor(0, 1);
    lcd.print("Disarm & Restart");
    while (true) {}
  }

}

void loop() {
  radio.startListening();
  radio.flush_tx();
  // poll buttons
  buttons.button1 = digitalRead(BUTTON1);
  buttons.button2 = digitalRead(BUTTON2);
  buttons.arm = digitalRead(ARM_SWITCH);
  // #ifdef DEBUG
  //   Serial.print("Button1 Value: "); Serial.println(buttons.button1);
  //   Serial.print("Button2 Value: "); Serial.println(buttons.button2);
  //   Serial.print("Arm Switch Value: "); Serial.println(buttons.arm);
  // #endif
  
  // Check for new button data
  if (buttons.button1 != buttons_last.button1 || buttons.button2 != buttons_last.button2 || buttons.arm != buttons_last.arm) {
    #ifdef DEBUG
      Serial.println("NEW BUTTON DATA");
      Serial.print("Button1 Value: "); Serial.println(buttons.button1);
      Serial.print("Button2 Value: "); Serial.println(buttons.button2);
      Serial.print("Arm Switch Value: "); Serial.println(buttons.arm);
    #endif

    
    // Update LEDs
    digitalWrite(LED1, !buttons.button1);
    digitalWrite(LED2, !buttons.button2);
    digitalWrite(LED_ARM, !buttons.arm);

    //Procecss pyro ignition inputs
    arc_data.ptx_status = (buttons.arm == LOW) ? STATUS_ARMED : STATUS_NORMAL;
    if (buttons.arm == LOW) {
        if (buttons.button1 == LOW) {
          while(digitalRead(BUTTON1) == LOW) {}
          arc_data.pyro_1 = pyroConfirm(1, BUTTON1);
          lcdReset = true;
        }
        else if (buttons.button2 == LOW) {
          while(digitalRead(BUTTON2) == LOW) {}
          arc_data.pyro_1 = pyroConfirm(2, BUTTON2);
          lcdReset = true;
        }
    }
    
    
    // Only send a transmission if a pyro command is issued (temporary measure to decrease number of transmissions)
    if(arc_data.pyro_1 == IGNITE || arc_data.pyro_2 == IGNITE) {
      // Send data struct
      #ifdef DEBUG
      Serial.println("SENDING DATA");
      Serial.print("ptx_status: "); Serial.println((arc_data.ptx_status == STATUS_ARMED) ? "STATUS_ARMED" : "STATUS_NORMAL");
      Serial.print("pyro_1: "); Serial.println((arc_data.pyro_1 == IGNITE) ? "IGNITE" : "NO_COMMAND");
      Serial.print("pyro_2: "); Serial.println((arc_data.pyro_2 == IGNITE) ? "IGNITE" : "NO_COMMAND");
      #endif
      radio.stopListening();
      bool success = radio.write(&arc_data, sizeof(arc_data));
      delay(5);
      lcd.clear();
      if(success) {
        lcd.setCursor(3, 0);
        lcd.print("TX Success!");
      } else {
        lcd.setCursor(3, 0);
        lcd.print("TX Failure!");
      }
      delay(2000);
      lcd.clear();
      radio.startListening();
      radio.flush_tx();
    }

    // Reset Data
    arc_data.pyro_1 = NO_COMMAND;
    arc_data.pyro_2 = NO_COMMAND;



    // Update buttons_last
    buttons_last.button1 = buttons.button1;
    buttons_last.button2 = buttons.button2;
    buttons_last.arm = buttons.arm;
    delay(20);
  }

  // Recieve data from ARIES
  if(radio.available()) {
    last_recieve_time = millis();
    radio.read(&aries_data, sizeof(aries_data));
    #ifdef DEBUG
      Serial.println("DATA RECIEVED");
      Serial.print("Resistance 1: "); Serial.println(aries_data.resistance_1);
      Serial.print("Resistance 2: "); Serial.println(aries_data.resistance_2);
      Serial.print("Pyro 1 Feedback: "); Serial.println(aries_data.pyro_1_feedback);
      Serial.print("Pyro 2 Feedback: "); Serial.println(aries_data.pyro_2_feedback);
      Serial.print("PRX Status: "); Serial.println(aries_data.prx_status);
      Serial.print("Last Recieve Time: "); Serial.println(last_recieve_time);
      Serial.print("TX FIFO full: "); Serial.println((radio.isFifo(true) == 2));
    #endif
    // Update LCD
    if(lcdReset == true) {
      lcd.clear();
      lcdReset = false;
    }
    lcd.setCursor(0, 0);
    lcd.print("Ch1 R: ");
    lcd.setCursor(7, 0);
    lcd.print(aries_data.resistance_1);
    lcd.setCursor(0, 1);
    lcd.print("Ch2 R: ");
    lcd.setCursor(7, 1);
    lcd.print(aries_data.resistance_2);
    lcd.setCursor(12, 0);
    switch (aries_data.prx_status) {
      case STATUS_NORMAL:
        lcd.print("NORM");
      break;
      case STATUS_ARMED:
        lcd.print("ARM");
      break;
      case STATUS_ERROR:
        lcd.print("ERR");
      break;
    }
  }

  if ((millis() - last_recieve_time) > 2000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Connection!");
    lcdReset = true;
    delay(100);
  }

}

