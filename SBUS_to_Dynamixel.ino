//*******************************************************************************************
//*  Radio Control SBUS / PPM to Dynamixel Servo adaptor                                    *
//*  ===================================================                                    *
//*  Board: Seeed Studio XIAO SAMD21                                                        *
//*                                                                                         *
//*  This board can control any Dynamixel Servo, either 5V or 12V types from a regular      *
//*  RC Receiver.  Either use PPM to control a single Dynamixel (set the Dxl ID to 1)       *
//*  Or use SBUS.  SBUS channels 1 to 16 map to Dynamixel ID's 1 to 16.                     *
//*  The settings switches select basic options, including SBUS / PPM Input                 *
//*                                                                                         *
//*  

#include "sbus.h"
#include "wiring_private.h" // For pinPeripheral
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <SimpleKalmanFilter.h>


#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
#define DXL_DIR_PIN  5
#define RC_PIN 0 // D0 on XIAO SAMD21
#define SMOOTHING_PIN A3 // A3 on XIAO SAMD21

#define SETTING_1 D1 // Settings Dip-switch - Single or Multi-turn
#define SETTING_2 D2 // Settings Dip-switch - Slow or Fast movement
#define SETTING_3 D4 // Settings Dip-switch - Position or Velocity control
#define SETTING_4 D8 // Settings Dip-switch - PPM / SBUS

//These options are selected vis DIP Switches on the board.
#define DXL_RANGE_LONG   22500 //Range of movement from -DXL_RANGE to +DXL_RANGE.  Roughly +/- 5 turns
#define DXL_RANGE_SHORT   2250 //Range of movement from -DXL_RANGE to +DXL_RANGE.  This is about right for +/-180 deg as the Tx doesn't output the full 500 to 2500uS range

#define DXL_SPEED_SLOW   100 //values from 1 to 255.  1 is V-SLOW!  0 to set max speed  
#define DXL_SPEED_FAST   200 //250 gives an OK, not to agressive speed

#define MIN_PULSE_WIDTH 1000  // Minimum pulse width in microseconds
#define MAX_PULSE_WIDTH 2000 // Maximum pulse width in microseconds

int DXL_RANGE = DXL_RANGE_SHORT; //Default to single turn
int DXL_SPEED = DXL_SPEED_SLOW;

bool SBUS_Enable = false;

// Global arrays to store channel values
int16_t channels[16];
bool validID[16];
// Create an array of SimpleKalmanFilter objects
SimpleKalmanFilter kalmanFilter[16] = {
  SimpleKalmanFilter(1, 1, 0.01),  // Initialize each filter object with your desired parameters
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01), 
  SimpleKalmanFilter(1, 1, 0.01) 
};
int smoothedPosition[16]; // Start at neutral position
long positionOffset[16];
long initialPulseWidth[16];
int validCount = 0; //Count of valid ID's

/* Create a custom HardwareSerial instance on SERCOM2 */
//Uart SerialSBUS(&sercom2, 0, 1, SERCOM_RX_PAD_3, UART_TX_PAD_2);
Uart SerialSBUS(&sercom0, A9, A10, SERCOM_RX_PAD_1, UART_TX_PAD_2);

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&SerialSBUS);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&SerialSBUS);
/* SBUS data */
bfs::SbusData data;

uint8_t DXL_ID = 0; //Typically, this will be 1 but will be set to the first DXL ID found

const float DXL_PROTOCOL_VERSION = 2.0;
int Smoothness = 0;

unsigned long startupTimer = 0;

bool postStartup1 = false;
bool postStartup2 = false;

unsigned long errorTimeout = millis(); //Timeout if DXL or PPM fails while running






unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 20; // Update every 20ms (50Hz)

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

using namespace ControlTableItem;

const int receiverPin = 0;  // Digital pin D0 connected to the RC receiver

volatile unsigned long startTime = 0;  // Variable to store the start time of the pulse
volatile unsigned long pulseWidth = 0; // Variable to store the pulse width
unsigned long currentPulseWidth;




// Function to reset micros counter
void resetMicros() {
    noInterrupts();       // Disable interrupts
    SysTick->VAL = 0;     // Clear the SysTick timer value
    interrupts();         // Enable interrupts
}

// ISR for handling the rising and falling edges of the pulse
void handlePulse() {
    if(micros()> 60*60*1000000) resetMicros();        // Reset micros every 60 mins

    if (digitalRead(receiverPin) == HIGH) {
        // Rising edge detected
        startTime = micros(); // Record the start time
        
    } else {
        // Falling edge detected
        unsigned long width = micros() - startTime; // Calculate pulse width
        
        // Constrain pulse width to expected range and update pulseWidth
        if (width >= 500 && width <= 2500) {
            pulseWidth = width;
            digitalWrite(PIN_LED3, HIGH); //LED Off
        } else {
            //pulseWidth = 0; // Set to 0 to indicate an out-of-bounds error
            digitalWrite(PIN_LED3, LOW); //LED On
        }
    }
}

void setup() {
  Serial.begin(115200); // Initialize serial communication
  delay(2000);
  pinMode(receiverPin, INPUT_PULLUP); // Set the receiver pin as an input
  pinMode(D9, INPUT); // Set the receiver pin as an input
  pinMode(LED_BUILTIN, OUTPUT); // Set the receiver pin as an input
  pinMode(PIN_LED2, OUTPUT); // Set the receiver pin as an input
  pinMode(PIN_LED3, OUTPUT); // Set the receiver pin as an input

  //Lightshow to indicate everything is running
  digitalWrite(LED_BUILTIN, LOW); //Active Low
  digitalWrite(PIN_LED2, LOW); //Active Low
  digitalWrite(PIN_LED3, LOW); //Active Low

  //Settings Pins
  pinMode(SETTING_1, INPUT_PULLUP); // Set the receiver pin as an input
  pinMode(SETTING_2, INPUT_PULLUP); // Set the receiver pin as an input
  pinMode(SETTING_3, INPUT_PULLUP); // Set the receiver pin as an input
  pinMode(SETTING_4, INPUT_PULLUP); // Set the receiver pin as an input

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);

  Serial.println("Setup SBUS to Dynamixel");


  //Are there any Dynamixels connected?
  for(int id=1; id<17;id++){
    if(dxl.ping(id)){
       Serial.print(" *** Dynamixel found 57kbps with ID = "); Serial.println(id); 
       if(DXL_ID==0) DXL_ID = id; //Set default ID to first servo found
       validID[id-1] = true;
       validCount++; //kep track of number of dynamixels found
       delay(20);
    }    
  }

  if(validCount==0) {
    //try again but using higher serial rate
    dxl.begin(115200);
    for(int id=1; id<17;id++){
      Serial.print(" *** Dynamixel found 115kbps with ID = "); Serial.println(id); 
      if(dxl.ping(id)){
        Serial.print(" *** Dynamixel found"); 
        if(DXL_ID==0) DXL_ID = id; //Set default ID to first servo found
        validID[id-1] = true;
        validCount++; //kep track of number of dynamixels found
        delay(200);
      }
    }
  }

  Settings();

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  //250 Equates to about 90 deg per second
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 250);

  Serial.println("Wait for valid Dynamixel Comms ");
  for(int i=0; i<100;i++){
    for(int id = 1; id<17; id++){
      if(validID[id-1]){
        //Serial.print("-");
        //Read current position as initial value
        smoothedPosition[id-1] = dxl.getPresentPosition(id);
        if(smoothedPosition[id-1]==0) {
          //This id is not valid after all.  set to invalid
          validID[id-1] = false;
          validCount--; 
        } else {
          //Serial.print("DXL Position: "); Serial.print(smoothedPosition[id]);
          kalmanFilter[id-1].updateEstimate(smoothedPosition[id-1]); //Pre-fill Kalman filter window
        }
      }
    }
  }
  digitalWrite(LED_BUILTIN, HIGH); //Active Low

  Settings();

  for(int id = 0; id<16; id++){
    if(validID[id]){
      positionOffset[id] = dxl.getPresentPosition(id+1);  //Position the DXL thinks it's in.  Treat this as zero
    }
  }


  if(SBUS_Enable){
    //Setup pins for Serial2
    digitalWrite(PIN_LED2, HIGH); //Active Low
    pinPeripheral(A10, PIO_SERCOM_ALT);
    pinPeripheral(A9, PIO_SERCOM_ALT);
    SerialSBUS.begin(100000, SERIAL_8E2); // 100000 baud, 8E2 configuration

    sbus_rx.Begin();
    //sbus_tx.Begin();

    while(!sbus_rx.Read()); //wait for data
    data = sbus_rx.data();
    for (int8_t id = 0; id < data.NUM_CH; id++) {
      int IDPlus = id+1;
      if(validID[id]){ //Only update valid ID's
        initialPulseWidth[id] = data.ch[id];

        Serial.print("InitialPulseWidth("); Serial.print(id+1); Serial.print(") = "); Serial.println(initialPulseWidth[id]);
        //long position = map(smoothedPosition[IDPlus], 0, 2000, -DXL_RANGE, DXL_RANGE) + positionOffset[IDPlus];      
        //initialPulseWidth[IDPlus] = position;

        //SBUS Values range from 0 to 2000. 
        long initialPosition = map(initialPulseWidth[id], 0, 2000, -DXL_RANGE, DXL_RANGE);
        Serial.print("InitialPosition = "); Serial.println(initialPosition);
        //Two options for Position offset:
        //   1: power-up position of dynamixel is zero
        //or 2: startup position is proportional to RC
        //Choose one or the other
        //positionOffset = positionOffset - initialPosition;  //Use dynamixel & Transmitter power up position as zero 

        positionOffset[id] = -(initialPosition % 4096); //Use absolute position of transmitter
        Serial.print("positionOffset("); Serial.print(id+1); Serial.print(") = "); Serial.println(positionOffset[id]);

      }      
    }   
  

  } else {
    digitalWrite(PIN_LED2, HIGH); //Active Low
    // Attach interrupt to the receiver pin for both rising and falling edges
    attachInterrupt(digitalPinToInterrupt(RC_PIN), handlePulse, CHANGE);

    //Wait until RC Signal valid
    Serial.println("Wait for valid PPM Signal ");
    while(initialPulseWidth[DXL_ID] == 0) {
      Serial.print(".");
      noInterrupts(); // Ensure atomic read
      if(pulseWidth>1000) {
        initialPulseWidth[DXL_ID] = pulseWidth;
      } else {
        initialPulseWidth[DXL_ID] = 0;
      }
      interrupts();
    }
    long initialPosition = map(initialPulseWidth[DXL_ID], MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, -DXL_RANGE, DXL_RANGE);
    //Two options for Position offset:
    //   1: power-up position of dynamixel is zero
    //or 2: startup position is proportional to RC
    //Choose one or the other
    //positionOffset = positionOffset - initialPosition;  //Use dynamixel & Transmitter power up position as zero 

    positionOffset[DXL_ID] = -(initialPosition % 4096); //Use absolute position of transmitter
    Serial.println("");
    Serial.print("Position Offset: "); Serial.println(positionOffset[DXL_ID]);

    
    digitalWrite(PIN_LED2, HIGH); //Active Low
  }


  startupTimer = millis();

}

void loop() {
  // The main loop can perform other tasks while pulseWidth is updated in the background

  if((millis()-startupTimer) > 2000) postStartup1 = true;
  if((millis()-startupTimer) > 5000) postStartup2 = true;



  //DXL Velocity initially slow.  Once everything has started, increase to running velocity
  if(postStartup2){
    for (int8_t id = 1; id < 16; id++) {
      if(validID[id-1]){
        dxl.writeControlTableItem(PROFILE_VELOCITY, id, DXL_SPEED);
      }
    }
  }

  if (SBUS_Enable) {
    //SBUS has been enabled (Setting switch 4)
      decodeSBUSChannels();
  } else {
  // The main loop can perform other tasks while pulseWidth is updated in the background
    noInterrupts(); // Ensure atomic read
    if(pulseWidth>500) {
      currentPulseWidth = pulseWidth;
    } else {
      currentPulseWidth = 0;
    }
    interrupts();


    if((currentPulseWidth == 0) || (validCount==0)){ 
      if(millis()-errorTimeout > 5000){
      //Lost RC or Dynamixel for 5 seconds+
      //Reboot
      NVIC_SystemReset();
      }
    } else {
      errorTimeout = millis();
    }


    // Apply smoothing and update servo position
    if ((millis() - lastUpdateTime >= updateInterval)&&(currentPulseWidth>0)) {
      //updateSmoothedPosition(currentPulseWidth);

      int rawAnalog = analogRead(SMOOTHING_PIN);
      Smoothness = map(rawAnalog, 0, 1024, 0, 100);

      
      //Convert PPM pulse length range to SBUS equivelent
      currentPulseWidth = constrain(currentPulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
      currentPulseWidth = map(currentPulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 200, 1800);

      smoothedPosition[DXL_ID] = smoothSignal(DXL_ID,currentPulseWidth, Smoothness);


      long position = map(smoothedPosition[DXL_ID], 200, 1800, -DXL_RANGE, DXL_RANGE) + positionOffset[DXL_ID];

      //Don't update the servo for a couple of seconds at startup.  Allow smoothing buffer to fill
      if(postStartup1){
        setDynamixel(DXL_ID, position);
        delay(1);
      }
      lastUpdateTime = millis();
    }
  }
}


// SERCOM2 handler for SerialSBUS - used to read SBUS
extern "C" {
  void SERCOM0_Handler(void) {
    SerialSBUS.IrqHandler();
  }
}






// Function to decode SBUS channels
void decodeSBUSChannels() {
int rawAnalog = analogRead(SMOOTHING_PIN);

  Smoothness = map(rawAnalog, 0, 1024, 0, 100);
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    for (int8_t id = 0; id < data.NUM_CH; id++) {
      if(validID[id]){ //Only update valid ID's
        int sbusPos = constrain(data.ch[id],200,1800); 
        smoothedPosition[id] = smoothSignal(id,sbusPos, Smoothness);        
        //Serial.print("ID:"); Serial.print(id); Serial.print(" = "); Serial.print(smoothedPosition[id]); Serial.print(" | ");
        long position = map(smoothedPosition[id], 200, 1800, -DXL_RANGE, DXL_RANGE) + positionOffset[id];
        //Serial.print("ID:"); Serial.print(id); Serial.print(" = "); Serial.print(position); Serial.print(" | ");
        //Serial.print("DXL Pos: "); Serial.println(dxl.getPresentPosition(id+1));
        if(postStartup1) setDynamixel(id+1, position);  //Allow smoothing to settle before sending data to servo
        delay(1);
        //Serial.println();
      }      
    }
       
  }
}


void Settings() {
//Read the settings DIP Switch
  pinMode(SETTING_1, INPUT_PULLUP); 
  pinMode(SETTING_2, INPUT_PULLUP); 
  pinMode(SETTING_3, INPUT_PULLUP); 
  pinMode(SETTING_4, INPUT_PULLUP); 

  //DIP SWITCH 1 = Single / Multi-turn
  if(digitalRead(SETTING_1)==HIGH){
    DXL_RANGE = DXL_RANGE_LONG; //10 Turn
    Serial.println("SETTING Range = 10 turn");
  } else {
    DXL_RANGE = DXL_RANGE_SHORT; //single turn
    Serial.println("SETTING Range = Single turn");
  }
  
  //DIP SWITCH 2 = Slow / Fast.  
  if(digitalRead(SETTING_2)==LOW){
    DXL_SPEED = DXL_SPEED_SLOW; //50% speed
    Serial.println("SETTING Speed = 50%");
  } else {
    DXL_SPEED = DXL_SPEED_FAST; //100% speed
    Serial.println("SETTING Speed = 100%");
  }

  //DIP SWITCH 3 = Position / Wheel Mode.  
  if(digitalRead(SETTING_3)==HIGH) Serial.println("SETTING Mode = Velocity");
  if(digitalRead(SETTING_3)==LOW) Serial.println("SETTING Mode = Position");


  for (int8_t id = 0; id < data.NUM_CH; id++) {
    if(validID[id]){ //Only update valid ID's
      if(digitalRead(SETTING_3)==LOW){
        dxl.torqueOff(id+1);
        dxl.setOperatingMode(id+1, OP_EXTENDED_POSITION); //Multi turn - Position range -1,000,000 to +1,000,000 ish.  4096 per 360 degree revolution
        dxl.torqueOn(id+1);
        dxl.writeControlTableItem(PROFILE_VELOCITY, id+1, DXL_SPEED);
      } else {
        dxl.torqueOff(id+1);
        dxl.setOperatingMode(id+1, OP_VELOCITY); //Wheel mode
        dxl.torqueOn(id+1);
        dxl.writeControlTableItem(PROFILE_VELOCITY, id+1, DXL_SPEED);
      }
    }
  }
  //DIP SWITCH 4 = Servo test function  
  if(digitalRead(SETTING_4)==HIGH){
    //Switch is ON
    SBUS_Enable = true;
    Serial.println("SETTING Input = SBUS");
  } else {
    //Switch is OFF
    SBUS_Enable = false;
    Serial.println("SETTING Input = PPM");
  }

  int rawAnalog = analogRead(SMOOTHING_PIN);
  Smoothness = map(rawAnalog, 0, 1024, 0, 100);

  Serial.print("Smoothing = "); Serial.print(Smoothness); Serial.println("%");
 
}

void setDynamixel(int dxl_id, int posVel){
//Set position or velocity depending on current mode
  if(digitalRead(SETTING_3)==LOW){
    //Position
    dxl.setGoalPosition(dxl_id, posVel);
  } else {
    //Velocity
    //posVel has a range of -2024 to +2024.  Convert this to -32767 to 32767
    long dxlVelo = constrain(posVel,-2048,2048);
    dxlVelo = map(posVel,-2048,2048,-250,250);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 0);
    //Add a hysteresis band
    if(abs(dxlVelo)<20) dxlVelo = 0;
    if(dxlVelo>0) dxlVelo = dxlVelo-20;
    if(dxlVelo<0) dxlVelo = dxlVelo+20;
    dxl.setGoalVelocity(dxl_id, dxlVelo);  // Set speed in wheel mode
    //Serial.print("DXL Velocity = "); Serial.println(posVel);
  }
}



void updateSmoothedPosition(int SBUS_ID, long pulseWidth) {
  // Read and scale analog input
  int rawAnalog = analogRead(SMOOTHING_PIN);
  Smoothness = map(rawAnalog, 0, 1024, 0, 100);

  // Constrain pulseWidth to extended RC range
  int targetPosition = constrain(pulseWidth, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  
  // Calculate smoothing factor (0.0 to 1.0)
  float smoothingFactor = 1.0 - (Smoothness / 100.0);
  
  // Apply smoothing
  smoothedPosition[SBUS_ID] = smoothedPosition[SBUS_ID] * (1 - smoothingFactor) + targetPosition * smoothingFactor;
    
}



float smoothSignal(int SBUS_ID, float noisyData, unsigned int smoothingPercentage) {
    
    
    // Ensure smoothingPercentage is within 0-100 range
    smoothingPercentage = constrain(smoothingPercentage, 0, 100);
    
    // Calculate the measurement noise based on the smoothing percentage
    float measurementNoise = map(smoothingPercentage, 0, 100, 0.01,200);
    kalmanFilter[SBUS_ID].setMeasurementError(measurementNoise);
    // Apply the Kalman filter
    float smoothedValue = kalmanFilter[SBUS_ID].updateEstimate(noisyData);
    
    // Return the smoothed value as an unsigned long
    return smoothedValue;
}



