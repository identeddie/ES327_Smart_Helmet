#include <LoRa_APP.h>
#include <Arduino.h>
#include <ADS1X15.h>
#include <EEPROM.h>
#include <Adafruit_PM25AQI.h>
#include <MPU6050_light.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//------------------------------------------------------ DEFINITIONS -------------------------------------------------------
 
#define LoraWan_RGB 1

#define DEFAULT_RF_FREQUENCY                        864000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 15        // Payload Size
#define numStored                                   5

#define lifeColor           0x7D0097
#define warningColor        0xE9F210
#define defconColor         0xE70000
#define immobileColor       0x0038D6
#define sensorFaultColor    0x21C600

//#define pmData "010510255010"

//------------------------------------------------------ GLOBALS -------------------------------------------------------

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
char transfer[BUFFER_SIZE];

char packetBuffer[numStored][BUFFER_SIZE];

static RadioEvents_t RadioEvents;

void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
void OnRxTimeout( void );
bool checkBuffer(void);

typedef enum
{
    LOWPOWER,
    RX,
    TX
}States_t;

States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;

long timer = 0;
long txTimer = 0;
long procTimer = 0;
long lifeTimer = 0;
long sensorTimer = 0;

uint32_t currentFreq = DEFAULT_RF_FREQUENCY;

char prot;
char flags;

int bufferCount;
int teamSize;
char teamLifeCheck;

char lifeCheck;
int group;
bool leadStatus;

char setupValidation;
bool assigned;
bool localized;
bool firstNormalMessage;
bool pmTime;
bool transferTime;
bool packetsInBuffer;
bool lifeCheckTime;

MPU6050 mpu(Wire);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
ADS1115 ADS(0x48);
float f = 0;
float gasMA [30];
float gasAvg = 0;
int idxGas = 0;
float sumGas = 0;

char pmPacket[12];

float mpuMA[60];
int idx = 0;
float sum = 0;
float avg = 0;
float stdDev = 0;
float net = 0;


//                  SETUP AREA
//-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-//
                                                                 //
#define nodeCount    4               // Number of Nodes          //
#define ID           3               // Individual Node ID       //
                                                                 //
char setupSignal[] = "8081010100";   //Init script for Nodes     //
                                                                 //
#define mpuStaticThreshold 0.02                                  //
#define mpuBlastThreshold  3                                     //
#define gasThreshold       2.4                                   //
                                                                 //
//-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-//

/*  Protocol List
 *  0 - Waiting
 *  1 - Life Check
 *  2 - Assignment
 *  3 - Standard Operation
 *  4 - Defcon 1
 *  5 - Defcon 2
 */

/////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

    delay(100*ID); //NO IDEA

    Wire.begin();
    byte status = mpu.begin();
    mpu.calcOffsets(true,true); // gyro and accelerometer

    ADS.begin();
    ADS.setGain(0);
    f = ADS.toVoltage();
    ADS.requestADC(0);
  
    Serial.begin(115200);
    EEPROM.begin(1); //Start EEPROM with 1 byte, for team/lead assignment

    //FOR TESTING ONLY
    EEPROM.write(0, 0);

    Serial.print("ID: ");
    Serial.println(ID);

    assigned = false;
    localized = false;
    firstNormalMessage = true;
    pmTime = false;
    transferTime = false;
    packetsInBuffer = false;
    lifeCheckTime = false;

    txTimer = millis();

    Rssi=0;
    prot = '0';

    bufferCount = 0;
    teamSize = 0;
    teamLifeCheck = 0;

    flags = 0x00;
    lifeCheck = 0x00;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;

    Radio.Init( &RadioEvents );
    Radio.SetChannel(currentFreq);
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    randomSeed(Radio.Random()); //Generate random seed with radio

    char storedSetup = EEPROM.read(0); //Collect saved init data from EEPROM

    //All nodes start in RX mode, besides admin node
    if (ID == 0) {
      state = TX;
      prot = '1'; //Admin node starts by initilaizing life check
    } else if (storedSetup) {
      leadStatus = storedSetup & (1 << 7); //Get node's leader status
      storedSetup &= 0x7F; //Remove leader bit with mask
      group = (int)storedSetup; //Rest of bits refer to group (team) number
      assigned = true;
      state = RX;
      Serial.println("Recovered Assignment");
    } else {
      state = RX;
    }

    turnOffRGB();
}

/////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  //If statements for LED indication
  if ((prot == '1') || ((prot == '3') && (flags & (1 << 4)))) {
    turnOnRGB(lifeColor, 0); //Life Check Color (Pink)
  } else if ((prot == '3') && (flags & (1 << 1))) {
    turnOnRGB(immobileColor, 0); //Immobility Color (Blue)
  } else if (prot == '4') {
    turnOnRGB(warningColor, 0); //Warning Color (Yellow)
  } else if (prot == '5') {
    turnOnRGB(defconColor, 0);//Evacuation Color (Red)
  }

  //TX timing
  if ((millis() - txTimer) >= (random(3000, 6000))) {
    state = TX; //Go into TX time at a random time
  } else {
    state = RX; //Otherwise default to RX
  }

  if (prot == '0') {
    state = RX; //Overwrite to RX if protocol is 0

  } else if ((prot - '0') >= 3) {
    //if ((millis() - sensorTimer) >= 1000) {
      sensorTimer = millis();

      mpu.update(); //Update gyro/accel readings
      net = sqrt(mpu.getAccX()*mpu.getAccX() + mpu.getAccY()*mpu.getAccY() + mpu.getAccZ()*mpu.getAccZ()); //Get resultant acceleration vector

      sum -= mpuMA[idx]; //Remove previous value from sum, if exists
      mpuMA[idx] = net; //Update current cell with reading
      sum += mpuMA[idx]; //Add new reading to sum
      idx = (idx + 1) % 60; //Update index for future use. 30%30 wraps to index 0
      avg = sum / 60; //Get average of stored readings

      float var = 0; //From moving averege, get standard deviation
      for (int i = 0; i < 60; i++) {
        var += pow((mpuMA[i] - avg), 2);
      }
      var /= 60;
      stdDev = sqrt(var);

      if (ADS.isBusy() == false) {  //Asynchronous data collection for MiCS-5524
        int16_t gasRead = ADS.getValue();
        ADS.requestADC(0);

        sumGas -= gasMA[idxGas]; //Same process as mpu6050
        gasMA[idxGas] = gasRead * f; //multiply by f to get value as a voltage
        sumGas += gasMA[idxGas];
        idxGas = (idxGas + 1) % 30;
        gasAvg = sumGas / 30;
      }
    //}
    
    /*if ((net > mpuBlastThreshold) || (stdDev < mpuStaticThreshold)) { //static threshold should be < !!!!!!!!!!!
      Serial.println("MPU HAZARD DETECTED!");
      flags |= 1; //Set movement flag
      if (ID == 0) {
        flags &= 0; //Overwrite for Admin
      }
    } else if (gasAvg > gasThreshold) {
      Serial.println("GAS HAZARD DETECTED!");
      flags |= 1 << 1; //Set gasWarn bit
    }*/
         
    if (millis() - timer > (120000 + (rand() % 60000))) { //Gather PM readings every 15 minutes SHOULD be 900000 on FIRST
      flags |= (1 << 3);
      Serial.println("It is PM Time");
      pmTime = true;
      timer = millis();
    } else if (pmTime && (millis() - timer > 15000)) {
      Serial.println("PM Time Over");
      pmTime = false;
      flags &= (0 << 3);
    }

    if (ID == 0) { //Overwrite PM Time toggle if admin
      pmTime = false;
      flags &= (0 << 3);
    }

    if (pmTime) { //Read data from PMSA003I
      PM25_AQI_Data data;
      aqi.read(&data);

      uint16_t pmData[6];
      pmData[0] = (uint16_t)data.particles_03um; //Make sure all readings are 16-bit (for two byte packet structure)
      pmData[1] = (uint16_t)data.particles_05um;
      pmData[2] = (uint16_t)data.particles_10um;
      pmData[3] = (uint16_t)data.particles_25um;
      pmData[4] = (uint16_t)data.particles_50um;
      pmData[5] = (uint16_t)data.particles_100um;

      for (int i = 0; i < 6; i ++) { //Split each reading into bytes (chars)
        char low = pmData[i] & 0xFF;
        char high = pmData[i] >> 8;
        pmPacket[i*2] = high;  //Fill accordingly into pmPacket
        pmPacket[i*2+1] = low;
      } 
    }
  }
  

  //Buffer process timing
  if ((millis() - procTimer) >= (random(1500, 3000)) && (packetsInBuffer)) { //Process First In Packet in buffer
    rxProcess();
    procTimer = millis();
  }

  //LifeCheck Initializer
  if (((prot - '0') == 3) && leadStatus && (ID != 0) && !lifeCheckTime) {
    if ((millis() - lifeTimer) >= 60000) {
      flags |= (1 << 4);
      lifeCheckTime = true;
      lifeCheck |= (1 << ID);
      Serial.println("Initializaing local Life Check.");
    }
  }

  if (!flags && (prot == '3') && !leadStatus) { //Forces to RX if nothing going on
    state = RX;
  }
  
  switch(state)
  {
    case TX:
        sprintf(txpacket,"%c",prot); //Assigns current protocol to first character of packet

        //Different packet structure for each protocol

        //Prot 1: Life Check
        if ((prot - '0') == 1) {
          sprintf(txpacket+strlen(txpacket), "%c", lifeCheck); //Append lifeCheck with relevant validation bits filled

        //Prot 2: Initialization
        } else if ((prot - '0') == 2) {
          if ((ID == 0) && !assigned) { //Admin node processes assignment packet to setup nodes
            char *out = initProcessing();
            sprintf(txpacket+strlen(txpacket),"%s", out);
            free(out);
          } else if (assigned) {
            memcpy(txpacket, transfer, BUFFER_SIZE); //Otherwise, forward initialization packet
            txpacket[nodeCount + 1] = setupValidation;
          }

        //Prot 3: Normal
        } else if ((prot - '0') == 3) {
          if (!flags) {
            sprintf(txpacket+strlen(txpacket),"%d", group); //First character after protocol indicates group
            sprintf(txpacket+strlen(txpacket),"%c", flags); //Second character after protocol inidicates Flag/Warning Bits
          } else if (lifeCheckTime) {
            sprintf(txpacket+strlen(txpacket),"%d", group); //First character after protocol indicates group
            sprintf(txpacket+strlen(txpacket),"%c", flags); //Second character after protocol inidicates Flag/Warning Bits
            sprintf(txpacket+strlen(txpacket),"%c", lifeCheck); //Send lifeCheck bits
          } else if (transferTime) {
            memcpy(txpacket, transfer, BUFFER_SIZE); //If already sent just transfer recieved PM packets
            transferTime = false;
          } else if (pmTime) {
            sprintf(txpacket+strlen(txpacket),"%d", group);
            sprintf(txpacket+strlen(txpacket),"%c", flags);
            sprintf(txpacket+strlen(txpacket),"%s", pmPacket);
          }
          
        } else if ((prot - '0') == 4) {
          if (flags & (1 << 2)) {
            sprintf(txpacket+strlen(txpacket),"%c", 0x01); //Gas warning
          } else if (flags & 1) {
            sprintf(txpacket+strlen(txpacket),"%c", 0x02); //Immobility warning
          }
        } else if ((prot - '0') == 5) {
        }

        while (!(Radio.IsChannelFree(MODEM_LORA, currentFreq, -100, 1))) {
          delay(1);
        }
        Serial.printf("\r\nsending packet \"%s\"\r\n", txpacket);
        Radio.Standby();
        Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
        delay(100);
        Radio.Rx(0);

        //state = LOWPOWER;
        break;
    case RX:
        //turnOnRGB(0,0);
        Radio.Rx(0);
        //state = LOWPOWER;
        break;
    case LOWPOWER:
      //lowPowerHandler();
        break;
        default:
            break;
  }
    Radio.IrqProcess( );
}

//------------------------------------------------------ LORA INTERRUPT FUNCTIONS -------------------------------------------------------

void OnTxDone(void) {
  Serial.println("TX done......");
  txTimer = millis();
  /*if ((prot == '3') && (leadStatus == 1)) {
    currentFreq = currentFreq + (200000 * group);
    Radio.SetChannel(currentFreq);
  }*/
        
  state=RX;
}

void OnTxTimeout(void) {
  Serial.println("TX Timeout......");
  state=RX;
}

void OnRxTimeout(void) {
  Serial.println("Didn't recieve anything");
  delay(rand() % 1001);
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) {
  Rssi = rssi;
  rxSize = size;
  
  memcpy(rxpacket, payload, size ); //Copy over packet recieved for processing
  Serial.printf("\r\nreceived packet:\"%s\" Rssi %d, Length %d\r\n",rxpacket, Rssi, rxSize);
  
  if ((bufferCount < numStored) && (rxpacket[0] >= prot)) {
    if ((bufferCount == 0) || ((bufferCount > 0) && checkBuffer())) {
      memcpy(packetBuffer[bufferCount], rxpacket, size ); //Copy over packet recieved for processing
      packetBuffer[bufferCount][rxSize] = '\0';
      //Radio.Sleep( ); //Sleep radio to pause communications
      
      bufferCount++;
      packetsInBuffer = true;
  
      //Serial.println("wait to send next packet");
      //state=TX;
    } else {
      Serial.println("Packet Match Found in Buffer.");
    }
  } else {
    Serial.println("Buffer Filled or Protocol Ingored.");
  }
}

//------------------------------------------------------ RX PACKET PROCESSING -------------------------------------------------------

void rxProcess() {
  Radio.Sleep();
  char temp;
  strncpy(&temp, packetBuffer[0], 1); //Take first character from packet (protocol)

  if ((temp - '0') > (prot - '0')) { //If recieved packet protocol is of higher priority, increase local protocol
    prot = temp;    
  }

  Serial.printf("Protocol: %c\r\n", prot);
  
  // Life Check Protocol
  if ((prot - '0') == 1) {
    Serial.print("lifeCheck IN: ");
    Serial.println(packetBuffer[0][1], BIN);  //Process relevant LifeCheck bytes

    packetBuffer[0][1] |= (1 << ID); //Fill Life unique ID dependent acknowledgement bit
    
    if ((lifeCheck != packetBuffer[0][1])) { //If local LifeCheck bits are not identical to recieved pakcet, merge
      lifeCheck |= packetBuffer[0][1];
    }

    Serial.print("lifeCheck OUT: ");
    Serial.println(lifeCheck, BIN);
    
    if ((packetBuffer[0][1] == 0x0F) && (ID == 0)) { //Only admin node can finalize a life check
      Serial.println("All alive");
      prot = '2'; //Advance to Initialization stage
    }

  // Initialization Protocol
  } else if (((prot - '0') == 2) && ((temp - '0') == 2)) {
    if (!assigned && (rxSize > 2)) { //If not assigned then follow steps for assignment

      lifeCheck = 0x00; //Clear lifeCheck bits for future reassessments
      
      char setup_in;

      memcpy(transfer, packetBuffer[0], BUFFER_SIZE);
      Serial.printf("Payload Transfered: %s\r\n", transfer);
    
      strncpy(&setup_in, (packetBuffer[0] + ID + 1), 1); //Extract node specific assignment character
      strncpy(&setupValidation, (packetBuffer[0] + nodeCount + 1), 1); //Extract assignment validation bits

      EEPROM.write(0, setup_in); //Write assignment byte to EEPROM to allow for reassignment after power loss
      EEPROM.commit();

      Serial.print("Setup bits: ");
      Serial.println(setup_in, BIN);
      Serial.print("Validation bits: ");
      Serial.println(setupValidation, BIN);

      if (!(setupValidation & (1 << ID))) { //If device specific validation bit not filled, need to assign
        setupValidation |= (1 << ID); //Fill validation bit

        leadStatus = setup_in & (1 << 7); //Get node's leader status
        setup_in &= 0x7F; //Clear the leader indicator bit
        group = (int)setup_in; //Rest of bits refer to group (team) number

        assigned = true; 

        Serial.printf("Leader? %d, ", leadStatus);
        Serial.printf("Group: %d\r\n", group);
        if (leadStatus) {
          teamLeadInstructions();
        }
      }
    } else {
      //If already assigned copy recieved packet and update validation bits
      setupValidation |= packetBuffer[0][nodeCount+1];
      Serial.printf("Assignment Validation Bits: ");
      Serial.println(packetBuffer[0][nodeCount+1], BIN);
    }

    if ((setupValidation == 0x0F) && (ID == 0)){ //Only admin node can finalize initialization
      prot = '3';
      Serial.println("All assigned");
      timer = millis();
    }

  //Normal Protocol
  } else if (((prot - '0') == 3) && ((temp - '0') == 3)) {

    if (firstNormalMessage) {
      firstNormalMessage = false;
      timer = millis();
    }

    assigned = false; //Reset assign in case of reassignment
    
    if (localized == false) { //If not localized, go to team's channel/sub-band 
      currentFreq = currentFreq + (200000 * group);
      Radio.SetChannel(currentFreq); //Unique group channel (tested bandwidth is 200 kHz (not 125 kHz)
      Serial.print("Group Channel: ");
      Serial.println(currentFreq);
      localized = true;
    } else if (prot == '5') {
      currentFreq = DEFAULT_RF_FREQUENCY;
      Radio.SetChannel(DEFAULT_RF_FREQUENCY);
      localized = false;
    }
    
    strncpy(&temp, (packetBuffer[0] + 1), 1); //Get Group of sender
    char errTemp;
    strncpy(&errTemp, (packetBuffer[0] + 2), 1); //Get Flags/Error Codes
    
    if ((packetBuffer[0][2] & 1) || (packetBuffer[0][2] & (1 << 2))) { //If any cross-val (gas) or gyro flags are raised it is an issue
      prot = '4';
    } else if (!(packetBuffer[0][2] | (0 << 4)) && lifeCheckTime) {
      lifeCheckTime = false;
      lifeCheck = 0;
      flags &= (0 << 4);
    } else if (packetBuffer[0][2] & (1 << 4)) {  //INTERNAL TEAM LIFE CHECK
      if (!leadStatus) {
        lifeCheckTime = true;
        flags |= (1 << 4);
      }
      Serial.println("Internal Life Checking Mode");
      Serial.print("IN: ");
      Serial.println(packetBuffer[0][3], BIN);
      
      packetBuffer[0][3] |= (1 << ID); //Fill Life unique ID dependent acknowledgement bit
      lifeCheck |= packetBuffer[0][3];

      Serial.print("OUT: ");
      Serial.println(lifeCheck, BIN);

      if ((packetBuffer[0][3] == teamLifeCheck) && leadStatus) { //Only admin node can finalize a life check
        lifeCheckTime = false;
        lifeCheck = 0;
        lifeTimer = millis();
        Serial.println("All alive in team.");
        flags &= (0 << 4);
      }
      
    } else {
      if ((packetBuffer[0][2] & 1) || (packetBuffer[0][2] & (1 << 1))) { //If cross-val already asserted or if movement flag raised
        prot = '4';
      } else if (errTemp & (1 << 1)) { //If gas warning bit active
        flags |= (1 << 1); //Raise cross-val flag
      } else if (packetBuffer[0][2] & (1 << 3)) { //Normal procedure (If PM flag is set)
        //PM DATA PROCESS
        memcpy(transfer, packetBuffer[0], BUFFER_SIZE);
        transferTime = true;

        if ((leadStatus == 1) && (ID != 0)) { //If a leader, go into admin channel and forward packet
          currentFreq = DEFAULT_RF_FREQUENCY;
          Radio.SetChannel(currentFreq);
          Radio.Standby();
          while (!(Radio.IsChannelFree(MODEM_LORA, currentFreq, -100, 1))) {
            delay(1);
          }
          Serial.println("Off to admin:");
          Radio.Send( (uint8_t *)transfer, strlen(txpacket) );
          delay(100);
          currentFreq = currentFreq + (200000 * group);
          Radio.SetChannel(currentFreq); //Unique group channel (tested bandwidth is 200 kHz (not 125 kHz)
          Radio.Rx(0);
        }

        if ((ID == 0) && (rxSize > 6)) { //Admin data collection process would go here, now just simulates with Serial
          transferTime = false;
          char pm[12];
          strncpy(pm, (packetBuffer[0] + 3), 12);
          Serial.printf("Recieved PM data: %s\r\n", pm);
        }
      }
    }
  } else if (((prot - '0') == 4) && ((temp - '0') == 4)) {

  } else if (((prot - '0') == 5) && ((temp - '0') == 5)) {
    EEPROM.write(0, 0);
    currentFreq = DEFAULT_RF_FREQUENCY;
    Radio.SetChannel(DEFAULT_RF_FREQUENCY);
    localized = false;
  }
  // Free the current packet
  for (int i = 0; i < (bufferCount - 1); i++) {
      memcpy(packetBuffer[i], packetBuffer[i + 1], BUFFER_SIZE);
      memset(packetBuffer[i+1], 0, BUFFER_SIZE);
  }
  bufferCount--;
  if (bufferCount == 0) {
    packetsInBuffer = false;
  }
}

//------------------------------------------------------ MISC FUNCTIONS -------------------------------------------------------

char * initProcessing() { //Converting Initialization string into processable char array
  int len = sizeof(setupSignal)/sizeof(*setupSignal);
  char *processed = (char*)malloc(len + 1);

  for (int i = 0, j = 0; i < len; i += 2, j++) {
    char part[3] = {setupSignal[i], setupSignal[i+1], '\0'};
    processed[j] = (char)strtol(part, NULL, 16);    
  }

  processed[len] = '\0';

  return processed;
}

bool checkBuffer() { //Checks through whole buffer to see if there is already an identical packet
  for (int i = 0; i < bufferCount; i++) {
    if (strcmp(packetBuffer[i], rxpacket) == 0) {
      return false;
    }
  }
  return true;
}

void teamLeadInstructions() {
  for (int i = 1; i < nodeCount + 1; i++) {
    packetBuffer[0][i] &= 0x7F;
    int tempTeam = (int)packetBuffer[0][i];
    if (tempTeam == group) {
      teamSize++;
      teamLifeCheck |= (1 << (i-1));
    }
  }
  Serial.printf("Current team will have %d members.\r\n", teamSize);
  Serial.print("This team's LifeCheck Key: ");
  Serial.println(teamLifeCheck, BIN);
}
