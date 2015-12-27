//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <EEPROM.h>

//Fona set-ups
//Fona libraries located here - C:\Program Files (x86)\Arduino\libraries\Adafruit_FONA
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>Gtm
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#ifdef __AVR__
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
#else
HardwareSerial *fonaSerial = &Serial1;
#endif
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

//Assign the Chip Select signal to pin 10.
 int8_t CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1
char values[10]; //This buffer will hold values read from the ADXL345 registers.

int x,y,z; //These variables will be used to hold the x,y and z axis accelerometer values.
int calibrationCount=12; // # times called calibrate fxn

const int numReadings = 20; //Number of readings for average
uint8_t numResults; //Number of readings to write out

int8_t mode = 0; //Operating Mode; 0=calibrate, 1=monitor, 2= collect, 3=flush and send buffer

//# times each mode should execute
int monitorDelay=500;
int writeDelay=100;

int monitorCount=0; //used to determine when to call processSMS()

//average z used to determine if event occurred
int zAverage = 0;

//event check
boolean eventCheck = false;

//EEPROM Config
int address;
int startAddress = 99; // start address for earthquake data, provides 100 bytes of configuration data

//Fona variables
uint8_t type; //Fona Type
char sendto[21]="4158069938"; //phone number to send SMS
char replybuffer[255]; //character buffer used for SMS communications

void setup(){ 
 //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

  resetError(); //resetting error counter
  
//Initialize FONA
  Serial.println("Start-up...");
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    addError ();
    while (1);
  }
  type = fona.type();
  strcpy(replybuffer,"FONA is OK");
  sendSMS(replybuffer,sendto);
  Serial.print(F("Found "));
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print(F("SIM card IMEI: ")); 
    Serial.println(imei);
  }

  //initialize GPRS, required to get network time
  initGPRS();

  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x00);
  
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  

  setNumResults(20); //intial # results to capture
  enableSMSSend (); //by default SMS messages are enabled
 
  deleteSMS(); //clean-up any SMS's on SIM card
  
}

void loop(){
  
  if (mode==0) {
    calibrateADXL(calibrationCount);
  }
  else if (mode ==1) {
    monitorEvent(monitorDelay);
    
    //drives frequency of SMS processing
    if (monitorCount <25) {monitorCount=monitorCount+1; } else {processSMS();
                                                                monitorCount=0;}
      }
  else if (mode ==2) {
      writeEventLog(getNumResults(),startAddress); //write out event log and send
  }
}

void calibrateADXL (int calCount) {
        Serial.println(F("Calibrating ADXL..."));
        delay(10);  //add small delay to ADXL reading
        int zPrior = getReading();
        zAverage=0; //reset zAverage

        for (int count = 0; count<calCount; count++) {
         
        if (quakeEvent(getReading(), zPrior)) {Serial.println(F("Not Stable..."));
                                               break; //stop calibration process
                                               }
          else {      
                  zAverage=calcAvgAccel (getReading(),count,zAverage);
                  delay(100); //short delay for callibration
        }
        }
        mode=1;
        Serial.println(F("Monitoring..."));
}
  
void monitorEvent(int mdelay) {
      eventCheck = quakeEvent(getReading(), zAverage); //check if seismic event has occurred
      printResults(); //print to terminal current x,y,z readings
      if (eventCheck == true) {mode= 2;
                              eventCheck=false;
                              strcpy(replybuffer,"Event detected");
                              if (getSMSSend()==1) {sendSMS(replybuffer,sendto);}
                                  }
      delay(mdelay);
}

void writeEventLog(int count, int start) {

       Serial.println(F("Write to eeprom"));
       int address=startAddress;
       
       for (int writeCount = 0; writeCount<count; writeCount++) {
         EEPROM.write(address, getReading()); //write to current address
         address = address + 1;  // advance to the next address
         if (address == 512) {break;} // Stop writing if address is out of bounds of EEPROM
       
         Serial.println(z, DEC);         
         delay(writeDelay);
       }
       
       Serial.println(F("Finished Writing"));
       sendEventLog(getNumResults()); //send event log
       mode=0; //re-callibrate ADXL      
}

void sendEventLog (int count) {

          Serial.println(F("Transmitting")); 
          int messageCounter=0; //counter used to break down messages into 140 byte chunks
          String value;
          char cValue[4]; //signed three digit character array
          int address=startAddress;
          int8_t valueInt; //value read which is appended to message, supports range of -128 to 127
                  
       for (int readCount = 0; readCount<count; readCount++) {

          valueInt = EEPROM.read(address); //get ADXL value
          value = String(valueInt,DEC);//convert int to string, length of 3 to 4 characters
          value.toCharArray(cValue,4); //copy value to CValue

          int i=0;
          while(i<value.length())
            {
            replybuffer[messageCounter+i]    = cValue[i];
            ++i;
            }

            replybuffer[ messageCounter + value.length()] = ',';  //add comma to seperate ADXL values
            messageCounter=messageCounter+value.length()+1;       //increment message counter by messageCounter+ lenght()

        if (messageCounter>140) {

          if (getSMSSend()==1) {sendSMS(replybuffer,sendto);}
              messageCounter=0;                                
        }
        
              address = address + 1;  // advance to the next address
       }
              if (getSMSSend()==1) {sendSMS(replybuffer,sendto);
                                    }
              mode=0;
}

void processSMS () {

  // read the number of SMS's!

          int8_t smsnum = fona.getNumSMS();

        if (smsnum < 0) {
          Serial.println(F("No SMS's"));
        } else {
          Serial.print(smsnum);
          Serial.println(F(" SMS's on SIM card!"));
        }

  for (int smsMessage = 1; smsMessage <=smsnum; smsMessage++) {

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsMessage, replybuffer, 250)) {
          Serial.println("Failed!");
          addError ();
        }

        Serial.print(F("FROM: ")); 
        Serial.println(replybuffer);

        //phone number
        //char phoneNumber[11];
        //strncpy(phoneNumber, replybuffer, 12);

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsMessage, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println(F("Failed to read SMS"));
          addError ();
        }

        //control type  
        char controlType[2];
        controlType[0]=replybuffer[0];
        controlType[1]=replybuffer[1];
        controlType[2]=replybuffer[2];
        
        Serial.print(F("Control Type:")); 
        Serial.println(controlType);

        //control value
        char controlValue;
        controlValue= replybuffer[4];

        deleteSMS(smsMessage); //remove the SMS message from queue
        
        if (!setControl(controlType,controlValue,smsMessage)) {

          strcpy(replybuffer,"Stupid commands!");
          sendSMS (replybuffer,sendto);

        }
       
         
 

  } //close for loop
}

boolean setControl (char controlT[2], char controlV, int smsMessage) {
  boolean match;

  if (strcmp(controlT, "Nmr")  == 0) {  
        setNumResults(controlV); //write ASCII value for # records
        strcpy(replybuffer,"Updating Results");
        sendSMS (replybuffer,sendto);
        match=true;
    } else if 
     (strcmp(controlT, "Sdl")  == 0) {  
        setWriteDelay(controlV); //write ASCII value for # records
        strcpy(replybuffer,"Setting Write Delay");
        sendSMS (replybuffer,sendto);
        match=true;
    } else if 
     (strcmp(controlT, "Res")  == 0) {  
        enableSMSSend();// set to SMS send mode
        strcpy(replybuffer,"Collecting");
        sendSMS (replybuffer,sendto);
        writeEventLog(getNumResults(),startAddress); //write out event log and send
        match=true;
    } else if 
     (strcmp(controlT, "SMS")  == 0) {  
        enableSMSSend(); //write out event log and send
        strcpy(replybuffer,"SMS enabled...");
        sendSMS (replybuffer,sendto); 
        match=true;
    } else if 
     (strcmp(controlT, "NMS")  == 0) {  
        disableSMSSend(); //write out event log and send
        strcpy(replybuffer,"SMS disabled");
        sendSMS (replybuffer,sendto); 
        match=true;
    } else if 
     (strcmp(controlT, "Rss")  == 0) {
       getRSSI(); //copy signal to reply buffer
       sendSMS (replybuffer,sendto); 
        match=true;
    } else if 
     (strcmp(controlT, "Gtm")  == 0) {  
       getTime(); //copy time to replybuffer
       sendSMS (replybuffer,sendto); 
       match=true;
    }
    else if 
     (strcmp(controlT, "Err")  == 0) {  
       getError();
       sendSMS(replybuffer,sendto); 
       match=true;
    }
    else if 
     (strcmp(controlT, "Rst")  == 0) {  
        strcpy(replybuffer,"Restarting Fona");
        sendSMS (replybuffer,sendto); 
        deleteSMS(smsMessage); //remove the SMS message from queue
        delay(50);
        softReset(); 
    }
  else {match=false;}
 return match;
} 

void softReset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");  
}

void setNumResults (char controlV) {
  EEPROM.write(0, controlV);
}

void setWriteDelay (char controlV) {
  int cV = (int)controlV;
  cV = (controlV/33*50) + (controlV-33)*4;
  writeDelay=cV; 
}


int8_t getNumResults () {
    int8_t numResultInt8_t = EEPROM.read(0); // read a byte from the current address
    return numResultInt8_t;
}

void enableSMSSend () {
  EEPROM.write(1, 1);
}

void disableSMSSend () {
  EEPROM.write(1, 2);
}

void getError () {
    int8_t numResultInt8_t = EEPROM.read(3); // read a byte from the current address
    setCharMessage(numResultInt8_t);
    Serial.println("Get Error");    
    Serial.println(numResultInt8_t);    
    
}

void addError () {
    int8_t numResultInt8_t = EEPROM.read(3); // read a byte from the current address
    ++numResultInt8_t;
    EEPROM.write(3,numResultInt8_t); // write new value
    Serial.println("Adding Error");
    Serial.println(numResultInt8_t);
    
}

void resetError () {
    EEPROM.write(3,0); // write new value
}


int8_t getSMSSend () {
    int8_t getSMSSendInt8_t = EEPROM.read(1); // read a byte from the current address
    return getSMSSendInt8_t;
}

void deleteSMS() {

        int8_t smsnum = fona.getNumSMS();

        if (smsnum < 0) {
          Serial.println(F("No SMS's"));
        } else {
          Serial.print(smsnum);
          Serial.println(F("SMS's on SIM card!"));
        }

  for (int smsMessage = 1; smsMessage <=smsnum; smsMessage++) {
          deleteSMS(smsMessage);
  }
}

    
void deleteSMS(int smsn) {
        if (fona.deleteSMS(smsn)) {
          Serial.println(F("Delete SMS"));
        } else {
          Serial.println(F("Couldn't delete"));
          addError ();
        }

}

void initGPRS () {
        // turn GPRS on
        if (!fona.enableGPRS(true)) {Serial.println(F("Failed to turn on GPRS"));
                                      addError ();
                                    }
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org"))){Serial.println(F("Failed to enable NTP time sync"));
                                                              addError ();
                                                              }
          
}

void getRSSI () {
        // read the RSSI
        uint8_t n = fona.getRSSI();
        int8_t r;
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        
        setCharMessage(r);

}

void setCharMessage (int8_t r) {
  
          int messageCounter=0; //counter used to break down messages into 140 byte chunks
          String value = String(r,DEC);
          char cValue[4];
          value.toCharArray(cValue,4); //copy value to CValue
          memset(replybuffer, 0, 255);        

       int i=0;
          while(i<value.length())
            {
            replybuffer[messageCounter+i]    = cValue[i];
            ++i;
            }
  
}

void getTime () {
        char buffer[23];
        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        replybuffer[0]=buffer[10];
        replybuffer[1]=buffer[11];
        replybuffer[2]=buffer[12];
        replybuffer[3]=buffer[13];
        replybuffer[4]=buffer[14];
        replybuffer[5]=buffer[15];
        replybuffer[6]=buffer[16];
        replybuffer[7]=buffer[17];
        replybuffer[8]='\0';
}

void sendSMS (String thisMessage,char recipient[21]) {
      Serial.println(F("Send SMS"));

       if (!fona.sendSMS(recipient, replybuffer)) {Serial.println(F("Failed to send SMS"));
                                                   fona.sendSMS(recipient, replybuffer); //retry
                                                   addError ();
                                                   } 
       else {Serial.println(F("Sent!"));
             memset(replybuffer, 0, 255);
        }
}

int8_t getReading () {
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);
  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1]<<8)|(int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3]<<8)|(int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5]<<8)|(int)values[4];


  //Implement filter to keep ADXL values between int8_t range -128 to 127
  if (z<-128) 
        {z=-128;} 
      else if (z>127) 
          {z=127;}
  return z;  
}

int calcAvgAccel (int z, float index, int currAvg) {
  float weightNew = 1/index;
  float weightOld = (1-weightNew);
  int avg = z*weightNew+currAvg*weightOld;
  return avg;  
}


boolean quakeEvent(int z, int zAverage) {
  boolean check = false;
  int8_t delta = abs(z - zAverage);
  if (delta >10) {check = true;} 
  return check;
}

void printResults () {
  //Print the results to the terminal.
  Serial.print(F("x Val:"));
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(eventCheck);
  Serial.print(',');
  Serial.print(F("y val:"));
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.print(F("z val:"));
  Serial.print(z, DEC);
  Serial.print(',');
  Serial.print(F("z Avg:"));
  Serial.print(zAverage,DEC);
  Serial.print(',');
  Serial.print(F("z Diff:"));
  Serial.println(z-zAverage,DEC);
}  

//This function will write a value to a register on the ADXL345.
//Parameters:
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}


//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}
