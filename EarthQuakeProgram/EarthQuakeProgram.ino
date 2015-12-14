//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <EEPROM.h>

//Fona set-ups
//Fona libraries located here - C:\Program Files (x86)\Arduino\libraries\Adafruit_FONA
#include <SoftwareSerial.h>
#include <Adafruit_FONA.h>
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#ifdef __AVR__
#include <SoftwareSerial.h>
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
int zPrior; //prior value of x used for callibration
const int numReadings = 20; //Number of readings for average
const int numResults = 50; //Number of readings to write out
int8_t mode = 0; //Operating Mode; 0=calibrate, 1=monitor, 2= collect, 3=flush and send buffer

//# times each mode should execute
int8_t calibCount=1; // # times called calibrate fxn
int8_t writeCount=0; // # times writing to EEPROM

//average z used to determine if event occurred
int zAverage = 0;

//event check
boolean eventCheck = false;

//EEPROM Config
int8_t address = 0; // the current address
int8_t valueInt; //value read

//Fona variables
uint8_t type; //Fona Type
char sendto[21]="4158069938"; //phone number to send SMS
char message[141]; //SMS message sent to Fona
String dataMessage; //data info
byte messageCounter=0;
char replybuffer[255]; //character buffer used for SMS communications


void setup(){ 
 //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

//Initialize FONA
  Serial.println("Fona start-up...");
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print(F("SIM card IMEI: ")); Serial.println(imei);
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
}

void loop(){
  
  if (mode==0) {
        if (calibCount == 1) {Serial.println(F("Calibrate ADXL..."));
                              delay(10);  //add small delay for ADXL 
                              zPrior = getReading();
                              processSMS();
                              }

        if (quakeEvent(getReading(), zPrior)) {Serial.println(F("Not Stable..."));
                                               calibCount=1; //reset calibration
                                               Serial.print(F("z:"));
                                               Serial.print(z, DEC);
                                               Serial.print(',');
                                               Serial.print(F("z Diff:"));
                                               Serial.println(z-zPrior,DEC);
                                               delay(10000);}
          else {      
                  zAverage=calcAvgAccel (getReading(),calibCount,zAverage);
                  calibCount = calibCount+1;        
                  Serial.print(calibCount, DEC); //
                  Serial.print(":Z Value:");
                  Serial.println(z, DEC);}

        if (calibCount == numReadings) {mode=1;
                                       calibCount=1; //reset calibration count
                                       Serial.println(F("Monitoring..."));}      

      delay(100); //short delay for callibration
  }
  else if (mode ==1) {
      eventCheck = quakeEvent(getReading(), zAverage);   
      printResults();
      if (eventCheck == true) {mode= 2;
                              eventCheck=false;
                              String SMSMessage="Event detected @ "; //SMS message constructed by program
                              SMSMessage= SMSMessage + getTime();
                              Serial.println(SMSMessage);                             
                              //sendSMS(SMSMessage);
                                  }
      delay(500);
      }
  else if (mode ==2) {
       if (writeCount == 0) {address=0;
                             Serial.println(F("Write to eeprom"));}
       EEPROM.write(address, getReading()); //write to current address
       address = address + 1;  // advance to the next address
       if (address == 512) {address = 0;} // Go back to 0 when we hit 512.
       writeCount = writeCount +1;
       Serial.print(address, DEC);
       Serial.print(":z=");
       Serial.println(z, DEC);
       if (writeCount == numResults) {mode=3;
                                   writeCount=0;
                                   address =0;
                                   Serial.println(F("Finished"));}
      delay(250);   
  }
  else if (mode ==3) {
       if (address == 0) {Serial.println(F("Transmitting"));
                         }
        valueInt = EEPROM.read(address); // read a byte from the current address
        Serial.print(address, DEC);
        Serial.print(":z=");
        Serial.println(valueInt, DEC);

        if (messageCounter <28) {dataMessage = dataMessage + valueInt + ",";
                                 messageCounter = messageCounter+1;
                                 } 
                               else { 
                                 //sendSMS(dataMessage); //send data in 140 character chunks           
                                      messageCounter = 0;
                                      dataMessage = "";
                               } 
        address = address + 1;    // advance to the next address of the EEPROM
        if (address == numResults) {mode=0;
                                    address=0;
                                    //sendSMS(dataMessage); //send remainder of data 
                                    Serial.println(F("Mem flushed"));}
      delay(50);   
  }

}

void processSMS () {

  // read the number of SMS's!
        int8_t smsnum = fona.getNumSMS(); //smsnum is the # of sms on sim card
        Serial.print(F("SMS's in queue: "));
        Serial.print(smsnum);

  for (int smsMessage = 1; smsMessage <=smsnum; smsMessage++) {

        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsMessage, replybuffer, 250)) {
          Serial.println("Failed!");
        }

        Serial.print(F("FROM: ")); 
        Serial.println(replybuffer);

        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsMessage, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println("Failed!");
        }

        Serial.print(F("***** SMS #")); Serial.print(smsMessage);
        Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
        Serial.println(replybuffer);
        Serial.println(F("*****"));

  }
}

void deleteSMS() {
  // delete an SMS
        uint8_t smsn; //smsn to be deleted

        if (fona.deleteSMS(smsn)) {
          Serial.println(F("OK!"));
        } else {
          Serial.println(F("Couldn't delete"));
        }

}

void initGPRS () {
        // turn GPRS on
        if (!fona.enableGPRS(true))
          Serial.println(F("Failed to turn on"));
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          Serial.println(F("Failed to enable"));
}

String getTime () {
        // read the time
        char buffer[23];
        fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
        String time = String(buffer);
        time.remove(0, 10); //remove date
        time.remove(8);    // remove trailing #'s
        Serial.println(time);
        return time;
}

void sendSMS (String thisMessage) {
      Serial.println(F("Send SMS"));
      thisMessage.toCharArray(message, 141);   
       if (!fona.sendSMS(sendto, message)) {Serial.println(F("Failed"));} 
       else {Serial.println(F("Sent!"));
        }
}

int getReading () {
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
