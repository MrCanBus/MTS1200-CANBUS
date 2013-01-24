#include <SD.h>       /* Library from Adafruit.com */
#include <SoftwareSerial.h>
#include <SPI.h>
#include "mcp2515.h"

/*
V003 Corrected Joystick input and statments (set to high).  Added event marker J/S up
v002 Cleaned up and simplified programming.  
 Added checks for can bus available.  
 No lights setting up.
 D8 Ready but no can bus messages available
 D8, D7 Capturing (press J/S to stop)
 D7 waiting for J/S down to restart */
//v001 added time field
//based on Bus_monitor_FAZd_SD2
//Added SD Card writing
//Compiles in 1.0.1 arduino
// Added in clear the filters and masks from Listener_TWO.
// works on Laguna.  Reports that Laguna is using almost no extended
// #include <CAN.h> can placed below 
//  Works on Laguna but only after filters are changed by running other program -Thaniel

/* This program is a CAN-bus monitor.  It listens on a
 * CAN bus and prints out the IDs and data fields of
 * any CAN messages it hears.  */

#define DEFAULT_CAN_ID  0x0555
// added for filter and can changes
#define MASK_0 0x20
#define MASK_1 0x24
#define FILTER_0 0x00
#define FILTER_1 0x04
#define FILTER_2 0x08
#define FILTER_3 0x10
#define FILTER_4 0x14
#define FILTER_5 0x18

//Pin definitions
#define SCK_PIN   13  //Clock pin
#define MISO_PIN  12  //Mater in Slave output
#define MOSI_PIN  11  //Master out Slave input
#define SS_PIN    10  //SS pin
#define SD_PIN    9  //pin for SD card control
#define RESET_PIN 2
#define INT_PIN 3

//Joystick pin definitions
#define UP     A1
#define RIGHT  A2
#define DOWN   A3
#define CLICK  A4
#define LEFT   A5

#define WRITE 0x02 //read and write comands for SPI


/* Operation Modes */
enum CAN_MODE {
  CAN_MODE_NORMAL, 		/* Transmit and receive as normal */
  CAN_MODE_SLEEP,	        /* Low power mode */
  CAN_MODE_LOOPBACK,		/* Test mode; anything "sent" appears in the receive buffer without external signaling */  							 
  CAN_MODE_LISTEN_ONLY,         /* Receive only; do not transmit */
  CAN_MODE_CONFIG,		/* Default; Allows writing to config registers */
  CAN_MODE_COUNT
};

/* Supported can bus speeds  
 500 Kbps       = MCP2515_SPEED_500000,
 250 Kbps       = MCP2515_SPEED_250000,
 125 Kbps       = MCP2515_SPEED_125000,
 100 Kbps       = MCP2515_SPEED_100000,
 62.5 Kbps      = MCP2515_SPEED_62500,
 50 Kbps        = MCP2515_SPEED_50000,
 31.25 Kbps     = MCP2515_SPEED_31250,
 25 Kbps        = MCP2515_SPEED_25000,
 20 Kbps        = MCP2515_SPEED_20000,
 15.625 Kbps    = MCP2515_SPEED_15625, */

unsigned long time;  //used for time stamp

//CanMessage message;
int led = 8;
int led2 = 7;
byte i;
uint8_t  message;

/** A flag indicating whether this is an extended CAN message */
uint8_t extended;

/** The ID of the CAN message.  The ID is 29 bytes long if the
 * extended flag is set, or 11 bytes long if not set. */
uint32_t id;

/** The number of bytes in the data field (0-8) */
uint8_t len;

/** Array containing the bytes of the CAN message.  This array
 * may be accessed directly to set or read the CAN message.
 * This field can also be set by the setTypeData functions and
 * read by the getTypeData functions. */
uint8_t data[8];
/* Define Joystick connection */


Sd2Card card;
SdVolume volume;
SdFile root;
SdFile file;

//************************ Setup ****************************
void setup()
{
  //Definitions for SD card

  // store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

  // set the slaveSelectPin as an output 
  pinMode (SCK_PIN,OUTPUT);
  pinMode (MISO_PIN,INPUT);
  pinMode (MOSI_PIN, OUTPUT);
  pinMode (SS_PIN, OUTPUT);
  pinMode(RESET_PIN,OUTPUT);
  pinMode(INT_PIN,INPUT);
  pinMode(led,OUTPUT); //setup LED
  pinMode(led2,OUTPUT); //setup LED

  digitalWrite(INT_PIN,HIGH);

  //Set up Joystick Pins
  pinMode(UP,INPUT);
  pinMode(DOWN,INPUT);
  pinMode(LEFT,INPUT);
  pinMode(RIGHT,INPUT);
  pinMode(CLICK,INPUT);

  /* Enable internal pull-ups on JS pins */
  digitalWrite(UP, HIGH);       
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);
  digitalWrite(CLICK, HIGH);

  // initialize CAN bus
  CAN_begin(MCP2515_SPEED_500000);  // set can baud rate
  CAN_setMode (CAN_MODE_LISTEN_ONLY); // set can mode

  Serial.begin(115200);  //set up serial port

  // Print headings on Screen *********************** 
  Serial.println("microSec, len, ARBID, Ext, B0, B1, B2, B3, B4, B5, B6, B7");
  delay(1000);
  Serial.println("Init SD card");  


  //************************* SD CARD SETUP *******************
  // initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
  // breadboards.  use SPI_FULL_SPEED for better performance.
  if (!card.init(SPI_FULL_SPEED,SD_PIN)) error("card.init failed");

  // initialize a FAT volume
  if (!volume.init(&card)) error("volume.init failed");

  // open the root directory
  if (!root.openRoot(&volume)) error("openRoot failed");

}

void loop()
{
  if (!CAN_available()) {
    Serial.println("Can bus not available.  Check connections");
    digitalWrite(led,HIGH);  //D8 light means waiting for Can to be connected
  }
  while(!CAN_available()) {
  } //waiting for Canbus to be connected
  Serial.println("Can Bus Connected and available");
  // create a new file
  char name[] = "WRITE00.TXT";
  for (uint8_t i = 0; i < 100; i++) {
    name[5] = i/10 + '0';
    name[6] = i%10 + '0';
    if (file.open(&root, name, O_CREAT | O_EXCL | O_WRITE)) break;
  }
  if (!file.isOpen()) error ("file.create");
  Serial.print("Writing to: ");
  Serial.println(name);
  // write header
  // file.writeError = 0;
  file.print("microSec, len, ARBID, Ext, B0, B1, B2, B3, B4, B5, B6, B7");
  file.println();  

  //**** instructions on how to stop print to screen
  Serial.println("Press J/S click to Stop");  
  //  while (digitalRead(DOWN) == 0);
  //  { Serial.println("waiting");}
  digitalWrite(led,HIGH);  //Turn on 2 lights when capturing
  digitalWrite(led2,HIGH); //Turn on 2 lights when capturing

  CAN_Capture();

}//end loop

void CAN_Capture (){

  while(1){
    if (CAN_available()) {  //is a message recieved
      time = micros();  //capture time when message was recieved
      CAN_getMessage (); //subroutine for extracting message from buffer

      //send messages to the SD card
      file.print (time);
      file.print (",");

      file.print (len, HEX);
      file.print (",");

      file.print (id, HEX);
      file.print (",");

      file.print (extended, HEX);

      for (i=0; i<len; i++) {
        file.print (';');
        file.print (data[i], HEX);
      }

      //check if it should stop
      if(digitalRead(CLICK) == 0){
        file.close();
        Serial.println("File closed, Data saved");
        digitalWrite(led,LOW);
        digitalWrite(led2,LOW);
        while(1){ //loop waiting for down
          digitalWrite(led2,HIGH); //D7 when ready to restart
          if(digitalRead(DOWN) == 0) {//wait for down to be pressed
            return;
          } //END IF down pressed
        }//end while after capture finishes
      }// end if click
      if(digitalRead(UP) == 0){ // mark event
        file.print(";Event");
      }//end if up 
    }//end if can avail
    file.println ("");  //end line for next reading
  }//end main while loop
}//end void

//*********************** Can cpp  ***************************************
/*
 * Copyright (c) 2010-2011 by Kevin Smith <faz@fazjaxton.net>
 * MCP2515 CAN library for arduino. */

void CAN_CanMessage ()
{
  extended = 0;
  id = DEFAULT_CAN_ID;
  len = 0;
}
// **************************** setByteData *******************
void CAN_setByteData (byte b)
{
  len = 1;
  data[0] = b;
}

//*******************  setIntData  ***************************
void setIntData (int i)
{
  len = 2;

  /* Big-endian network byte ordering */
  data[0] = i >> 8;
  data[1] = i & 0xff;
}

//******************* setLongData  ***************************
void CAN_setLongData (long l)
{
  len = 4;

  /* Big-endian network byte ordering */
  data[0] = (l >> 24) & 0xff;
  data[1] = (l >> 16) & 0xff;
  data[2] = (l >>  8) & 0xff;
  data[3] = (l >>  0) & 0xff;
}

//*********************** setData  *****************************
void CAN_setData (const uint8_t *data, uint8_t len)
{
  byte i;

  //  this->len = len;
  len = len;
  for (i=0; i<len; i++) {
    //    this->data[i] = data[i];

  }
}

//*********************** setData  **************************
void CAN_setData (const char *data, uint8_t len)
{
  CAN_setData ((const uint8_t *)data, len);
}

//********************* send  ******************************
void CAN_send ()
{
  mcp2515_set_msg (0, id, data, len, extended);
  mcp2515_request_tx (0);
}

//****************** getByteFromData **********************

byte CAN_getByteFromData()
{
  return data[0];
}

//**************** getIntFromData ************************
int CAN_getIntFromData ()
{
  int val;

  val |= (uint16_t)data[0] << 8;
  val |= (uint16_t)data[1] << 0;

  return val;
}

//**************** getLongFromData
long CAN_getLongFromData ()
{
  long val;

  val |= (uint32_t)data[0] << 24;
  val |= (uint32_t)data[1] << 16;
  val |= (uint32_t)data[2] <<  8;
  val |= (uint32_t)data[3] <<  0;

  return val;
}

//********************** getData *******************
void CAN_getData (uint8_t *data)
{
  byte i;

  for (i=0; i<len; i++) {
    //    data[i] = this->data[i];
  }
}

//******************** getData ***********************
void CAN_getData (char *data)
{
  CAN_getData ((uint8_t *)data);
}

/*
 * CANClass
 */

//********************** Begin ***********************
void CAN_begin(uint32_t bit_time) {
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  mcp2515_init (bit_time);

  CAN_resetFiltersAndMasks();  
}


//******************************** reset filters and Masks  ****************
// added to set filters -- from another program
void CAN_resetFiltersAndMasks() {
  //disable first buffer
  CAN_setMaskOrFilter(MASK_0,   0b00000000, 0b00000000, 0b00000000, 0b00000000);
  CAN_setMaskOrFilter(FILTER_0, 0b00000000, 0b00000000, 0b00000000, 0b00000000);
  CAN_setMaskOrFilter(FILTER_1, 0b00000000, 0b00000000, 0b00000000, 0b00000000);

  //disable the second buffer
  CAN_setMaskOrFilter(MASK_1,   0b00000000, 0b00000000, 0b00000000, 0b00000000); 
  CAN_setMaskOrFilter(FILTER_2, 0b00000000, 0b00000000, 0b00000000, 0b00000000);
  CAN_setMaskOrFilter(FILTER_3, 0b00000000, 0b00000000, 0b00000000, 0b00000000); 
  CAN_setMaskOrFilter(FILTER_4, 0b00000000, 0b00000000, 0b00000000, 0b00000000);
  CAN_setMaskOrFilter(FILTER_5, 0b00000000, 0b00000000, 0b00000000, 0b00000000); 
}

//***************************** set mask or FIlter **********************
void CAN_setMaskOrFilter(byte mask, byte b0, byte b1, byte b2, byte b3) {
  CAN_setMode(CAN_MODE_CONFIG); 
  CAN_setRegister(mask, b0);
  CAN_setRegister(mask+1, b1);
  CAN_setRegister(mask+2, b2);
  CAN_setRegister(mask+3, b3);
  CAN_setMode(CAN_MODE_LISTEN_ONLY); 
}

//************************** Set register ******************************
void CAN_setRegister(byte reg, byte value) {
  //  mcp2515_write_reg (reg, value); this should be instead to write the registers
  //static void mcp2515_write_reg (uint8_t addr, uint8_t buf)
  // used from other program.  Replaced by above 
  digitalWrite(SS_PIN, LOW);
  delay(10);
  SPI.transfer(WRITE);
  SPI.transfer(reg);
  SPI.transfer(value);
  delay(10);
  digitalWrite(SS_PIN, HIGH);
  delay(10);

}
// end from another program



///*********************** end ***********************
void CAN_end() {
  SPI.end ();
}

//********************* setMode *********************
void CAN_setMode (uint8_t mode)
{
  mcp2515_set_mode (mode);
}

//******************** ready **********************
uint8_t CAN_ready ()
{
  return mcp2515_msg_sent ();
}

//****************** Available *********************
boolean CAN_available ()
{
  return (boolean)mcp2515_msg_received();
}

//***************** getMessage ***********************
void CAN_getMessage ()
{
  uint8_t extended;
  extended = mcp2515_get_msg (0, &id, data, &len);
}
// ************************** SD card Error ************************
void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);

  Serial.print("SD error");

  if (card.errorCode()) {
    PgmPrint("SD error: ");
    Serial.print(card.errorCode(), HEX);

    Serial.print(',');
    Serial.println(card.errorData(), HEX);

  }
  while(1);
}

//CANClass CAN;

/*
/**
 * A class representing a single CAN message.  The message can be built
 * using the send<Type>Data functions, or the bytes of the message can
 * be set directly by accessing the public data[] array.  This class is
 * also used to retrieve a message that has been received.  The data
 * can be read using the get<type>Data functions, or can be read directly
 * by accessing the public data[] array.
 */
//class CanMessage {
//    public:
/** A flag indicating whether this is an extended CAN message */
//        uint8_t extended;
/** The ID of the CAN message.  The ID is 29 bytes long if the
 * extended flag is set, or 11 bytes long if not set. */
//        uint32_t id;
/** The number of bytes in the data field (0-8) */
//        uint8_t len;
/** Array containing the bytes of the CAN message.  This array
 * may be accessed directly to set or read the CAN message.
 * This field can also be set by the setTypeData functions and
 * read by the getTypeData functions. */
//        uint8_t data[8];

//        CanMessage();

/**
 * Simple interface to set up a CAN message for sending a byte data
 * type.  When received, this message should be unpacked with the
 * getByteData function.  This interface only allows one byte to be
 * packed into a message.  To pack more data, access the data array
 * directly.
 * @param b - The byte to pack into the message.
 */
//        void setByteData (byte b);

/**
 * Simple interface to set up a CAN message for sending an int data
 * type.  When received, this message should be unpacked with the
 * getIntData function.  This interface only allows one int to be
 * packed into a message.  To pack more data, access the data array
 * directly.
 * @param i - The int to pack into the message.
 */
//       void setIntData (int i);

/**
 * Simple interface to set up a CAN message for sending a long data
 * type.  When received, this message should be unpacked with the
 * getLongData function.  This interface only allows one long to be
 * packed into a message.  To pack more data, access the data array
 * directly.
 * @param l - The long to pack into the message.
 */
//        void setLongData (long l);

/**
 * A convenience function for copying multiple bytes of data into
 * the message.
 * @param data - The data to be copied into the message
 * @param len  - The size of the data
 */
//        void setData (const uint8_t *data, uint8_t len);
//        void setData (const char *data, uint8_t len);

/**
 * Send the CAN message.  Once a message has been created, this
 * function sends it.
 */
//        void send();

/**
 * Simple interface to retrieve a byte from a CAN message.  This
 * should only be used on messages that were created using the
 * setByteData function on another node.
 * @return The byte contained in the message.
 */
//        byte getByteFromData ();

/**
 * Simple interface to retrieve an int from a CAN message.  This
 * should only be used on messages that were created using the
 * setIntData function on another node.
 * @return The int contained in the message.
 */
//        int getIntFromData ();

/**
 * Simple interface to retrieve a long from a CAN message.  This
 * should only be used on messages that were created using the
 * setLongData function on another node.
 * @return The long contained in the message.
 */
//        long getLongFromData ();

/**
 * A convenience function for copying multiple bytes out of a
 * CAN message.
 * @param data - The location to copy the data to.
 */
//        void getData (uint8_t *data);
//        void getData (char *data);
//};

//class CANClass {
//	public:
/**
 * Call before using any other CAN functions.
 * @param bit_time - Desired width of a single bit in nanoseconds.
 *                   The CAN_SPEED enumerated values are set to
 *                   the bit widths of some common frequencies.
 */
//		static void begin(uint32_t bit_time);

/** Call when all CAN functions are complete */
//		static void end();

/**
 * Set operational mode.
 * @param mode - One of the CAN_MODE enumerated values */
//		static void setMode(uint8_t mode);

/** Check whether a message may be sent */
//       static uint8_t ready ();

/**
 * Check whether received CAN data is available.
 * @return True if a message is available to be retrieved.
 */
//        static boolean available ();

/**
 * Retrieve a CAN message.
 * @return A CanMessage containing the retrieved message
 */
//        static CanMessage getMessage (); */




