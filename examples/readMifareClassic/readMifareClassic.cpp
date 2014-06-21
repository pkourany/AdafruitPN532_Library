/**************************************************************************/
/*! 
    @file     readMifareClassic.cpp
    @author   Adafruit Industries, Paul Kourany, Technobly
    @license  BSD (see license.txt)

    This example will wait for any ISO14443A card or tag, and
    depending on the size of the UID will attempt to read from it.
   
    If the card has a 4-byte UID it is probably a Mifare
    Classic card, and the following steps are taken:
   
    Reads the 4 byte (32 bit) ID of a MiFare Classic card.
    Since the classic cards have only 32 bit identifiers you can stick
    them in a single variable and use that to compare card ID's as a
    number. This doesn't work for ultralight cards that have longer 7
    byte IDs!
   
    Note that you need the baud rate to be 115200 because we need to
    print out the data and read from the card at the same time!

This is an example sketch for the Adafruit PN532 NFC/RFID breakout boards
This library works with the Adafruit NFC breakout 
  ----> https://www.adafruit.com/products/364
 
Check out the links above for our tutorials and wiring diagrams 
These chips use SPI to communicate, 4 required to interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!
*/
/**************************************************************************/
#include "Adafruit_PN532.h"

// Uncomment for faster debugging!
#include "spark_disable_wlan.h"

// SPI Mode defines
#define SCK_PIN  (A3)
#define MOSI_PIN (A5)
#define SS_PIN   (A2)
#define MISO_PIN (A4)

// I2C Mode defines
#define IRQ_PIN  (D2) // This is how the PN532 Shield gets ready status in I2C mode
#define RST_PIN  (D3) // Necessary for I2C mode

// IMPORTANT! CONFIGURE to use SPI or I2C mode:
// Set PN532_MODE to PN532_SPI_MODE or PN532_I2C_MODE on line 32 of Adafruit_PN532.h
// If using SPI mode, an additional option is to set PN532_HW_SPI to 1 for hardware
// SPI, or 0 for software SPI on line 35 of Adafruit_PN532.h
#if PN532_MODE == PN532_SPI_MODE
  Adafruit_PN532 nfc(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
#elif PN532_MODE == PN532_I2C_MODE
  Adafruit_PN532 nfc(IRQ_PIN, RST_PIN);
#endif

void setup(void) {
  Serial.begin(115200); // Make sure your serial terminal is closed before power the Core.
  while(!Serial.available()) {
    #ifdef SPARK_WLAN_SETUP
      SPARK_WLAN_Loop(); // Open serial terminal and Press ENTER.
    #endif
  }
  Serial.println("Hello!");

  nfc.begin();

  uint32_t versiondata;
  do {
    versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
      Serial.print("Didn't find PN53x board");
      delay(1000);
      #ifdef SPARK_WLAN_SETUP
        SPARK_WLAN_Loop(); // Keep connected to the Cloud while re-trying
      #endif
    }
  }
  while(!versiondata);

  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  
  Serial.println("Waiting for an ISO14443A Card ...");
}


void loop(void) {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  
  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    
    if (uidLength == 4)
    {
      // We probably have a Mifare Classic card ... 
      uint32_t cardid = uid[0];
      cardid <<= 8;
      cardid |= uid[1];
      cardid <<= 8;
      cardid |= uid[2];  
      cardid <<= 8;
      cardid |= uid[3]; 
      Serial.print("Seems to be a Mifare Classic card #");
      Serial.println(cardid);
    }
    Serial.println("");
  }
}
