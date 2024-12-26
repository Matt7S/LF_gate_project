# Freenove RFID Lib for Pico

## Description
This is an Arduino library used to drive Pico to control RFID modules.
Please note that our library supports Raspberry Pi Pico/RP2040 package versions up to 4.0.2.
Based on the example program "SerialNumberRead" in Arduino IDE. The source code repository is here:
https://github.com/Freenove/Freenove_RFID_Lib_for_Pico/tree/main/examples/SerialNumberRead



## Examples:

Here are some simple examples.

### SerialNumberRead
This example allows your Pico driver RFID module to obtain the UID and type of the card.
```
#include <SPI.h>
#include <Freenove_RFID_Lib_for_Pico.h>

//D10 - CS Pin、D5 - RST Pin
RFID rfid(5,6);   
unsigned char status;
unsigned char str[MAX_LEN];  //MAX_LEN is 16, the maximum length of the array

void setup()
{
  Serial.begin(115200);
  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.begin();
  rfid.init(); //initialization
}

void loop()
{
  //Search card, return card types
  if (rfid.findCard(PICC_REQIDL, str) == MI_OK) {
    Serial.println("Find the card!");
    // Show card type
    ShowCardType(str);
    //Anti conflict detection, reading card serial number
    if (rfid.anticoll(str) == MI_OK) {
      Serial.print("The card's number is  : ");
      //Display card serial number
      for(int i = 0; i < 4; i++){
        Serial.print(0x0F & (str[i] >> 4),HEX);
        Serial.print(0x0F & str[i],HEX);
      }
      Serial.println("");
    }
    //Card selection (locking the card to prevent majority reading, removing this line will result in continuous card reading)
    rfid.selectTag(str);
  }
  rfid.halt();  //Command the card to enter sleep mode
}

void ShowCardType(unsigned char * type)
{
  Serial.print("Card type: ");
  if(type[0]==0x04 && type[1]==0x00) 
    Serial.println("MFOne-S50");
  else if(type[0]==0x02 && type[1]==0x00)
    Serial.println("MFOne-S70");
  else if(type[0]==0x44 && type[1]==0x00)
    Serial.println("MF-UltraLight");
  else if(type[0]==0x08 && type[1]==0x00)
    Serial.println("MF-Pro");
  else if(type[0]==0x44 && type[1]==0x03)
    Serial.println("MF Desire");
  else
    Serial.println("Unknown");
}

```

## Usage
```
RFID rfid(5, 6);
```
* Construction. Create a RFID object.

```
rfid.init();
```
* Initialization data, ready for communication.
```
rfid.findCard(PICC_REQIDL, str);
```
* Search card, return card types.

```
ShowCardType(str);
```
* Show card type.

```
rfid.anticoll(str);
```
* Anti conflict detection, reading card serial number.

```
rfid.selectTag(str);
```
* Card selection (locking the card to prevent majority reading, removing this line will result in continuous card reading).

```
rfid.halt();
```
* Command the card to enter sleep mode.

```
Find the card!
Card type: MFOne-S50
The card's number is  : 5CFB4117
```
*Placing the card near the RFID module will print the type and UID of the card,As shown above.