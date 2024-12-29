/*
  ReadAndWrite
  Read the serial number of the IC card.
 */

#include <SPI.h>
#include <Freenove_RFID_Lib_for_Pico.h>

//D10 - CS Pin、D5 - RST Pin
RFID rfid(5,6);   

//4-byte card serial number, with the 5th byte being the checksum byte
unsigned char serNum[5];
unsigned char status;
unsigned char str[MAX_LEN];
unsigned char blockAddr;        //Select block addresses 0-63 for operation

//Write card data, the data you want to write (16 bytes)
unsigned char writeDate[16] ={
  'W', 'e', 'l', 'c', 'o', 'm', 'e', ' ', 'M', 'F', 'R', 'C', '5', '2', '2', 0};

//The original password for sector A consists of 16 sectors, each with a password of 6 bytes
unsigned char sectorKeyA[16][16] = {
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,
  {  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  } ,};

void setup()
{
  Serial.begin(115200);
  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(2);
  SPI.setTX(3);
  SPI.begin();
  rfid.init();
}

void loop()
{
  //Find Card
  rfid.findCard(PICC_REQIDL, str);
    //Anti conflict detection, reading card serial number
    if (rfid.anticoll(str) == MI_OK){
      
      Serial.print("The card's number is  : ");
      //Display card serial number
      for(int i = 0; i < 4; i++){
        Serial.print(0x0F & (str[i] >> 4),HEX);
        Serial.print(0x0F & str[i],HEX);
      }
      Serial.println("");
      memcpy(rfid.serNum,str,5);
    }
  //Select card, return card capacity (lock card to prevent multiple reads and writes)
  rfid.selectTag(rfid.serNum);

  //Write data to the card data block (within 16 bytes), taking data block 4 as an example
  writeCard(4);
  //Read data from data block, taking data block 4 as an example
  readCard(4);
 
  rfid.halt();
} 

//Write data card
void writeCard(int blockAddr){
  
  if (rfid.auth(PICC_AUTHENT1A, blockAddr, sectorKeyA[blockAddr/4], rfid.serNum) == MI_OK)  //authentication
  {
    //Write Data
    //status = rfid.write((blockAddr/4 + 3*(blockAddr/4+1)), sectorKeyA[0]);
    Serial.print("set the new card password, and can modify the data of the Sector: ");
    Serial.println(blockAddr/4,DEC);
    //Select blocks in the sector to write data
    if(rfid.write(blockAddr, writeDate) == MI_OK){
      Serial.println("Write card OK!");
    }
  }
}

//Read Card
void readCard(int blockAddr){
 
  if ( rfid.auth(PICC_AUTHENT1A, blockAddr, sectorKeyA[blockAddr/4], rfid.serNum) == MI_OK)  //authentication
  {
    //Select blocks in the sector to read data
    Serial.print("Read from the blockAddr of the card : ");
    Serial.println(blockAddr,DEC);
    if( rfid.read(blockAddr, str) == MI_OK){
      Serial.print("The data is : ");
      Serial.println((char *)str);
    }
  }
}
