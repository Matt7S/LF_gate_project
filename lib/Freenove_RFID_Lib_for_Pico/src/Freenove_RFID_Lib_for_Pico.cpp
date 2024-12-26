#include <Arduino.h>
#include <Freenove_RFID_Lib_for_Pico.h>

RFID::RFID(int chipSelectPin, int NRSTPD, int irqPin)
{
  _chipSelectPin = chipSelectPin;
  _NRSTPD = NRSTPD;

  pinMode(_chipSelectPin,OUTPUT);     // Set pin _chipSelectPin as output and connect it to the module enable port
  digitalWrite(_chipSelectPin, LOW);


  pinMode(_NRSTPD,OUTPUT);            // Set pin NRSTPD as output, non reset or power down
  digitalWrite(_NRSTPD, HIGH);

  if (irqPin != -1) {
    _irqPin = irqPin;
    pinMode(_irqPin, INPUT_PULLUP);
  }
}

/******************************************************************************
 * User API
 ******************************************************************************/

/******************************************************************************
 * Function name��		 init
 * Function Description��Initialize RC522
 * Input Parameter��	 NULL
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::init()
{
  digitalWrite(_NRSTPD,HIGH);

  reset();

  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  writeMFRC522(TModeReg, 0x8D);   //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  writeMFRC522(TPrescalerReg, 0x3E);  //TModeReg[3..0] + TPrescalerReg
  writeMFRC522(TReloadRegL, 30);
  writeMFRC522(TReloadRegH, 0);
  writeMFRC522(TxAutoReg, 0x40);    //100%ASK
  writeMFRC522(ModeReg, 0x3D);    // CRC valor inicial de 0x6363

  //ClearBitMask(Status2Reg, 0x08); //MFCrypto1On=0
  //writeMFRC522(RxSelReg, 0x86);   //RxWait = RxSelReg[5..0]
  //writeMFRC522(RFCfgReg, 0x7F);     //RxGain = 48dB

  antennaOn();    //Open the antenna
}

/******************************************************************************
 * Function name��		 reset
 * Function Description��Reset RC522
 * Input Parameter��	 NULL
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::reset()
{
  writeMFRC522(CommandReg, PCD_RESETPHASE);
}

/******************************************************************************
 * Function name��		 writeMFRC522
 * Function Description��Write a byte of data to a register of MFRC522
 * Input Parameter��	 addr--Register address��val--The value to be written
 * Return Value��		 NULL
 ******************************************************************************/
SPISettings spisettings(1000000, MSBFIRST, SPI_MODE0);
void RFID::writeMFRC522(unsigned char addr, unsigned char val)
{
  digitalWrite(_chipSelectPin, LOW);
  SPI.beginTransaction(spisettings);

  //address format��0XXXXXX0
  SPI.transfer((addr<<1)&0x7E);
  SPI.transfer(val);

  SPI.endTransaction();
  digitalWrite(_chipSelectPin, HIGH);
}

/******************************************************************************
 * Function name��		 readMFRC522
 * Function Description��Read a byte of data from a register of MFRC522
 * Input Parameter��	 addr--Register address
 * Return Value��		 Return a byte of data read
 ******************************************************************************/
unsigned char RFID::readMFRC522(unsigned char addr)
{
  unsigned char val;
  digitalWrite(_chipSelectPin, LOW);
  SPI.transfer(((addr<<1)&0x7E) | 0x80);
  val =SPI.transfer(0x00);
  digitalWrite(_chipSelectPin, HIGH);
  return val;
}

/******************************************************************************
 * Function name��		 setBitMask
 * Function Description��Set RC522 register bit
 * Input Parameter��	 reg--Register address;mask--Set bit mask
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::setBitMask(unsigned char reg, unsigned char mask)
{
  unsigned char tmp;
  tmp = readMFRC522(reg);
  writeMFRC522(reg, tmp | mask);  // set bit mask
}

/******************************************************************************
 * Function name��		 clearBitMask
 * Function Description��Clear RC522 register bit
 * Input Parameter��	 reg--Register address;mask--Clear bit mask
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::clearBitMask(unsigned char reg, unsigned char mask)
{
  unsigned char tmp;
  tmp = readMFRC522(reg);
  writeMFRC522(reg, tmp & (~mask));  // clear bit mask
}

/******************************************************************************
 * Function name��		 antennaOn
 * Function Description��Turn on the antenna, and there should be at least
 *						 1ms interval between each activation or deactivation
 *						 of the sky hazard transmission
 * Input Parameter��	 NULL
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::antennaOn(void)
{
  unsigned char temp;

  temp = readMFRC522(TxControlReg);
  if (!(temp & 0x03))
  {
    setBitMask(TxControlReg, 0x03);
  }
}

/******************************************************************************
 * Function name��		 antennaOff
 * Function Description��Turn off the antenna, and there should be at least
 *						 1ms interval between each activation or deactivation
 *						 of the natural hazard transmission
 * Input Parameter��	 NULL
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::antennaOff(void)
{
  unsigned char temp;

  temp = readMFRC522(TxControlReg);
  if (!(temp & 0x03))
  {
    clearBitMask(TxControlReg, 0x03);
  }
}

/******************************************************************************
 * Function name��		 calculateCRC
 * Function Description��Calculate the CRC of MF522
 * Input Parameter��	 pIndata--To read CRC data;
 *						 len--Data Length;
 *						 pOutData--Calculated CRC result
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::calculateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
  unsigned char i, n;

  clearBitMask(DivIrqReg, 0x04);      //CRCIrq = 0
  setBitMask(FIFOLevelReg, 0x80);     //Clear FIFO pointer
  //Write_MFRC522(CommandReg, PCD_IDLE);

  //Write data to FIFO
  for (i=0; i<len; i++)
    writeMFRC522(FIFODataReg, *(pIndata+i));
  writeMFRC522(CommandReg, PCD_CALCCRC);

  //Waiting for CRC calculation to complete
  i = 0xFF;
  do
  {
    n = readMFRC522(DivIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x04));      //CRCIrq = 1

  //Read CRC calculation results
  pOutData[0] = readMFRC522(CRCResultRegL);
  pOutData[1] = readMFRC522(CRCResultRegM);
}

/******************************************************************************
 * Function name��		 MFRC522ToCard
 * Function Description��RC522 and ISO14443 card communication
 * Input Parameter��	 command--MF522 Command Word;
 *						 sendData--Data sent to the card through RC522;
 *						 sendLen--The length of the data sent
 *						 backData--Received card returns data
 *						 backLen--Return the bit length of the data
 * Return Value��		 returned MI_SOK
 ******************************************************************************/
unsigned char RFID::MFRC522ToCard(unsigned char command, unsigned char *sendData, unsigned char sendLen, unsigned char *backData, unsigned int *backLen)
{
  unsigned char status = MI_ERR;
  unsigned char irqEn = 0x00;
  unsigned char waitIRq = 0x00;
  unsigned char lastBits;
  unsigned char n;
  unsigned int i;

  switch (command)
  {
    case PCD_AUTHENT:   //Authentication Card Password
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:  //Sending data from FIFO
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  writeMFRC522(CommIEnReg, irqEn|0x80); //Allow interrupt requests
  clearBitMask(CommIrqReg, 0x80);       //Clear all interrupt request bits
  setBitMask(FIFOLevelReg, 0x80);       //FlushBuffer=1, FIFO initialization

  writeMFRC522(CommandReg, PCD_IDLE);   //No action, cancel the current command

  //Write data to FIFO
  for (i=0; i<sendLen; i++)
    writeMFRC522(FIFODataReg, sendData[i]);

  //Execute Command
  writeMFRC522(CommandReg, command);
  if (command == PCD_TRANSCEIVE)
    setBitMask(BitFramingReg, 0x80);    //StartSend=1,transmission of data starts

  //Waiting for data reception to complete
  i = 2000; //Adjust the maximum waiting time for operating M1 card according to the clock frequency, which is 25ms
  do
  {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = readMFRC522(CommIrqReg);
    i--;
  }
  while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  clearBitMask(BitFramingReg, 0x80);      //StartSend=0

  if (i != 0)
  {
    if(!(readMFRC522(ErrorReg) & 0x1B)) //BufferOvfl Collerr CRCErr ProtecolErr
    {
      status = MI_OK;
      if (n & irqEn & 0x01)
        status = MI_NOTAGERR;     //??

      if (command == PCD_TRANSCEIVE)
      {
        n = readMFRC522(FIFOLevelReg);
        lastBits = readMFRC522(ControlReg) & 0x07;
        if (lastBits)
          *backLen = (n-1)*8 + lastBits;
        else
          *backLen = n*8;

        if (n == 0)
          n = 1;
        if (n > MAX_LEN)
          n = MAX_LEN;

        //Read the data received from the FIFO
        for (i=0; i<n; i++)
          backData[i] = readMFRC522(FIFODataReg);
      }
    }
    else
      status = MI_ERR;
  }

  //SetBitMask(ControlReg,0x80);           //timer stops
  //Write_MFRC522(CommandReg, PCD_IDLE);

  return status;
}


/******************************************************************************
 * Function name��		 findCard
 * Function Description��Search for cards, read card type models
 * Input Parameter��	 reqMode--Card search method;
 *							0x4400 = Mifare_UltraLight
 *                    		0x0400 = Mifare_One(S50)
 *                    		0x0200 = Mifare_One(S70)
 *                    		0x0800 = Mifare_Pro(X)
 *                    		0x4403 = Mifare_DESFire
 * Return Value��		 returned MI_SOK
 ******************************************************************************/
unsigned char RFID::findCard(unsigned char reqMode, unsigned char *TagType)
{
  unsigned char status;
  unsigned int backBits;      //���յ�������λ��

  writeMFRC522(BitFramingReg, 0x07);    //TxLastBists = BitFramingReg[2..0] ???

  TagType[0] = reqMode;
  status = MFRC522ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10))
    status = MI_ERR;

  return status;
}

/******************************************************************************
 * Function name��		 anticoll
 * Function Description��Anti conflict detection, reading the card serial 
 *						 number of the selected card
 * Input Parameter��	 serNum--Return the 4-byte card serial number,
 *						 with the 5th byte being the checksum byte;
 * Return Value��		 returned MI_SOK
 ******************************************************************************/
unsigned char RFID::anticoll(unsigned char *serNum)
{
  unsigned char status;
  unsigned char i;
  unsigned char serNumCheck=0;
  unsigned int unLen;

  clearBitMask(Status2Reg, 0x08);   //TempSensclear
  clearBitMask(CollReg,0x80);     //ValuesAfterColl
  writeMFRC522(BitFramingReg, 0x00);    //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;

  status = MFRC522ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  if (status == MI_OK)
  {
    //Verify card serial number
	for (i=0; i<4; i++){
	  *(serNum+i)  = serNum[i];
      serNumCheck ^= serNum[i];
	}
    if (serNumCheck != serNum[i]){
      status = MI_ERR;
	}
  }

  setBitMask(CollReg, 0x80);    //ValuesAfterColl=1

  return status;
}

/******************************************************************************
 * Function name��		 auth
 * Function Description��Verify card password 
 * Input Parameter��	 authMode--Password verification mode
 *						 	0x60 = Verify key A
 *                     		0x61 = Verify key B
 *           			 BlockAddr--block address
 *           			 Sectorkey--Sector password
 *           			 serNum--Card serial number, 4 bytes
 * Return Value��		 returned MI_SOK
 ******************************************************************************/
unsigned char RFID::auth(unsigned char authMode, unsigned char BlockAddr, unsigned char *Sectorkey, unsigned char *serNum)
{
  unsigned char status;
  unsigned int recvBits;
  unsigned char i;
  unsigned char buff[12];

  //Verify Instruction + Block Address + Sector Password + Card Serial Number
  buff[0] = authMode;
  buff[1] = BlockAddr;
  for (i=0; i<6; i++)
    buff[i+2] = *(Sectorkey+i);
  for (i=0; i<4; i++)
    buff[i+8] = *(serNum+i);
    
  status = MFRC522ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);
  if ((status != MI_OK) || (!(readMFRC522(Status2Reg) & 0x08)))
    status = MI_ERR;

  return status;
}

/******************************************************************************
 * Function name��		 read
 * Function Description��Read block data
 * Input Parameter��	 blockAddr--block address;recvData--Read block data
 * Return Value��		 returned MI_SOK
 ******************************************************************************/
unsigned char RFID::read(unsigned char blockAddr, unsigned char *recvData)
{
  unsigned char status;
  unsigned int unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  calculateCRC(recvData,2, &recvData[2]);
  status = MFRC522ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

  if ((status != MI_OK) || (unLen != 0x90))
    status = MI_ERR;

  return status;
}

/******************************************************************************
 * Function name��		 write
 * Function Description��Write block data
 * Input Parameter��	 blockAddr--block address;
 *						 writeData--Write 16 bytes of data to the block
 * Return Value��		 returned MI_SOK
 ******************************************************************************/
unsigned char RFID::write(unsigned char blockAddr, unsigned char *writeData)
{
  unsigned char status;
  unsigned int recvBits;
  unsigned char i;
  unsigned char buff[18];

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  calculateCRC(buff, 2, &buff[2]);
  status = MFRC522ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

  if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    status = MI_ERR;

  if (status == MI_OK)
  {
    for (i=0; i<16; i++)    //?FIFO?16Byte?? Datos a la FIFO 16Byte escribir
      buff[i] = *(writeData+i);
      
    calculateCRC(buff, 16, &buff[16]);
    status = MFRC522ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
      status = MI_ERR;
  }

  return status;
}

/******************************************************************************
 * Function name��		 selectTag
 * Function Description��Select card, read card storage capacity
 * Input Parameter��	 serNum--Enter the card serial number
 * Return Value��		 returned card capacity
 ******************************************************************************/
unsigned char RFID::selectTag(unsigned char *serNum)
{
  unsigned char i;
  unsigned char status;
  unsigned char size;
  unsigned int recvBits;
  unsigned char buffer[9];

  //ClearBitMask(Status2Reg, 0x08);                        //MFCrypto1On=0

  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;

  for (i=0; i<5; i++)
    buffer[i+2] = *(serNum+i);

  calculateCRC(buffer, 7, &buffer[7]);
  
  status = MFRC522ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
  if ((status == MI_OK) && (recvBits == 0x18))
    size = buffer[i];
  else
    size = 0;
  return size;
}

/******************************************************************************
 * Function name��		 Halt
 * Function Description��Command the card to enter sleep mode
 * Input Parameter��	 NULL
 * Return Value��		 NULL
 ******************************************************************************/
void RFID::halt()
{
  unsigned char status;
  unsigned int unLen;
  unsigned char buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  calculateCRC(buff, 2, &buff[2]);

  status = MFRC522ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}


void RFID::handleIRQ()
{
    _interruptFlag = true; // Ustawienie flagi przerwania
    _lastInterruptTime = millis(); // Zapisanie czasu przerwania
    //disableCardDetectionInterrupt(); // Wyłączenie obsługi przerwań
}

void RFID::handleIRQWrapper()
{
    if (_instance != nullptr) {
        _instance->handleIRQ();
    }
}


void RFID::enableCardDetectionInterrupt()
{
    if (_irqPin != -1) {
        // Enable interrupt for card detection (IdleIEn bit in CommIEnReg)
        attachInterrupt(digitalPinToInterrupt(_irqPin), handleIRQWrapper, FALLING);
        writeMFRC522(CommIEnReg, 0x20); // Enable IdleIEn bit
        writeMFRC522(DivlEnReg, 0x00);  // Disable all interrupts in DivIEnReg

        // Clear all interrupt request bits
        clearCardDetectionInterrupt();
    }
}

void RFID::disableCardDetectionInteqrupt()
{
    if (_irqPin != -1) {
        // Disable all interrupt sources
        detachInterrupt(digitalPinToInterrupt(_irqPin));
        writeMFRC522(CommIEnReg, 0x00); // Disable all interrupt bits in CommIEnReg
        writeMFRC522(DivlEnReg, 0x00);  // Disable all interrupt bits in DivIEnReg

        // Clear all interrupt request bits
        clearCardDetectionInterrupt();
    }
}

void RFID::clearCardDetectionInterrupt()
{
    if (_irqPin != -1) {
        // Clear all interrupt request bits
        writeMFRC522(CommIrqReg, 0x7F); // Clear all interrupt bits in CommIrqReg
        writeMFRC522(DivIrqReg, 0x7F);  // Clear all interrupt bits in DivIrqReg
    }
}

bool RFID::isInterruptTriggered()
{
    if (_irqPin != -1) {
        // Read the state of the IRQ pin
        return digitalRead(_irqPin) == HIGH;
    }
    return false;
}

