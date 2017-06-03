/*
 * SPI Flash memory library for anarduino.
 * This works with 256byte/page SPI flash memory (typically SPANSION S25FL127S)
 * For instance a 128MBit (16Mbyte) flash chip will have 65536 pages: 256*65536 = 16777216 bytes (16MKbytes)
 * This library uses the same function ones as the Moteino SPIFlash and is interned to be used for wireless programming
 * using the Moteino terminology for Anarduino
 * See http://lowpowerlab.com/blog/category/moteino/wireless-programming/ and https://github.com/LowPowerLab/WirelessProgramming
 * Therefore only a limited set of SPANSION programming functions are implemented to erase, read and write the Flash memory
 *
 * NOTES:
 *		The Anarduino SPANSION Flash memory (S25FL127S) uses a SPI command interface that is slightly different than the one used by the
 *		Moteino WINBOND (W25X40CL).
 *       bulkErase() takes quite a long time (typically 45 seconds for 16 MBytes)
 * 		4. The WINBOND Unique Identifier 8 Bytes value is replaced by a 12 last Bytes of the fisrt 16 Bytes OTP (Manufacturer One Time Program) which is obtained using the readUniqueId () command
 *			Note from the specs:
 *				The OTP 16 lowest address bytes are programmed by Spansion with a 128-bit random number. Only Spansion is able to program these bytes.
 *		5. The JEDEC identifier for the S25FL127S is 0x12018
 *			Note:
 *				1st Byte:  0x01 Manufacturer ID for Spansion
 *				2nd Byte:  0x20 (128 Mb) Device ID Most Significant Byte - Memory Interface Type
 *				3rd Byte:  0x18 (128 Mb) Device ID Least Significant Byte - Density
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
*/

#include <SPIFlashB.h>

SPIFlashA::SPIFlashA(uint8_t slaveSelectPin, uint32_t jedecID) {
  _slaveSelectPin = slaveSelectPin;
  _jedecID = jedecID;
}

/// Select the flash chip
void SPIFlashA::select() {
  noInterrupts();
  //save current SPI settings
  _SPCR = SPCR;					// Required if Multiple SPI are used (typically RFM69)
  _SPSR = SPSR;
  //set FLASH chip SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4); //decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  SPI.begin();
  digitalWrite(_slaveSelectPin, LOW);
}

/// UNselect the flash chip
void SPIFlashA::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
  //restore SPI settings to what they were before talking to the FLASH chip
  SPCR = _SPCR;				// Required if Multiple SPI are used (typically RFM69)
  SPSR = _SPSR;
  interrupts();
}

/// setup SPI, read device ID etc...
boolean SPIFlashA::initialize()
{
  _SPCR = SPCR;				// Required if Multiple SPI are used (typically RFM69)
  _SPSR = SPSR;
  pinMode(_slaveSelectPin, OUTPUT);
  unselect();
  while (busy());		// Ensure the memory is ready after power up or restart

  if (_jedecID == 0 || readDeviceId() == _jedecID) {
    command(SPIFLASH_STATUSWRITE, true); // Write Status Register
    SPI.transfer(0);                     // Global Unprotect
    unselect();
    return true;
  }
  return false;
}

/// Get the manufacturer and device ID bytes (as a long)
long SPIFlashA::readDeviceId()
{
	long jedecid = 0;
#if defined(__AVR_ATmega32U4__) // Arduino Leonardo, MoteinoLeo
  command(SPIFLASH_IDREAD); // Read JEDEC ID
#else
  select();
  SPI.transfer(SPIFLASH_IDREAD);
#endif
  jedecid |= (long) SPI.transfer(0) <<16;
  jedecid |= (long) SPI.transfer(0) << 8;
  jedecid |= (long) SPI.transfer(0);
  unselect();
  return jedecid;
}

/// Get the 64 bit unique identifier, stores it in provided uniqueId[8] and return a pointer to it
byte* SPIFlashA::readUniqueId(byte uniqueId[12])
{
  command(SPIFLASH_MACREAD);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);
  for (byte i=0;i<12;i++)				// Change from 8 to 12 for SPANSION
    uniqueId[i] = SPI.transfer(0);
  unselect();
  return uniqueId;
}

/// read 1 byte from flash memory
byte SPIFlashA::readByte(long addr) {
  command(SPIFLASH_ARRAYREADLOWFREQ);
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  byte result = SPI.transfer(0);
  unselect();
  return result;
}

/// read unlimited # of bytes
void SPIFlashA::readBytes(long addr, void* buf, word len) {
  command(SPIFLASH_ARRAYREAD);
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  SPI.transfer(0); //"dont care"
  for (word i = 0; i < len; ++i)
    ((byte*) buf)[i] = SPI.transfer(0);
  unselect();
}

/// Send a command to the flash chip, pass TRUE for isWrite when its a write command
void SPIFlashA::command(byte cmd, boolean isWrite){
#if defined(__AVR_ATmega32U4__) // Arduino Leonardo, MoteinoLeo
  DDRB |= B00000001;            // Make sure the SS pin (PB0 - used by RFM12B on MoteinoLeo R1) is set as output HIGH!
  PORTB |= B00000001;
#endif
  if (isWrite)
  {
    command(SPIFLASH_WRITEENABLE); // Write Enable
    unselect();
  }
  //wait for any write/erase to complete
  //  a time limit cannot really be added here without it being a very large safe limit
  //  that is because some chips can take several seconds to carry out a chip erase or other similar multi block or entire-chip operations
  //  a recommended alternative to such situations where chip can be or not be present is to add a 10k or similar weak pulldown on the
  //  open drain MISO input which can read noise/static and hence return a non 0 status byte, causing the while() to hang when a flash chip is not present
  while(busy());
  select();
  SPI.transfer(cmd);
}

/// check if the chip is busy erasing/writing
boolean SPIFlashA::busy()
{
  return readStatus() & 1;
}

/// return the STATUS register
byte SPIFlashA::readStatus()
{
  select();
  SPI.transfer(SPIFLASH_STATUSREAD);
  byte status = SPI.transfer(0);
  unselect();
  return status;
}


/// Write 1 byte to flash memory
/// WARNING: you can only write to previously erased memory locations (see datasheet)
///          use the block erase commands to first clear memory (write 0xFFs)
void SPIFlashA::writeByte(long addr, uint8_t byt) {
  command(SPIFLASH_BYTEPAGEPROGRAM, true);  // Byte/Page Program
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  SPI.transfer(byt);
  unselect();
}

/// write 1-256 bytes to flash memory
/// WARNING: you can only write to previously erased memory locations (see datasheet)
///          use the block erase commands to first clear memory (write 0xFFs)
/// WARNING: if you write beyond a page boundary (or more than 256bytes),
///          the bytes will wrap around and start overwriting at the beginning of that same page
///          see datasheet for more details
void SPIFlashA::writeBytes(long addr, const void* buf, uint16_t len) {
  command(SPIFLASH_BYTEPAGEPROGRAM, true);  // Byte/Page Program
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  for (uint16_t i = 0; i < len; i++)
    SPI.transfer(((byte*) buf)[i]);
  unselect();
}

/// erase entire flash memory array
/// may take several seconds depending on size, but is non blocking
/// so you may wait for this to complete using busy() or continue doing
/// other things and later check if the chip is done with busy()
/// note that any command will first wait for chip to become available using busy()
/// so no need to do that twice
void SPIFlashA::bulkErase() {
  command(SPIFLASH_CHIPERASE, true);
  unselect();
}

/// erase a 4Kbyte block
void SPIFlashA::blockErase4K(long addr) {
  command(SPIFLASH_BLOCKERASE_4K, true); // Block Erase
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  unselect();
}

/// erase a 64Kbyte block
void SPIFlashA::blockErase64K(long addr) {
  command(SPIFLASH_BLOCKERASE_64K, true); // Block Erase
  SPI.transfer(addr >> 16);
  SPI.transfer(addr >> 8);
  SPI.transfer(addr);
  unselect();
}

void SPIFlashA::circular_log(uint32_t& addr, uint8_t* payload, size_t size)
{
  // TODO: use faster FIFO writeBytes(addr,payload,length), but check block boundaries else wrap around will occur!
  // Writing uses 50mA, see http://www.cypress.com/file/177961/download
  /*
    10.5.1.1 Page Programming
    Page Programming is done by loading a Page Buffer with data to be programmed and issuing a programming
    command to move data from the buffer to the memory array. This sets an upper limit on the amount of data
    that can be programmed with a single programming command. Page Programming allows up to a page size
    (either 256 or 512 bytes) to be programmed in one operation. The page size is determined by a configuration
    bit (SR2[6]). The page is aligned on the page size address boundary. It is possible to program from one bit up
    to a page size in each Page programming operation. It is recommended that a multiple of 16-byte length and
    aligned Program Blocks be written. For the very best performance, programming should be done in full,
    aligned, pages of 512 bytes aligned on 512-byte boundaries with each Page being programmed only once.
  */
  for(size_t n=0; n<size; ++n)
  {
    if((addr & 0xFFF) == 0) // entering a new 4KB sector? then we need to erase it first
    {
      blockErase4K(addr / 0x1000);
      while(busy());
    }
    writeByte(addr, payload[n]);
    while(busy());
    ++addr;
    if(addr==0x1000000) // 16MByte wrap around
      addr= 0;
  }
  while(busy());
}

/// cleanup
void SPIFlashA::end() {
  SPI.end();
}
