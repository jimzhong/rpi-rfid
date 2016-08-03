#ifndef __RFID_H__
#define __RFID_H__

#include <stdint.h>

#define PIN_RST  26

//Mifare constants
#define MF_KEY_SIZE  6
#define MF_ACK   0xA

typedef uint8_t byte;
typedef byte bool;

typedef struct {
    byte		size;			// Number of bytes in the UID. 4, 7 or 10.
    byte		uidByte[10];
    byte		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
} Uid;

typedef struct {
    byte		keyByte[MF_KEY_SIZE];
} MIFARE_Key;

//High level functions
void PCD_Init();
void PCD_Deinit();
byte PCD_Version();

int PICC_Select(
    Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
	byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
);
int PICC_HaltA();
int PICC_IsNewCardPresent();

int MIFARE_Write(
    byte blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
	byte *buffer,	///< The 16 bytes to write to the PICC
	byte bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
);

int MIFARE_Read(
    byte blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
	byte *buffer,		///< The buffer to store the data in
	byte *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
);

int PICC_DumpMifareClassicSector(
    Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
	MIFARE_Key *key,	///< Key A for the sector.
	byte sector			///< The sector to dump, 0..39.
);

void PICC_DumpMifareClassic(
    Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
	int piccType,	///< One of the PICC_Type enums.
	MIFARE_Key *key		///< Key A used for all sectors.
);


int PICC_GetType(byte SAK);


//Mid level functions
void PCD_AntennaOn();
void PCD_AntennaOff();

void PCD_HardReset();
void PCD_Reset();

char * GetStatusCodeName(int code);

int PCD_Authenticate(
    byte command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
	byte blockAddr, 	///< The block number. See numbering in the comments in the .h file.
	MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
	Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
);

void PCD_StopCrypto1();

int PCD_CalculateCRC(
    byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
    byte length,	///< In: The number of bytes to transfer.
	byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
);

int PCD_MIFARE_Transceive(
    byte *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
	byte sendLen,		///< Number of bytes in sendData.
	bool acceptTimeout	///< True => A timeout is also success
);

int PCD_CommunicateWithPICC(
    byte command,		///< The command to execute. One of the PCD_Command enums.
	byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
	byte sendLen,		///< Number of bytes to transfer to the FIFO.
	byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
	byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
	byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
	byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
	bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
);

int PCD_TransceiveData(
    byte *sendData,		///< Pointer to the data to transfer to the FIFO.
	byte sendLen,		///< Number of bytes to transfer to the FIFO.
	byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
	byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
	byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
	byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
	bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
);

int PICC_REQA_or_WUPA(
    byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
    byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
);

int PICC_RequestA(
    byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
	byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
);


//Low level functions
byte PCD_ReadRegister(byte reg);
void PCD_WriteRegister(byte reg, byte value);

void PCD_SetRegisterBitMask(byte reg, byte mask);
void PCD_ClearRegisterBitMask(byte reg, byte mask);

void PCD_WriteRegisterFromBuffer(byte reg, byte len, byte *buf);
void PCD_ReadRegisterToBuffer(byte reg, byte len, byte *buf, byte rxAlign);


//MFRC522 Registers
//Page 1: Command and status
#define	CommandReg				 0x01 << 1	// starts and stops command execution
#define	ComIEnReg				 0x02 << 1	// enable and disable interrupt request control bits
#define	DivIEnReg				 0x03 << 1	// enable and disable interrupt request control bits
#define	ComIrqReg				 0x04 << 1	// interrupt request bits
#define	DivIrqReg				 0x05 << 1	// interrupt request bits
#define	ErrorReg				 0x06 << 1	// error bits showing the error status of the last command executed
#define	Status1Reg				 0x07 << 1	// communication status bits
#define	Status2Reg				 0x08 << 1	// receiver and transmitter status bits
#define	FIFODataReg				 0x09 << 1	// input and output of 64 byte FIFO buffer
#define	FIFOLevelReg			 0x0A << 1	// number of bytes stored in the FIFO buffer
#define	WaterLevelReg			 0x0B << 1	// level for FIFO underflow and overflow warning
#define	ControlReg				 0x0C << 1	// miscellaneous control registers
#define	BitFramingReg			 0x0D << 1	// adjustments for bit-oriented frames
#define	CollReg					 0x0E << 1	// bit position of the first bit-collision detected on the RF interface

//Page 1: Command
#define	ModeReg					 0x11 << 1	// defines general modes for transmitting and receiving
#define	TxModeReg				 0x12 << 1	// defines transmission data rate and framing
#define	RxModeReg				 0x13 << 1	// defines reception data rate and framing
#define	TxControlReg			 0x14 << 1	// controls the logical behavior of the antenna driver pins TX1 and TX2
#define	TxASKReg				 0x15 << 1	// controls the setting of the transmission modulation
#define	TxSelReg				 0x16 << 1	// selects the internal sources for the antenna driver
#define	RxSelReg				 0x17 << 1	// selects internal receiver settings
#define	RxThresholdReg			 0x18 << 1	// selects thresholds for the bit decoder
#define	DemodReg				 0x19 << 1	// defines demodulator settings
#define	MfTxReg					 0x1C << 1	// controls some MIFARE communication transmit parameters
#define	MfRxReg					 0x1D << 1	// controls some MIFARE communication receive parameters
#define	SerialSpeedReg			 0x1F << 1	// selects the speed of the serial UART interface

//Page 2: Configuration
#define	CRCResultRegH			 0x21 << 1	// shows the MSB and LSB values of the CRC calculation
#define	CRCResultRegL			 0x22 << 1
#define	ModWidthReg				 0x24 << 1	// controls the ModWidth setting?
#define	RFCfgReg				 0x26 << 1	// configures the receiver gain
#define	GsNReg					 0x27 << 1	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define	CWGsPReg				 0x28 << 1	// defines the conductance of the p-driver output during periods of no modulation
#define	ModGsPReg				 0x29 << 1	// defines the conductance of the p-driver output during periods of modulation
#define	TModeReg				 0x2A << 1	// defines settings for the internal timer
#define	TPrescalerReg			 0x2B << 1	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
#define	TReloadRegH				 0x2C << 1	// defines the 16-bit timer reload value
#define	TReloadRegL				 0x2D << 1
#define	TCounterValueRegH		 0x2E << 1	// shows the 16-bit timer value
#define	TCounterValueRegL		 0x2F << 1

// Page 3: Test Registers
#define	TestSel1Reg				 0x31 << 1	// general test signal configuration
#define	TestSel2Reg				 0x32 << 1	// general test signal configuration
#define	TestPinEnReg			 0x33 << 1	// enables pin output driver on pins D1 to D7
#define	TestPinValueReg			 0x34 << 1	// defines the values for D1 to D7 when it is used as an I/O bus
#define	TestBusReg				 0x35 << 1	// shows the status of the internal test bus
#define	AutoTestReg				 0x36 << 1	// controls the digital self test
#define	VersionReg				 0x37 << 1	// shows the software version
#define	AnalogTestReg			 0x38 << 1	// controls the pins AUX1 and AUX2
#define	TestDAC1Reg				 0x39 << 1	// defines the test value for TestDAC1
#define	TestDAC2Reg				 0x3A << 1	// defines the test value for TestDAC2
#define	TestADCReg				 0x3B << 1		// shows the value of ADC I and Q channels


//PCD commands
#define PCD_Idle				 0x00		// no action cancels current command execution
#define	PCD_Mem					 0x01		// stores 25 bytes into the internal buffer
#define	PCD_GenerateRandomID	 0x02		// generates a 10-byte random ID number
#define	PCD_CalcCRC				 0x03		// activates the CRC coprocessor or performs a self test
#define	PCD_Transmit			 0x04		// transmits data from the FIFO buffer
#define	PCD_NoCmdChange			 0x07		// no command change can be used to modify the CommandReg register bits without affecting the command for example the PowerDown bit
#define	PCD_Receive				 0x08		// activates the receiver circuits
#define	PCD_Transceive 			 0x0C		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
#define	PCD_MFAuthent 			 0x0E		// performs the MIFARE standard authentication as a reader
#define	PCD_SoftReset			 0x0F		// resets the MFRC522

//PICC general commands
#define	PICC_CMD_REQA			 0x26		// REQuest command Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
#define	PICC_CMD_WUPA			 0x52		// Wake-UP command Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
#define	PICC_CMD_CT				 0x88		// Cascade Tag. Not really a command but used during anti collision.
#define	PICC_CMD_SEL_CL1		 0x93		// Anti collision/Select Cascade Level 1
#define	PICC_CMD_SEL_CL2		 0x95		// Anti collision/Select Cascade Level 2
#define	PICC_CMD_SEL_CL3		 0x97		// Anti collision/Select Cascade Level 3
#define	PICC_CMD_HLTA			 0x50		// HaLT command Type A. Instructs an ACTIVE PICC to go to state HALT.

// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf Section 9)
// Use PCD_MFAuthent to authenticate access to a sector then use these commands to read/write/modify the blocks on the sector.
// The read/write commands can also be used for MIFARE Ultralight.
#define	PICC_CMD_MF_AUTH_KEY_A	 0x60		// Perform authentication with Key A
#define	PICC_CMD_MF_AUTH_KEY_B	 0x61		// Perform authentication with Key B
#define	PICC_CMD_MF_READ		 0x30		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
#define	PICC_CMD_MF_WRITE		 0xA0		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
#define	PICC_CMD_MF_DECREMENT	 0xC0		// Decrements the contents of a block and stores the result in the internal data register.
#define	PICC_CMD_MF_INCREMENT	 0xC1		// Increments the contents of a block and stores the result in the internal data register.
#define	PICC_CMD_MF_RESTORE		 0xC2		// Reads the contents of a block into the internal data register.
#define	PICC_CMD_MF_TRANSFER	 0xB0		// Writes the contents of the internal data register to a block.

// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf Section 8.6)
// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
#define	PICC_CMD_UL_WRITE		 0xA2		// Writes one 4 byte page to the PICC.


//Status code
#define STATUS_OK   0
#define STATUS_ERROR    1
#define STATUS_COLLISION    2
#define STATUS_TIMEOUT  3
#define STATUS_NO_ROOM  4
#define STATUS_INTERNAL_ERROR   5
#define STATUS_INVALID  6
#define STATUS_CRC_WRONG    7
#define STATUS_MIFARE_NACK  255


//PICC card type
#define		PICC_TYPE_UNKNOWN		0
#define		PICC_TYPE_ISO_14443_4	1	// PICC compliant with ISO/IEC 14443-4
#define		PICC_TYPE_ISO_18092		2 	// PICC compliant with ISO/IEC 18092 (NFC)
#define		PICC_TYPE_MIFARE_MINI	3	// MIFARE Classic protocol 320 bytes
#define		PICC_TYPE_MIFARE_1K		4	// MIFARE Classic protocol 1KB
#define		PICC_TYPE_MIFARE_4K		5	// MIFARE Classic protocol 4KB
#define		PICC_TYPE_MIFARE_UL		6	// MIFARE Ultralight or Ultralight C
#define		PICC_TYPE_MIFARE_PLUS	7	// MIFARE Plus
#define		PICC_TYPE_TNP3XXX		8	// Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure
#define		PICC_TYPE_NOT_COMPLETE	255	// SAK indicates UID is not complete.

#endif
