#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <wiringPi.h>

#include "rfid.h"
#include "bitspi.h"

void PCD_HardReset()
{
    digitalWrite(PIN_RST, LOW);
    //hold RST down for 50 msecs
    delay(50);
    digitalWrite(PIN_RST, HIGH);
}


void PCD_Reset()
{
    PCD_WriteRegister(CommandReg, PCD_SoftReset);
    delay(50);
    while (PCD_ReadRegister(CommandReg) & (1<<4));
	// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
}


byte PCD_ReadRegister(byte reg)
{
    byte value;
    SPI_begin_transaction();
	SPI_transfer(0x80 | (reg & 0x7E));			// MSB == 1 is for reading. LSB is not used in address.
	value = SPI_transfer(0);					// Read the value back. Send 0 to stop reading.
	SPI_end_transaction();
    // printf("read %x as %x\n", reg, value);
	return value;
}

void PCD_ReadRegisterToBuffer(byte reg, byte len, byte *buf, byte rxAlign)
{
	byte address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	byte index = 0;							// Index in buf array.

    if (len == 0)
		return;
    SPI_begin_transaction();
	len--;								// One read is performed outside of the loop
	SPI_transfer(address);					// Tell MFRC522 which address we want to read

	while (index < len)
    {
		if (index == 0 && rxAlign)
        {	// Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			byte mask = 0;
            byte i;
			for (i = rxAlign; i <= 7; i++)
				mask |= (1 << i);

			// Read value and tell that we want to read the same address again.
			byte value = SPI_transfer(address);
			// Apply mask to both current value of values[0] and the new data in value.
			buf[0] = (buf[index] & ~mask) | (value & mask);
		}
		else
            // Normal case
			buf[index] = SPI_transfer(address);	// Read value and tell that we want to read the same address again.

		index++;
	}
    buf[index] = SPI_transfer(0);	// Read the final byte. Send 0 to stop reading.
	SPI_end_transaction();
}

void PCD_WriteRegisterFromBuffer(byte reg, byte len, byte *buf)
{
    SPI_begin_transaction();
	SPI_transfer(reg & 0x7E);			// MSB == 1 is for reading. LSB is not used in address.
    while (len--)
    {
        SPI_transfer(*(buf++));
    }
	SPI_end_transaction();
}

void PCD_WriteRegister(byte reg, byte value)
{
    // printf("write %x to %x.\n", value, reg);
    SPI_begin_transaction();
	SPI_transfer(reg & 0x7E);			// MSB == 1 is for reading. LSB is not used in address.
    SPI_transfer(value);					// Read the value back. Send 0 to stop reading.
	SPI_end_transaction();
}

void PCD_SetRegisterBitMask(byte reg, byte mask)
{
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
}

void PCD_ClearRegisterBitMask(byte reg, byte mask)
{
    byte tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
}

void PCD_AntennaOn()
{
    byte value;
    value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03)
        PCD_WriteRegister(TxControlReg, value | 0x03);
}

void PCD_AntennaOff()
{
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}

void PCD_Init()
{
    SPI_init();

    pinMode(PIN_RST, OUTPUT);
    //do not select the chip for now
    PCD_HardReset();

    //configure a timeout of 25ms
    PCD_WriteRegister(TModeReg, 0x80);
    PCD_WriteRegister(TPrescalerReg, 0xA9);
    PCD_WriteRegister(TReloadRegH, 0x03);
    PCD_WriteRegister(TReloadRegL, 0xE8);

    //force 100% ASK
    PCD_WriteRegister(TxASKReg, 0x40);
    //Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363
    PCD_WriteRegister(ModeReg, 0x3D);

    PCD_AntennaOn();
}

void PCD_Deinit()
{
    SPI_deinit();
    digitalWrite(PIN_RST, LOW);
    pinMode(PIN_RST, INPUT);
}

byte PCD_Version()
{
    return PCD_ReadRegister(VersionReg);
}

int PICC_Select(
    Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
	byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
)
{
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	int result;
	byte count;
	byte index;
	byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte.
	byte *responseBuffer;
	byte responseLength;

	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9

	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}

	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.

	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = FALSE;
	while (!uidComplete) {
        // printf("cascadeLevel = %u\n", cascadeLevel);
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;

			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;

			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = FALSE;						// Never used in CL3.
				break;

			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}

		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}

		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = FALSE;
		while (!selectDone) {
            // printf("selecting.\n");
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//printf(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//printf(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}

			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, FALSE);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
                // printf("collision found.\n");
				byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = TRUE; // No more anticollision
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)

        // printf("select done\n");

		// We do not check the CBB - it was constructed by us above.

		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}

		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = TRUE;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)

    // printf("uid complete.\n");

	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()


int PCD_CalculateCRC(
    byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
    byte length,	///< In: The number of bytes to transfer.
	byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
)
{
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterFromBuffer(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73�s.
	int i = 5000;
	byte n;
	while (1) {
		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL);
	result[1] = PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
} // End PCD_CalculateCRC()


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
)
{
	byte n, _validBits;
	unsigned int i;

	// Prepare values for BitFramingReg
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterFromBuffer(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86�s.
	i = 6000;
	while (1) {
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
            printf("timer expired.");
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other conditions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
            printf("counter expired.");
            return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = PCD_ReadRegister(FIFOLevelReg);			// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;									// Number of bytes returned
		PCD_ReadRegisterToBuffer(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2];
		int status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()


int PCD_TransceiveData(
    byte *sendData,		///< Pointer to the data to transfer to the FIFO.
	byte sendLen,		///< Number of bytes to transfer to the FIFO.
	byte *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
	byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
	byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default NULL.
	byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
	bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
)
{
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()


int PICC_IsNewCardPresent()
{
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);
	int result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

int PICC_RequestA(
    byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
	byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
)
{
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

int PICC_REQA_or_WUPA(
    byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
    byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
)
{
	byte validBits;
	int status;

	if (bufferATQA == NULL || *bufferSize < 2)	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;

	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, FALSE);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 */
int PICC_HaltA()
{
	int result;
	byte buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, FALSE);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()


int PCD_Authenticate(
    byte command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
	byte blockAddr, 	///< The block number. See numbering in the comments in the .h file.
	MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
	Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
)
{
	byte waitIRq = 0x10;		// IdleIRq
    byte i;

	// Build command buffer
	byte sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (i = 0; i < MF_KEY_SIZE; i++)
    	// 6 key bytes
		sendData[2+i] = key->keyByte[i];

	for (i = 0; i < 4; i++)
        // The first 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i];

	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, sendData, sizeof(sendData), NULL, NULL, NULL, 0, FALSE);
}

void PCD_StopCrypto1()
{
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08);
}

int MIFARE_Read(
    byte blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
	byte *buffer,		///< The buffer to store the data in
	byte *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
)
{
	int result;
	// Sanity check
	if (buffer == NULL || *bufferSize < 18)
		return STATUS_NO_ROOM;
	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK)
		return result;
	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, TRUE);
}


int MIFARE_Write(
    byte blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
	byte *buffer,	///< The 16 bytes to write to the PICC
	byte bufferSize	///< Buffer size, must be at least 16 bytes. Exactly 16 bytes are written.
)
{
	int result;
	// Sanity check
	if (buffer == NULL || bufferSize < 16)
		return STATUS_INVALID;

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	byte cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2, FALSE); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		return result;

	// Step 2: Transfer the data
	result = PCD_MIFARE_Transceive(buffer, bufferSize, FALSE); // Adds CRC_A and checks that the response is MF_ACK.
	if (result != STATUS_OK)
		return result;

	return STATUS_OK;
}


int PCD_MIFARE_Transceive(
    byte *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
	byte sendLen,		///< Number of bytes in sendData.
	bool acceptTimeout	///< True => A timeout is also success
)
{
	int result;
	byte cmdBuffer[18]; // We need room for 16 bytes data and 2 bytes CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen > 16)
		return STATUS_INVALID;

	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != STATUS_OK)
		return result;

	sendLen += 2;

	// Transceive the data, store the reply in cmdBuffer[]
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	byte cmdBufferSize = sizeof(cmdBuffer);
	byte validBits = 0;
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, FALSE);
	if (acceptTimeout && result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()


int PICC_GetType(byte sak)
{
	// http://www.nxp.com/documents/application_note/AN10833.pdf
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak)
    {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
}

void PICC_DumpMifareClassic(
    Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
	int piccType,	///< One of the PICC_Type enums.
	MIFARE_Key *key		///< Key A used for all sectors.
)
{
	byte no_of_sectors = 0;
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;

		case PICC_TYPE_MIFARE_1K:
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;

		case PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;

		default: // Should not happen. Ignore.
			break;
	}

	// Dump sectors, highest address first.
	if (no_of_sectors)
    {
		printf("Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits\n");
        int8_t i;
		for (i = no_of_sectors - 1; i >= 0; i--)
			PICC_DumpMifareClassicSector(uid, key, i);
	}
	PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()

int PICC_DumpMifareClassicSector(
    Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
	MIFARE_Key *key,	///< Key A for the sector.
	byte sector			///< The sector to dump, 0..39.
)
{
	int status;
	byte firstBlock;		// Address of lowest address to dump actually last block dumped)
	byte no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.

	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	byte c1, c2, c3;		// Nibbles
	byte c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	byte g[4];				// Access bits for each of the four groups.
	byte group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group

	// Determine position and size of sector.
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return STATUS_INVALID;
	}

	// Dump blocks, highest address first.
	byte byteCount;
	byte buffer[18];
	byte blockAddr;
    int8_t blockOffset;
	isSectorTrailer = TRUE;

	for (blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--)
    {
		blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer)
        {
			if(sector < 10)
				printf("   "); // Pad with spaces
			else
				printf("  "); // Pad with spaces
			printf("%d", sector);
			printf("   ");
		}
		else
			printf("       ");

		// Block number
		if(blockAddr < 10)
			printf("   "); // Pad with spaces
		else {
			if(blockAddr < 100)
				printf("  "); // Pad with spaces
			else
				printf(" "); // Pad with spaces
		}
		printf("%d", blockAddr);
		printf("  ");
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer)
        {
			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK)
            {
				printf("PCD_Authenticate() failed: ");
				printf("%s\n", GetStatusCodeName(status));
				return status;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = MIFARE_Read(blockAddr, buffer, &byteCount);
		if (status != STATUS_OK)
        {
			printf("MIFARE_Read() failed: %s\n", GetStatusCodeName(status));
			continue;
		}
		// Dump data
        byte index;
		for (index = 0; index < 16; index++)
        {
			if(buffer[index] < 0x10)
				printf(" 0");
			else
				printf(" ");
			printf("%x", buffer[index]);
			if ((index % 4) == 3) {
				printf(" ");
			}
		}
		// Parse sector trailer data
		if (isSectorTrailer)
        {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = FALSE;
		}

		// Which access group is this block in?
		if (no_of_blocks == 4)
        {
			group = blockOffset;
			firstInGroup = TRUE;
		}
		else
        {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}

		if (firstInGroup)
        {
			// Print access bits
			printf(" [ ");
			printf("%d", (g[group] >> 2) & 1); printf(" ");
			printf("%d", (g[group] >> 1) & 1); printf(" ");
			printf("%d", (g[group] >> 0) & 1);
			printf(" ] ");
			if (invertedError)
				printf(" Inverted access bits did not match! ");
		}

		if (group != 3 && (g[group] == 1 || g[group] == 6))
        {
            // Not a sector trailer, a value block
			long value = ((long)(buffer[3])<<24) | ((long)(buffer[2])<<16) | ((long)(buffer[1])<<8) | (long)(buffer[0]);
			printf(" Value=0x%lx", value);
			printf(" Adr=0x%x", buffer[12]);
		}
		printf("\n");
	}

	return STATUS_OK;
}

char * GetStatusCodeName(int code)
{
	switch (code)
    {
		case STATUS_OK:				return "Success.";
		case STATUS_ERROR:			return "Error in communication.";
		case STATUS_COLLISION:		return "Collission detected.";
		case STATUS_TIMEOUT:		return "Timeout in communication.";
		case STATUS_NO_ROOM:		return "A buffer is not big enough.";
		case STATUS_INTERNAL_ERROR:	return "Internal error in the code. Should not happen.";
		case STATUS_INVALID:		return "Invalid argument.";
		case STATUS_CRC_WRONG:		return "The CRC_A does not match.";
		case STATUS_MIFARE_NACK:	return "A MIFARE PICC responded with NAK.";
		default:					return "Unknown error";
	}
}
