/* floppybridge_abstract
*
* Copyright 2021 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This library defines a standard interface for connecting physical disk drives to WinUAE
*
* Derived classes must be implemented so they are unlikely to cause a delay in any function
* as doing so would cause audio and mouse cursors to stutter
* 
* This is free and unencumbered released into the public domain.
* See the file COPYING for more details, or visit <http://unlicense.org>.
*
*/

#pragma once

#include <Windows.h>
#include <functional>

// In MSVC++ you can now use the word 'abstract' rather than =0 to make a pure virtual function.
// If you dont have support for this, uncomment this line
// #define abstract =0


class FloppyDiskBridge {
public:
	// Definition of the type of drive
	enum class DriveTypeID { dti35DD, dti35HD, dti5255SD };

	FloppyDiskBridge() {};
	// This is just to force this being virtual
	virtual ~FloppyDiskBridge() {};

	// Call to start the system up.  Return false if it fails
	virtual bool initialise() abstract;

	// Returns the name of interface.  This pointer should remain valid after the class is destroyed
	virtual const TCHAR* getDriveIDName() abstract;

	// Return the 'bit cell' time in uSec.  Standard DD Amiga disks this would be 2uS, HD disks would be 1us I guess, but mainly used for =4 for SD I think
	virtual unsigned char getBitSpeed() { return 2; };

	// Return the type of disk connected.  This is used to tell WinUAE if we're DD or HD
	virtual DriveTypeID getDriveTypeID() abstract;

	// Call to get the last error message.  Length is the size of the buffer in TCHARs, the function returns the size actually needed
	virtual unsigned int getLastErrorMessage(TCHAR* errorMessage, unsigned int length) { return 0; };

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




	// Reset the drive.  This should reset it to the state it would be at powerup, ie: motor switched off etc.  The current cylinder number can be 'unknown' at this point
	virtual bool resetDrive(int trackNumber) abstract;




	/////////////////////// Head movement Controls //////////////////////////////////////////
	// Return TRUE if the drive is currently on cylinder 0
	virtual bool isAtCylinder0() abstract;

	// Return the number of cylinders the drive supports.  Eg: 80 or 82 (or 40)
	virtual unsigned char getMaxCylinder() abstract;

	// Seek to a specific cylinder
	virtual void gotoCylinder(int cylinderNumber, bool side) abstract;

	// Return the current cylinder number we're on
	virtual unsigned char getCurrentCylinderNumber() abstract;



	/////////////////////// Drive Motor Controls /////////////////////////////////////////////
	// Return true if the motor is spinning, but not necessarly up to speed
	virtual bool isMotorRunning() abstract;

	// Turn on and off the motor
	virtual void setMotorStatus(bool turnOn, bool side) abstract;

	// Returns TRUE if the drive is ready (ie: the motor has spun up to speed to speed)
	virtual bool isReady() abstract;



	/////////////////////// Disk Detection ///////////////////////////////////////////////////
	// Return TRUE if there is a disk in the drive.  This is usually called after gotoCylinder
	virtual bool isDiskInDrive() abstract;

	// Check if the disk has changed.  This should be set to TRUE if the disk has been removed from the drive
	virtual bool hasDiskChanged() abstract;



	/////////////////////// Reading Data /////////////////////////////////////////////////////
	// Return TRUE if we're at the INDEX marker/sensor.  mfmPositionBits is in BITS
	virtual bool isMFMPositionAtIndex(int mfmPositionBits) abstract;

	// This returns a single MFM bit at the position provided
	virtual bool getMFMBit(const int mfmPositionBits) abstract;

	// This asks the time taken to read the bit at mfmPositionBits.  1000=100%, <1000 data is read faster, >1000 data is read slower.
	// This number is used in a loop (scaled) starting with a number, and decrementing by this number.
	// Each loop a single bit is read.  So the smaller the number, the more loops that occur, and the more bits that are read
	virtual int getMFMSpeed(const int mfmPositionBits) abstract;

	// This is called in both modes.  It is called when WinUAE detects a full revolution of data has been read.  This could allow you to switch to a different recording of the same cylinder if needed.
	virtual void mfmSwitchBuffer(bool side) abstract;

	// Return the maximum size of bits available in this revolution.  1 - this is the maximimum passed to getMFMBit
	virtual int maxMFMBitPosition() abstract;


	/////////////////////// Writing Data /////////////////////////////////////////////////////

	// Submits a single WORD of data received during a DMA transfer to the disk buffer.  This needs to be saved.  It is usually flushed when commitWriteBuffer is called
	// You should reset this buffer if side or track changes, mfmPosition is provided purely for any index sync you may wish to do
	virtual void writeShortToBuffer(bool side, unsigned int track, unsigned short mfmData, int mfmPosition) abstract;

	// Return TRUE if the currently insrted disk is write protected
	virtual bool isWriteProtected() abstract;

	// Requests that any data received via writeShortToBuffer be saved to disk. The side and track should match against what you have been collected
	// and the buffer should be reset upon completion.  You should return the new tracklength (maxMFMBitPosition) with optional padding if needed
	virtual unsigned int commitWriteBuffer(bool side, unsigned int track) abstract;

};


