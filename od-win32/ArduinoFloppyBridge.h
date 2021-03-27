/* ArduinoFloppyBridge (and writer) for WinUAE
*
* Copyright (C) 2021 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Library General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Library General Public License for more details.
*
* You should have received a copy of the GNU Library General Public
* License along with this library; if not, see http://www.gnu.org/licenses/
*/

//////////////////////////////////////////////////////////////////////////////////////////
// This class provides an interface between WinUAE and the Arduino Floppy Reader/Writer //
//////////////////////////////////////////////////////////////////////////////////////////
//
// ROMTYPE_ARDUINOREADER_WRITER must be defined for this to work


#pragma once

#ifdef ROMTYPE_ARDUINOREADER_WRITER

#include "floppybridge_abstract.h"
#include "ArduinoInterface.h"
#include <thread>
#include <functional>
#include <queue>


// Maximum data we can receive.  
#define ARD_MFM_BUFFER_MAX_TRACK_LENGTH			0x3800

// Max number of cylinders we can offer
#define ARD_MAX_CYLINDER_BRIDGE 82

class ArduinoFloppyDiskBridge : public FloppyDiskBridge {
private:
	enum class QueueCommand { qcTerminate, qcMotorOn, qcMotorOff, writeMFMData, qcGotoToTrack, qcSelectDiskSide};
	 
	// Data to hold in the queue
	struct QueueInfo {
		QueueCommand command;
		union {
			int i;
			bool b;
		} option;
	};

	// Hardware connection
	ArduinoFloppyReader::ArduinoInterface* m_io;

	// Track data to write
	struct TrackToWrite {
		// Which track to write it to
		unsigned char mfmBuffer[ARD_MFM_BUFFER_MAX_TRACK_LENGTH];

		// Which side to write it to
		ArduinoFloppyReader::DiskSurface side;

		// Which track to write to
		unsigned char trackNumber;

		// Size of the above buffer in bits
		unsigned int floppyBufferSizeBits;

		// Should we start writing from the INDEX pulse?
		bool writeFromIndex;
	};

	// A list of writes that still need to happen to a disk
	std::vector<TrackToWrite> m_pendingTrackWrites;
	// Protect the above vector in the multithreadded environment
	CRITICAL_SECTION m_pendingWriteLock;
	// The current track being written to (or more accurately read from WinUAE)
	TrackToWrite m_currentWriteTrack;
	int m_currentWriteStartMfmPosition;
	bool m_delayStreaming;
	DWORD m_delayStreamingStart;

	// Cache of drive read data
	struct MFMCache {
		// Buffer for current data.  This is a circular buffer
		ArduinoFloppyReader::MFMSample mfmBuffer[ARD_MFM_BUFFER_MAX_TRACK_LENGTH];

		// If this is a complete revolution or not
		bool ready;

		// Size fo the mfm data we actually have (in bits) so far
		int amountReadInBits;
	};

	// Cache history
	struct MFMCaches {
		// Currently being read by WinUAE version of this track
		MFMCache current;
		// The track we're about to read in
		MFMCache next;
		std::vector<unsigned char> startBitPatterns;
	};
	
	// Cache of entire disk (or what we have so far)
	MFMCaches m_mfmRead[ARD_MAX_CYLINDER_BRIDGE][2];

	// The main thread
	std::thread* m_control;

	int m_comPort;					// Comport we're connecting on

	int m_currentTrack;				// what we tell WinUAE
	int m_actualCurrentCylinder;	// Where we actually are

	// Error messages
	std::string m_lastError;
	std::wstring m_lastErrorW;
	
	// Disk and drive status
	bool m_writeProtected;					
	bool m_diskInDrive;
	
	// Motor Status
	bool m_motorSpinningUp;					// True if the drive is spinning up, but not at speed
	DWORD m_motorSpinningUpStart;			// When it started spinnign up (GetTickCount)
	bool m_motorIsReady;					// set when the above cycle completes
	bool m_isMotorRunning;					// Direct - from WinUAE's point fo view

	DWORD m_lastDiskCheckTime;				// When we last checked to see if a disk was still in the drive (you cant tell without moving the head)
	DWORD m_lastDriveStepTime;				// When the drive head last moved or changed side

	ArduinoFloppyReader::DiskSurface m_floppySide;				// What WinUAE thinks
	ArduinoFloppyReader::DiskSurface m_actualFloppySide;		// Where we currently are

	// Thread sync
	std::queue<QueueInfo> m_queue;								// Queue of stuff the thread needs to do
	CRITICAL_SECTION m_queueProtect;							// Lock around the memory of the above queue
	CRITICAL_SECTION m_switchBufferLock;						// Lock to protect buffers while being switched
	HANDLE m_queueSemaphore;									// To notify the thread of something being added to thr queue
	HANDLE m_readBufferAvailable;								// Gets set the moment a new buffer is ready (event)

	void pushOntoQueue(const QueueInfo& info);

	// The main thread
	void mainThread();
	// Add a command for the thread to process
	void queueCommand(QueueCommand command, bool optionB);
	void queueCommand(QueueCommand command, int optionI=0);

	// Handle processing the command
	void processCommand(const QueueInfo& info);

	// Handle background reading
	void handleBackgroundDiskRead();

	// Reset the current MFM buffer used by readMFMBit()
	void resetMFMCache();

	// Called to reverse initialise.  This will automatically be called in the destructor too.
	void terminate();

	// Handle disk side change
	void switchDiskSide(bool side);

	// Process the queue.  Return TRUE if the thread should quit
	bool processQueue();

	// This is called to switch to a different copy of the track so multiple revolutions can ve read
	void internalSwitchCylinder(int cylinder, ArduinoFloppyReader::DiskSurface side);

	// Save a new disk side and switch it in if it can be
	void saveNextBuffer(int cylinder, ArduinoFloppyReader::DiskSurface side);

	// Reset and clear out any data we have received thus far
	void resetWriteBuffer();

public:
	// Drive is 0 to 3, comPort is probably 1-9
	ArduinoFloppyDiskBridge(const int device_settings);
	virtual ~ArduinoFloppyDiskBridge();

	// Call to start the system up
	virtual bool initialise() override;

	// Returns the name of interface.  This pointer should remain valid after the class is destroyed
	virtual const TCHAR* getDriveIDName() override;

	// Call to get the last error message.  Length is the size of the buffer in TCHARs, the function returns the size actually needed
	virtual unsigned int getLastErrorMessage(TCHAR* errorMessage, unsigned int length) override;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Return TRUE if the drive is currently on cylinder 0
	virtual bool isAtCylinder0() override;

	// Return the number of cylinders the drive supports.  Eg: 80 or 82 (or 40)
	virtual unsigned char getMaxCylinder() override { return ARD_MAX_CYLINDER_BRIDGE; };

	// Reset the drive.  This should reset it to the state it would be at powerup
	virtual bool resetDrive(int trackNumber) override;

	// Return true if the motor is spinning
	virtual bool isMotorRunning() override;

	// Set the status of the motor. 
	virtual void setMotorStatus(bool side, bool turnOn) override;

	// Returns TRUE when the last command requested has completed
	virtual bool isReady() override;

	// Return TRUE if there is a disk in the drive, else return false.  Some drives dont detect this until the head moves once
	virtual bool isDiskInDrive() override;

	// Check if the disk has changed.  This should reset its status after being called
	virtual bool hasDiskChanged() override;

	// Return the current track number we're on
	virtual unsigned char getCurrentCylinderNumber() override;

	// Return the type of disk connected
	virtual DriveTypeID getDriveTypeID() override;

	// Return TRUE if the currently insrted disk is write protected
	virtual bool isWriteProtected() override;
	
	// Return the maximum size of the internal track buffer in BITS
	virtual int maxMFMBitPosition() override;

	// This is called to switch to a different copy of the track so multiple revolutions can ve read
	virtual void mfmSwitchBuffer(bool side) override;

	// Get the speed at this position.  1000=100%.  
	virtual int getMFMSpeed(const int mfmPositionBits) override;

	// While not doing anything else, the library should be contnoeously streaming the current track if the motor is on.  mfmBufferPosition is in BITS 
	virtual bool getMFMBit(const int mfmPositionBits) override;

	// Seek to a specific track
	virtual void gotoCylinder(int trackNumber, bool side) override;

	// Submits a single WORD of data received during a DMA transfer to the disk buffer.  This needs to be saved.  It is usually flushed when commitWriteBuffer is called
	// You should reset this buffer if side or track changes.  mfmPosition is provided purely for any index sync you may wish to do
	virtual void writeShortToBuffer(bool side, unsigned int track, unsigned short mfmData, int mfmPosition) override;

	// Requests that any data received via writeShortToBuffer be saved to disk. The side and track should match against what you have been collected
	// and the buffer should be reset upon completion.  You should return the new tracklength (maxMFMBitPosition) with optional padding if needed
	virtual unsigned int commitWriteBuffer(bool side, unsigned int track) override;

	// Return TRUE if we're at the INDEX marker
	virtual bool isMFMPositionAtIndex(int mfmPositionBits) override;

};



#endif