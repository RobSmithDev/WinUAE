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

#include "floppybridge_config.h"

#ifdef ROMTYPE_ARDUINOREADER_WRITER

#include "floppybridge_abstract.h"
#include "ArduinoFloppyBridge.h"
#include "ArduinoInterface.h"
#include <tchar.h>
#include <string>
#include <codecvt>
#include <locale>

using namespace ArduinoFloppyReader;


// Assume a disk rotates every at 300rpm.
// Assume the bitcel size is 2uSec
// This means 5 revolutions per second
// Each revolution is 200 milliseconds
// 200ms / 2uSec = 100000 bits
// 100000 bits is 12500 bytes
// Assume the drive spin speed tolerance is +/- 3%, so 
// Biggest this can be is: 12886 bytes (-3 %)
// Perfect this can be is: 12500 bytes 
// Smallest this can be is: 12135 bytes (+3 %)
#define THEORETICAL_MINIMUM_TRACK_LENGTH		12134

// How many bits of data we receive each "chunk" from the device
#define NUM_BITS_IN_CHUNK				128

// The number of MS we should allow the interface to simulate the head moving and not reporting any data at all when a track changes.  From some of the PDFs I read, this should vary between 18-50ms. 
// The larger this number, the less chance of this interface stalling WinUAE and the better track seeking we get
// Strangly enough software seems VERY tolerant of receiving no data for quite some time
#define DRIVE_STEP_GARBAGE_TIME 500

// Auto-sense the disk, just like Amiga OS will, be do this if it hasnt 
#define DISKCHANGE_BEFORE_INSERTED_CHECK_INTERVAL 2500

// We need to poll the drive once theres a disk in there ot know when its been removed.  This is the poll interval
#define DISKCHANGE_ONCE_INSERTED_CHECK_INTERVAL 500

// Auto-sense the disk, just like Amiga OS will, be do this if it hasnt - this is for the non-modded hardware that can't autodetect
#define DISKCHANGE_BEFORE_INSERTED_CHECK_INTERVAL_NONMOD 3000

// We need to poll the drive once theres a disk in there to know when its been removed.   - this is for the non-modded hardware that can't autodetect
#define DISKCHANGE_ONCE_INSERTED_CHECK_INTERVAL_NONMOD 3000

// How log we simulate the drive taking to spin up to speed.
#define DISK_SPINUP_TIME 750

// The Cylinder number to start using Write Precomp of 125ns at:
#define WRITE_PRECOMP_START 40


static const TCHAR* DriverName = _T("Arduino Floppy Disk Reader/Writer, https://amiga.robsmithdev.co.uk");


// This is a quick shorthand as I wasnt sure which side UAE thought was upper and lower
inline DiskSurface boolSideToDiskSurface(const bool surface) {
	return (surface) ? DiskSurface::dsUpper : DiskSurface::dsLower;
}
inline const bool diskSurfaceToBoolSide(const DiskSurface side) {
	return (side == DiskSurface::dsUpper);
}

// Handle disk side change
void ArduinoFloppyDiskBridge::switchDiskSide(bool side) {
	if (boolSideToDiskSurface(side) != m_floppySide) {
		resetWriteBuffer();
		m_floppySide = boolSideToDiskSurface(side);
		if (!m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
			ResetEvent(m_readBufferAvailable);
		}
		m_lastDriveStepTime = GetTickCount();
		queueCommand(QueueCommand::qcSelectDiskSide, side);
	}
}

// comPort is probably 1-9
ArduinoFloppyDiskBridge::ArduinoFloppyDiskBridge(const int device_settings) : m_comPort((device_settings & 0x0F) + 1), m_motorSpinningUp(false), m_motorSpinningUpStart(0), m_actualCurrentCylinder(0),
m_control(nullptr), m_io(nullptr), m_currentTrack(0), m_motorIsReady(false), m_isMotorRunning(false), m_floppySide(DiskSurface::dsLower) {
	InitializeCriticalSection(&m_queueProtect);
	InitializeCriticalSection(&m_switchBufferLock);
	InitializeCriticalSection(&m_pendingWriteLock);

	m_readBufferAvailable = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_queueSemaphore = CreateSemaphore(NULL, 0, 100, NULL);
	memset(&m_mfmRead, 0, sizeof(m_mfmRead));;
}

// Free
ArduinoFloppyDiskBridge::~ArduinoFloppyDiskBridge() {
	terminate();

	CloseHandle(m_queueSemaphore);
	CloseHandle(m_readBufferAvailable);
	DeleteCriticalSection(&m_queueProtect);
	DeleteCriticalSection(&m_switchBufferLock);
	DeleteCriticalSection(&m_pendingWriteLock);
}

// Add a command for the thread to process
void ArduinoFloppyDiskBridge::queueCommand(QueueCommand command, int optionI) {
	QueueInfo info;
	info.command = command;
	info.option.i = optionI;

	pushOntoQueue(info);
}

// Add a command for the thread to process
void ArduinoFloppyDiskBridge::queueCommand(QueueCommand command, bool optionB) {
	QueueInfo info;
	info.command = command;
	info.option.b = optionB;

	pushOntoQueue(info);
}

// Add to queue
void ArduinoFloppyDiskBridge::pushOntoQueue(const QueueInfo& info) {
	EnterCriticalSection(&m_queueProtect);
	m_queue.push(info);
	LeaveCriticalSection(&m_queueProtect);
	ReleaseSemaphore(m_queueSemaphore, 1, NULL);

	// Make it drop out of streaming if thats what its doing
	if (m_io) m_io->abortReadStreaming();
}

// Process the queue.  Return TRUE if the thread should quit
bool ArduinoFloppyDiskBridge::processQueue() {
	EnterCriticalSection(&m_queueProtect);
	if (m_queue.size()) {

		QueueInfo cmd = m_queue.front();
		m_queue.pop();

		LeaveCriticalSection(&m_queueProtect);

		// Special exit condition
		if (cmd.command == QueueCommand::qcTerminate) return true;

		processCommand(cmd);
	}
	else {
		LeaveCriticalSection(&m_queueProtect);
	}
	return false;
}

// The main thread
void ArduinoFloppyDiskBridge::mainThread() {
	m_lastDiskCheckTime = GetTickCount();

	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);

	SetThreadDescription(GetCurrentThread(), _T("ArduinoFloppyDiskBridge"));

	for (;;) {

		if (WaitForSingleObject(m_queueSemaphore, m_motorIsReady ? 1 : 250) == WAIT_OBJECT_0) {
			if (processQueue())
				return;
		}
		else {
			// Trigger background reading if we're not busy
			if (m_motorIsReady) {
				if ((!m_delayStreaming) || ((m_delayStreaming) && (GetTickCount() - m_delayStreamingStart > 100))) 
					handleBackgroundDiskRead();
			}

			// Is it time for us to poll the drive?
			bool readyForDiskCheck = false;
			
			if (m_io->getFirwareVersion().fullControlMod) {
				readyForDiskCheck = ((GetTickCount() - m_lastDiskCheckTime > DISKCHANGE_ONCE_INSERTED_CHECK_INTERVAL) && (m_diskInDrive)) ||
					((GetTickCount() - m_lastDiskCheckTime > DISKCHANGE_BEFORE_INSERTED_CHECK_INTERVAL) && (!m_diskInDrive));
			}
			else {
				// This version we cant detect disk change to easily.  So we do it less often as it impacts the disk more.
				readyForDiskCheck = ((GetTickCount() - m_lastDiskCheckTime > DISKCHANGE_ONCE_INSERTED_CHECK_INTERVAL_NONMOD) && (m_diskInDrive)) ||
									((GetTickCount() - m_lastDiskCheckTime > DISKCHANGE_BEFORE_INSERTED_CHECK_INTERVAL_NONMOD) && (!m_diskInDrive));
			}

			// If the queue is empty, the motor isnt on, and we think theres a disk in the drive we periodically check as we dont have any other way to find out
			if ((readyForDiskCheck) && (m_queue.size() < 1)) {

				// Monitor for disk
				m_lastDiskCheckTime = GetTickCount();

				m_io->checkForDisk(true);
				m_writeProtected = m_io->checkIfDiskIsWriteProtected(false) == DiagnosticResponse::drWriteProtected;
			}
		}

		if ((m_motorSpinningUp) && (GetTickCount() - m_motorSpinningUpStart >= DISK_SPINUP_TIME)) {
			m_motorSpinningUp = false;
			m_motorIsReady = true;
			m_lastDriveStepTime = GetTickCount();
		}


		bool lastDiskState = m_diskInDrive;
		m_diskInDrive = m_io->isDiskInDrive();
		if (lastDiskState != m_diskInDrive) {
			// Erase track cache 
			if (!m_diskInDrive) resetMFMCache();
		}
	}
}

// Reset the previously setup queue
void ArduinoFloppyDiskBridge::resetMFMCache() {
	EnterCriticalSection(&m_switchBufferLock);
	for (int a = 0; a < ARD_MAX_CYLINDER_BRIDGE; a++)
		for (int c = 0; c < 2; c++) {
			m_mfmRead[a][c].startBitPatterns.clear();
			memset(&m_mfmRead[a][c].next, 0, sizeof(m_mfmRead[a][c].next));
			memset(&m_mfmRead[a][c].current, 0, sizeof(m_mfmRead[a][c].current));
		}
	resetWriteBuffer();
	ResetEvent(m_readBufferAvailable);
	LeaveCriticalSection(&m_switchBufferLock);
}

// Save a new disk side and switch it in if it can be
void ArduinoFloppyDiskBridge::saveNextBuffer(int cylinder, DiskSurface side) {

	// Save the new buffer
	EnterCriticalSection(&m_switchBufferLock);
	if (m_mfmRead[cylinder][(int)side].next.amountReadInBits) {
		m_mfmRead[cylinder][(int)side].next.ready = true;
	}
	LeaveCriticalSection(&m_switchBufferLock);

	// Stop of the above blocked the buffer
	if (!m_mfmRead[cylinder][(int)side].next.ready) return;

	// Go live now?
	if (!m_mfmRead[cylinder][(int)side].current.ready) {

		internalSwitchCylinder(cylinder, side);

		// This test *should* always be true
		if ((cylinder == m_currentTrack) && (side == m_floppySide))
			SetEvent(m_readBufferAvailable);
	}
}

// Handle reading the disk data in the background while the queue is idle
void ArduinoFloppyDiskBridge::handleBackgroundDiskRead() {
	// Dont do anythign until the motor is ready.  The flag that checks for disk change will also report spin ready status if the drive hasnt spun up properly yet
	if (!isReady()) return;

	// If we already have the next buffer full then stop
	if (m_mfmRead[m_actualCurrentCylinder][(int)m_actualFloppySide].next.ready) {
		if (!m_mfmRead[m_actualCurrentCylinder][(int)m_actualFloppySide].current.ready)
			OutputDebugStringA("BUG BUG BUG - This should not happen\n");
		return;
	}

	// Make sure the right surface is selected
	m_io->selectSurface(m_actualFloppySide);
	unsigned int trackPositionInBytes = 0;

	MFMCache& trackData = m_mfmRead[m_actualCurrentCylinder][(int)m_actualFloppySide].next;
	trackData.amountReadInBits = 0;
	trackData.ready = false;
	
	// Grab full revolutions if possible. 
	if (m_io->readCurrentTrackStream(NUM_BITS_IN_CHUNK, 1, m_mfmRead[m_actualCurrentCylinder][(int)m_actualFloppySide].startBitPatterns,
		[this, &trackData, &trackPositionInBytes](const MFMSample* mfmData, const unsigned dataLengthInBits, const bool isEndOfRevolution) -> bool {
			// Something waiting to be processed? Abort read streaming immediately
			if (m_queue.size()) {
				trackData.amountReadInBits = 0;
				return false;
			}

			// Some data arrived from the drive.  Its too much an we can't take it all on.  This should'nt happen as a complete revolution should have been found ages ago
			unsigned int endPos = trackPositionInBytes + dataLengthInBits / 8;
			if (endPos >= ARD_MFM_BUFFER_MAX_TRACK_LENGTH) {

				// Work out how many bytes we can safely copy - this should never happen
				unsigned int bytesRemaining = (ARD_MFM_BUFFER_MAX_TRACK_LENGTH)-trackPositionInBytes;
				if (bytesRemaining) {
					memcpy(&trackData.mfmBuffer[trackPositionInBytes], mfmData, bytesRemaining * sizeof(MFMSample));
				}

				trackData.amountReadInBits = ARD_MFM_BUFFER_MAX_TRACK_LENGTH * 8;

				saveNextBuffer(m_actualCurrentCylinder, m_actualFloppySide);

				// We have to stop here.  We dont know where the data started and ended, so at any minute isEndOfRevolution might be set and we'd end up with an incomplete revolution.
				// If everythign is working proeprly and the drive is not broken, we should NEVER get into this area.  This is a belt and braces last chance, which for AmigaDOS is usually fine and resilient enough to work with this mess!
				return false;
			}
			else {
				// dataLengthInBits is only not a full bytes worth if isEndOfRevolution is true.
				int lengthInBytes = dataLengthInBits / 8;

				// Weird number of bits?
				if (dataLengthInBits & 7)
					lengthInBytes++;

				// Copy the data received.
				memcpy(&trackData.mfmBuffer[trackPositionInBytes], mfmData, lengthInBytes * sizeof(MFMSample));

				// Note where the end is
				trackData.amountReadInBits += dataLengthInBits;
				trackPositionInBytes += lengthInBytes;

				// Is this the end of a full revolution?
				if (isEndOfRevolution) {
					trackPositionInBytes = 0;

					saveNextBuffer(m_actualCurrentCylinder, m_actualFloppySide);

					// If the buffer is now empty, continue.
					if (!m_mfmRead[m_actualCurrentCylinder][(int)m_actualFloppySide].next.ready) return true;

					// No more space
					return false;
				}

				// Not at the end of the revolution, so we want more data
				return true;
			}
		}) == DiagnosticResponse::drNoDiskInDrive) {
		m_diskInDrive = false;
	}


	// If we get here then any data in the buffer would just get discarded anyway, and we dont want to promise we hvae data that we aren't reading.
	if (!trackData.ready) trackData.amountReadInBits = 0;

	// Just flag this so we dont get an immediate check
	m_lastDiskCheckTime = GetTickCount();
}

// Return TRUE if we're at the INDEX marker - we fake the start of the buffer being where the index marker is.
bool ArduinoFloppyDiskBridge::isMFMPositionAtIndex(int mfmPositionBits) {
	if (m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
		return (mfmPositionBits == 0) || (mfmPositionBits == m_mfmRead[m_currentTrack][(int)m_floppySide].current.amountReadInBits);
	}
	return mfmPositionBits == 0;
}

// Return the maximum size of the internal track buffer in BITS
int ArduinoFloppyDiskBridge::maxMFMBitPosition() {
	if (m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready)
		return m_mfmRead[m_currentTrack][(int)m_floppySide].current.amountReadInBits;

	// If there is no buffer ready, it's difficult to tell WinUAE what it want to know, as we dont either.  So we supply a absolute MINIMUM that *should* be available on a disk
	// As this is dynamically read each time it *shouldnt* be a problem and by the time it hopefullt reaches it the buffer will have gone live
	return max(THEORETICAL_MINIMUM_TRACK_LENGTH * 8, m_mfmRead[m_currentTrack][(int)m_floppySide].next.amountReadInBits);
}

// This is called to switch to a different copy of the track so multiple revolutions can ve read
void ArduinoFloppyDiskBridge::mfmSwitchBuffer(bool side) {
	switchDiskSide(side);
	internalSwitchCylinder(m_currentTrack, m_floppySide);
}

// This is called to switch to a different copy of the track so multiple revolutions can ve read
void ArduinoFloppyDiskBridge::internalSwitchCylinder(int cylinder, DiskSurface side) {
	EnterCriticalSection(&m_switchBufferLock);

	if (m_mfmRead[cylinder][(int)side].next.ready) {		
		m_mfmRead[cylinder][(int)side].current = m_mfmRead[cylinder][(int)side].next;
		m_mfmRead[cylinder][(int)side].next.amountReadInBits = 0;
		m_mfmRead[cylinder][(int)side].next.ready = false;
	}
	LeaveCriticalSection(&m_switchBufferLock);
}


// Get the speed at this position.  1000=100%.  
int ArduinoFloppyDiskBridge::getMFMSpeed(const int mfmPositionBits) {
	// If there is no data available at this point it means we will have to stall WinUAE. This will cause the mouse/frame rate to stutter
	// But theres nothign we can do about this.  Simulation vs our implementation reality issues.
	// Stop if no disk in the drive, or we're still in the 'grace period' when the drive steps to a different track
	if ((!m_diskInDrive) || (!m_motorIsReady)) return 1000;

	// Internally manage loops until the data is ready
	const int mfmPositionByte = mfmPositionBits >> 3;
	const int mfmPositionBit = 7 - (mfmPositionBits & 7);

	if (m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
		int speed = (10 * (int)(m_mfmRead[m_currentTrack][(int)m_floppySide].current.mfmBuffer[mfmPositionByte].speed[mfmPositionBit]));
		if (speed < 700)  speed = 700;
		if (speed > 3000)  speed = 3000;
		return speed;
	}

	if ((GetTickCount() - m_lastDriveStepTime < DRIVE_STEP_GARBAGE_TIME)) return 1000;

	// No full buffer ready yet.
	if (!m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
		if (mfmPositionBits < m_mfmRead[m_currentTrack][(int)m_floppySide].next.amountReadInBits) {
			int speed = (10 * (int)(m_mfmRead[m_currentTrack][(int)m_floppySide].next.mfmBuffer[mfmPositionByte].speed[mfmPositionBit]));
			if (speed < 700)  speed = 700;
			if (speed > 3000) speed = 3000;
			return speed;
		}
	}

	return 1000;
}

// Read a single bit from the data stream.   this should call triggerReadWriteAtIndex where appropriate
bool ArduinoFloppyDiskBridge::getMFMBit(const int mfmPositionBits) {
	// If there is no data available at this point it means we will have to stall WinUAE. This will cause the mouse/frame rate to stutter
	// But theres nothign we can do about this.  Simulation vs our implementation reality issues.
	// Stop if no disk in the drive, or we're still in the 'grace period' when the drive steps to a different track
	if ((!m_diskInDrive) || (!m_motorIsReady)) return 0;

	// Internally manage loops until the data is ready
	const int mfmPositionByte = mfmPositionBits >> 3;
	const int mfmPositionBit = 7 - (mfmPositionBits & 7);

	if (m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
		return (m_mfmRead[m_currentTrack][(int)m_floppySide].current.mfmBuffer[mfmPositionByte].mfmData & (1 << mfmPositionBit)) != 0;
	}

	// Hopefully this will be enough to catch most of the 'stalling' - we're simulating the drive settling after head movement
	if ((GetTickCount() - m_lastDriveStepTime < DRIVE_STEP_GARBAGE_TIME)) return 0;

	// Try to wait for the data.  A full revolution should be about 200ms, but given its trying to INDEX align, worse case this could be approx 400ms.
	// Average should be 300ms based on starting to read when the head is at least half way from the index.
	const int DelayBetweenChecks = 5;
	for (int a = 0; a < 600 / DelayBetweenChecks; a++) {

		// No full buffer ready yet.
		if (!m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {

			if (mfmPositionBits < m_mfmRead[m_currentTrack][(int)m_floppySide].next.amountReadInBits) {
				return (m_mfmRead[m_currentTrack][(int)m_floppySide].next.mfmBuffer[mfmPositionByte].mfmData & (1 << mfmPositionBit)) != 0;
			}
		}
		else {
			// If a buffer is available, go for it!
			return (m_mfmRead[m_currentTrack][(int)m_floppySide].current.mfmBuffer[mfmPositionByte].mfmData & (1 << mfmPositionBit)) != 0;
		}

		// Causes a delay of 5ms or until a buffer is made live
		WaitForSingleObject(m_readBufferAvailable, DelayBetweenChecks);
	}

	// If we get here, we give up and return nothing. So we behave like no data
	return 0;
}

// Handle processing the command
void ArduinoFloppyDiskBridge::processCommand(const QueueInfo& info) {
	if (!m_io) return;

	// See what command is being ran
	switch (info.command) {
	case QueueCommand::qcMotorOn:
		//  With V1.8 fo the firmware, reading/writing mode becomes irreverent
		m_io->enableReading(true, false, true);
		m_motorSpinningUp = true;
		m_motorSpinningUpStart = GetTickCount();
		break;

	case QueueCommand::qcGotoToTrack:
	{
		m_lastDriveStepTime = GetTickCount();
		bool ignoreDiskCheck = (m_motorSpinningUp) && (!m_motorIsReady);

		if (!m_io->getFirwareVersion().fullControlMod) {
			ignoreDiskCheck |= ((GetTickCount() - m_lastDiskCheckTime <= DISKCHANGE_ONCE_INSERTED_CHECK_INTERVAL_NONMOD) && (m_diskInDrive)) &&
							   ((GetTickCount() - m_lastDiskCheckTime <= DISKCHANGE_BEFORE_INSERTED_CHECK_INTERVAL_NONMOD) && (!m_diskInDrive));
		}

		m_io->selectTrack(info.option.i, TrackSearchSpeed::tssNormal, ignoreDiskCheck);
		if (!ignoreDiskCheck) m_lastDiskCheckTime = GetTickCount();
		m_writeProtected = m_io->checkIfDiskIsWriteProtected(false) == DiagnosticResponse::drWriteProtected;
		m_actualCurrentCylinder = info.option.i;
		m_lastDriveStepTime = GetTickCount();
	}
	break;

	case QueueCommand::qcMotorOff:
		m_io->enableReading(false, false);
		// and note its switched off
		m_motorSpinningUp = false;
		m_motorIsReady = false;
		break;

	case QueueCommand::qcSelectDiskSide:
		m_lastDriveStepTime = GetTickCount();  // slight abuse
		m_actualFloppySide = boolSideToDiskSurface(info.option.b);
		m_io->selectSurface(m_actualFloppySide);
		break;

	case QueueCommand::writeMFMData:
		// Grab the item
		EnterCriticalSection(&m_pendingWriteLock);
		if (m_pendingTrackWrites.size()) {
			TrackToWrite track = m_pendingTrackWrites.front();
			m_pendingTrackWrites.erase(m_pendingTrackWrites.begin());
			LeaveCriticalSection(&m_pendingWriteLock);

			// Is there data to write?
			if (track.floppyBufferSizeBits) {
				if (m_actualCurrentCylinder != track.trackNumber) {
					m_actualCurrentCylinder = track.trackNumber;
					m_io->selectTrack(track.trackNumber, TrackSearchSpeed::tssFast);
				}
				if (m_actualFloppySide != track.side) {
					m_actualFloppySide = track.side;
					m_io->selectSurface(track.side);
				}

				// This isnt great.  IF this fails it might be hard for WinUAE to know.
				int numBytes = track.floppyBufferSizeBits / 8;
				if (track.floppyBufferSizeBits & 7) numBytes++;
				m_io->writeCurrentTrackPrecomp(track.mfmBuffer, (track.floppyBufferSizeBits + 7) / 8, track.writeFromIndex, m_actualCurrentCylinder >= WRITE_PRECOMP_START);
				//m_io->writeCurrentTrack(track.mfmBuffer, (track.floppyBufferSizeBits + 7) / 8, track.writeFromIndex);
				m_mfmRead[m_actualCurrentCylinder][(int)m_actualFloppySide].current.ready = false;  // invalidate it
				m_lastDriveStepTime = GetTickCount();  // slight abuse to stop stalling while we read the track back
				m_delayStreaming = false;
			}
		} else LeaveCriticalSection(&m_pendingWriteLock);

		break;
	}
}

// Called to reverse initialise.  This will automatically be called in the destructor too.
void ArduinoFloppyDiskBridge::terminate() {

	if (m_control) {
		queueCommand(QueueCommand::qcTerminate);

		if (m_control->joinable())
			m_control->join();
		m_control = nullptr;
	}

	if (m_io) {
		// Turn everythign off
		m_io->enableReading(false);
		m_io->closePort();
		delete m_io;
		m_io = nullptr;
	}

	m_lastError = "";
}

// Startup.  This will display an error message if theres an issue with the interface
bool ArduinoFloppyDiskBridge::initialise() {
	// Stop first
	if (m_control) terminate();

	m_currentTrack = 0;
	m_isMotorRunning = false;
	m_motorIsReady = false;
	m_writeProtected = true; // to start with
	m_diskInDrive = false;

	// Clear down the queue
	EnterCriticalSection(&m_queueProtect);
	while (m_queue.size()) m_queue.pop();
	LeaveCriticalSection(&m_queueProtect);
	while (WaitForSingleObject(m_queueSemaphore, 0) == WAIT_OBJECT_0);

	m_io = new ArduinoFloppyReader::ArduinoInterface();

	DiagnosticResponse error = m_io->openPort(m_comPort);

	if (error == DiagnosticResponse::drOK) {
		// See which version it is
		FirmwareVersion fv = m_io->getFirwareVersion();
		if ((fv.major <= 1) && (fv.minor < 8)) {
			// Must be at least V1.8
			char buf[20];
			sprintf_s(buf, "%i.%i", fv.major, fv.minor);
			m_lastError = "Arduino Floppy Reader/Writer Firmware is Out Of Date\n\nWinUAE requires V1.8 (and ideally with the modded circuit design).\n\n";
			m_lastError += "You are currently using V" + std::string(buf) + ".  Please update the firmware.";
		}
		else {
			// Dont want this warning more than once
			static bool shownHardwareWarning = false;

			if ((!fv.fullControlMod) && (!shownHardwareWarning)) {
				shownHardwareWarning = true;
				MessageBox(GetDesktopWindow(), _T("The Arduino Reader/Writer hasn't had the 'hardware mod' applied for optimal WinUAE Support.\nThis mod is highly recommended for best experience."), _T("Arduino Reader/Writer"), MB_OK | MB_ICONINFORMATION);
			}

			m_io->findTrack0();
			m_io->checkForDisk(true);
			m_actualFloppySide = DiskSurface::dsLower;
			m_floppySide = DiskSurface::dsLower;
			m_io->selectSurface(m_floppySide);
			m_diskInDrive = m_io->isDiskInDrive();

			m_control = new std::thread([this]() {
				this->mainThread();
				});

			return true;
		}
	}
	else {
		m_lastError = m_io->getLastErrorStr();
	}


	delete m_io;
	m_io = nullptr;


	return false;
}

// Returns the name of interface.  This pointer should remain valid after the class is destroyed
const TCHAR* ArduinoFloppyDiskBridge::getDriveIDName() {
	return DriverName;
}

// Call to get the last error message.  Length is the size of the buffer in TCHARs, the function returns the size actually needed
unsigned int ArduinoFloppyDiskBridge::getLastErrorMessage(TCHAR* errorMessage, unsigned int length) {
#ifdef UNICODE
	using convert_t = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_t, wchar_t> strconverter;

	m_lastErrorW = strconverter.from_bytes(m_lastError);
	if (m_lastErrorW.length() + 1 > length) return m_lastErrorW.length() + 1; else
		wcscpy_s(errorMessage, length, m_lastErrorW.c_str());
#else
	if (m_lastError.length() + 1 > length) return m_lastError.length() + 1; else
		strcpy_s(errorMessage, length, m_lastError.c_str());
#endif
	return 0;
}

// Return TRUE if the drive is currently on track 0
bool ArduinoFloppyDiskBridge::isAtCylinder0() {
	return (m_io) && (m_currentTrack == 0);
}

// Reset the drive.  This should reset it to the state it would be at powerup
bool ArduinoFloppyDiskBridge::resetDrive(int trackNumber) {
	// Delete all future writes
	EnterCriticalSection(&m_pendingWriteLock);
	m_pendingTrackWrites.clear();
	LeaveCriticalSection(&m_pendingWriteLock);

	// Stop background reading
	setMotorStatus(diskSurfaceToBoolSide(m_floppySide), false);

	// Reset back to unknown disk
	resetMFMCache();

	// Ready
	return true;
}

// Return true if the motor is spinning
bool ArduinoFloppyDiskBridge::isMotorRunning() {
	return m_isMotorRunning;
}

// Set the status of the motor. 
void ArduinoFloppyDiskBridge::setMotorStatus(bool side, bool turnOn) {
	switchDiskSide(side);

	if (m_isMotorRunning == turnOn) return;
	m_isMotorRunning = turnOn;

	m_motorIsReady = false;
	m_motorSpinningUp = false;
	queueCommand(turnOn ? QueueCommand::qcMotorOn : QueueCommand::qcMotorOff);
}

// Returns TRUE when the last command requested has completed
bool ArduinoFloppyDiskBridge::isReady() {
	return (m_motorIsReady) && (!m_motorSpinningUp);
}

// Return TRUE if there is a disk in the drive, else return false.  Some drives dont detect this until the head moves once
bool ArduinoFloppyDiskBridge::isDiskInDrive() {
	return m_diskInDrive;
}

// Check if the disk has changed.  This should reset its status after being called
bool ArduinoFloppyDiskBridge::hasDiskChanged() {
	return !m_diskInDrive;
}

// Seek to a specific track
void ArduinoFloppyDiskBridge::gotoCylinder(int trackNumber, bool side) {
	if (m_currentTrack == trackNumber) return;
	resetWriteBuffer();
	m_currentTrack = trackNumber;

	m_lastDriveStepTime = GetTickCount();

	bool queueUpdated = false;
	switchDiskSide(side);

	// We want to see if there are other 'goto track' commands in the queue just before this one.  If there is, we can replace them
	EnterCriticalSection(&m_queueProtect);
	if (m_queue.size()) {
		if (m_queue.back().command == QueueCommand::qcGotoToTrack) {
			m_queue.back().option.i = trackNumber;
			queueUpdated = true;
			if (!m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
				ResetEvent(m_readBufferAvailable);
			}
		}
	}
	LeaveCriticalSection(&m_queueProtect);

	// Nope? Well we'll just add it then
	if (!queueUpdated) {
		if (!m_mfmRead[m_currentTrack][(int)m_floppySide].current.ready) {
			ResetEvent(m_readBufferAvailable);
		}
		queueCommand(QueueCommand::qcGotoToTrack, trackNumber);
	}
}

// Return the current track number we're on
unsigned char ArduinoFloppyDiskBridge::getCurrentCylinderNumber() {
	return m_currentTrack;
}

// Return the type of disk connected
FloppyDiskBridge::DriveTypeID ArduinoFloppyDiskBridge::getDriveTypeID() {
	return DriveTypeID::dti35DD;
}

// Return TRUE if the currently insrted disk is write protected
bool ArduinoFloppyDiskBridge::isWriteProtected() {
	return m_writeProtected;
}

// Reset and clear out any data we have received thus far
void ArduinoFloppyDiskBridge::resetWriteBuffer() {
	m_currentWriteTrack.writeFromIndex = false;
	m_currentWriteTrack.floppyBufferSizeBits = 0;
	m_currentWriteTrack.trackNumber = -1;
	m_currentWriteStartMfmPosition = 0;
}

// Submits a single WORD of data received during a DMA transfer to the disk buffer.  This needs to be saved.  It is usually flushed when commitWriteBuffer is called
// You should reset this buffer if side or track changes
void ArduinoFloppyDiskBridge::writeShortToBuffer(bool side, unsigned int track, unsigned short mfmData, int mfmPosition) {
	switchDiskSide(side);
	gotoCylinder(track, side);

	// Prevent background reading while we're busy
	m_delayStreaming = true;
	m_delayStreamingStart = GetTickCount();
	m_io->abortReadStreaming();


	// Check there is enough space left.  Bytes to bits, then 16 bits for the next block of data
	if (m_currentWriteTrack.floppyBufferSizeBits < (ARD_MFM_BUFFER_MAX_TRACK_LENGTH * 8) - 16) {
		if (m_currentWriteTrack.floppyBufferSizeBits == 0) {
			m_currentWriteTrack.trackNumber = track;
			m_currentWriteTrack.side = boolSideToDiskSurface(side);
			m_currentWriteStartMfmPosition = mfmPosition;
		}
		m_currentWriteTrack.mfmBuffer[m_currentWriteTrack.floppyBufferSizeBits >> 3] = mfmData >> 8;
		m_currentWriteTrack.mfmBuffer[(m_currentWriteTrack.floppyBufferSizeBits >> 3) + 1] = mfmData & 0xFF;
		m_currentWriteTrack.floppyBufferSizeBits += 16;
	}
}

// Requests that any data received via writeShortToBuffer be saved to disk. The side and track should match against what you have been collected
// and the buffer should be reset upon completion.  You should return the new tracklength (maxMFMBitPosition) with optional padding if needed
unsigned int ArduinoFloppyDiskBridge::commitWriteBuffer(bool side, unsigned int track) {
	switchDiskSide(side);
	gotoCylinder(track, side);

	// Prevent background reading while we're busy
	m_delayStreaming = true;
	m_delayStreamingStart = GetTickCount();
	m_io->abortReadStreaming();

	// If there was data?
	if ((m_currentWriteTrack.floppyBufferSizeBits) && (m_currentWriteTrack.trackNumber == track) && (m_currentWriteTrack.side == boolSideToDiskSurface(side))) {
		// Roughly accurate	
		m_currentWriteTrack.writeFromIndex = (m_currentWriteStartMfmPosition <= 10) || (m_currentWriteStartMfmPosition >= maxMFMBitPosition() - 10);

		EnterCriticalSection(&m_pendingWriteLock);
		m_pendingTrackWrites.push_back(m_currentWriteTrack);
		queueCommand(QueueCommand::writeMFMData);

		// Prevent old data being read back by WinUAE
		EnterCriticalSection(&m_switchBufferLock);
		MFMCaches* cache = &m_mfmRead[track][(int)m_floppySide];
		cache->current.ready = false;
		cache->next.amountReadInBits = 0;
		cache->next.ready = false;
		LeaveCriticalSection(&m_switchBufferLock);
		LeaveCriticalSection(&m_pendingWriteLock);
	}

	resetWriteBuffer();

	return maxMFMBitPosition();
}


#endif
