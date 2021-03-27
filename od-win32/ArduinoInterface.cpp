/* ArduinoFloppyReader (and writer)
*
* Copyright (C) 2017-2021 Robert Smith (@RobSmithDev)
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

////////////////////////////////////////////////////////////////////////////////////////
// Class to manage the communication between the computer and the Arduino    V2.5     //
////////////////////////////////////////////////////////////////////////////////////////
//
// Purpose:
// The class handles the command interface to the arduino.  It doesn't do any decoding
// Just open ports, switch motors on and off, seek to tracks etc.
//
//
//

#include "ArduinoInterface.h"
#include <sstream>
#include <vector>
#include <queue>



using namespace ArduinoFloppyReader;

// Command that the ARDUINO Sketch understands
#define COMMAND_VERSION            '?'
#define COMMAND_REWIND             '.'
#define COMMAND_GOTOTRACK          '#'
#define COMMAND_HEAD0              '['
#define COMMAND_HEAD1              ']'
#define COMMAND_READTRACK          '<'
#define COMMAND_ENABLE             '+'
#define COMMAND_DISABLE            '-'
#define COMMAND_WRITETRACK         '>'
#define COMMAND_ENABLEWRITE        '~'
#define COMMAND_DIAGNOSTICS        '&'
#define COMMAND_SWITCHTO_DD		   'D'   // Requires Firmware V1.6
#define COMMAND_SWITCHTO_HD		   'H'   // Requires Firmware V1.6
#define COMMAND_DETECT_DISK_TYPE   'M'	 // currently not implemented here

// New commands for more direct control of the drive.  Some of these are more efficient or dont turn the disk motor on for modded hardware
#define COMMAND_READTRACKSTREAM    '{'    // Requires Firmware V1.8
#define COMMAND_WRITETRACKPRECOMP  '}'    // Requires Firmware V1.8
#define COMMAND_CHECKDISKEXISTS    '^'    // Requires Firmware V1.8 (and modded hardware for fast version)
#define COMMAND_ISWRITEPROTECTED   '$'    // Requires Firmware V1.8
#define COMMAND_ENABLE_NOWAIT      '*'    // Requires Firmware V1.8
#define COMMAND_GOTOTRACK_REPORT   '='	  // Requires Firmware V1.8

#define SPECIAL_ABORT_CHAR  'x'

// When streaming, this amount of bit sequences is checked at the end of the track to properly work out where the overlap is as the INDEX pulse isnt accurate enough
// This is probably much higher than needs to be
#define OVERLAP_WINDOW_SIZE			32

#pragma pack(1) 
/* This is the format of the data received from the Arduino */
struct ArduinoPacket {
	bool isIndex;

	unsigned char readSpeed;
	unsigned char mfm;
};
#pragma pack()

// Convert the last executed command that had an error to a string
std::string lastCommandToName(LastCommand cmd) {
	switch (cmd) {
	case LastCommand::lcOpenPort:		return "OpenPort";
	case LastCommand::lcGetVersion:		return "GetVersion";
	case LastCommand::lcEnableWrite:	return "EnableWrite";
	case LastCommand::lcRewind:			return "Rewind";
	case LastCommand::lcDisableMotor:	return "DisableMotor";
	case LastCommand::lcEnableMotor:	return "EnableMotor";
	case LastCommand::lcGotoTrack:		return "GotoTrack";
	case LastCommand::lcSelectSurface:	return "SelectSurface";
	case LastCommand::lcReadTrack:		return "ReadTrack";
	case LastCommand::lcWriteTrack:		return "WriteTrack";
	case LastCommand::lcRunDiagnostics:	return "RunDiagnostics";
	case LastCommand::lcSwitchDiskMode: return "SetCapacity";
	case LastCommand::lcReadTrackStream: return "ReadTrackStream";
	case LastCommand::lcCheckDiskInDrive: return "CheckDiskInDrive";
	case LastCommand::lcCheckDiskWriteProtected: return "CheckDiskWriteProtected";

	default:							return "Unknown";
	}
}

// Uses the above fields to constructr a suitable error message, hopefully useful in diagnosing the issue
const std::string ArduinoInterface::getLastErrorStr() const {
	std::stringstream tmp;
	switch (m_lastError) {
	case DiagnosticResponse::drOldFirmware: return "The Arduino is running an older version of the firmware/sketch.  Please re-upload.";
	case DiagnosticResponse::drOK: return "Last command completed successfully.";
	case DiagnosticResponse::drPortInUse: return "The specified COM port is currently in use by another application.";
	case DiagnosticResponse::drPortNotFound: return "The specified COM port was not found.";
	case DiagnosticResponse::drAccessDenied: return "The operating system denied access to the specified COM port.";
	case DiagnosticResponse::drComportConfigError: return "We were unable to configure the COM port using the SetCommConfig() command.";
	case DiagnosticResponse::drBaudRateNotSupported: return "The COM port does not support the 2M baud rate required by this application.";
	case DiagnosticResponse::drErrorReadingVersion: return "An error occured attempting to read the version of the sketch running on the Arduino.";
	case DiagnosticResponse::drErrorMalformedVersion: return "The Arduino returned an unexpected string when version was requested.  This could be a baud rate mismatch or incorrect loaded sketch.";
	case DiagnosticResponse::drCTSFailure: return "Diagnostics report the CTS line is not connected correctly or is not behaving correctly.";
	case DiagnosticResponse::drTrackRangeError: return "An error occured attempting to go to a track number that was out of allowed range.";
	case DiagnosticResponse::drWriteProtected: return "Unable to write to the disk.  The disk is write protected.";
	case DiagnosticResponse::drPortError: return "An unknown error occured attempting to open access to the specified COM port.";
	case DiagnosticResponse::drDiagnosticNotAvailable: return "CTS diagnostic not available, command GetCommModemStatus failed to execute.";
	case DiagnosticResponse::drSelectTrackError: return "Arduino reported an error seeking to a specific track.";
	case DiagnosticResponse::drTrackWriteResponseError: return "Error receiving status from Arduino after a track write operation.";
	case DiagnosticResponse::drSendDataFailed: return "Error sending track data to be written to disk.  This could be a COM timeout.";
	case DiagnosticResponse::drRewindFailure: return "Arduino was unable to find track 0.  This could be a wiring fault or power supply failure.";
	case DiagnosticResponse::drNoDiskInDrive: return "No disk in drive";
	case DiagnosticResponse::drWriteTimeout: return "The Arduino could not receive the data quick enough to write to disk. Try connecting via USB2 and not using a USB hub.\n\nIf this still does not work, turn off precomp if you are using it.";
	case DiagnosticResponse::drFramingError: return "The Arduino received bad data from the PC. This could indicate poor connectivity, bad baud rate matching or damaged cables.";
	case DiagnosticResponse::drSerialOverrun: return "The Arduino received data faster than it could handle. This could either be a fault with the CTS connection or the USB/serial interface is faulty";
	case DiagnosticResponse::drError:	tmp << "Arduino responded with an error running the " << lastCommandToName(m_lastCommand) << " command.";
		return tmp.str();

	case DiagnosticResponse::drReadResponseFailed:
		switch (m_lastCommand) {
		case LastCommand::lcGotoTrack: return "Unable to read response from Arduino after requesting to go to a specific track";
		case LastCommand::lcReadTrack: return "Gave up trying to read a full track from the disk.";
		case LastCommand::lcWriteTrack: return "Unable to read response to requesting to write a track.";
		default: tmp << "Error reading response from the Arduino while running command " << lastCommandToName(m_lastCommand) << ".";
			return tmp.str();
		}

	case DiagnosticResponse::drSendFailed:
		if (m_lastCommand == LastCommand::lcGotoTrack)
			return "Unable to send the complete select track command to the Arduino.";
		else {
			tmp << "Error sending the command " << lastCommandToName(m_lastCommand) << " to the Arduino.";
			return tmp.str();
		}

	case DiagnosticResponse::drSendParameterFailed:	tmp << "Unable to send a parameter while executing the " << lastCommandToName(m_lastCommand) << " command.";
		return tmp.str();
	case DiagnosticResponse::drStatusError: tmp << "An unknown response was was received from the Arduino while executing the " << lastCommandToName(m_lastCommand) << " command.";
		return tmp.str();

	default: return "Unknown error.";
	}
}

// Constructor for this class
ArduinoInterface::ArduinoInterface() {
	m_abortStreaming = true;
	m_comPort = INVALID_HANDLE_VALUE;
	m_version = { 0,0,false };
	m_lastError = DiagnosticResponse::drOK;
	m_lastCommand = LastCommand::lcGetVersion;
	m_inWriteMode = false;
	m_isWriteProtected = false;
	m_diskInDrive = false;
	m_abortSignalled = false;
	m_isStreaming = false;
}

// Free me
ArduinoInterface::~ArduinoInterface() {
	abortReadStreaming();
	closePort();
}

// Checks if the disk is write protected.  If forceCheck=false then the last cached version is returned.  This is also updated by checkForDisk() as well as this function
DiagnosticResponse ArduinoInterface::checkIfDiskIsWriteProtected(bool forceCheck) {
	if (!forceCheck) {
		return m_isWriteProtected ? DiagnosticResponse::drWriteProtected : DiagnosticResponse::drOK;
	}

	// Test manually
	m_lastCommand = LastCommand::lcCheckDiskWriteProtected;

	// If no hardware support then return no change
	if ((m_version.major == 1) && (m_version.minor < 8)) {
		m_lastError = DiagnosticResponse::drOldFirmware;
		return m_lastError;
	}

	// Send and update flag
	m_lastError = checkForDisk(true);
	if ((m_lastError == DiagnosticResponse::drStatusError) || (m_lastError == DiagnosticResponse::drOK)) {
		m_lastCommand = LastCommand::lcCheckDiskWriteProtected;

		if (m_isWriteProtected)  m_lastError = DiagnosticResponse::drWriteProtected;
	}

	return m_lastError;
}

// This only works normally after the motor has been stepped in one direction or another.  This requires the 'advanced' configuration
DiagnosticResponse ArduinoInterface::checkForDisk(bool forceCheck) {
	if (!forceCheck) {
		return m_diskInDrive ? DiagnosticResponse::drOK : DiagnosticResponse::drNoDiskInDrive;
	}

	// Test manually
	m_lastCommand = LastCommand::lcCheckDiskInDrive;

	// If no hardware support then return no change
	if ((m_version.major == 1) && (m_version.minor < 8)) {
		m_lastError = DiagnosticResponse::drOldFirmware;
		return m_lastError;
	}

	char response;
	m_lastError = runCommand(COMMAND_CHECKDISKEXISTS, '\0', &response);
	if ((m_lastError == DiagnosticResponse::drStatusError) || (m_lastError == DiagnosticResponse::drOK)) {
		if (response == '#') m_lastError = DiagnosticResponse::drNoDiskInDrive;
		m_diskInDrive = m_lastError != DiagnosticResponse::drNoDiskInDrive;

		// Also read the write protect status
		if (!deviceRead(&response, 1, true)) {
			m_lastError = DiagnosticResponse::drReadResponseFailed;

			return m_lastError;
		}

		m_isWriteProtected = response == '1';
	}

	return m_lastError;
}

// Check if an index pulse can be detected from the drive
DiagnosticResponse ArduinoInterface::testIndexPulse() {
	// Port opned.  We need to check what happens as the pin is toggled
	m_lastError = runCommand(COMMAND_DIAGNOSTICS, '3');
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcRunDiagnostics;
		return m_lastError;
	}
	return m_lastError;
}

// Check if data can be detected from the drive
DiagnosticResponse ArduinoInterface::testDataPulse() {
	// Port opned.  We need to check what happens as the pin is toggled
	m_lastError = runCommand(COMMAND_DIAGNOSTICS, '4');
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcRunDiagnostics;
		return m_lastError;
	}
	return m_lastError;
}

// Check CTS status by asking the device to set it and then checking what happened
DiagnosticResponse ArduinoInterface::testCTS(const unsigned int portNumber) {
	m_lastError = openPort(portNumber, false);
	if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

	for (int a = 1; a <= 10; a++) {
		// Port opned.  We need to check what happens as the pin is toggled
		m_lastError = runCommand(COMMAND_DIAGNOSTICS, (a & 1) ? '1' : '2');
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcRunDiagnostics;
			closePort();
			return m_lastError;
		}
		Sleep(1);
		// Check the state of it
		DWORD mask;

		if (!GetCommModemStatus(m_comPort, &mask)) {
			closePort();
			m_lastError = DiagnosticResponse::drDiagnosticNotAvailable;
			return m_lastError;
		}

		// This doesnt actually run a command, this switches the CTS line back to its default setting
		m_lastError = runCommand(COMMAND_DIAGNOSTICS);

		if (((mask & MS_CTS_ON) != 0) ^ ((a & 1) != 0)) {
			// If we get here then the CTS value isn't what it should be
			closePort();
			m_lastError = DiagnosticResponse::drCTSFailure;
			return m_lastError;
		}
		// Pass.  Try the other state
		Sleep(1);
	}

	closePort();

	return DiagnosticResponse::drOK;
}

// Attempts to open the reader running on the COM port number provided.  Port MUST support 2M baud
DiagnosticResponse ArduinoInterface::openPort(const unsigned int portNumber, bool enableCTSflowcontrol) {
	m_lastCommand = LastCommand::lcOpenPort;
	closePort();

	// Communicate with the serial port
	char buffer[20];
	sprintf_s(buffer, "\\\\.\\COM%i", portNumber);
	m_comPort = CreateFileA(buffer, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);

	// No com port? Error!
	if (m_comPort == INVALID_HANDLE_VALUE) {
		int i = GetLastError();
		switch (i) {
		case ERROR_FILE_NOT_FOUND:  m_lastError = DiagnosticResponse::drPortNotFound;
			return m_lastError;
		case ERROR_ACCESS_DENIED:   m_lastError = DiagnosticResponse::drPortInUse;
			return m_lastError;
		default: m_lastError = DiagnosticResponse::drPortError;
			return m_lastError;
		}
	}

	// Prepare communication settings
	COMMCONFIG config;
	DWORD comConfigSize = sizeof(config);
	memset(&config, 0, sizeof(config));

	GetCommConfig(m_comPort, &config, &comConfigSize);
	config.dwSize = sizeof(config);
	config.dcb.DCBlength = sizeof(config.dcb);
	config.dcb.BaudRate = 2000000;  // 2M baudrate
	config.dcb.ByteSize = 8;        // 8-bit
	config.dcb.fBinary = true;
	config.dcb.Parity = false;
	config.dcb.fOutxCtsFlow = enableCTSflowcontrol;  // Turn on CTS flow control
	config.dcb.fOutxDsrFlow = false;
	config.dcb.fDtrControl = DTR_CONTROL_DISABLE;
	config.dcb.fDsrSensitivity = false;
	config.dcb.fNull = false;
	config.dcb.fTXContinueOnXoff = false;
	config.dcb.fRtsControl = RTS_CONTROL_DISABLE;
	config.dcb.fAbortOnError = false;
	config.dcb.StopBits = 0;  // 1 stop bit
	config.dcb.fOutX = 0;
	config.dcb.fInX = 0;
	config.dcb.fErrorChar = 0;
	config.dcb.fAbortOnError = 0;
	config.dcb.fInX = 0;

	m_abortSignalled = false;

	// Try to put the serial port in the mode we require
	if (!SetCommConfig(m_comPort, &config, sizeof(config))) {
		// If it failed something went wrong.  We'll change the baud rate to see if its that
		config.dcb.BaudRate = 9600;
		if (!SetCommConfig(m_comPort, &config, sizeof(config))) {
			closePort();
			m_lastError = DiagnosticResponse::drComportConfigError;
			return m_lastError;
		}
		else {
			closePort();
			m_lastError = DiagnosticResponse::drBaudRateNotSupported;
			return m_lastError;
		}
	}

	// Delay, and then disable DTR.  This will restart most Arduinos.  A good way to get the device into a known state, but we shouldnt ly
	EscapeCommFunction(m_comPort, SETDTR);
	Sleep(150);
	EscapeCommFunction(m_comPort, CLRDTR);
	Sleep(150);
	// Possibly a bit overkill
	SetupComm(m_comPort, RAW_TRACKDATA_LENGTH * 2, RAW_TRACKDATA_LENGTH);

	applyCommTimeouts(false);

	unsigned char blank;
	DWORD read;

	// This is to clear streaming mode if it gets stuck
	blank = SPECIAL_ABORT_CHAR;
	WriteFile(m_comPort, &blank, 1, &read, NULL);

	// Quickly force streaming to be aborted
	m_abortStreaming = true;

	// And wait to make sure all data has gone
	while (ReadFile(m_comPort, &blank, 1, &read, NULL))
		if (read < 1) break;


	// Request version from the Arduino device running our software
	m_lastError = runCommand(COMMAND_VERSION);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcGetVersion;
		closePort();
		return m_lastError;
	}

	// Version is always 4 bytes, we're gonna read them in one by one, and if any are wrong we exit with an error
	char versionBuffer[4];
	if (!deviceRead(versionBuffer, 4, true)) {
		closePort();
		m_lastError = DiagnosticResponse::drErrorReadingVersion;
		return m_lastError;
	}

	// Detect advanced controls
	m_version.fullControlMod = versionBuffer[2] == ',';
	if (m_version.fullControlMod) versionBuffer[2] = '.';

	// Now check the response
	if ((versionBuffer[0] != 'V') || (versionBuffer[2] != '.')) {
		closePort();
		m_lastError = DiagnosticResponse::drErrorMalformedVersion;
		return m_lastError;
	}

	// Looks like its formatted correctly.  There's a good chance this is our device
	m_version.major = versionBuffer[1] - '0';
	m_version.minor = versionBuffer[3] - '0';

	if (((m_version.major == 1) && (m_version.minor < 2)) || (m_version.major == 0)) {
		// Ok, success
		m_lastError = DiagnosticResponse::drOldFirmware;
		return m_lastError;
	}

	// Ok, success
	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}

// Apply and change the timeouts on the com port
void ArduinoInterface::applyCommTimeouts(bool shortTimeouts) {
	// Setup port timeouts
	COMMTIMEOUTS timeouts;
	timeouts.WriteTotalTimeoutConstant = 2000;
	timeouts.WriteTotalTimeoutMultiplier = 200;

	if (shortTimeouts) {
		timeouts.ReadIntervalTimeout = 10;
		timeouts.ReadTotalTimeoutConstant = 5;
		timeouts.ReadTotalTimeoutMultiplier = 2;
	}
	else {
		timeouts.ReadIntervalTimeout = 2000;
		timeouts.ReadTotalTimeoutConstant = 2000;
		timeouts.ReadTotalTimeoutMultiplier = 200;
	}
	SetCommTimeouts(m_comPort, &timeouts);
}

// Closes the port down
void ArduinoInterface::closePort() {
	if (m_comPort != INVALID_HANDLE_VALUE) {
		// Force the drive to power down
		enableReading(false);
		// Reset the Arduino, incase it gets left on
		EscapeCommFunction(m_comPort, SETDTR);
		Sleep(10);
		EscapeCommFunction(m_comPort, CLRDTR);
		// And close the handle
		CloseHandle(m_comPort);
		m_comPort = INVALID_HANDLE_VALUE;
	}
	m_inWriteMode = false;
	m_isWriteProtected = false;
	m_diskInDrive = false;
}

// Returns true if the track actually contains some data, else its considered blank or unformatted
bool ArduinoInterface::trackContainsData(const RawTrackData& trackData) const {
	int zerocount = 0, ffcount = 0;
	unsigned char lastByte = trackData[0];
	for (unsigned int counter = 1; counter < RAW_TRACKDATA_LENGTH; counter++) {
		if (trackData[counter] == lastByte) {
			switch (lastByte) {
			case 0xFF: ffcount++; zerocount = 0; break;
			case 0x00: ffcount = 0; zerocount++; break;
			default: zerocount = 0; ffcount = 0;
			}
		}
		else {
			lastByte = trackData[counter];
			zerocount = 0; ffcount = 0;
		}
	}

	// More than this in a row and its bad
	return ((ffcount < 20) && (zerocount < 20));
}

// Turns on and off the writing interface.  If irError is returned the disk is write protected
DiagnosticResponse ArduinoInterface::enableWriting(const bool enable, const bool reset) {
	if (enable) {
		// Enable the device
		m_lastError = runCommand(COMMAND_ENABLEWRITE);
		if (m_lastError == DiagnosticResponse::drError) {
			m_lastCommand = LastCommand::lcEnableWrite;
			m_lastError = DiagnosticResponse::drWriteProtected;
			return m_lastError;
		}
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcEnableWrite;
			return m_lastError;
		}
		m_inWriteMode = true;

		// Reset?
		if (reset) {
			// And rewind to the first track
			m_lastError = findTrack0();
			if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

			// Lets know where we are
			return selectSurface(DiskSurface::dsUpper);
		}
		m_lastError = DiagnosticResponse::drOK;
		return m_lastError;
	}
	else {
		// Disable the device
		m_lastError = runCommand(COMMAND_DISABLE);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcDisableMotor;
			return m_lastError;
		}
		m_inWriteMode = false;

		return m_lastError;
	}
}

DiagnosticResponse ArduinoInterface::findTrack0() {
	// And rewind to the first track
	char status = '0';
	m_lastError = runCommand(COMMAND_REWIND, '\000', &status);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcRewind;
		if (status == '#') return DiagnosticResponse::drRewindFailure;
		return m_lastError;
	}
	return m_lastError;
}

// Turns on and off the reading interface
DiagnosticResponse ArduinoInterface::enableReading(const bool enable, const bool reset, const bool dontWait) {
	m_inWriteMode = false;
	if (enable) {
		// Enable the device
		m_lastError = runCommand(dontWait ? COMMAND_ENABLE_NOWAIT : COMMAND_ENABLE);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcEnableMotor;
			return m_lastError;
		}

		// Reset?
		if (reset) {
			m_lastError = findTrack0();
			if (m_lastError != DiagnosticResponse::drOK) return m_lastError;

			// Lets know where we are
			return selectSurface(DiskSurface::dsUpper);
		}
		m_lastError = DiagnosticResponse::drOK;
		m_inWriteMode = m_version.fullControlMod;
		return m_lastError;

	}
	else {
		// Disable the device
		m_lastError = runCommand(COMMAND_DISABLE);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcDisableMotor;
			return m_lastError;
		}

		return m_lastError;
	}
}

// Check and switch to HD disk
DiagnosticResponse ArduinoInterface::setDiskCapacity(bool switchToHD_Disk) {
	// Disable the device
	m_lastError = runCommand(switchToHD_Disk ? COMMAND_SWITCHTO_HD : COMMAND_SWITCHTO_DD);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcSwitchDiskMode;
		return m_lastError;
	}

	return m_lastError;
}

// Select the track, this makes the motor seek to this position
DiagnosticResponse ArduinoInterface::selectTrack(const unsigned char trackIndex, const TrackSearchSpeed searchSpeed, bool ignoreDiskInsertCheck) {
	if (trackIndex > 81) {
		m_lastError = DiagnosticResponse::drTrackRangeError;
		return m_lastError; // no chance, it can't be done.
	}

	// And send the command and track.  This is sent as ASCII text as a result of terminal testing.  Easier to see whats going on
	bool isV18 = (m_version.major > 1) || ((m_version.major == 1) && (m_version.minor >= 8));
	char buf[8];
	if (isV18) {
		char flags = 0;
		switch (searchSpeed) {
		case TrackSearchSpeed::tssNormal:   flags = 1; break;                    // Speed - 1
		case TrackSearchSpeed::tssFast:     flags = 2; break;                    // Speed - 2
		case TrackSearchSpeed::tssVeryFast: flags = 3; break;					// Speed - 3
		}
		if (!ignoreDiskInsertCheck) flags |= 4;
		sprintf_s(buf, "%c%02i%c", COMMAND_GOTOTRACK_REPORT, trackIndex, flags);
	}
	else {
		sprintf_s(buf, "%c%02i", COMMAND_GOTOTRACK, trackIndex);
	}

	// Send track number. 
	if (!deviceWrite(buf, strlen(buf))) {
		m_lastCommand = LastCommand::lcGotoTrack;
		m_lastError = DiagnosticResponse::drSendFailed;
		return m_lastError;
	}

	// Get result
	char result;

	if (!deviceRead(&result, 1, true)) {
		m_lastCommand = LastCommand::lcGotoTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	switch (result) {
	case '2':  m_lastError = DiagnosticResponse::drOK; break; // already at track.  No op needed.  V1.8 only
	case '1':   m_lastError = DiagnosticResponse::drOK;
		if (isV18) {
			// Read extended data
			char status;
			if (!deviceRead(&status, 1, true)) {
				m_lastError = DiagnosticResponse::drReadResponseFailed;
				return m_lastError;
			}
			// 'x' means we didnt check it
			if (status != 'x') m_diskInDrive = status == '1';

			// Also read the write protect status
			if (!deviceRead(&status, 1, true)) {
				m_lastError = DiagnosticResponse::drReadResponseFailed;
				return m_lastError;
			}
			m_isWriteProtected = status == '1';
		}
		break;
	case '0':   m_lastCommand = LastCommand::lcGotoTrack;
		m_lastError = DiagnosticResponse::drSelectTrackError;
		break;
	default:	m_lastCommand = LastCommand::lcGotoTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		break;
	}

	return m_lastError;
}

// Choose which surface of the disk to read from
DiagnosticResponse ArduinoInterface::selectSurface(const DiskSurface side) {
	m_lastError = runCommand((side == DiskSurface::dsUpper) ? COMMAND_HEAD0 : COMMAND_HEAD1);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcSelectSurface;
		return m_lastError;
	}
	return m_lastError;
}

void writeBit(RawTrackData& output, int& pos, int& bit, int value) {
	if (pos >= RAW_TRACKDATA_LENGTH) return;

	output[pos] <<= 1;
	output[pos] |= value;
	bit++;
	if (bit >= 8) {
		pos++;
		bit = 0;
	}
}

void unpack(const RawTrackData& data, RawTrackData& output) {
	int pos = 0;
	int index = 0;
	int p2 = 0;
	memset(output, 0, sizeof(output));
	while (pos < RAW_TRACKDATA_LENGTH) {
		// Each byte contains four pairs of bits that identify an MFM sequence to be encoded

		for (int b = 6; b >= 0; b -= 2) {
			unsigned char value = (data[index] >> b) & 3;
			switch (value) {
			case 0:
				// This can't happen, its invalid data but we account for 4 '0' bits
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				break;
			case 1: // This is an '01'
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 1);
				break;
			case 2: // This is an '001'
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 1);
				break;
			case 3: // this is an '0001'
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 0);
				writeBit(output, pos, p2, 1);
				break;
			}
		}
		index++;
		if (index >= sizeof(data)) return;
	}
	// There will be left-over data
}

// Read RAW data from the current track and surface 
DiagnosticResponse ArduinoInterface::readCurrentTrack(RawTrackData& trackData, const bool readFromIndexPulse) {
	m_lastError = runCommand(COMMAND_READTRACK);

	RawTrackData tmp;

	// Allow command retry
	if (m_lastError != DiagnosticResponse::drOK) {
		// Clear the buffer
		deviceRead(&tmp, RAW_TRACKDATA_LENGTH);
		m_lastError = runCommand(COMMAND_READTRACK);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcReadTrack;
			return m_lastError;
		}
	}

	unsigned char signalPulse = readFromIndexPulse ? 1 : 0;
	if (!deviceWrite(&signalPulse, 1)) {
		m_lastCommand = LastCommand::lcReadTrack;
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	// Keep reading until he hit RAW_TRACKDATA_LENGTH or a null byte is recieved
	int bytePos = 0;
	int readFail = 0;
	for (;;) {
		unsigned char value;
		if (deviceRead(&value, 1, true)) {
			if (value == 0) break; else
				if (bytePos < RAW_TRACKDATA_LENGTH) tmp[bytePos++] = value;
		}
		else {
			readFail++;
			if (readFail > 4) {
				m_lastCommand = LastCommand::lcReadTrack;
				m_lastError = DiagnosticResponse::drReadResponseFailed;
				return m_lastError;
			}
		}
	}
	unpack(tmp, trackData);
	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}



// In Debug at least, std::queue was too slow, might be memory allocation overhead, not sure.  So a simple queue implementation was used below
#define QUEUE_SIZE (OVERLAP_WINDOW_SIZE*4)

// At leats in debug, std::queue was too slow
template <class queueType>
class FastQueue {
private:
	queueType buffer[QUEUE_SIZE];
	unsigned int readHead = 0;
	unsigned int writeHead = 0;
	unsigned int bytes = 0;
public:
	inline queueType front() const { return buffer[readHead]; };
	inline queueType next() {
		queueType tmp = front();
		pop();
		return tmp;
	}
	inline void pop() {
		if (bytes) {
			readHead = (readHead + 1) % QUEUE_SIZE;
			bytes--;
		}
		else {
#ifdef _DEBUG
			OutputDebugStringA("READ QUEUE OVERFLOW\n");
#endif
		}
	};
	inline unsigned int size() const { return bytes; };
	inline void push(queueType c) {
		if (bytes < QUEUE_SIZE) {
			buffer[writeHead] = c;
			writeHead = (writeHead + 1) % QUEUE_SIZE;
			bytes++;
		}
		else {
#ifdef _DEBUG
			OutputDebugStringA("WRITE QUEUE OVERFLOW\n");
#endif
		}
	}
};


// Locates the part of this entire buffer that matches startSequence the best.  Returns how many sequences from currentBitSequences + futureBitSequences are needed to complete the revolution
static int findSlidingWindow(const std::vector<unsigned char>& searchSequence, const FastQueue<ArduinoPacket>& futureBitSequences, const FastQueue<ArduinoPacket>& currentBitSequences) {
	if (futureBitSequences.size() < OVERLAP_WINDOW_SIZE) return 0;
	if (currentBitSequences.size() < OVERLAP_WINDOW_SIZE) return 0;
	if (searchSequence.size() < OVERLAP_WINDOW_SIZE) return 0;

	// Make a vector with all the data from the queue
	std::vector<unsigned char> searchArea;
	FastQueue<ArduinoPacket> copy = currentBitSequences;
	while (copy.size()) {
		searchArea.push_back(copy.next().mfm);
	}

	copy = futureBitSequences;
	while (copy.size()) {
		searchArea.push_back(copy.next().mfm);
	}

	int bestIndex = currentBitSequences.size() - 1;
	int bestScore = 0;
	int midPoint = (searchArea.size() - searchSequence.size()) / 2;

	// Now find the pattern.  We fan out from where the index was actually detected to nsure it has the highest chance of being right
	for (int a = 0; a <= midPoint; a++) {
		for (int direction = -1; direction <= 1; direction += 2) {
			int startIndex = midPoint + (direction * a);
			int score = 0;

			for (unsigned int pos = 0; pos < searchSequence.size(); pos++)
				if ((startIndex + pos >= 0) && (startIndex + pos < searchArea.size())) // This line shouldnt be requied but best to be safe
					if (searchSequence[pos] == searchArea[startIndex + pos]) score++;

			if (score > bestScore) {
				bestIndex = startIndex;
				bestScore = score;

				// Stop straight away if we get to the maximum possible score
				if (score == searchSequence.size()) {
					a = midPoint + 1;
					break;
				}
			}
		}
	}
	return bestIndex;
}

// Streaming version
inline void writeStreamBit(MFMSample* output, unsigned int& pos, unsigned int& bit, unsigned int value, unsigned int valuespeed, unsigned int maxLength) {
	if (pos >= maxLength) return;

	output[pos].mfmData <<= 1;
	output[pos].mfmData |= value;

	// If the data read quicker, then valuespeed will be smaller.  
	// If it read slower, then valuespeed will be larger
	//smaller=more bits

	// ~3-4 is the middle, or normal
	output[pos].speed[7 - bit] = valuespeed;

	bit++;
	if (bit >= 8) {
		pos++;
		bit = 0;
	}
}

#define BITCELL_SIZE_IN_NS 2000L


// Handle adding these bits to the output buffer
static void outputBitSequence(const ArduinoPacket& value, MFMSample* buffer, unsigned int& pos, unsigned int& bit, const int maxBufferSize) {
	// Convert into a more useful format
	int sequence = (value.mfm == 0) ? 2 : value.mfm - 1;

	// Calculate what the 'ticks' would be
	int ticksInNS = 3000 + (sequence * 2000) + (((long long)(64 + ((int)value.readSpeed) * 2000) / 128));
	int speed = (((long long)ticksInNS * 100L) / (((long long)(sequence + 2)) * BITCELL_SIZE_IN_NS));

	for (int a = 0; a <= sequence; a++)
		writeStreamBit(buffer, pos, bit, 0, speed, maxBufferSize);
	writeStreamBit(buffer, pos, bit, (sequence == 3) ? 0 : 1, speed, maxBufferSize);
}

// Handle sending and updating the buffer.  Returns FALSE if we should abort
static bool flushAndPush(MFMSample* buffer, unsigned int& pos, unsigned int& bit, const unsigned int maxBlockSize, const unsigned int maxBufferSize, std::function<bool(const MFMSample* mfmData, const unsigned dataLength, const bool isEndOfRevolution)> dataStream, bool flushALL = false) {
	int flushSize = flushALL ? 0 : maxBlockSize;

	// Do we have enough data? (we hold some back as part of the end of track "slide")
	if ((int)pos > flushSize) {
		// Do the callback
		MFMSample* bufStart = buffer;
		int startBytes = pos;
		// And output
		while ((int)pos > flushSize) {
			int amountToSend = maxBlockSize;
			if ((int)pos < amountToSend) amountToSend = pos;

			if (!dataStream(bufStart, amountToSend * 8, false)) return false;
			pos -= amountToSend;
			bufStart += amountToSend;
		}

		// Now move everything back to the start so data can remain intact, but only if really needed
		if ((bit > 0) || (pos > 0)) {
			// Move memory allowed memory to overlap
			MoveMemory(buffer, bufStart, ((maxBufferSize * sizeof(MFMSample)) - (startBytes * sizeof(MFMSample)) + sizeof(MFMSample)));
			// Pos should now be correct anyway
		}
		else {
			pos = 0;
		}
	}

	if (flushALL) {
		if (pos > 0) {
			// Something has gone wrong if we get here
		}
		if (bit) {
			// Theres still some bits remaining.  These need to be at the MSB end though, so we'll shift them
			int shiftAmount = 8 - bit;
			buffer->mfmData <<= shiftAmount;
			//buffer->speed += (shiftAmount * 100);
			if (!dataStream(buffer, bit, true)) return false;
			bit = 0;
		}
		else {
			// No data left.  We'll snd an 'end of revolution' notification
			if (!dataStream(buffer, 0, true)) return false;
		}
	}

	return true;
}

// Read RAW data from the current track and surface selected, every time maxBlockSize bytes is received the function is called with data.
// IF you return FALSE to the function the stream will be stopped.  You can also abort the stream with the function below.
// Once the callback is called with isEndOfRevolution then this block completes an EXACT revolution of data from the disk!
// dataLengthInBits will always be aligned to a byte boundary unless isEndOfRevolution is true
// startBitPatterns is somethign to key off for future revolutions of the same disk, its automatically managed, you just need to either supply it or leave it blank (for a new disk)
// maxBlockSize is in bytes
DiagnosticResponse ArduinoInterface::readCurrentTrackStream(const unsigned int maxBlockSize, const unsigned int maxRevolutions, std::vector<unsigned char>& startBitPatterns, std::function<bool(const MFMSample* mfmData, const unsigned dataLengthInBits, const bool isEndOfRevolution)> dataStream) {
	if ((m_version.major == 1) && (m_version.minor < 8)) {
		LastCommand::lcReadTrackStream;
		m_lastError = DiagnosticResponse::drOldFirmware;
		return m_lastError;
	}

	// Who would do this, right?
	if (maxBlockSize < 1) {
		m_lastError = DiagnosticResponse::drError;
		m_lastCommand = LastCommand::lcReadTrackStream;
		return m_lastError;
	}

	m_lastError = runCommand(COMMAND_READTRACKSTREAM);

	// Allow command retry
	if (m_lastError != DiagnosticResponse::drOK) {
		// Clear the buffer
		RawTrackData tmp;
		deviceRead(&tmp, RAW_TRACKDATA_LENGTH);
		m_lastError = runCommand(COMMAND_READTRACKSTREAM);
		if (m_lastError != DiagnosticResponse::drOK) {
			m_lastCommand = LastCommand::lcReadTrackStream;
			return m_lastError;
		}
	}

	m_isStreaming = true;

	// It's just gonna keep coming!
	int bytePos = 0;
	int readFail = 0;

	// The +5 is for unpacking to still have space
	const unsigned int maxBufferSize = maxBlockSize + (OVERLAP_WINDOW_SIZE * 2) + 10;   // make sure we have enough room
	MFMSample* buffer = (MFMSample*)malloc(maxBufferSize * sizeof(MFMSample));

	// Probably a bit overkill for something that is unlikely to happen anyway
	if (buffer == nullptr) {
		abortReadStreaming();
		unsigned char byteRead;
		// Ensure the buffer is clear
		do {
			if (!deviceRead(&byteRead, 1, true)) byteRead = 0;
		} while (byteRead != 0);

		m_lastCommand = LastCommand::lcReadTrackStream;
		m_lastError = DiagnosticResponse::drSendParameterFailed;

		m_isStreaming = false;
		return m_lastError;
	}

	unsigned int pos = 0;
	unsigned int index = 0;
	unsigned int bit = 0;

	m_abortStreaming = false;
	m_abortSignalled = false;
	int abortSequence = 0;

	bool startIndexFound = false;
	int received = 0;

	FastQueue<ArduinoPacket> futureBitSequences;
	FastQueue<ArduinoPacket> currentBitSequences;
	FastQueue<ArduinoPacket> oldBitSequences;

	// Clear if incomplete
	if (startBitPatterns.size() < OVERLAP_WINDOW_SIZE)
		startBitPatterns.clear();
	bool oldSequenceEnabled = startBitPatterns.size() >= OVERLAP_WINDOW_SIZE;

	applyCommTimeouts(true);

	int skipIndex = 0;

	int loops = 0;

	DWORD bytesRead;
	unsigned char tempReadBuffer[64];


	for (;;) {

		// More efficient to read several bytes in one go		
		if (!ReadFile(m_comPort, tempReadBuffer, m_abortSignalled ? 1 : sizeof(tempReadBuffer), &bytesRead, NULL)) bytesRead = 0;

		for (int a = 0; a < bytesRead; a++) {
			if (m_abortSignalled) {
				switch (tempReadBuffer[a]) {
				case 'X': if (abortSequence == 0) abortSequence++; else abortSequence = 0;  break;
				case 'Y': if (abortSequence == 1) abortSequence++; else abortSequence = 0; break;
				case 'Z': if (abortSequence == 2) abortSequence++; else abortSequence = 0; break;
				case SPECIAL_ABORT_CHAR: if (abortSequence == 3) abortSequence++; else abortSequence = 0; break;
				case '1': if (abortSequence == 4) {
					m_isStreaming = false;
					PurgeComm(m_comPort, PURGE_RXCLEAR | PURGE_TXCLEAR);
					m_lastCommand = LastCommand::lcReadTrackStream;
					m_lastError = DiagnosticResponse::drOK;
					free(buffer);
					applyCommTimeouts(false);
					return m_lastError;
				}
						else abortSequence = 0;
					break;
				default:
					abortSequence = 0;
					break;
				}
			}
			else {

				unsigned char byteRead = tempReadBuffer[a];

				ArduinoPacket pkt;
				pkt.isIndex = (byteRead & 0x80) != 0;
				pkt.mfm = (byteRead >> 5) & 0x03;
				pkt.readSpeed = (byteRead & 0x07) * 16;
				futureBitSequences.push(pkt);

				pkt.isIndex = false;
				pkt.mfm = (byteRead >> 3) & 0x03;
				pkt.readSpeed = (byteRead & 0x07) * 16;
				futureBitSequences.push(pkt);


				// Once we have enough, start going through the data
				while ((futureBitSequences.size() > OVERLAP_WINDOW_SIZE * 2) && (!m_abortStreaming)) {
					ArduinoPacket nextData = futureBitSequences.next();

					// Now add it to the previous sequences, but only if an index has been detected
					if (startIndexFound) {
						currentBitSequences.push(nextData);

						while ((currentBitSequences.size() > OVERLAP_WINDOW_SIZE * 2) && (!m_abortStreaming)) {
							// Write whats left here into the output stream
							outputBitSequence(currentBitSequences.next(), buffer, pos, bit, maxBufferSize);

							// Handle sending and updating the buffer.  Returns FALSE if we should abort
							if (!flushAndPush(buffer, pos, bit, maxBlockSize, maxBufferSize, dataStream)) {
								abortReadStreaming();
							}
						}
					}
					else {
						// This happens if a sequence is provided, and we havent found the start yet.  There could be mis-alignment if we dont store some of the bits before INDEX detected
						if ((oldSequenceEnabled) && (startBitPatterns.size())) {
							oldBitSequences.push(nextData);
							while (oldBitSequences.size() > OVERLAP_WINDOW_SIZE * 2) oldBitSequences.pop();
						}
					}

					// Build up the start buffer
					if ((startIndexFound) && (startBitPatterns.size() < OVERLAP_WINDOW_SIZE)) {
						startBitPatterns.push_back(nextData.mfm);
					}

					// So, "value" and "isIndex" are now in the middle of all of this sliding data
					if (skipIndex) skipIndex--;
					if ((nextData.isIndex) && (skipIndex == 0)) {
						// Index found.
						if (!startIndexFound) {
							// So, we detected an index pulse, and we had a previously supplied sequence.  This index might have been slightly off, so we may need the bits from oldBitSequences to correct this.
							if ((oldSequenceEnabled) && (oldBitSequences.size())) {
								int i = findSlidingWindow(startBitPatterns, futureBitSequences, oldBitSequences);
								// We need to discard all sequences before "i" and add anything left from oldBitSequences into currentBitSequences
								while ((i > 0) && (oldBitSequences.size())) {
									oldBitSequences.pop();
									i--;
								}
								while ((i > 0) && (futureBitSequences.size())) {
									futureBitSequences.pop();
									i--;
								}
								currentBitSequences = oldBitSequences;
								startIndexFound = true;
							}
							else {
								// This is the first sequence for the index.  We'll record this.
								if (startBitPatterns.size() < OVERLAP_WINDOW_SIZE) startBitPatterns.push_back(nextData.mfm);
								startIndexFound = true;
								// And dont forget we want to process and output this data
								currentBitSequences.push(nextData);
							}
						}
						else {
							loops++;
							// This marks the END of the current cycle, and the start of the next.
								// At this point 'value' is at the back of currentBitSequences()
								// This shouldn't really be anymore than +/- about 3 bit cells from experience
							int i = findSlidingWindow(startBitPatterns, futureBitSequences, currentBitSequences);

							// "i" returns where to cut the buffer off at.  This is how many patterns need adding to complete the track.  Not sure why this varies EVEN WITH this code
							std::vector<ArduinoPacket> patternsToOutput;
							while ((i > 0) && (currentBitSequences.size())) {
								patternsToOutput.push_back(currentBitSequences.front());
								currentBitSequences.pop();
								i--;
							}
							while ((i > 0) && (futureBitSequences.size())) {
								patternsToOutput.push_back(futureBitSequences.front());
								futureBitSequences.pop();
								i--;
							}

							// Now patternsToOutput is whats left that needs to be flushed to finish the revolution.  Then the buffers should just re-fill themselves
							for (ArduinoPacket& value : patternsToOutput) {
								// Process this sequence
								outputBitSequence(value, buffer, pos, bit, maxBufferSize);

								// Handle sending and updating the buffer.  Returns FALSE if we should abort
								if (!flushAndPush(buffer, pos, bit, maxBlockSize, maxBufferSize, dataStream, false)) {
									abortReadStreaming();
								}
							}

							if (loops >= (int)maxRevolutions) {
								loops = 0;

								// Flush and close the buffer. Returns FALSE if we should abort
								if (!flushAndPush(buffer, pos, bit, maxBlockSize, maxBufferSize, dataStream, true)) {
									abortReadStreaming();

									// Populate the new sequence, this allows any incorrect bits to update to the new incorrect bits
									if (futureBitSequences.size() + currentBitSequences.size() >= OVERLAP_WINDOW_SIZE) {
										startBitPatterns.clear();

										while (futureBitSequences.size()) {
											currentBitSequences.push(futureBitSequences.next());
										}

										while (startBitPatterns.size() < OVERLAP_WINDOW_SIZE) {
											startBitPatterns.push_back(currentBitSequences.front().mfm);
											currentBitSequences.pop();
										}
									}
								}
								else {
									while (futureBitSequences.size()) {
										currentBitSequences.push(futureBitSequences.next());
									}
									std::swap(currentBitSequences, futureBitSequences);
									startBitPatterns.clear();
									skipIndex = futureBitSequences.size() + 1;
								}
							}
							else {
								while (futureBitSequences.size()) {
									currentBitSequences.push(futureBitSequences.next());
								}
								std::swap(currentBitSequences, futureBitSequences);
								startBitPatterns.clear();
								skipIndex = futureBitSequences.size() + 1;
							}
						}
					}
				}
			}
		}
		if (bytesRead < 1) {
			readFail++;
			if (readFail > 20) {
				m_abortStreaming = false; // force the 'abort' command to be sent
				abortReadStreaming();
				m_lastCommand = LastCommand::lcReadTrackStream;
				m_lastError = DiagnosticResponse::drReadResponseFailed;
				m_isStreaming = false;
				free(buffer);
				applyCommTimeouts(false);
				return m_lastError;
			}
			else Sleep(1);
		}
	}
}

// Stops the read streamming immediately and any data in the buffer will be discarded.
bool ArduinoInterface::abortReadStreaming() {
	if ((m_version.major == 1) && (m_version.minor < 8)) {
		return false;
	}

	if (!m_isStreaming) return true;

	if (!m_abortStreaming) {
		unsigned char command = SPECIAL_ABORT_CHAR;

		m_abortSignalled = true;
		//	PurgeComm(m_comPort, PURGE_RXCLEAR | PURGE_TXCLEAR);

			// Send the command
		if (!deviceWrite(&command, 1)) {
			return false;
		}

	}
	m_abortStreaming = true;
	return true;
}

// Read a bit from the data provided
inline int readBit(const unsigned char* buffer, const unsigned int maxLength, int& pos, int& bit) {
	if (pos >= (int)maxLength) {
		bit--;
		if (bit < 0) {
			bit = 7;
			pos++;
		}
		return (bit & 1) ? 0 : 1;
	}

	int ret = (buffer[pos] >> bit) & 1;
	bit--;
	if (bit < 0) {
		bit = 7;
		pos++;
	}

	return ret;
}

#define PRECOMP_NONE 0x00
#define PRECOMP_ERLY 0x04   
#define PRECOMP_LATE 0x08   

/* If we have a look at the previous and next three bits, assume the current is a '1', then these are the only valid combinations that coudl be allowed
	I have chosen the PRECOMP rule based on trying to get the current '1' as far away as possible from the nearest '1'

	   LAST		 CURRENT      NEXT			PRECOMP			Hex Value
	0	0	0    	1	   0	0	0		Normal
	0	0	0    	1	   0	0	1       Early			0x09
	0	0	0    	1	   0	1	0       Early			0x0A
	0	1	0    	1	   0	0	0       Late			0x28
	0	1	0    	1	   0	0	1       Late			0x29
	0	1	0    	1	   0	1	0       Normal
	1	0	0    	1	   0	0	0       Late			0x48
	1	0	0    	1	   0	0	1       Normal
	1	0	0    	1	   0	1	0       Early			0x4A
*/

// The precomp version of the above. Dont use the above function directly to write precomp mode, it wont work.  Data must be passed with an 0xAA each side at least
DiagnosticResponse ArduinoInterface::writeCurrentTrackPrecomp(const unsigned char* mfmData, const unsigned short numBytes, const bool writeFromIndexPulse, bool usePrecomp) {
	m_lastCommand = LastCommand::lcWriteTrack;
	if ((m_version.major == 1) && (m_version.minor < 8)) return DiagnosticResponse::drOldFirmware;

	// First step is we need to re-encode the supplied buffer into a packed format with precomp pre-calculated.
	// Each nybble looks like: xxyy
	// where xx is: 0=no precomp, 1=-ve, 2=+ve
	//		 yy is: 0: 4us,		1: 6us,		2: 8us,		3: 10us

	// *4 is a worse case situation, ie: if everything was a 01.  The +128 is for any extra padding
	const unsigned int maxOutSize = (numBytes * 4) + 16;
	unsigned char* outputBuffer = (unsigned char*)malloc(maxOutSize);

	if (!outputBuffer) {
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	// Original data was written from MSB downto LSB
	int pos = 0;
	int bit = 7;
	unsigned int outputPos = 0;
	unsigned char sequence = 0xAA;  // start at 10101010
	unsigned char* output = outputBuffer;
	int lastCount = 2;

	// Re-encode the data into our format and apply precomp.  The +8 is to ensure theres some padding around the edge which will come out as 010101 etc
	while (pos < numBytes + 8) {
		*output = 0;

		for (int i = 0; i < 2; i++) {
			int b, count = 0;

			// See how many zero bits there are before we hit a 1
			do {
				b = readBit(mfmData, numBytes, pos, bit);
				sequence = ((sequence << 1) & 0x7F) | b;
				count++;
			} while (((sequence & 0x08) == 0) && (pos < numBytes + 8));

			// Validate range
			if (count < 2) count = 2;  // <2 would be a 11 sequence, not allowed
			if (count > 5) count = 5;  // max we support 01, 001, 0001, 00001

			// Write to stream. Based on the rules above we apply some precomp
			int precomp = 0;
			if (usePrecomp) {
				switch (sequence) {
				case 0x09:
				case 0x0A:
				case 0x4A: // early;
					precomp = PRECOMP_ERLY;
					break;

				case 0x28:
				case 0x29:
				case 0x48: // late
					precomp = PRECOMP_LATE;
					break;

				default:
					precomp = PRECOMP_NONE;
					break;
				}
			}
			else precomp = PRECOMP_NONE;

			*output |= ((lastCount - 2) | precomp) << (i * 4);
			lastCount = count;
		}

		output++;
		outputPos++;
		if (outputPos >= maxOutSize) {
			// should never happen
			free(outputBuffer);
			m_lastError = DiagnosticResponse::drSendParameterFailed;
			return m_lastError;
		}
	}

	m_lastError = internalWriteTrack(outputBuffer, outputPos, writeFromIndexPulse, true);

	free(outputBuffer);

	return m_lastError;
}

// Writes RAW data onto the current track
DiagnosticResponse ArduinoInterface::writeCurrentTrack(const unsigned char* data, const unsigned short numBytes, const bool writeFromIndexPulse) {
	return internalWriteTrack(data, numBytes, writeFromIndexPulse, false);
}


// Writes RAW data onto the current track
DiagnosticResponse ArduinoInterface::internalWriteTrack(const unsigned char* data, const unsigned short numBytes, const bool writeFromIndexPulse, bool usePrecomp) {
	// Fall back if older firmware
	if ((m_version.major == 1) && (m_version.minor < 8) && usePrecomp) {
		return DiagnosticResponse::drOldFirmware;
	}
	m_lastError = runCommand(usePrecomp ? COMMAND_WRITETRACKPRECOMP : COMMAND_WRITETRACK);
	if (m_lastError != DiagnosticResponse::drOK) {
		m_lastCommand = LastCommand::lcWriteTrack;
		return m_lastError;
	}
	unsigned char chr;
	if (!deviceRead(&chr, 1, true)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	// 'N' means NO Writing, aka write protected
	if (chr == 'N') {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drWriteProtected;
		return m_lastError;
	}
	if (chr != 'Y') {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drStatusError;
		return m_lastError;
	}

	// Now we send the number of bytes we're planning on transmitting
	unsigned char b = (numBytes >> 8);
	if (!deviceWrite(&b, 1)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}
	b = numBytes & 0xFF;
	if (!deviceWrite(&b, 1)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	// Explain if we want index pulse sync writing (slower and not required by normal AmigaDOS disks)
	b = writeFromIndexPulse ? 1 : 0;
	if (!deviceWrite(&b, 1)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drSendParameterFailed;
		return m_lastError;
	}

	unsigned char response;
	if (!deviceRead(&response, 1, true)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	if (response != '!') {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drStatusError;

		return m_lastError;
	}

	if (!deviceWrite((const void*)data, numBytes)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drSendDataFailed;
		return m_lastError;
	}

	if (!deviceRead(&response, 1, true)) {
		m_lastCommand = LastCommand::lcWriteTrack;
		m_lastError = DiagnosticResponse::drTrackWriteResponseError;
		return m_lastError;
	}

	// If this is a '1' then the Arduino didn't miss a single bit!
	if (response != '1') {
		m_lastCommand = LastCommand::lcWriteTrack;
		switch (response) {
		case 'X':m_lastError = DiagnosticResponse::drWriteTimeout; break;
		case 'Y': m_lastError = DiagnosticResponse::drFramingError; break;
		case 'Z': m_lastError = DiagnosticResponse::drSerialOverrun; break;
		default:
			m_lastError = DiagnosticResponse::drStatusError;
			break;
		}
		return m_lastError;
	}

	m_lastError = DiagnosticResponse::drOK;
	return m_lastError;
}

// Run a command that returns 1 or 0 for its response
DiagnosticResponse ArduinoInterface::runCommand(const char command, const char parameter, char* actualResponse) {
	unsigned char response;

	// Send the command
	if (!deviceWrite(&command, 1)) {
		m_lastError = DiagnosticResponse::drSendFailed;
		return m_lastError;
	}

	// Only send the parameter if its not NULL
	if (parameter != '\0')
		if (!deviceWrite(&parameter, 1)) {
			m_lastError = DiagnosticResponse::drSendParameterFailed;
			return m_lastError;
		}

	// And read the response
	if (!deviceRead(&response, 1, true)) {
		m_lastError = DiagnosticResponse::drReadResponseFailed;
		return m_lastError;
	}

	if (actualResponse) *actualResponse = response;

	// Evaluate the response
	switch (response) {
	case '1': m_lastError = DiagnosticResponse::drOK;
		break;
	case '0': m_lastError = DiagnosticResponse::drError;
		break;
	default:  m_lastError = DiagnosticResponse::drStatusError;
		break;
	}
	return m_lastError;
}

// Read a desired number of bytes into the target pointer
bool ArduinoInterface::deviceRead(void* target, const unsigned int numBytes, const bool failIfNotAllRead) {
	DWORD read;
	if (m_comPort == INVALID_HANDLE_VALUE) return false;

	if (!ReadFile(m_comPort, target, numBytes, &read, NULL)) return false;

	if (read < numBytes) {
		if (failIfNotAllRead) return false;

		// Clear the unread bytes
		char* target2 = ((char*)target) + read;
		memset(target2, 0, numBytes - read);
	}

	return true;
}

// Writes a desired number of bytes from the the pointer
bool ArduinoInterface::deviceWrite(const void* source, const unsigned int numBytes) {
	DWORD written;
	if (m_comPort == INVALID_HANDLE_VALUE) return false;

	return (WriteFile(m_comPort, source, numBytes, &written, NULL)) && (written == numBytes);
}
