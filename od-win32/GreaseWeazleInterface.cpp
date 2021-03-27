/* GreaseWeazle C++ Interface for reading and writing Amiga Disks
*
* By Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
* 
* Based on the excellent code by Keir Fraser <keir.xen@gmail.com>
* https://github.com/keirf/Greaseweazle/
* 
* Used the Arduino Reader/Writer library as a template to work from
*
* This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */


#include "GreaseWeazleInterface.h"
#include <sstream>
#include <vector>
#include <queue>
#include <SetupAPI.h>
#include <Devpropdef.h>
//#include <Devpkey.h>
#pragma comment(lib, "setupapi.lib")

#define ONE_NANOSECOND 1000000000UL
#define BITCELL_SIZE_IN_NS 2000L

using namespace GreaseWeazle;


static const DEVPROPKEY DEVPKEY_Device_BusReportedDeviceDesc2 = { {0x540b947e, 0x8b40, 0x45bc, 0xa8, 0xa2, 0x6a, 0x0b, 0x89, 0x4c, 0xbd, 0xa2}, 4 };
static const DEVPROPKEY DEVPKEY_Device_InstanceId2 = { {0x78c34fc8, 0x104a, 0x4aca, 0x9e, 0xa4, 0x52, 0x4d, 0x52, 0x99, 0x6e, 0x57}, 256 };   // DEVPROP_TYPE_STRING


// When streaming, this amount of bit sequences is checked at the end of the track to properly work out where the overlap is as the INDEX pulse isnt accurate enough
// This is probably much higher than needs to be
#define OVERLAP_WINDOW_SIZE			32

// Command names
static const char* CmdStr[] = {
	"GetInfo",
	"Update",
	"Seek",
	"Head",
	"SetParams",
	"GetParams",
	"Motor",
	"ReadFlux",
	"WriteFlux",
	"GetFluxStatus",
	"GetIndexTimes",
	"SwitchFwMode",
	"Select",
	"Deselect",
	"SetBusType",
	"SetPin",
	"Reset",
	"EraseFlux",
	"SourceBytes",
	"SinkBytes" 
};

// Error messages
static const char* AckStr[] = {
	"Okay",
	"Bad Command",
	"No Index",
	"Track 0 not found",
	"Flux Overflow",
	"Flux Underflow",
	"Disk is Write Protected",
	"No drive unit selected",
	"No bus type (eg. Shugart, IBM/PC) specified",
	"Invalid unit number",
	"Not a modifiable pin",
	"Invalid cylinder" 
};

enum class GetInfo { Firmware = 0, BandwidthStats = 1 };
// ## Cmd.{Get,Set}Params indexes
enum class Params { Delays = 0 };
// ## Flux read stream opcodes, preceded by 0xFF byte
enum class FluxOp { Index = 1, Space = 2, Astable = 3};

#pragma pack(1) 

/* CMD_READ_FLUX */
struct GWReadFlux {
	/* Maximum ticks to read for (or 0, for no limit). */
	unsigned int ticks;
	/* Maximum index pulses to read (or 0, for no limit). */
	unsigned short max_index;
	/* Linger time, in ticks, to continue reading after @max_index pulses. */
	unsigned int max_index_linger; /* default: 500 microseconds */
};

/* CMD_WRITE_FLUX */
struct GWWriteFlux {
	/* If non-zero, start the write at the index pulse. */
	unsigned char cue_at_index;
	/* If non-zero, terminate the write at the next index pulse. */
	unsigned char terminate_at_index;
};

#pragma pack()

struct Sequence {
	unsigned char sequence;
	unsigned short speed;     // speed compared to what it should be (for this sequence)
	bool atIndex;			// At index
};

// We're working in nanoseconds
struct PLLData {
	unsigned int freq = 0;			  // sample frequency in Hz
	int ticks = 0;					// This can go negative

	// Used fot tracking the output
	int pos = 0;
	int bit = 0;

	bool enableWrite = false;
};

// In Debug at least, std::queue was too slow, might be memory allocation overhead, not sure.  So a simple queue implementation was used below
#define QUEUE_SIZE (OVERLAP_WINDOW_SIZE*4)

template <class queueType>
class FastQueue2 {
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

// Constructor for this class
GreaseWeazleInterface::GreaseWeazleInterface() {
	m_comPort = INVALID_HANDLE_VALUE;
	m_diskInDrive = false;

	m_currentBusType = BusType::IBMPC;
	m_currentDriveIndex = 0;

	memset(&m_gwVersionInformation, 0, sizeof(m_gwVersionInformation));
	memset(&m_gwDriveDelays, 0, sizeof(m_gwDriveDelays));
}

// Free me
GreaseWeazleInterface::~GreaseWeazleInterface() {
	closePort();
}
																																			  // Extract the VID and PID from the hardware id
static void getPidVid(std::string deviceString, int& vid, int& pid) {
	int a = deviceString.find("VID_");
	if (a != std::string::npos) vid = strtol(deviceString.substr(a + 4).c_str(), NULL, 16); else vid = 0;
	a = deviceString.find("PID_");
	if (a != std::string::npos) pid = strtol(deviceString.substr(a + 4).c_str(), NULL, 16); else pid = 0;
}

// Search and find the Greaseweazle COM port
int findPortNumber() {
	// Query for hardware
	HDEVINFO hDevInfoSet = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
	if (hDevInfoSet == INVALID_HANDLE_VALUE) return -1;
		
	// Scan for items
	DWORD devIndex = 0;
	SP_DEVINFO_DATA devInfo;
	devInfo.cbSize = sizeof(SP_DEVINFO_DATA);

	int portFound = -1;
	int maxScore = 0;
	
	// Enum devices
	while (SetupDiEnumDeviceInfo(hDevInfoSet, devIndex, &devInfo)) {
		HKEY key = SetupDiOpenDevRegKey(hDevInfoSet, &devInfo, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
		if (key != INVALID_HANDLE_VALUE) {
			char name[128];
			DWORD nameLength = 128;

			// Get the COM Port Name
			if (RegQueryValueExA(key, "PortName", NULL, NULL, (LPBYTE)name, &nameLength) == ERROR_SUCCESS) {
				std::string pn = name;

				// Check it starts with COM
				if ((pn.length() >= 4) && (pn.substr(0, 3) == "COM")) {
					int score = 0;
					int comPortNumber = atoi(pn.substr(3).c_str());

					// Get the hardware ID
					nameLength = 128;
					if (score < 20) {
						DWORD dwType;
						if (SetupDiGetDeviceRegistryPropertyA(hDevInfoSet, &devInfo, SPDRP_HARDWAREID, &dwType, (LPBYTE)name, 128, &nameLength))
						{
							int vid, pid;
							getPidVid(name, vid, pid);
							// properly-assigned PID. Guaranteed to be Greaseweazle.
							if ((vid == 0x1209) && (pid == 0x4d69)) {
								score+= 20;
							}
							else // old shared Test PID. It's not guaranteed to be Greaseweazle.
								if ((vid == 0x1209) && (pid == 0x0001)) {
									score+= 10;
								}
						}
					}

					DEVPROPTYPE type = 0;
					WCHAR productName[128];

					// Greaseweazle "product"
					if (SetupDiGetDeviceProperty(hDevInfoSet, &devInfo, &DEVPKEY_Device_BusReportedDeviceDesc2, &type, (PBYTE)productName, 128, 0, 0)) {
						if (wcscmp(productName, L"Greaseweazle") == 0) {
							score += 10;
						}
					}

					// Finally, look for the serial number
					// Greaseweazle "product"
					if (SetupDiGetDeviceProperty(hDevInfoSet, &devInfo, &DEVPKEY_Device_InstanceId2, &type, (PBYTE)productName, 128, 0, 0)) {
						if (wcsstr(productName, L"\\GW")) {
							// Add more
							score += 10;
						}
					}

					// Keep this port if its achieved a place on our high score table
					if (score > maxScore) {
						maxScore = score;
						portFound = comPortNumber;
					}
				}
			}
			RegCloseKey(key);
		}

		devIndex++;
	}

	SetupDiDestroyDeviceInfoList(hDevInfoSet);

	return portFound;
}

// Attempts to open the reader running on the COM port number provided.  Port MUST support 2M baud
GWResponse GreaseWeazleInterface::openPort(bool useDriveA) {
	closePort();

	m_motorIsEnabled = false;

	// Find and detect the port
	int gwPortNumber = findPortNumber();

	// If no device detected?
	if (gwPortNumber < 1) {
		return GWResponse::drPortNotFound;
	}

	// Communicate with the serial port
	char buffer[20];
	sprintf_s(buffer, "\\\\.\\COM%i", gwPortNumber);
	m_comPort = CreateFileA(buffer, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, 0);

	// No com port? Error!
	if (m_comPort == INVALID_HANDLE_VALUE) {
		int i = GetLastError();
		switch (i) {
			case ERROR_FILE_NOT_FOUND:  return GWResponse::drPortNotFound;
			case ERROR_ACCESS_DENIED:   return GWResponse::drPortInUse;
			default: return GWResponse::drPortError;
		}
	}

	// Prepare communication settings
	COMMCONFIG config;
	DWORD comConfigSize = sizeof(config);
	memset(&config, 0, sizeof(config));

	GetCommConfig(m_comPort, &config, &comConfigSize);
	config.dwSize = sizeof(config);
	config.dcb.DCBlength = sizeof(config.dcb);
	config.dcb.BaudRate = 9600; 
	config.dcb.ByteSize = 8;        // 8-bit
	config.dcb.fBinary = true;
	config.dcb.Parity = false;
	config.dcb.fOutxCtsFlow = false;
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

	// Try to put the serial port in the mode we require
	if (!SetCommConfig(m_comPort, &config, sizeof(config))) {		
		closePort();
		return GWResponse::drComportConfigError;
	}

	applyCommTimeouts(false);
	// Clear any pending data
	PurgeComm(m_comPort, PURGE_TXCLEAR | PURGE_RXCLEAR);

	// Lets ask GW it's firmware version
	Ack response = Ack::Okay;
	if (!sendCommand(Cmd::GetInfo, (unsigned char)GetInfo::Firmware, response)) {
		PurgeComm(m_comPort, PURGE_TXCLEAR | PURGE_RXCLEAR);

		if (!sendCommand(Cmd::GetInfo, (unsigned char)GetInfo::Firmware, response)) {
			closePort();
			return GWResponse::drErrorMalformedVersion;
		}
	};
	DWORD read;	
	if (!ReadFile(m_comPort, &m_gwVersionInformation, sizeof(m_gwVersionInformation), &read, NULL)) read = 0;
	if (read != 32) {
		closePort();
		return GWResponse::drErrorMalformedVersion;
	}
	if ((m_gwVersionInformation.major == 0) && (m_gwVersionInformation.minor < 25)) {
		closePort();
		return GWResponse::drOldFirmware;
	}
	if (m_gwVersionInformation.is_main_firmware == 0) {
		closePort();
		return GWResponse::drInUpdateMode;
	}

	// Reset it
	if (!sendCommand(Cmd::Reset, nullptr, 0, response)) {
		closePort();
		return GWResponse::drErrorMalformedVersion;
	};

	// Now get the drive delays
	if (!sendCommand(Cmd::GetParams, (unsigned char)Params::Delays, response, sizeof(m_gwDriveDelays))) {
		closePort();
		return GWResponse::drErrorMalformedVersion;
	};
	if (!ReadFile(m_comPort, &m_gwDriveDelays, sizeof(m_gwDriveDelays), &read, NULL)) read = 0;
	if (read != sizeof(m_gwDriveDelays)) {
		closePort();
		return GWResponse::drErrorMalformedVersion;
	};

	// Select the drive we want to communicate with
	m_currentBusType = BusType::IBMPC;
	m_currentDriveIndex = useDriveA ? 0 : 1;

	if (!sendCommand(Cmd::SetBusType, (unsigned char)m_currentBusType, response)) {
		closePort();
		return GWResponse::drError;
	}

	// Ok, success
	return GWResponse::drOK;
}

// Change the drive delays as required
bool GreaseWeazleInterface::updateDriveDelays() {
	unsigned char buffer[11];
	buffer[0] = (unsigned char)Params::Delays;
	memcpy_s(buffer + 1, 10, &m_gwDriveDelays, sizeof(m_gwDriveDelays));

	Ack response = Ack::Okay;
	if (!sendCommand(Cmd::SetParams, buffer, 11, response)) return false;

	return true;
}

// Trigger drive select
bool GreaseWeazleInterface::selectDrive(bool select) {
	Ack response = Ack::Okay;

	if (select) {
		return sendCommand(Cmd::Select, m_currentDriveIndex, response);
	}
	else {
		return sendCommand(Cmd::Deselect, nullptr, 0, response);
	}
}

// Apply and change the timeouts on the com port
void GreaseWeazleInterface::applyCommTimeouts(bool shortTimeouts) {
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
void GreaseWeazleInterface::closePort() {
	if (m_comPort != INVALID_HANDLE_VALUE) {
		// Force the drive to power down
		enableMotor(false);

		// And close the handle
		CloseHandle(m_comPort);
		m_comPort = INVALID_HANDLE_VALUE;
	}
}

// Turns on and off the reading interface
GWResponse GreaseWeazleInterface::enableMotor(const bool enable, const bool dontWait) {
	Ack response = Ack::Okay;

	std::vector<unsigned char> params;

	unsigned int delay = dontWait ? 10 : 750;
	if (delay != m_gwDriveDelays.motor_delay) {
		m_gwDriveDelays.motor_delay = delay;
		updateDriveDelays();
	}
	
	unsigned char buffer[2] = { m_currentDriveIndex, ((unsigned char)(enable ? 1 : 0)) };
	if (!sendCommand(Cmd::Motor, buffer, 2, response)) {
		return GWResponse::drError;
	}

	if (response == Ack::Okay) m_motorIsEnabled = enable;

	return (response == Ack::Okay) ? GWResponse::drOK : GWResponse::drError;
}

// Select the track, this makes the motor seek to this position
GWResponse GreaseWeazleInterface::selectTrack(const unsigned char trackIndex, const TrackSearchSpeed searchSpeed, bool ignoreDiskInsertCheck) {
	if (trackIndex > 81) {
		return GWResponse::drTrackRangeError; // no chance, it can't be done.
	}

	unsigned short newSpeed = m_gwDriveDelays.step_delay;
	switch (searchSpeed) {
		case TrackSearchSpeed::tssSlow:		newSpeed = 8000;  break;
		case TrackSearchSpeed::tssNormal:	newSpeed = 7000;  break;
		case TrackSearchSpeed::tssFast:		newSpeed = 6000;  break;
		case TrackSearchSpeed::tssVeryFast: newSpeed = 5000;  break;
	}
	if (newSpeed != m_gwDriveDelays.step_delay) {
		m_gwDriveDelays.step_delay = newSpeed;
		updateDriveDelays();
	}	

	selectDrive(true);

	Ack response = Ack::Okay;
	sendCommand(Cmd::Seek, trackIndex, response);
	selectDrive(false);

	// We have to check for a disk manually
	if (!ignoreDiskInsertCheck) {
		checkForDisk(true);
	}


	switch (response) {
	case Ack::NoTrk0: return GWResponse::drRewindFailure;
	case Ack::Okay: return GWResponse::drOK;
	default: return GWResponse::drSelectTrackError;
	}
}

// Search for track 0
GWResponse GreaseWeazleInterface::findTrack0() {
	return selectTrack(0, TrackSearchSpeed::tssFast, true);
}

// Choose which surface of the disk to read from
GWResponse GreaseWeazleInterface::selectSurface(const DiskSurface side) {

	Ack response = Ack::Okay;
	sendCommand(Cmd::Head, (side == DiskSurface::dsUpper ? 1 : 0), response);

	return (response == Ack::Okay) ? GWResponse::drOK : GWResponse::drError;
}

// send a command out to the GW and receive its response.  Returns FALSE on error
bool GreaseWeazleInterface::sendCommand(Cmd command, void* params, unsigned int paramsLength, Ack& response, unsigned char extraResponseSize) {
	std::vector<unsigned char> data;
	data.resize(paramsLength + 2);
	data[0] = (unsigned char)command;
	data[1] = 2 + paramsLength + (extraResponseSize>0 ? 1 : 0);
	if ((params) && (paramsLength)) memcpy_s(((unsigned char*)data.data()) + 2, paramsLength, params, paramsLength);
	if (extraResponseSize>0) data.push_back(extraResponseSize);

	// Write request
	DWORD written = 0;
	if (!WriteFile(m_comPort, data.data(), data.size(), &written, NULL)) return false;
	if (written != data.size()) {
		response = Ack::BadCommand;
		return false;
	}

	// Read response
	unsigned char responseMsg[2];
	if (!ReadFile(m_comPort, (LPVOID)responseMsg, 2, &written, NULL)) return false;
	if (written != 2) {
		response = Ack::BadCommand;
		return false;
	}

	response = (Ack)responseMsg[1];

	// Was it the response to the command we issued?
	if (responseMsg[0] != (unsigned char)command) {
		response = Ack::BadCommand;
		return false;
	}

	return true;
}
bool GreaseWeazleInterface::sendCommand(Cmd command, const std::vector<unsigned char>& params, Ack& response, unsigned char extraResponseSize) {
	return sendCommand(command, (void*)params.data(), params.size(), response, extraResponseSize);
}
bool GreaseWeazleInterface::sendCommand(Cmd command, unsigned char param, Ack& response, unsigned char extraResponseSize) {
	return sendCommand(command, (void*)(&param), 1, response, extraResponseSize);
}

// Taken from optimised.c and modified
static int read_28bit(FastQueue2<unsigned char>& queue) {
	int x;
	x = queue.next() >> 1;
	x |= (queue.next() & 0xfe) << 6;
	x |= (queue.next() & 0xfe) << 13;
	x |= (queue.next() & 0xfe) << 20;

	return x;
}

// write mfm bit into the output stream
static void writeBit(RawTrackData& output, int& pos, int& bit, int value) {
	if (pos >= RAW_TRACKDATA_LENGTH) return;

	output[pos] <<= 1;
	output[pos] |= value;
	bit++;
	if (bit >= 8) {
		pos++;
		bit = 0;
	}
}

// Convert from nSec to TICKS
static int nSecToTicks(int nSec, int sampleFrequency) {
	return ((long long)nSec * (long long)sampleFrequency) / (long long)ONE_NANOSECOND;
}

// Convert from TICKS to nSec
static int ticksToNSec(int ticks, int sampleFrequency) {
	return ((long long)ticks * ONE_NANOSECOND) / (long long)sampleFrequency;
}

// Process queue and work out whats going on - returns TRUE if it detects an index
void processQueue(FastQueue2<unsigned char>& queue, PLLData& pllData, RawTrackData& trackData) {
	int val;

	// This only happens potentially for the last byte received from the stream
	if (queue.size() < 1) return;

	unsigned char i = queue.front();
	if (i == 255) {
		// This only happens potentially for the last byte received from the stream
		if (queue.size() < 6) return;
		queue.pop();

		// Get opcode
		switch ((FluxOp)queue.next()) {
		case FluxOp::Index:
			val = read_28bit(queue);
			pllData.enableWrite = true;
			return;
			break;
		case FluxOp::Space:
			pllData.ticks += read_28bit(queue);
			break;
		default: return; // this isnt allowed.
		}
	}
	else {
		if (i < 250) {
			val = i;
			queue.pop();

		}
		else {
			if (queue.size() < 2) return;
			queue.pop();

			val = 250 + (i - 250) * 255;
			val += queue.next() - 1;
		}

		pllData.ticks += val;

		// Work out the actual time this tick took in nanoSeconds.
		int tickInNS = ticksToNSec(pllData.ticks, pllData.freq);

		// Enough bit-cells?
		if (tickInNS > BITCELL_SIZE_IN_NS) {
			tickInNS -= BITCELL_SIZE_IN_NS;

			// Clock out the sequence
			while (tickInNS > (BITCELL_SIZE_IN_NS / 2)) {
				if (pllData.enableWrite) writeBit(trackData, pllData.pos, pllData.bit, 0);
				tickInNS -= BITCELL_SIZE_IN_NS;
			}
			if (pllData.enableWrite) writeBit(trackData, pllData.pos, pllData.bit, 1);
		}
		pllData.ticks = 0;
	}
}

// Read RAW data from the current track and surface 
GWResponse GreaseWeazleInterface::readCurrentTrack(RawTrackData& trackData, const bool readFromIndexPulse) {
	GWReadFlux header;
	if (readFromIndexPulse) {
		header.ticks = 0;
		header.max_index = 1;
		header.max_index_linger = nSecToTicks(220 * 1000 * 1000, m_gwVersionInformation.sample_freq);;
	}
	else {
		header.ticks = nSecToTicks(220 * 1000 * 1000, m_gwVersionInformation.sample_freq);
		header.max_index = 0;
		header.max_index_linger = 0;
	}
	
	selectDrive(true);

	// Write request
	Ack response = Ack::Okay;
	if (!sendCommand(Cmd::ReadFlux, (void*)&header, sizeof(header), response)) {
		selectDrive(false);
		return GWResponse::drReadResponseFailed;
	}

	DWORD bytesRead;
	FastQueue2<unsigned char> queue;

	PLLData pllData;
	pllData.freq = m_gwVersionInformation.sample_freq;
	pllData.enableWrite = !readFromIndexPulse;

	for (;;) {
		// Read a single byte
		unsigned char byte;
		if (!ReadFile(m_comPort, &byte, 1, &bytesRead, NULL)) bytesRead = 0;
		if (bytesRead) queue.push(byte);

		if (byte == 0) {
			processQueue(queue, pllData, trackData);
			break;
		}
		// If theres this many we can process as this is the maximum we need to process
		while (queue.size() >= 6) {
			processQueue(queue, pllData, trackData);
		}
	};

	// Check for errors
	response = Ack::Okay;
	sendCommand(Cmd::GetFluxStatus, nullptr, 0, response);

	selectDrive(false);

	switch (response) {
	case Ack::FluxOverflow: return GWResponse::drSerialOverrun;
	case Ack::NoIndex: return GWResponse::drNoDiskInDrive;
	case Ack::Okay: return GWResponse::drOK;
	default: return GWResponse::drReadResponseFailed;
	}
}

// Read RAW data from the current track and surface 
GWResponse GreaseWeazleInterface::checkForDisk(bool force) {
	if (force) {
		GWReadFlux header;
		header.ticks = 0;
		header.max_index = 2;
		header.max_index_linger = 0;

		bool alreadySpun = m_motorIsEnabled;
		if (!alreadySpun)
			if (enableMotor(true, false) != GWResponse::drOK) return GWResponse::drOK;

		selectDrive(true);

		// Write request
		Ack response = Ack::Okay;
		if (!sendCommand(Cmd::ReadFlux, (void*)&header, sizeof(header), response)) {
			selectDrive(false);
			return GWResponse::drOK;
		}

		DWORD bytesRead;

		for (;;) {
			// Read a single byte
			unsigned char byte;
			if (!ReadFile(m_comPort, &byte, 1, &bytesRead, NULL)) bytesRead = 0;
			if (byte == 0) break;
		};

		// Check for errors
		response = Ack::Okay;
		sendCommand(Cmd::GetFluxStatus, nullptr, 0, response);

		selectDrive(false);

		if (!alreadySpun) enableMotor(false, false);

		m_diskInDrive = response != Ack::NoIndex;
	}
	return m_diskInDrive ? GWResponse::drOK : GWResponse::drNoDiskInDrive;
}

// Read a bit from the MFM data provided
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

// Write 28bit value ot the output
void write28bit(int value, std::vector<unsigned char>& output) {
	output.push_back(1 | (value << 1) & 255);
	output.push_back(1 | (value >> 6) & 255);
	output.push_back(1 | (value >> 13) & 255);
	output.push_back(1 | (value >> 20) & 255);
}

// Write data to the disk
GWResponse GreaseWeazleInterface::writeCurrentTrackPrecomp(const unsigned char* mfmData, const unsigned short numBytes, const bool writeFromIndexPulse, bool usePrecomp) {
	std::vector<unsigned char> outputBuffer;

	// Original data was written from MSB downto LSB
	int pos = 0;
	int bit = 7;
	unsigned int outputPos = 0;
	unsigned char sequence = 0xAA;  // start at 10101010
	int lastCount = 2;

	const int nfa_thresh = (int)((float)150e-6 * (float)m_gwVersionInformation.sample_freq);
	const int precompTime = 140;   // Amiga default precomp amount (140ns)
	int extraTimeFromPrevious = 0;

	// Re-encode the data into our format and apply precomp.  The +1 is to ensure theres some padding around the edge 
	while (pos < numBytes + 1) {
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
		
		// Calculate the time for this in nanoseconds
		int timeInNS = extraTimeFromPrevious + (count * 2000);     // 2=4000, 3=6000, 4=8000, (5=10000 although is isnt strictly allowed)

		int precompDirection = 0;

		if (usePrecomp) {
			switch (sequence) {
			case 0x09:
			case 0x0A:
			case 0x4A: // early;
				timeInNS -= precompTime;
				extraTimeFromPrevious = precompTime;
				break;

			case 0x28:
			case 0x29:
			case 0x48: // late
				timeInNS += precompTime;
				extraTimeFromPrevious = -precompTime;
				break;

			default:
				extraTimeFromPrevious = 0;
				break;
			}
		}

		const int ticks = nSecToTicks(timeInNS, m_gwVersionInformation.sample_freq);
		
		// Encode for GW
		if (ticks>0)
			if (ticks < 250) {
				outputBuffer.push_back(ticks);
			}
			else {
				if (ticks > nfa_thresh) {
					outputBuffer.push_back(255);
					outputBuffer.push_back((unsigned char)FluxOp::Space);   // Space
					write28bit(ticks, outputBuffer);
					outputBuffer.push_back(255);
					outputBuffer.push_back((unsigned char)FluxOp::Space);   // Space
				}
				else {
					unsigned int high = (ticks - 250) / 255;
					if (high < 5) {
						outputBuffer.push_back(250 + high);
						outputBuffer.push_back(1 + (ticks - 250) % 255);
					}
					else {
						outputBuffer.push_back(255);
						outputBuffer.push_back((unsigned char)FluxOp::Space);   // Space
						write28bit(ticks - 249, outputBuffer);
						outputBuffer.push_back(249);
					}
				}
			}
	}
	
	outputBuffer.push_back(0); // finish	

	selectDrive(true);

	// Header
	GWWriteFlux header;
	header.cue_at_index = writeFromIndexPulse ? 1 : 0;
	header.terminate_at_index = 0;

	// Write request
	Ack response = Ack::Okay;
	if (!sendCommand(Cmd::WriteFlux, (void*)&header, sizeof(header), response)) {
		selectDrive(false);
		return GWResponse::drReadResponseFailed;
	}

	if (response == Ack::Wrprot) {
		selectDrive(false);
		return GWResponse::drWriteProtected;
	}

	if (response != Ack::Okay) {
		selectDrive(false);
		return GWResponse::drReadResponseFailed;
	}

	// Write it
	DWORD read;
	if (!WriteFile(m_comPort, outputBuffer.data(), outputBuffer.size(), &read, NULL)) read = 0;
	if (read != outputBuffer.size()) {
		selectDrive(false);
		return GWResponse::drReadResponseFailed;
	}

	// Sync with GW
	unsigned char sync;
	if (!ReadFile(m_comPort, &sync, 1, &read, NULL)) read = 0;
	if (read != 1) {
		selectDrive(false);
		return GWResponse::drReadResponseFailed;
	}

	// Check the value of SYNC
	response = Ack::Okay;
	sendCommand(Cmd::GetFluxStatus, nullptr, 0, response);

	selectDrive(false);

	switch (response) {
		case Ack::FluxUnderflow: return GWResponse::drSerialOverrun;
		case Ack::Wrprot: return GWResponse::drWriteProtected;
		case Ack::Okay: return GWResponse::drOK;
		default: return GWResponse::drReadResponseFailed;
	}
}

// Locates the part of this entire buffer that matches startSequence the best.  Returns how many sequences from currentBitSequences + futureBitSequences are needed to complete the revolution
static int findSlidingWindow(const std::vector<unsigned char>& searchSequence, const FastQueue2< Sequence>& futureBitSequences, const FastQueue2<Sequence>& currentBitSequences) {
	if (futureBitSequences.size() < OVERLAP_WINDOW_SIZE) return 0;
	if (currentBitSequences.size() < OVERLAP_WINDOW_SIZE) return 0;
	if (searchSequence.size() < OVERLAP_WINDOW_SIZE) return 0;

	// Make a vector with all the data from the queue
	std::vector<unsigned char> searchArea;
	FastQueue2<Sequence> copy = currentBitSequences;
	while (copy.size()) {
		searchArea.push_back(copy.front().sequence);
		copy.pop();
	}

	copy = futureBitSequences;
	while (copy.size()) {
		searchArea.push_back(copy.front().sequence);
		copy.pop();
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
inline void writeStreamBit(GWMfmSample* output, unsigned int& pos, unsigned int& bit, unsigned int value, unsigned int valuespeed, unsigned int maxLength) {
	if (pos >= maxLength) return;

	output[pos].mfmData <<= 1;
	output[pos].mfmData |= value;

	// If the data read quicker, then valuespeed will be smaller.  
	// If it read slower, then valuespeed will be larger
	// smaller=more bits

	// valuespeed for GW is already scaled and will just need a *10 at the far end
	if (bit == 0) output[pos].speed = 0; // ensure it's reset
	output[pos].speed += valuespeed;  // get the average speed over 8 bits

	bit++;
	if (bit >= 8) {
		pos++;
		bit = 0;
	}
}

// Handle adding these bits to the output buffer
static void outputBitSequence(Sequence value, GWMfmSample* buffer, unsigned int& pos, unsigned int& bit, const int maxBufferSize) {
	for (int a=0; a<=value.sequence; a++)
		writeStreamBit(buffer, pos, bit, 0, value.speed, maxBufferSize);

	writeStreamBit(buffer, pos, bit, 1, value.speed, maxBufferSize);
}

// Handle sending and updating the buffer.  Returns FALSE if we should abort
static bool flushAndPush(GWMfmSample* buffer, unsigned int& pos, unsigned int& bit, const unsigned int maxBlockSize, const unsigned int maxBufferSize, std::function<bool(const GWMfmSample* samples, const unsigned dataLengthInBits, bool isEndOfRevolution)> dataStream, bool flushALL = false) {
	int flushSize = flushALL ? 0 : maxBlockSize;

	// Do we have enough data? (we hold some back as part of the end of track "slide")
	if ((int)pos > flushSize) {
		// Do the callback
		GWMfmSample* bufStart = buffer;
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
			MoveMemory(buffer, bufStart, ((maxBufferSize * sizeof(GWMfmSample)) - (startBytes * sizeof(GWMfmSample)) + sizeof(GWMfmSample)));
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
			buffer->speed += (shiftAmount * 100);   // add speed for the missing bits
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

// Process queue and work out whats going on - return 0 for nothing, 1 for data, 2 for index
int unpackStreamQueue(FastQueue2<unsigned char>& queue, PLLData& pllData, Sequence& output) {
	int val;

	// This only happens potentially for the last byte received from the stream
	if (queue.size() < 1) return 0;

	unsigned char i = queue.front();
	if (i == 255) {
		// This only happens potentially for the last byte received from the stream
		if (queue.size() < 6) return 0;
		queue.pop();

		// Get opcode
		switch ((FluxOp)queue.next()) {
		case FluxOp::Index:
			val = read_28bit(queue);
			return 2;
			break;
		case FluxOp::Space:
			pllData.ticks += read_28bit(queue);
			break;
		default: return 0; // this isnt allowed.
		}
	}
	else {

		if (i < 250) {
			val = i;
			queue.pop();
		}
		else {
			if (queue.size() >= 2) {
				queue.pop();
				val = 250 + (i - 250) * 255;
				val += queue.next() - 1;
			}
			else return 0;
		}

		pllData.ticks += val;

		// Work out the actual time this tick took in nanoSeconds.
		int tickInNS = ticksToNSec(pllData.ticks, pllData.freq);

		// Enough bit-cells?
		if (tickInNS > BITCELL_SIZE_IN_NS) {
			output.sequence = 0;   // 1=01, 2=001, 3=0001, 4=00001 etc

			int t = tickInNS - BITCELL_SIZE_IN_NS;

			// Clock out the sequence
			while (t > (BITCELL_SIZE_IN_NS / 2)) {
				output.sequence++;
				t -= BITCELL_SIZE_IN_NS;
			}
			if (output.sequence < 1) output.sequence = 1;
			output.sequence--;					
			output.speed = ((long long)tickInNS * 100L) / (((long long)(output.sequence + 2)) * BITCELL_SIZE_IN_NS);

			// Save it
			pllData.ticks = 0;

			return 1;
		}
		
	}
	return 0;
}

// Streaming version with timing information
GWResponse GreaseWeazleInterface::readCurrentTrackStream(const unsigned int maxBlockSize, const unsigned int maxRevolutions, std::vector<unsigned char>& startBitPatterns, std::function<bool(const GWMfmSample* samples, const unsigned dataLengthInBits, bool isEndOfRevolution)> dataStream) {
	// Setup with number of revolutions and to run slightly more than 1 rotation after it
	GWReadFlux header;
	header.ticks = 0;
	header.max_index = maxRevolutions;
	header.max_index_linger = nSecToTicks(210 * 1000 * 1000, m_gwVersionInformation.sample_freq);;

	DWORD bytesRead;
	FastQueue2<unsigned char> queue;

	PLLData pllData;
	pllData.freq = m_gwVersionInformation.sample_freq;

	FastQueue2<Sequence> futureBitSequences;
	FastQueue2<Sequence> currentBitSequences;
	FastQueue2<Sequence> oldBitSequences;

	// Clear if incomplete
	if (startBitPatterns.size() < OVERLAP_WINDOW_SIZE) startBitPatterns.clear();
	bool oldSequenceEnabled = startBitPatterns.size() >= OVERLAP_WINDOW_SIZE;

	unsigned char byte = 1;
	Sequence sequence = { 0,100,false };

	unsigned int pos = 0;
	unsigned int index = 0;
	unsigned int bit = 0;
	bool startIndexFound = false;
	int loops = 0;
	bool m_abort = false;

	const unsigned int maxBufferSize = maxBlockSize + (OVERLAP_WINDOW_SIZE * 2) + 10;   // make sure we have enough room
	GWMfmSample* buffer = (GWMfmSample*)malloc(maxBufferSize * sizeof(GWMfmSample));
	if (buffer == nullptr) return GWResponse::drError;

	selectDrive(true);

	// Write request
	Ack response = Ack::Okay;
	int skipIndex = 0;
	if (!sendCommand(Cmd::ReadFlux, (void*)&header, sizeof(header), response)) {
		free(buffer);
		selectDrive(false);
		return GWResponse::drReadResponseFailed;
	}

	do {
		// Read a single byte
		if (byte)
			if (!ReadFile(m_comPort, &byte, 1, &bytesRead, NULL)) bytesRead = 0;
		// If theres this many we can process as this is the maximum we need to process
		if (!m_abort) {
			if ((bytesRead) && (byte>0)) queue.push(byte);
			
			bool exitLoop = false;
			do {
				switch (unpackStreamQueue(queue, pllData, sequence)) {
				case 0: exitLoop = true;
						break; // nothing to proces
				case 1:
					// Data
					futureBitSequences.push(sequence);
					sequence.atIndex = false;
					break;
				case 2:
					// index
					sequence.atIndex = true;
					break;
				}
			} while (!exitLoop);

			// Process
			int limit = (byte == 0) ? 0 : OVERLAP_WINDOW_SIZE * 2;
			while ((futureBitSequences.size() > limit) && (!m_abort)) {
				Sequence value = futureBitSequences.next();

				// Now add it to the previous sequences, but only if an index has been detected
				if (startIndexFound) {
					currentBitSequences.push(value);

					while (currentBitSequences.size() > limit) {
						// Write whats left here into the output stream
						outputBitSequence(currentBitSequences.next(), buffer, pos, bit, maxBufferSize);

						// Handle sending and updating the buffer.  Returns FALSE if we should abort
						if (!flushAndPush(buffer, pos, bit, maxBlockSize, maxBufferSize, dataStream))
							m_abort = true;
					}
				}
				else {
					// This happens if a sequence is provided, and we havent found the start yet.  There could be mis-alignment if we dont store some of the bits before INDEX detected
					if ((oldSequenceEnabled) && (startBitPatterns.size())) {
						oldBitSequences.push(value);
						while (oldBitSequences.size() > OVERLAP_WINDOW_SIZE * 2) oldBitSequences.pop();
					}
				}

				// Build up the start buffer
				if ((startIndexFound) && (startBitPatterns.size() < OVERLAP_WINDOW_SIZE)) {
					startBitPatterns.push_back(value.sequence);
				}

				// So, "value" and "isIndex" are now in the middle of all of this sliding data
				if (skipIndex) skipIndex--;
				if ((value.atIndex) && (skipIndex == 0)) {
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
							if (startBitPatterns.size() < OVERLAP_WINDOW_SIZE) startBitPatterns.push_back(value.sequence);
							startIndexFound = true;
							// And dont forget we want to process and output this data
							currentBitSequences.push(value);
						}
					}
					else {
						loops++;
						// This marks the END of the current cycle, and the start of the next.
							// At this point 'value' is at the back of currentBitSequences()
							// This shouldn't really be anymore than +/- about 3 bit cells from experience
						int i = findSlidingWindow(startBitPatterns, futureBitSequences, currentBitSequences);

						// "i" returns where to cut the buffer off at.  This is how many patterns need adding to complete the track.  Not sure why this varies EVEN WITH this code
						std::vector<Sequence> patternsToOutput;
						while ((i > 0) && (currentBitSequences.size())) {
							patternsToOutput.push_back(currentBitSequences.next());
							i--;
						}
						while ((i > 0) && (futureBitSequences.size())) {
							patternsToOutput.push_back(futureBitSequences.next());
							i--;
						}

						// Now patternsToOutput is whats left that needs to be flushed to finish the revolution.  Then the buffers should just re-fill themselves
						for (Sequence& value : patternsToOutput) {
							// Process this sequence
							outputBitSequence(value, buffer, pos, bit, maxBufferSize);

							// Handle sending and updating the buffer.  Returns FALSE if we should abort
							if (!flushAndPush(buffer, pos, bit, maxBlockSize, maxBufferSize, dataStream, false)) {
								m_abort = true;
							}
						}

						if (loops >= (int)maxRevolutions) {
							loops = 0;

							// Flush and close the buffer. Returns FALSE if we should abort
							if (!flushAndPush(buffer, pos, bit, maxBlockSize, maxBufferSize, dataStream, true)) {
								m_abort = true;

								// Populate the new sequence, this allows any incorrect bits to update to the new incorrect bits
								if (futureBitSequences.size() + currentBitSequences.size() >= OVERLAP_WINDOW_SIZE) {
									startBitPatterns.clear();

									while (futureBitSequences.size()) {
										currentBitSequences.push(futureBitSequences.next());
									}

									while (startBitPatterns.size() < OVERLAP_WINDOW_SIZE) {
										startBitPatterns.push_back(currentBitSequences.front().sequence);
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
								skipIndex = futureBitSequences.size()+1;
							}
						}
						else {
							while (futureBitSequences.size()) {
								currentBitSequences.push(futureBitSequences.next());
							}
							std::swap(currentBitSequences, futureBitSequences);
							startBitPatterns.clear();
							skipIndex = futureBitSequences.size()+1;
						}
					}
				}
			}
		}		
	} while ((byte) || (((futureBitSequences.size()) || (currentBitSequences.size())) && (!m_abort)));

	free(buffer);

	// Check for errors
	response = Ack::Okay;
	sendCommand(Cmd::GetFluxStatus, nullptr, 0, response);

	selectDrive(false);

	// Update this flag
	m_diskInDrive = response != Ack::NoIndex;

	switch (response) {
	case Ack::FluxOverflow: return GWResponse::drSerialOverrun;
	case Ack::NoIndex: return GWResponse::drNoDiskInDrive;
	case Ack::Okay: return GWResponse::drOK;
	default: return GWResponse::drReadResponseFailed;
	}


	return GWResponse::drOK;
}

