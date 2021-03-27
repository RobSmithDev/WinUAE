/* floppybridge_config
*
* Copyright 2021 Robert Smith (@RobSmithDev)
* https://amiga.robsmithdev.co.uk
*
* This library defines which interfaces are enabled within WinUAE
* 
*  This is free and unencumbered released into the public domain.
*  See the file COPYING for more details, or visit <http://unlicense.org>.
*
*/

#pragma once

/* rommgr.h has the following ROM TYPES reserved:  This is so that file does not need to be changed to support upto 16 new interfaces
#define ROMTYPE_FLOPYBRDGE0 0x0010008c
#define ROMTYPE_FLOPYBRDGE1 0x0010008d
#define ROMTYPE_FLOPYBRDGE2 0x0010008e
#define ROMTYPE_FLOPYBRDGE3 0x0010008f
#define ROMTYPE_FLOPYBRDGE4 0x00100090
#define ROMTYPE_FLOPYBRDGE5 0x00100091
#define ROMTYPE_FLOPYBRDGE6 0x00100092
#define ROMTYPE_FLOPYBRDGE7 0x00100093
#define ROMTYPE_FLOPYBRDGE8 0x00100094
#define ROMTYPE_FLOPYBRDGE9 0x00100095
#define ROMTYPE_FLOPYBRDGEA 0x00100096
#define ROMTYPE_FLOPYBRDGEB 0x00100097
#define ROMTYPE_FLOPYBRDGEC 0x00100098
#define ROMTYPE_FLOPYBRDGED 0x00100099
#define ROMTYPE_FLOPYBRDGEE 0x0010009A
#define ROMTYPE_FLOPYBRDGEF 0x0010009B
*/



// Arduino floppy reader/writer
#define ROMTYPE_ARDUINOREADER_WRITER			ROMTYPE_FLOPYBRDGE0
#include "ArduinoFloppyBridge.h"

// GreaseWeazle floppy reader/writer
#define ROMTYPE_GREASEWEAZLEREADER_WRITER		ROMTYPE_FLOPYBRDGE1
#include "GreaseWeazleBridge.h"


// Not the nicest way to do this, but here's how they get installed, for now
#define FACTOR_BUILDER(romtype, classname, settings) case romtype: bridge = new classname(settings); break;


// Build a list of what can be installed
#define BRIDGE_FACTORY(settings)																																\
		FACTOR_BUILDER(ROMTYPE_ARDUINOREADER_WRITER,ArduinoFloppyDiskBridge,settings)																			\
		FACTOR_BUILDER(ROMTYPE_GREASEWEAZLEREADER_WRITER,GreaseWeazleDiskBridge,settings)																		\



// This builds up the config options shown in in WinUAE.  
#define FLOPPY_BRIDGE_CONFIG																																	\
	{	_T("arduinoreaderwriter"), _T("Arduino Reader/Writer"), _T("RobSmithDev"),																				\
		NULL, NULL, NULL, NULL, ROMTYPE_ARDUINOREADER_WRITER | ROMTYPE_NOT, 0, 0, BOARD_IGNORE, true,															\
		bridge_drive_selection_config, 0, false, EXPANSIONTYPE_FLOPPY,																							\
		0, 0, 0, false, NULL, false, 0, arduino_reader_writer_options },																						\
	{	_T("greaseweazlewriter"), _T("GreaseWeazle Reader/Writer"), _T("Keir Fraser/Rob Smith"),																\
		NULL, NULL, NULL, NULL, ROMTYPE_GREASEWEAZLEREADER_WRITER | ROMTYPE_NOT, 0, 0, BOARD_IGNORE, true,														\
		bridge_drive_selection_config, 0, false, EXPANSIONTYPE_FLOPPY,																							\
		0, 0, 0, false, NULL, false, 0, greaseweazle_reader_writer_options },																					\


// And the options to add to the hardware extensions
#define FLOPPY_BRIDGE_CONFIG_OPTIONS																															\
	static const struct expansionboardsettings arduino_reader_writer_options[] = {{																				\
		_T("COM Port\0") _T("COM 1\0") _T("COM 2\0") _T("COM 3\0") _T("COM 4\0") _T("COM 5\0") _T("COM 6\0") _T("COM 7\0") _T("COM 8\0") _T("COM 9\0"),			\
		_T("port\0") _T("COM1\0") _T("COM2\0") _T("COM3\0") _T("COM4\0") _T("COM5\0") _T("COM6\0") _T("COM7\0") _T("COM8\0") _T("COM9\0"),						\
		true,false,0 }, { NULL }};																																\
	static const struct expansionboardsettings greaseweazle_reader_writer_options[] = {{																		\
		_T("Drive on Cable\0") _T("Drive A\0") _T("Drive B\0") ,																								\
		_T("drive\0") _T("drva\0") _T("drvb\0"),																												\
		true,false,0 }, { NULL }};																																\




	