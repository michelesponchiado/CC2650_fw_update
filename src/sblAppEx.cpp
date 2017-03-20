/******************************************************************************
*  Filename:       sblAppEx.cpp
*  Revised:        $Date: 2014-10-09 14:34:40 +0200 (to, 09 okt 2014) $
*  Revision:       $Revision: 32696 $
*
*  Description:    Serial Bootloader Library application example.
*                  This example enumerates all COM devices and lets you
*                  select which port to connect to. The example assumes the
*                  connected device is a CC2538 or a CC2650 and programs a blinky
*                  onto the device. After programming the blinky, bootloader
*                  mode may be forced by holding the SELECT button on 06EB
*                  when resetting the chip.
*
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/
#ifndef _GNU_SOURCE
	#define _GNU_SOURCE
#endif
#include <stdlib.h>
#include <stdint.h>
#include <sbllib.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <syslog.h>


#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <stdlib.h>
#include <time.h>
#include <sys/timeb.h>  /* ftime, timeb (for timestamp in millisecond) */
#include <sys/time.h>   /* gettimeofday, timeval (for timestamp in microsecond) */
#include "CC2650_fw_update.h"

#ifdef ANDROID
	#define def_selected_serial_port "/dev/ttymxc2"
#else
	#ifdef OLINUXINO
			#define def_selected_serial_port "/dev/ttyS1"
	#else
			#define def_selected_serial_port "/dev/ttyUSB1"
	#endif
#endif

#ifdef OLINUXINO_LIB
extern "C"
#else
extern
#endif// Calculate crc32 checksum the way CC2538 and CC2650 does it.
uint32_t calcCrcLikeChip(const unsigned char *pData, unsigned long ulByteCount)
{
    uint32_t d, ind;
    uint32_t acc = 0xFFFFFFFF;
    const uint32_t ulCrcRand32Lut[] =
    {
        0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
        0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
        0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
        0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
    };

    while ( ulByteCount-- )
    {
        d = *pData++;
        ind = (acc & 0x0F) ^ (d & 0x0F);
        acc = (acc >> 4) ^ ulCrcRand32Lut[ind];
        ind = (acc & 0x0F) ^ (d >> 4);
        acc = (acc >> 4) ^ ulCrcRand32Lut[ind];
    }

    return (acc ^ 0xFFFFFFFF);
}
#ifdef TARGET_UBUNTU
extern "C" int is_OK_do_CC2650_reset(unsigned int enable_boot_mode)
{
	//enable_boot_mode;
	return 1;
}
#endif

#ifndef LOCALMAIN

#ifndef OLINUXINO_LIB
static void open_syslog(void)
{
	openlog("CC2650_fw_upd", LOG_PID|LOG_CONS, LOG_DAEMON);
	my_log(LOG_INFO, "Log just started");
}
#endif

#ifdef OLINUXINO_LIB
extern "C" int
#else
extern int
#endif
is_OK_do_CC2650_reset(unsigned int enable_boot_mode);


#if 0
static int64_t get_current_epoch_time_ms(void)
{
	struct timeb timer_msec;
	int64_t timestamp_msec; // timestamp in milliseconds
	if (!ftime(&timer_msec))
	{
		timestamp_msec = ((long long int) timer_msec.time) * 1000ll +	(long long int) timer_msec.millitm;
	}
	else
	{
		timestamp_msec = -1;
	}
	return timestamp_msec;
}
#endif


//
// Including functions for millisecond precision timing.
// Taken from http://stackoverflow.com/questions/2494356/how-to-use-gettimeofday-or-something-equivalent-with-visual-studio-c-2008
//
#include <time.h>

extern "C" void my_log(int print_level, const char *fmt, ...);

/// Application status function (used as SBL status callback)
static void appStatus(char *pcText, bool bError)
{
    if(bError)
    {
    	my_log(LOG_ERR, "%s", pcText);
    }
    else
    {
    	my_log(LOG_INFO, "%s", pcText);
    }
}

#ifdef OLINUXINO_LIB
static uint32_t percentage_progress = 0;
#endif

/// Application progress function (used as SBL progress callback)
static void appProgress(uint32_t progress)
{
#ifdef OLINUXINO_LIB
	percentage_progress = progress;
#endif
}

#ifdef OLINUXINO_LIB
extern "C"
#endif
uint32_t get_CC2650_fw_update_progress(void)
{
#ifdef OLINUXINO_LIB
	return percentage_progress;
#else
	return 0;
#endif

}

// Defines
#define DEVICE_CC2538				0x2538
#define DEVICE_CC26XX				0x2650
#define CC2538_FLASH_BASE			0x00200000
#define CC26XX_FLASH_BASE			0x00000000
#ifndef OLINUXINO_LIB
static void my_at_exit(void)
{
	// at exit, close the system log
	closelog();
}
#endif


const char *tbl_msg_CC2650_fw_update_retcode[]=
{
	"OK",							// firmware updated OK
	"ERR_unable_to_reset_chip",				// unable to reset the CC2650 chip
	"ERR_unable_to_create_SBL_object", 		// unable to create serial boot-loader library object
	"ERR_unable_to_connect_with_the_chip",	// unable to connect with the chip
	"ERR_unable_to_open_the_firmware_file", 	//
	"ERR_unable_to_erase_the_flash",			// unable to erase the chips internal flash
	"ERR_writing_flash",						//
	"ERR_CRC_calculating_gone_bad",			//
	"ERR_CRC_checking_gone_bad",				//
	"ERR_chip_reset_gone_bad",				//
	"ERR_unable_to_seek_to_the_file_end",	//
	"ERR_unable_to_get_filesize",			//
	"ERR_unable_to_seek_where_the_file_begins", //
	"ERR_unable_to_allocate_the_read_buffer",
	"ERR_unable_to_read_the_whole_file",
	"ERR_unable_to_close_the_file",
	"ERR_filesize_too_small",
	"ERR_invalid_header_CRC32",
	"ERR_invalid_header_magic_key",
	"ERR_invalid_body_size",
	"ERR_invalid_body_CRC",
	"ERR_unable_to_reset_the_chip_in_normal_mode",
	"ERR_invalid_operation_requested",
};

#ifdef OLINUXINO_LIB
extern "C"
#endif
const char * get_msg_from_CC2650_fw_update_retcode(enum_do_CC2650_fw_update_retcode r)
{
	if (r >= sizeof(tbl_msg_CC2650_fw_update_retcode) / sizeof(tbl_msg_CC2650_fw_update_retcode[0]))
	{
		return "UNKNOWN";
	}
	return tbl_msg_CC2650_fw_update_retcode[r];
}

#ifdef OLINUXINO_LIB
extern "C"
#endif
enum_do_CC2650_fw_update_retcode do_CC2650_fw_operation(enum_CC2650_fw_operation op, const char *path_binary_file, type_ASACZ_CC2650_fw_update_header *p_dst_header)
{
	enum_do_CC2650_fw_update_retcode r = enum_do_CC2650_fw_update_retcode_OK;
#ifndef OLINUXINO_LIB
	open_syslog();
	atexit(my_at_exit);
#endif
	my_log(LOG_INFO, "%s + starts", __func__);

#ifdef OLINUXINO_LIB
	percentage_progress = 0;
#endif

	uint32_t do_fw_update = 0;

	switch(op)
	{
		case enum_CC2650_fw_operation_update_firmware:
		{
			my_log(LOG_INFO, "%s firmware update requested", __func__);
			do_fw_update = 1;
			break;
		}
		case enum_CC2650_fw_operation_get_file_header_info:
		{
			my_log(LOG_INFO, "%s firmware file get header info requested", __func__);
			break;
		}
		default:
		{
			my_log(LOG_ERR, "%s invalid operation requested %u", __func__, op);
			r = enum_do_CC2650_fw_update_retcode_ERR_invalid_operation_requested;
			break;
		}
	}

	//
	// START: Program Configuration
	//
	/* Device type. (Binary-coded decimal of the device
	     e.g. 0x2538 for CC2538 and 0x2650 for CC2650) */
	uint32_t deviceType = 0x2650;

	//
	// END: Program configuration
	//

	SblDevice *pDevice = NULL;		// Pointer to SblDevice object
	const char * fileName;			// File name to program
	uint32_t byteCount = 0;			// File size in bytes
    uint32_t fileCrc, devCrc;		// Variables to save CRC checksum
	uint32_t devFlashBase;	    	// Flash start address
	char * pvWrite = (char *)malloc(1);// Vector to application firmware in.
	FILE * file;		// File stream


	fileName = path_binary_file;

	if (do_fw_update)
	{
		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			if (!is_OK_do_CC2650_reset(1))
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_reset_chip;
				my_log(LOG_ERR, "ERR_unable_to_reset_chip");
			}
		}

		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{

			devFlashBase = CC26XX_FLASH_BASE;

			//
			// Set callback functions
			//
			SblDevice::setCallBackStatusFunction(&appStatus);
			SblDevice::setCallBackProgressFunction(&appProgress);

			//
			// Create SBL object
			//
			pDevice = SblDevice::Create(deviceType);
			if(pDevice == NULL)
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_create_SBL_object;
				my_log(LOG_ERR, "ERR_unable_to_create_SBL_object");
			}
		}
		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			unsigned int connected_OK = 0;
#define def_max_try_connect_CC2650 40
			int idx_try;
			for (idx_try = 0; !connected_OK && idx_try < def_max_try_connect_CC2650; idx_try++)
			{
				if (idx_try)
				{
					my_log(LOG_INFO, "trying again closing and reopening the serial port, try %u of %u", idx_try +1, def_max_try_connect_CC2650);
				}
				else
				{
					my_log(LOG_INFO, "Sleeping before opening the serial port %s", def_selected_serial_port);
				}
				usleep(100*1000);
				//
				// Connect to device
				//
				my_log(LOG_INFO, "Opening the serial port %s", def_selected_serial_port);
				if(pDevice->connect(def_selected_serial_port, false) != SBL_SUCCESS)
				{
					pDevice->CloseSerialPort();
				}
				else
				{
					connected_OK = 1;
				}
			}
			if (connected_OK)
			{
				my_log(LOG_INFO, "device connected OK");
			}
			else
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_connect_with_the_chip;
				my_log(LOG_ERR, "Opening the serial port %s something went wrong", def_selected_serial_port);
			}
		}
	}

	// read file with all of the checks needed
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		//
		// Read file
		//
		my_log(LOG_INFO, "Opening the file %s", fileName);
		file = fopen(fileName, "rb");
		if( !file)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_open_the_firmware_file;
			my_log(LOG_ERR, "unable to open the file %s", fileName);
		}
	}

	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		//
		// Get file size:
		//
		if (fseek(file, 0L, SEEK_END) != 0)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_seek_to_the_file_end;
			my_log(LOG_ERR, "unable to seek to the very end of the file");
		}
	}

	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		long int ft = ftell(file);
		byteCount = (uint32_t)ft;
		if (ft < 0)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_get_filesize;
			my_log(LOG_ERR, "unable to get the file size");
		}
		else
		{
			my_log(LOG_INFO, "Number of bytes in the file: %u", byteCount);
		}
	}
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (fseek(file, 0L, SEEK_SET) != 0)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_seek_where_the_file_begins;
			my_log(LOG_ERR, "unable to seek to the begin of the file");
		}
	}
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		//
		// Read data
		//
		pvWrite = (char*)realloc(pvWrite, byteCount);
		if (!pvWrite)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_allocate_the_read_buffer;
			my_log(LOG_ERR, "unable to allocate the read buffer");
		}
	}
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (fread((char*)&pvWrite[0], byteCount, 1, file) != 1)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_read_the_whole_file;
			my_log(LOG_ERR, "unable to read the whole file");
		}
	}
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (fclose(file) != 0)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_close_the_file;
			my_log(LOG_ERR, "unable to close the file");
		}
		file = NULL;
	}
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (byteCount <= sizeof(type_ASACZ_CC2650_fw_update_header))
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_filesize_too_small;
			my_log(LOG_ERR, "the file size is too small to contain both header and body");
		}
	}

	type_ASACZ_CC2650_fw_update_header * p_header = (type_ASACZ_CC2650_fw_update_header *)&pvWrite[0];
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		uint32_t calc_crc_header = calcCrcLikeChip((unsigned char *)p_header, sizeof(type_ASACZ_CC2650_fw_update_header) - sizeof(p_header->header_CRC32_CC2650));
		if (calc_crc_header != p_header->header_CRC32_CC2650)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_invalid_header_CRC32;
			my_log(LOG_ERR, "header has invalid header CRC : calculated is 0x%08X, header has 0x%08X", calc_crc_header, p_header->header_CRC32_CC2650);
		}
		else
		{
			my_log(LOG_INFO, "magic key  %s"	, p_header->magic_name);
			my_log(LOG_INFO, "ASCII ver. %s"	, p_header->ascii_version_number);
			my_log(LOG_INFO, "date       %s"	, p_header->date);
			my_log(LOG_INFO, "fw version %u.%u.%u"	, p_header->fw_version_major, p_header->fw_version_middle, p_header->fw_version_minor);
			my_log(LOG_INFO, "body size  %u"		, p_header->firmware_body_size);
			my_log(LOG_INFO, "BODY CRC   0x%08X"	, p_header->firmware_body_CRC32_CC2650);
			my_log(LOG_INFO, "header CRC 0x%08X"	, p_header->header_CRC32_CC2650);
		}
	}

	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (strcmp((const char*)p_header->magic_name, (const char*)def_magic_name_ASACZ_CC2650_fw_update_header))
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_invalid_header_magic_key;
			my_log(LOG_ERR, "header has invalid magic name %s", (const char*)p_header->magic_name);
		}
	}

	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (p_header->firmware_body_size + sizeof(type_ASACZ_CC2650_fw_update_header) != byteCount)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_invalid_body_size;
			my_log(LOG_ERR, "header has invalid body size; expected %u, read %u", p_header->firmware_body_size, (unsigned int)(byteCount - sizeof(type_ASACZ_CC2650_fw_update_header)));
		}
	}

	unsigned char * p_firmware_body = (unsigned char *)&pvWrite[0];
	uint32_t body_size = 0;
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		body_size = p_header->firmware_body_size;
		p_firmware_body = (unsigned char *)&pvWrite[0] + sizeof(type_ASACZ_CC2650_fw_update_header);
		//
		// Calculate file CRC checksum
		//
		my_log(LOG_ERR, "about to calculate the CRC");
		fileCrc = calcCrcLikeChip(p_firmware_body, body_size);
		my_log(LOG_ERR, "the CRC is 0x%X", fileCrc);
	}

	// check if valid
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		if (fileCrc != p_header->firmware_body_CRC32_CC2650)
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_invalid_body_CRC;
			my_log(LOG_ERR, "header has invalid body CRC; header has %u, calculated is %u", p_header->firmware_body_CRC32_CC2650, fileCrc);
		}
	}
	if (p_dst_header)
	{
		memset(p_dst_header, 0, sizeof(*p_dst_header));
		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			memcpy(p_dst_header, p_header, sizeof(*p_dst_header));
		}
	}

	if (do_fw_update)
	{

		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{

			//
			// Erasing as much flash needed to program firmware.
			//
			my_log(LOG_INFO, "erasing the flash, body_size = %u...", body_size);
			if(pDevice->eraseFlashRange(devFlashBase, body_size) != SBL_SUCCESS)
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_erase_the_flash;
				my_log(LOG_ERR, "erasing the flash something went wrong");
			}
		}

		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			//
			// Writing file to device flash memory.
			//
			my_log(LOG_INFO, "writing the flash...");
			if(pDevice->writeFlashRange(devFlashBase, body_size, (const char *)p_firmware_body) != SBL_SUCCESS)
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_writing_flash;
				my_log(LOG_ERR, "writing the flash something went wrong");
			}
		}
	
		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			//
			// Calculate CRC checksum of flashed content.
			//
			my_log(LOG_INFO, "Calculating the CRC...");
			if(pDevice->calculateCrc32(devFlashBase, body_size, &devCrc) != SBL_SUCCESS)
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_CRC_calculating_gone_bad;
				my_log(LOG_ERR, "Calculating the CRC something went wrong");

			}
		}

		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			//
			// Compare CRC checksums
			//
			if(fileCrc == devCrc)
			{
				my_log(LOG_INFO, "The CRC compares OK");
			}
			else
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_CRC_checking_gone_bad;
				my_log(LOG_ERR, "The CRC are different: file is 0x%X, device gives 0x%X", fileCrc, devCrc);
			}
		}
		if (r == enum_do_CC2650_fw_update_retcode_OK)
		{
			if(pDevice->reset() != SBL_SUCCESS)
			{
				r = enum_do_CC2650_fw_update_retcode_ERR_chip_reset_gone_bad;
				my_log(LOG_ERR, "ERR_chip_reset_gone_bad");
			}
		}

		pDevice->CloseSerialPort();

		// removes the reset from the CC2650
		if (!is_OK_do_CC2650_reset(0))
		{
			r = enum_do_CC2650_fw_update_retcode_ERR_unable_to_reset_the_chip_in_normal_mode;
			my_log(LOG_ERR, "ERR_unable_to_reset_the_chip_in_normal_mode");
		}
	}
	if (pvWrite)
	{
		free(pvWrite);
		pvWrite = NULL;
	}
	if (file)
	{
		fclose(file);
		file = NULL;
	}
	if (r == enum_do_CC2650_fw_update_retcode_OK)
	{
		my_log(LOG_INFO, "Procedure ends OK");
	}
	else
	{
		my_log(LOG_ERR, "Procedure ends with error: %u", (uint32_t)r);
	}
	my_log(LOG_INFO, "%s - ends", __func__);
	
	return r;
}

#endif


#ifndef OLINUXINO_LIB
// Syntax:
// <input binary firmware file path> <firmware type> <major firmware version number> <middle...> <minor...>
// <input binary firmware file path> is the pathname of the binary input file
// <firmware type> is in {"COORDINATOR","ROUTER","END_DEVICE"}
// <major>, <middle> and <minor> must be numbers between 0 and 255
// e.g. calling with the arguments /home/michele/workspace/CC2650_fw_update/versions/znp_coordinator_pro_secure_linkkeyjoin_2_6_5.bin COORDINATOR 2 6 5
// creates an output file named "ASACZ_CC2650fw_<firmware type>.<major>_<middle>_<minor> that contains
// the filename will be ASACZ_CC2650fw_COORDINATOR.2_6_5
// an header filled with all of the informations (magic key, CRC etc) needed to do a safe update of a CC2650 firmware
// for ASACZ application

typedef enum
{
	enum_ASACZ_CC2650fw_retcode_OK = 0,
	enum_ASACZ_CC2650fw_retcode_ERR_incorrect_number_of_arguments,
	enum_ASACZ_CC2650fw_retcode_ERR_unknown_fw_type,
	enum_ASACZ_CC2650fw_retcode_ERR_incorrect_firmware_version_number_major,
	enum_ASACZ_CC2650fw_retcode_ERR_incorrect_firmware_version_number_middle,
	enum_ASACZ_CC2650fw_retcode_ERR_incorrect_firmware_version_number_minor,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_open_input_file,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_seek_end_input_file,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_get_input_file_size,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_seek_begin_input_file,
	enum_ASACZ_CC2650fw_retcode_ERR_invalid_file_size,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_alloc_input_file_body_buffer,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_read_input_file_body,
	enum_ASACZ_CC2650fw_retcode_ERR_close_input_file,
	enum_ASACZ_CC2650fw_retcode_ERR_magic_name_too_long,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_ascii_fw_type,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_ascii_version_number,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_date,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_output_filename,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_open_output_file,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_write_header,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_write_body,
	enum_ASACZ_CC2650fw_retcode_ERR_unable_to_close_output_file,

	enum_ASACZ_CC2650fw_retcode_numof
}enum_ASACZ_CC2650fw_retcode;

// recognized firmware types, please put printable plain normal characters here, avoid blanks etc
const char *fw_types_OK[]=
{
	"COORDINATOR","ROUTER","END_DEVICE"
};

int main(int argc, char *argv[])
{
	enum_ASACZ_CC2650fw_retcode r = enum_ASACZ_CC2650fw_retcode_OK;
	char *filename_in = NULL;
	char *fw_type_in = NULL;
	uint32_t fw_type = 0;
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		if (argc < 6)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_incorrect_number_of_arguments;
		}
	}
	uint8_t fw_numbers[3];
	memset(fw_numbers, 0, sizeof(fw_numbers));
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		filename_in = argv[1];
		printf("The input filename is %s\n", filename_in);
		fw_type_in = argv[2];
		unsigned int i;
		unsigned int found_OK = 0;
		for (i = 0; !found_OK && i < sizeof(fw_types_OK)/sizeof(fw_types_OK[0]); i++)
		{
			if (strcmp(fw_types_OK[i], fw_type_in) == 0)
			{
				fw_type = i;
				found_OK = 1;
				printf("The firmware type is %u (%s)\n", fw_type, fw_types_OK[fw_type]);
				break;
			}
		}
		if (!found_OK)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unknown_fw_type;
		}
	}

	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		unsigned int i;
		for (i = 0; i < 3; i++)
		{
			char * endptr;
			long int n = strtol(argv[3 + i], &endptr, 0);
			if ((*endptr != 0) || (n < 0) || (n > 255))
			{
				r = (enum_ASACZ_CC2650fw_retcode)(enum_ASACZ_CC2650fw_retcode_ERR_incorrect_firmware_version_number_major + i);
				break;
			}
			fw_numbers[i] = n;
		}
		if (r == enum_ASACZ_CC2650fw_retcode_OK)
		{
			printf("The firmware version is %u.%u.%u\n", fw_numbers[0], fw_numbers[1], fw_numbers[2]);
		}
	}
	FILE *fin = NULL;
	FILE *fout = NULL;
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		fin = fopen(filename_in, "rb");
		if (!fin)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_open_input_file;
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		int result = fseek(fin, 0L, SEEK_END);
		if (result)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_seek_end_input_file;
		}
	}
	long l_filesize = 0;
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		l_filesize = ftell(fin);
		if (l_filesize < 0)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_get_input_file_size;
		}
		else if (l_filesize > 128 * 1024)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_invalid_file_size;
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		int result = fseek(fin, 0L, SEEK_SET);
		if (result)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_seek_begin_input_file;
		}
	}
	uint8_t * p_bin_file_body = NULL;
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		p_bin_file_body = (uint8_t *)malloc(l_filesize);
		if (!p_bin_file_body)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_alloc_input_file_body_buffer;
		}
		else
		{
			memset(p_bin_file_body, 0, l_filesize);
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		size_t n_read = fread(p_bin_file_body, l_filesize, 1, fin);
		if (n_read != 1)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_read_input_file_body;
		}
	}
	if (fin)
	{
		if (fclose(fin))
		{
			if (r == enum_ASACZ_CC2650fw_retcode_OK)
			{
				r = enum_ASACZ_CC2650fw_retcode_ERR_close_input_file;
			}
		}
	}
	type_ASACZ_CC2650_fw_update_header ASACZ_CC2650_fw_update_header = {0};
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		int n_needed = snprintf((char*)ASACZ_CC2650_fw_update_header.magic_name, sizeof(ASACZ_CC2650_fw_update_header.magic_name), "%s", def_magic_name_ASACZ_CC2650_fw_update_header);
		if (n_needed >= (int)sizeof(ASACZ_CC2650_fw_update_header.magic_name))
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_magic_name_too_long;
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		ASACZ_CC2650_fw_update_header.fw_type = fw_type;
		int n = snprintf((char *)ASACZ_CC2650_fw_update_header.ascii_fw_type, sizeof(ASACZ_CC2650_fw_update_header.ascii_fw_type), "%s",
				fw_types_OK[ASACZ_CC2650_fw_update_header.fw_type]
				);
		if (n >= (int)sizeof(ASACZ_CC2650_fw_update_header.ascii_fw_type))
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_ascii_fw_type;
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		ASACZ_CC2650_fw_update_header.firmware_body_size = l_filesize;
		ASACZ_CC2650_fw_update_header.fw_version_major = fw_numbers[0];
		ASACZ_CC2650_fw_update_header.fw_version_middle = fw_numbers[1];
		ASACZ_CC2650_fw_update_header.fw_version_minor = fw_numbers[2];
		ASACZ_CC2650_fw_update_header.firmware_body_CRC32_CC2650 = calcCrcLikeChip((const unsigned char *)p_bin_file_body, ASACZ_CC2650_fw_update_header.firmware_body_size);

		time_t rawtime;
		struct tm * timeinfo;

		time (&rawtime);
		timeinfo = localtime (&rawtime);
		int n = snprintf((char *)ASACZ_CC2650_fw_update_header.ascii_version_number, sizeof(ASACZ_CC2650_fw_update_header.ascii_version_number), "%u.%u.%u",
				ASACZ_CC2650_fw_update_header.fw_version_major,
				ASACZ_CC2650_fw_update_header.fw_version_middle,
				ASACZ_CC2650_fw_update_header.fw_version_minor
				);
		if (n >= (int)sizeof(ASACZ_CC2650_fw_update_header.ascii_version_number))
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_ascii_version_number;
		}
		else if (!strftime ((char *)ASACZ_CC2650_fw_update_header.date, sizeof(ASACZ_CC2650_fw_update_header.date), "%Y %b %d",timeinfo))
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_date;
		}
		else
		{
			printf("Header info:\n");
			printf("\t magic key  %s\n"	, ASACZ_CC2650_fw_update_header.magic_name);
			printf("\t ASCII fw   %s\n"	, ASACZ_CC2650_fw_update_header.ascii_fw_type);
			printf("\t ASCII ver. %s\n"	, ASACZ_CC2650_fw_update_header.ascii_version_number);
			printf("\t ASCII date %s\n"	, ASACZ_CC2650_fw_update_header.date);
			printf("\t fw type    %u\n"	, ASACZ_CC2650_fw_update_header.fw_type);
			printf("\t fw version %u.%u.%u\n"	, ASACZ_CC2650_fw_update_header.fw_version_major, ASACZ_CC2650_fw_update_header.fw_version_middle, ASACZ_CC2650_fw_update_header.fw_version_minor);
			printf("\t body size  %u\n"			, ASACZ_CC2650_fw_update_header.firmware_body_size);
			printf("\t body CRC   0x%08X\n"		, ASACZ_CC2650_fw_update_header.firmware_body_CRC32_CC2650);
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		int header_CRC_size = sizeof(ASACZ_CC2650_fw_update_header) - sizeof(ASACZ_CC2650_fw_update_header.header_CRC32_CC2650);
		ASACZ_CC2650_fw_update_header.header_CRC32_CC2650 = calcCrcLikeChip((const unsigned char *)&ASACZ_CC2650_fw_update_header, header_CRC_size);
		printf("\t header CRC 0x%08X\n"	, ASACZ_CC2650_fw_update_header.header_CRC32_CC2650);
	}
	char outfile_name[1024];
	memset(outfile_name, 0, sizeof(outfile_name));
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		int n = snprintf(outfile_name, sizeof(outfile_name), "ASACZ_CC2650fw_%s.%u_%u_%u"
				, ASACZ_CC2650_fw_update_header.ascii_fw_type
				, ASACZ_CC2650_fw_update_header.fw_version_major
				, ASACZ_CC2650_fw_update_header.fw_version_middle
				, ASACZ_CC2650_fw_update_header.fw_version_minor
				);
		if (n >= (int)sizeof(outfile_name))
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_sprint_output_filename;
		}
		else
		{
			fout = fopen(outfile_name, "wb");
			if (!fout)
			{
				r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_open_output_file;
			}
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		size_t n = fwrite(&ASACZ_CC2650_fw_update_header, sizeof(ASACZ_CC2650_fw_update_header), 1, fout);
		if (n != 1)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_write_header;
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		size_t n = fwrite(p_bin_file_body, l_filesize, 1, fout);
		if (n != 1)
		{
			r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_write_body;
		}
	}
	if (fout)
	{
		if (fclose(fout))
		{
			if (r == enum_ASACZ_CC2650fw_retcode_OK)
			{
				r = enum_ASACZ_CC2650fw_retcode_ERR_unable_to_close_output_file;
			}
		}
	}
	if (r == enum_ASACZ_CC2650fw_retcode_OK)
	{
		printf("Output generated OK to %s\n", outfile_name);
	}
	else
	{
		printf("Error %u\n", (uint32_t)r);
		printf("Syntax: %s <binary file> <firmware type> <major> <middle> <minor>\n", argv[0] );
		printf("<binary file> must contain the firmware");
		printf("\t <firmware type> is in ");
		{
			unsigned int i;
			for (i = 0; i < sizeof(fw_types_OK)/sizeof(fw_types_OK[0]); i++)
			{
				printf("%s ", fw_types_OK[i]);
			}
		}
		printf("\n");
		printf("<major> <middle> <minor> must be between 0 and 255");
	}


	return r;
}
#endif
