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


#include <vector>
#include <iostream>
#include <fstream>


#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>
#include <stdlib.h>
#include <time.h>
#include <sys/timeb.h>  /* ftime, timeb (for timestamp in millisecond) */
#include <sys/time.h>   /* gettimeofday, timeval (for timestamp in microsecond) */

#ifdef ANDROID
	#define def_selected_serial_port "/dev/ttymxc2"
#else
	#ifdef OLINUXINO
			#define def_selected_serial_port "/dev/ttyS1"
	#else
			#define def_selected_serial_port "/dev/ttyUSB1"
	#endif
#endif

static void open_syslog(void)
{
// this goes straight to /var/log/syslog file
//	Nov  3 12:16:53 localhost ASAC_Zlog[13149]: Log just started
//	Nov  3 12:16:53 localhost ASAC_Zlog[13149]: The application starts

	openlog("CC2650_fw_updlog", LOG_PID|LOG_CONS, LOG_DAEMON);
	syslog(LOG_INFO, "Log just started");
}

#include <sys/stat.h>
#include <fcntl.h>
int is_OK_do_CC2650_reset(unsigned int enable_boot_mode)
{
	int is_OK = 1;
#define gpio_base_path "/sys/class/gpio"
#define gpio_export_path gpio_base_path"/export"
#define gpio_value_path gpio_base_path"/value"

	syslog(LOG_INFO, "+%s\n", __func__);
	int fd_reset_low = -1;
	int fd_boot_enable_high = -1;
	char fname_reset[128];
	char fname_boot_enable[128];
	if (snprintf(fname_boot_enable, sizeof(fname_boot_enable), "%s/gpio0/value", gpio_base_path) < 0)
	{
		syslog(LOG_ERR, "ERROR doing snprintf %s\n", fname_boot_enable);
		is_OK = 0;
	}
	else if (snprintf(fname_reset, sizeof(fname_reset), "%s/gpio12/value", gpio_base_path) < 0)
	{
		syslog(LOG_ERR, "ERROR doing snprintf reset %s\n", fname_reset);
		is_OK = 0;
	}
	else
	{
		fd_reset_low = open((char *)fname_reset, O_WRONLY);
		fd_boot_enable_high = open((char *)fname_boot_enable, O_WRONLY);
		if (fd_reset_low < 0)
		{
			syslog(LOG_ERR, "ERROR doing open reset gpio %s", fname_reset);
			is_OK = 0;
		}
		else if (fd_boot_enable_high < 0)
		{
			syslog(LOG_ERR, "ERROR doing open boot enable gpio %s", fname_boot_enable);
			is_OK = 0;
		}
		else
		{
			typedef struct _type_cycle_op
			{
				unsigned int gpio_reset12_boot_0;
				unsigned int value;
				unsigned int delay_ms;
			}type_cycle_op;
			type_cycle_op cycle_op[3];

			// reset low (Active) then wait 100ms
			cycle_op[0].gpio_reset12_boot_0 = 12;
			cycle_op[0].value = 0;
			cycle_op[0].delay_ms = 100;

			// boot enable high (Active)/ low (NOT active) then wait 200ms
			cycle_op[1].gpio_reset12_boot_0 = 0;
			cycle_op[1].value = enable_boot_mode ? 1 : 0;
			cycle_op[1].delay_ms = 200;

			// reset high (NOT Active) then wait 10ms
			cycle_op[2].gpio_reset12_boot_0 = 12;
			cycle_op[2].value = 1;
			cycle_op[2].delay_ms = 10;
#if 0
			// boot enable low (NOT Active) then wait 10ms
			cycle_op[3].gpio_reset12_boot_0 = 0;
			cycle_op[3].value = 0;
			cycle_op[3].delay_ms = 10;
#endif
			unsigned int idx_loop;
			for (idx_loop = 0; is_OK && idx_loop < sizeof(cycle_op)/ sizeof(cycle_op[0]); idx_loop++)
			{
				type_cycle_op * p_cyc = &cycle_op[idx_loop];
				char val[256];
				int n= snprintf(val, sizeof(val), "%u", p_cyc->value);
				if (n > 0)
				{
					int val_len = strlen(val);
					int fd = p_cyc->gpio_reset12_boot_0 == 0 ? fd_boot_enable_high : fd_reset_low;
					int n = write(fd, val, val_len);
					if (n != val_len)
					{
						is_OK = 0;
						syslog(LOG_ERR, "ERROR doing write value %u gpio %u OK", p_cyc->value, p_cyc->gpio_reset12_boot_0);
					}
					else
					{
						syslog(LOG_INFO, "set value %u gpio %u OK", p_cyc->value, p_cyc->gpio_reset12_boot_0);
					}
				}
				else
				{
					is_OK = 0;
					syslog(LOG_ERR, "ERR doing snprintf value %u gpio %u OK", p_cyc->value, p_cyc->gpio_reset12_boot_0);
				}
			}
		}
	}

	if (fd_reset_low >= 0)
	{
		close(fd_reset_low);
	}
	if (fd_boot_enable_high >= 0)
	{
		close(fd_boot_enable_high);
	}

	syslog(LOG_INFO, "-%s returns is_OK = %u\n", __func__, is_OK);
	return	is_OK;
}

int64_t get_current_epoch_time_ms(void)
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


//
// Including functions for millisecond precision timing.
// Taken from http://stackoverflow.com/questions/2494356/how-to-use-gettimeofday-or-something-equivalent-with-visual-studio-c-2008
//
#include <time.h>


// Calculate crc32 checksum the way CC2538 and CC2650 does it.
int calcCrcLikeChip(const unsigned char *pData, unsigned long ulByteCount)
{
    unsigned long d, ind;
    unsigned long acc = 0xFFFFFFFF;
    const unsigned long ulCrcRand32Lut[] =
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

/// Application status function (used as SBL status callback)
void appStatus(char *pcText, bool bError)
{
    if(bError)
    {
    	syslog(LOG_ERR, "%s", pcText);
    }
    else
    {
    	syslog(LOG_INFO, "%s", pcText);
        //cout << pcText;
    }
}


/// Application progress function (used as SBL progress callback)
static void appProgress(uint32_t progress)
{
    fprintf(stdout, "\r%d%% ", progress);
    fflush(stdout);
}

// Time variables for calculating execution time.
static int64_t tBefore, tAfter;

extern int64_t get_current_epoch_time_ms(void);
// Start millisecond timer
static void getTime(void)
{
	tBefore = get_current_epoch_time_ms();
}

// Print time since getTime()
static void printTimeDelta(void)
{
	tAfter = get_current_epoch_time_ms();
    //printf("delta: %"PRI64 ms", (tAfter - tBefore));
}

// Defines
#define DEVICE_CC2538				0x2538
#define DEVICE_CC26XX				0x2650
#define CC2538_FLASH_BASE			0x00200000
#define CC26XX_FLASH_BASE			0x00000000

static void my_at_exit(void)
{
	// remove the reset from the CC2650
	is_OK_do_CC2650_reset(0);
}

// Application main function
int do_fw_update(void)
{
	open_syslog();

	atexit(my_at_exit);

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
	//int32_t devIdx;					//
    //ComPortElement pElements[10];	// An array for the COM ports that will be found by enumeration.
    //int32_t nElem = 10;				// Sets the number of COM ports to list by SBL.
    int32_t devStatus = -1;			// Hold SBL status codes
	std::string fileName;			// File name to program
	uint32_t byteCount = 0;			// File size in bytes
    uint32_t fileCrc, devCrc;		// Variables to save CRC checksum
	uint32_t devFlashBase;	    	// Flash start address
	static std::vector<char> pvWrite(1);// Vector to application firmware in.
	static std::ifstream file;		// File stream


	if (!is_OK_do_CC2650_reset(1))
	{
        goto error;
	}

	fileName = "/usr/znp_coordinator_pro_secure_linkkeyjoin_2_6_5.bin";
	devFlashBase = CC26XX_FLASH_BASE;

    //
    // Set callback functions
    //
    SblDevice::setCallBackStatusFunction(&appStatus);
    SblDevice::setCallBackProgressFunction(&appProgress);


    //
    // Print out header
    //
    //cout << "+-------------------------------------------------------------\n";
    //cout << "| Serial Bootloader Library Example Application for CC" << (deviceType >> 12 & 0xf) << (deviceType >> 8 & 0xf) << (deviceType >> 4 & 0xf) << (deviceType & 0xf) << "\n";
    //cout << "+-------------------------------------------------------------\n\n";
    //
    // Create SBL object
    //
    pDevice = SblDevice::Create(deviceType);
    if(pDevice == NULL) 
    { 
        //cout << "No SBL device object.\n";
        goto error;
    }

    //
    // Connect to device
    //
	syslog(LOG_INFO, "Opening the serial port %s", def_selected_serial_port);
	getTime();
    if(pDevice->connect(def_selected_serial_port, false) != SBL_SUCCESS)
    { 
    	syslog(LOG_INFO, "Opening the serial port %s soething went wrong", def_selected_serial_port);
    	goto error;
    }
	printTimeDelta();
	
    //
    // Read file
    //
	syslog(LOG_INFO, "Opening the file %s", fileName.c_str());
	file.open(fileName.c_str(), std::ios::binary);
    if(file.is_open())
    {
        //
        // Get file size:
        //
        file.seekg(0, std::ios::end);
        byteCount = (uint32_t)file.tellg();
        file.seekg(0, std::ios::beg);
    	syslog(LOG_INFO, "Number of bytes in the file: %u", byteCount);

        //
        // Read data
        //
        pvWrite.resize(byteCount);
        file.read((char*) &pvWrite[0], byteCount);
    }
    else   
    {
		//cout << "Unable to open file " << fileName.c_str();
    	syslog(LOG_ERR, "unable to open the file %s", fileName.c_str());
        goto error;
    }

    //
    // Calculate file CRC checksum
    //
    fileCrc = calcCrcLikeChip((unsigned char *)&pvWrite[0], byteCount);

	//
	// Erasing as much flash needed to program firmware.
	//
    //cout << "Erasing flash ...\n";
    getTime();
	syslog(LOG_INFO, "erasing the flash...");
    if(pDevice->eraseFlashRange(devFlashBase, byteCount) != SBL_SUCCESS)
    {
    	syslog(LOG_ERR, "erasing the flash something went wrong");
        goto error;
    }
    printTimeDelta();

	//
	// Writing file to device flash memory.
	//
    //cout << "Writing flash ...\n";
    getTime();
	syslog(LOG_INFO, "writing the flash...");
    if(pDevice->writeFlashRange(devFlashBase, byteCount, &pvWrite[0]) != SBL_SUCCESS)
    {
    	syslog(LOG_ERR, "writing the flash something went wrong");
        goto error;
    }
    printTimeDelta();

	//
	// Calculate CRC checksum of flashed content.
	//
    //cout << "Calculating CRC on device ...\n";
    getTime();
	syslog(LOG_INFO, "Calculating the CRC...");
    if(pDevice->calculateCrc32(devFlashBase, byteCount, &devCrc) != SBL_SUCCESS)
    {
    	syslog(LOG_ERR, "Calculating the CRC something went wrong");
        goto error;

    }

    printTimeDelta();

	//
	// Compare CRC checksums
	//
    //cout << "Comparing CRC ...\n";
    if(fileCrc == devCrc)
    {
    	syslog(LOG_INFO, "The CRC compares OK");
    }
    else
	{
    	syslog(LOG_ERR, "The CRC are different: file is 0x%X, device gives 0x%X", fileCrc, devCrc);
	}

    //cout << "Resetting device ...\n";
    if(pDevice->reset() != SBL_SUCCESS)
    {
        goto error;
    }
    //cout << "OK\n";

    //
    // If we got here, all succeeded. Jump to exit.
    //
    goto exit;

error:
    //cout << "\n\nAn error occurred: " << pDevice->getLastStatus();
exit:
	devStatus = 0;
	if(pDevice != NULL) {
		devStatus = pDevice->getLastStatus();
	}
	
	syslog(LOG_INFO, "The application closes");
	// at exit, close system log
	closelog();

	return devStatus;
}



int main()
{
	 return do_fw_update();
}
