/*
 * CC2650_fw_update.h
 *
 *  Created on: Feb 27, 2017
 *      Author: michele
 */

#ifndef INC_CC2650_FW_UPDATE_H_
#define INC_CC2650_FW_UPDATE_H_

#define def_magic_name_ASACZ_CC2650_fw_update_header "ASACZ_CC2650_fw_update_header"
typedef struct _type_ASACZ_CC2650_fw_update_header
{
	uint8_t magic_name[32];				// must be set to "ASACZ_CC2650_fw_update_header", and filled up with 0x00 in the remaining bytes
	uint8_t ascii_fw_type[32];			// must be set to "COORDINATOR" or "ROUTER" or "END_DEVICE"
	uint8_t ascii_version_number[32];	// must have the format "<major>.<middle>.<minor>"
	uint8_t date[32];					// must be set to "YYYY mmm dd"
	uint32_t fw_type;					// must be set to 0 for COORDINATOR, 1 for ROUTER, 2 for END_DEVICE
	uint32_t fw_version_major;			// the firmware version major number
	uint32_t fw_version_middle;			// the firmware version middle number
	uint32_t fw_version_minor;			// the firmware version minor number
	uint32_t firmware_body_size;		// the expected number of bytes in the firmware body, most of the times it should be 131072, i.e. 128 kBytes
	uint32_t firmware_body_CRC32_CC2650;// the CRC32 of the firmware body calculated as CC2650 does it, please see the calcCrcLikeChip routine
	uint32_t header_CRC32_CC2650;		// the CRC32 of the header (this field excluded), calculated as CC2650 does it, please see the calcCrcLikeChip routine

}__attribute__((__packed__)) type_ASACZ_CC2650_fw_update_header;

typedef enum
{
	enum_do_CC2650_fw_update_retcode_OK = 0,								// firmware updated OK
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_reset_chip,				// unable to reset the CC2650 chip
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_create_SBL_object, 		// unable to create serial boot-loader library object
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_connect_with_the_chip,	// unable to connect with the chip
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_open_the_firmware_file, 	//
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_erase_the_flash,			// unable to erase the chips internal flash
	enum_do_CC2650_fw_update_retcode_ERR_writing_flash,						//
	enum_do_CC2650_fw_update_retcode_ERR_CRC_calculating_gone_bad,			//
	enum_do_CC2650_fw_update_retcode_ERR_CRC_checking_gone_bad,				//
	enum_do_CC2650_fw_update_retcode_ERR_chip_reset_gone_bad,				//
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_seek_to_the_file_end,	//
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_get_filesize,			//
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_seek_where_the_file_begins, //
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_allocate_the_read_buffer,
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_read_the_whole_file,
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_close_the_file,
	enum_do_CC2650_fw_update_retcode_ERR_filesize_too_small,
	enum_do_CC2650_fw_update_retcode_ERR_invalid_header_CRC32,
	enum_do_CC2650_fw_update_retcode_ERR_invalid_header_magic_key,
	enum_do_CC2650_fw_update_retcode_ERR_invalid_body_size,
	enum_do_CC2650_fw_update_retcode_ERR_invalid_body_CRC,
	enum_do_CC2650_fw_update_retcode_ERR_unable_to_reset_the_chip_in_normal_mode,
	enum_do_CC2650_fw_update_retcode_ERR_invalid_operation_requested,

	enum_do_CC2650_fw_update_retcode_numof
}enum_do_CC2650_fw_update_retcode;

typedef enum
{
	enum_CC2650_fw_operation_update_firmware = 0,
	enum_CC2650_fw_operation_get_file_header_info,
	enum_CC2650_fw_operation_numof
}enum_CC2650_fw_operation;


#ifdef OLINUXINO_LIB
extern "C"
#endif
enum_do_CC2650_fw_update_retcode do_CC2650_fw_operation(enum_CC2650_fw_operation op, const char *path_binary_file, type_ASACZ_CC2650_fw_update_header *p_dst_header);

#ifdef OLINUXINO_LIB
extern "C"
#endif
const char * get_msg_from_CC2650_fw_update_retcode(enum_do_CC2650_fw_update_retcode r);

#ifdef OLINUXINO_LIB
extern "C"
#endif
uint32_t get_CC2650_fw_update_progress(void);
#endif /* INC_CC2650_FW_UPDATE_H_ */
