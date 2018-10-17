/**
 * YDLidar X4 driver
 * @brief Here we implement the driver to use the YD Lidar X4.
 */

#ifndef _LIDAR_X4_H	// ifndef _LIDAR_X4_  
#define _LIDAR_X4_H

#include "stdint.h"

/* Start scanning and output point cloud data */
#define START_SCAN 		0x60
/* Stop scanning */
#define STOP_SCAN			0x65
/* Get device information (model, firmware, hardawre conversion) */
#define DEVICE_INFO 	0x901
/* Get device health status */
#define DEVICE_STATUS 0x91
/* Soft reboot */
#define SOFT_REBOOT 	0x40

/**
 * Maximal size of data comming from the lidar, use to reserve 
 * an array to fill with data 
 */
#define LIDAR_MAX_MSG_SIZE 40

/**
 * Initalize the communication with the lidar 
 * by configuring the pin B10 and B11 and the 
 * usart3.
 * RX B11 green
 * TX B10 yellow
 */
void lidarInit(void);

/**
 * Send a command to the lidar.
 * For the moment command are defined with define but 
 * it's better to use an enum I think.
 * See above for the command you can send.
 */
void lidarSend(uint16_t command);

/**
 * 
 */
void lidarReadPacket(char* data);

#endif	// end _LIDAR_X4_
