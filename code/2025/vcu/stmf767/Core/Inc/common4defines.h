/*
 * common4defines.h
 *
 *  Created on: May 10, 2025
 *      Author: kadir
 */

#ifndef INC_COMMON4DEFINES_H_
#define INC_COMMON4DEFINES_H_

#include "stdint.h"



/* Değerler "Byte" cinsinden verilmiştir */

//+1 CRC8 için, yani 64 byte CRC ile
#define SPI_DATA_SIZE 63

/* Sürüş verileri uzunluğu */
#define DRIVETRAIN_COMMAND_SIZE 9
#define DRIVETRAIN_FEEDBACK_SIZE 12


/* Robot kol verileri uzunluğu */
#define ARM_COMMAND_SIZE 6
#define ARM_FEEDBACK_SIZE 12

/* Bilim komut verileri uzunluğu */
#define SCIENCE_COMMAND_SIZE 4
#define SCIENCE_FEEDBACK_SIZE 39


/* Küresel Uydu Konumlandırma Sistemi verisi uzunluğu */
#define GNSS_SIZE 16

/* UDP Haberleşme Paketi uzunlukları */
// Payload değişken
#define processIdSize 2
#define timeStampSize 2



#endif /* INC_COMMON4DEFINES_H_ */
