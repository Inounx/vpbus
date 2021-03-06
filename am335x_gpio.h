/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#ifndef AM335X_GPIO_H
#define AM335X_GPIO_H

//Adresses des blocs de registres des GPIO
#define GPIO0_BASE_ADDR 0x44E07000
#define GPIO1_BASE_ADDR 0x4804C000
#define GPIO2_BASE_ADDR 0x481AC000
#define GPIO3_BASE_ADDR 0x481AE000
#define GPIO_BLOCK_SIZE 0x1000

//Definitions des offsets d'accès pour les registres d'un block de GPIO
#define GPIO_REVISION           0x00
#define GPIO_SYSCONFIG          0x10
#define GPIO_EOI                0x20
#define GPIO_IRQSTATUS_RAW_0    0x24
#define GPIO_IRQSTATUS_RAW_1    0x28
#define GPIO_IRQSTATUS_0        0x2C
#define GPIO_IRQSTATUS_1        0x30
#define GPIO_IRQSTATUS_SET_0    0x34
#define GPIO_IRQSTATUS_SET_1    0x38
#define GPIO_IRQSTATUS_CLR_0    0x3C
#define GPIO_IRQSTATUS_CLR_1    0x40
#define GPIO_IRQWAKEN_0         0x44
#define GPIO_IRQWAKEN_1         0x48
#define GPIO_SYSSTATUS          0x114
#define GPIO_CTRL               0x130
#define GPIO_OE                 0x134
#define GPIO_DATAIN             0x138
#define GPIO_DATAOUT            0x13C
#define GPIO_LEVELDETECT_0      0x140
#define GPIO_LEVELDETECT_1      0x144
#define GPIO_RISINGDETECT       0x148
#define GPIO_FALLINGDETECT      0x14C
#define GPIO_DEBOUCENABLE       0x150
#define GPIO_DEBOUCINGTIME      0x154
#define GPIO_CLEARDATAOUT       0x190
#define GPIO_SETDATAOUT         0x194

#endif
