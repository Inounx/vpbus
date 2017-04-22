/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#ifndef AM335X_CONTROL_H
#define AM335X_CONTROL_H

//Adresses des blocs de registres des GPIO
#define CONTROL_MODULE_BASE_ADDR 0x44E10000
#define CONTROL_MODULE_BLOCK_SIZE 0x2000

//Definition des offsets d'accès pour les registres du block control modules
#define PAD_CONTROL_SPI0_SCLK           0x950
#define PAD_CONTROL_SPI0_D0             0x954
#define PAD_CONTROL_SPI0_D1             0x958
#define PAD_CONTROL_SPI0_CS0            0x95C
#define PAD_CONTROL_ECAP0_IN_PWM0_OUT   0x964
#define PAD_CONTROL_UART1_CTSn          0x978
#define PAD_CONTROL_UART1_RTSn          0x97C
#define PAD_CONTROL_UART1_RXD           0x980
#define PAD_CONTROL_UART1_TXD           0x984
#define PAD_CONTROL_XDMA_EVENT_INTR1    0x9B4
#define PAD_CONTROL_GPMC_AD12           0x830
#define PAD_CONTROL_GPMC_AD13           0x834
#define PAD_CONTROL_GPMC_AD14           0x838
#define PAD_CONTROL_GPMC_AD15           0x83C
#define PAD_CONTROL_GPMC_A0             0x840
#define PAD_CONTROL_GPMC_A1             0x844
#define PAD_CONTROL_GPMC_A2             0x848
#define PAD_CONTROL_GPMC_A3             0x84C
#define PAD_CONTROL_MCASP0_ACLKX        0x990
#define PAD_CONTROL_MCASP0_FSX          0x994
#define PAD_CONTROL_MCASP0_AXR0         0x998
#define PAD_CONTROL_MCASP0_AHCLKR       0x99C
#define PAD_CONTROL_MCASP0_ACLKR        0x9A0
#define PAD_CONTROL_MCASP0_FSR          0x9A4
#define PAD_CONTROL_MCASP0_AXR1         0x9A8
#define PAD_CONTROL_MCASP0_AHCLKX       0x9AC
#define PAD_CONTROL_GPMC_WAIT0          0x870
#define PAD_CONTROL_GPMC_WPN            0x874


//Définitions valeurs du registre PAD_CONTROL
#define PAD_CONTROL_MUX_MODE_0 0x00
#define PAD_CONTROL_MUX_MODE_1 0x01
#define PAD_CONTROL_MUX_MODE_2 0x02
#define PAD_CONTROL_MUX_MODE_3 0x03
#define PAD_CONTROL_MUX_MODE_4 0x04
#define PAD_CONTROL_MUX_MODE_5 0x05
#define PAD_CONTROL_MUX_MODE_6 0x06
#define PAD_CONTROL_MUX_MODE_7 0x07

#define PAD_CONTROL_PULL_ENABLE_MASK 0x08
#define PAD_CONTROL_PULL_ENABLE 0x00
#define PAD_CONTROL_PULL_DISABLE PAD_CONTROL_PULL_ENABLE_MASK

#define PAD_CONTROL_PULL_TYPE_MASK 0x10
#define PAD_CONTROL_PULL_DOWN 0x00
#define PAD_CONTROL_PULL_UP PAD_CONTROL_PULL_TYPE_MASK

#define PAD_CONTROL_RX_ACTIVE_MASK 0x20
#define PAD_CONTROL_RX_ACTIVE PAD_CONTROL_RX_ACTIVE_MASK
#define PAD_CONTROL_RX_DISABLED

#define PAD_CONTROL_SLEW_CTRL_MASK 0x40
#define PAD_CONTROL_SLEW_SLOW PAD_CONTROL_SLEW_CTRL_MASK
#define PAD_CONTROL_SLEW_FAST 0x00

#endif // AM335X_CONTROL_H
