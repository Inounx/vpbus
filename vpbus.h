/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#ifndef VPBUS_H
#define VPBUS_H

//Option de profilage du bus
//Affiche à chaque accès le temps et la bande passante
//#define PROFILE

//A activer pour n'autoriser que les accès alignés sur le bus
#define WORD_ADDRESSING_ONLY

//======================================================================
//Définitions non modifiables
#define BUS_SIZE (65536*2)
#define MAX_ADDRESS (BUS_SIZE - 2)

#define GPIO_ID(port, pin) (32 * port + pin)

//ALE P2.2
//Read P2.3
//Write P2.4
#define ALE_PIN_INDEX 2
#define READ_PIN_INDEX 3
#define WRITE_PIN_INDEX 4

#define P2PIN GPIO_ID(2, ALE_PIN_INDEX)

//AD0 P1.12
//AD1 P1.13
//AD2 P1.14
//AD3 P1.15
//AD4 P1.16
//AD5 P1.17
//AD6 P1.18
//AD7 P1.19
//AD8 P3.14
//AD9 P3.15
//AD10 P3.16
//AD11 P3.17
//AD12 P3.18
//AD13 P3.19
//AD14 P3.20
//AD15 P3.21
#define AD0_PIN_INDEX 12
#define AD1_PIN_INDEX 13
#define AD2_PIN_INDEX 14
#define AD3_PIN_INDEX 15
#define AD4_PIN_INDEX 16
#define AD5_PIN_INDEX 17
#define AD6_PIN_INDEX 18
#define AD7_PIN_INDEX 19
#define GPIO1_AD_PIN_MASK (0xFFuL << AD0_PIN_INDEX)
#define P1PIN GPIO_ID(1, AD0_PIN_INDEX)

#define AD8_PIN_INDEX 14
#define AD9_PIN_INDEX 15
#define AD10_PIN_INDEX 16
#define AD11_PIN_INDEX 17
#define AD12_PIN_INDEX 18
#define AD13_PIN_INDEX 19
#define AD14_PIN_INDEX 20
#define AD15_PIN_INDEX 21
#define GPIO3_AD_PIN_MASK (0xFFuL << AD8_PIN_INDEX)
#define P3PIN GPIO_ID(3, AD8_PIN_INDEX)


#endif //VPBUS_H
