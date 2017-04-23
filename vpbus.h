/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#ifndef VPBUS_H
#define VPBUS_H

#define GPIO_ID(port, pin) (32 * port + pin)

#define MAX_BUS_ADDRESS 0x1FF

//======================================================================
//Définitions modifiables en fonction du bus souhaité
//A0 P0.2
#define GPIO_A0 GPIO_ID(0, 2)

//A1 P0.3
#define GPIO_A1 GPIO_ID(0, 3)

//A2 P0.4
#define GPIO_A2 GPIO_ID(0, 4)

//A3 P0.5
#define GPIO_A3 GPIO_ID(0, 5)

//A4 P0.12
#define GPIO_A4 GPIO_ID(0, 12)

//A5 P0.13
#define GPIO_A5 GPIO_ID(0, 13)

//A6 P0.14
#define GPIO_A6 GPIO_ID(0, 14)

//A7 P0.15
#define GPIO_A7 GPIO_ID(0, 15)

//Read P0.30
//Write P0.31
#define GPIO_READ GPIO_ID(0, 30)
#define GPIO_WRITE GPIO_ID(0, 31)

//D0 P1.12
//D1 P1.13
//D2 P1.14
//D3 P1.15
//D4 P1.16
//D5 P1.17
//D6 P1.18
//D7 P1.19
//D8 P3.14
//D9 P3.15
//D10 P3.16
//D11 P3.17
//D12 P3.18
//D13 P3.19
//D14 P3.20
//D15 P3.21
#define GPIO_D0 GPIO_ID(1, 12)
#define GPIO_D1 GPIO_ID(1, 13)
#define GPIO_D2 GPIO_ID(1, 14)
#define GPIO_D3 GPIO_ID(1, 15)
#define GPIO_D4 GPIO_ID(1, 16)
#define GPIO_D5 GPIO_ID(1, 17)
#define GPIO_D6 GPIO_ID(1, 18)
#define GPIO_D7 GPIO_ID(1, 19)


#define GPIO_D8 GPIO_ID(3, 14)
#define GPIO_D9 GPIO_ID(3, 15)
#define GPIO_D10 GPIO_ID(3, 16)
#define GPIO_D11 GPIO_ID(3, 17)
#define GPIO_D12 GPIO_ID(3, 18)
#define GPIO_D13 GPIO_ID(3, 19)
#define GPIO_D14 GPIO_ID(3, 20)
#define GPIO_D15 GPIO_ID(3, 21)


#endif //VPBUS_H
