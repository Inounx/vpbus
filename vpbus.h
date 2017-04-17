/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#ifndef VPBUS_H
#define VPBUS_H

//======================================================================
//Définitions modifiables en fonction du bus souhaité

//Attention dans les bits "utiles" on ne compte pas le bit 0 qui est "virtuel"
//il n'apparait pas sur le bus, c'est juste qu'on adresse des mots de 16bits
//donc on accède aux mots par des adresses paires dons le bit 0 est toujours à 0
//Le nombre de bits utiles détermine seulement le nombre de mots adressables
//Pour le ramener à l'octet il faut logiquement multiplier par 2
//***Exemple pour 8bits :
// - Nbre de mots adressables : 256
// - Addresse min: 0
// - Addresse max: 0x1FE
//***Exemple pour 4bits :
// - Nbre de mots adressables : 16
// - Addresse min: 0
// - Addresse max: 0x1E
#define USABLE_ADDRESS_BITS 8

//======================================================================
//Définitions non modifiables
#define MAX_ADDRESS_BITS 8

#if USABLE_ADDRESS_BITS > MAX_ADDRESS_BITS
#error "USABLE_ADDRESS_BITS cannot be greter than MAX_ADDRESS_BITS!"
#else
#define MAX_BUS_ADDRESS ((1uL << (USABLE_ADDRESS_BITS + 1)) - 1)
#endif

//A0 P0.2
//A1 P0.3
//A2 P0.4
//A3 P0.5
//A4 P0.12
//A5 P0.13
//A6 P0.14
//A7 P0.15

//Read P0.7
//Write P0.20

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

#endif //VPBUS_H
