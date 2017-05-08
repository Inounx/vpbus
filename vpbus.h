/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#ifndef VPBUS_H
#define VPBUS_H

//Option de profilage du bus
//Affiche à chaque accès le temps et la bande passante
#define PROFILE

//A activer pour n'autoriser que les accès alignés sur le bus
#define WORD_ADDRESSING_ONLY

//======================================================================
//Définitions non modifiables
//Adresse max sur 16bits et on adresse des mots de 16bits

#define BUS_BASE_ADDRESS 0x01000000
#define BUS_SIZE (65536*2)
#define MAX_ADDRESS (BUS_SIZE - 2)

#define DEVICE_NAME "vpbus"

#endif //VPBUS_H
