/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#include "vpbus.h"
#include <linux/types.h>
#include "am335x_gpio.h"
#include "am335x_control.h"
#include <asm/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marc Laval");
MODULE_DESCRIPTION("VirtuAiles Parallel bus driver");

//================================================================================
//                              Définitions
//================================================================================

#define DRIVER_NAME "vpbus"
#define DEVICE_NAME DRIVER_NAME

typedef enum
{
    BusRead,
    BusWrite
} BusDirectivity;

typedef enum
{
    SeekFromStart = 0,
    SeekFromCurrentPos = 1,
    SeekFromEnd = 2
} SeekFrom;

//================================================================================
//                         Déclaration des fonctions
//================================================================================
//Fonctions LKM
static int __init vpbus_init(void);
static void __exit vpbus_exit(void);

//Fonctions d'interface chrdev
static int device_open(struct inode* i, struct file *f);
static int device_release(struct inode* i, struct file *f);
static long device_ioctl(struct file*, unsigned int, unsigned long);
static int device_read(struct file* f, char __user *data, size_t size, loff_t *l);
static int device_write(struct file* f, const char __user *data, size_t size, loff_t *l);
static loff_t device_seek(struct file* f, loff_t offset, int from);

//Fonctions internes
static void init_bus(void);
static void deinit_bus(void);
static void set_bus_address(uint16_t address);
static uint16_t read_bus(uint16_t address);
static void write_bus(uint16_t address, uint16_t data);


struct file_operations fops =
{
    .read = device_read,
    .write = device_write,
    .unlocked_ioctl = device_ioctl,
    .open = device_open,
    .release = device_release,
    .llseek = device_seek
};

module_init(vpbus_init);
module_exit(vpbus_exit);

//================================================================================
//                          Variables Globales
//================================================================================
/* Variables globales.
   Simple question de facilité pour la lecture du code d'exemple.
   À éviter au maximum en vrai.
 */
int major;
struct device *dev;
static struct class *class;
dev_t devt;

struct
{
    uint16_t currentAddress;
    BusDirectivity directivity;
} vpbus;

//================================================================================
//                              Fonctions
//================================================================================
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define TRY_RESERVE(my_gpio, error_label) if(!gpio_is_valid(my_gpio))\
{\
   printk(KERN_INFO "[%s] %s: invalid GPIO\n", DRIVER_NAME, TOSTRING(my_gpio));\
   status = -ENODEV;\
   goto error_label;\
}\
else if(!gpio_request(my_gpio, DRIVER_NAME))\
{\
    printk(KERN_INFO "[%s] %s: GPIO request failed\n", DRIVER_NAME, TOSTRING(my_gpio));\
    status = -EBUSY;\
    goto error_label;\
}

//----------------------------------------------------------------------
/*! \brief Fonction d'initialisation du module
 */
static int __init vpbus_init(void)
{
    int status;

    printk(KERN_INFO "[%s] Initializing...\n", DRIVER_NAME);

    major = register_chrdev(0, DRIVER_NAME, &fops);
    if(major < 0)
    {
        printk(KERN_ERR "[%s] Failed registering chrdev!\n", DRIVER_NAME);
        return major;
    }

    class = class_create(THIS_MODULE, DRIVER_NAME);
    if(IS_ERR(class))
    {
        printk(KERN_ERR "[%s] Failed to create class!\n", DRIVER_NAME);
        status = PTR_ERR(class);
        goto errorClass;
    }

    devt = MKDEV(major, 0);
    dev = device_create(class, NULL, devt, NULL, DRIVER_NAME);
    status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

    if(status != 0)
    {
        printk(KERN_ERR "[%s] Failed to create device\n", DRIVER_NAME);
        goto error;
    }

    TRY_RESERVE(GPIO_A0, error_A0)
    TRY_RESERVE(GPIO_A1, error_A1)
    TRY_RESERVE(GPIO_A2, error_A2)
    TRY_RESERVE(GPIO_A3, error_A3)
    TRY_RESERVE(GPIO_A4, error_A4)
    TRY_RESERVE(GPIO_A5, error_A5)
    TRY_RESERVE(GPIO_A6, error_A6)
    TRY_RESERVE(GPIO_A7, error_A7)

    TRY_RESERVE(GPIO_READ, error_READ)
    TRY_RESERVE(GPIO_WRITE, error_WRITE)

    TRY_RESERVE(GPIO_D0, error_D0)
    TRY_RESERVE(GPIO_D1, error_D1)
    TRY_RESERVE(GPIO_D2, error_D2)
    TRY_RESERVE(GPIO_D3, error_D3)
    TRY_RESERVE(GPIO_D4, error_D4)
    TRY_RESERVE(GPIO_D5, error_D5)
    TRY_RESERVE(GPIO_D6, error_D6)
    TRY_RESERVE(GPIO_D7, error_D7)

    TRY_RESERVE(GPIO_D8, error_D8)
    TRY_RESERVE(GPIO_D9, error_D9)
    TRY_RESERVE(GPIO_D10, error_D10)
    TRY_RESERVE(GPIO_D11, error_D11)
    TRY_RESERVE(GPIO_D12, error_D12)
    TRY_RESERVE(GPIO_D13, error_D13)
    TRY_RESERVE(GPIO_D14, error_D14)
    TRY_RESERVE(GPIO_D15, error_D15)

    return 0;

error_D15:
    gpio_free(GPIO_D15);
error_D14:
    gpio_free(GPIO_D14);
error_D13:
    gpio_free(GPIO_D13);
error_D12:
    gpio_free(GPIO_D12);
error_D11:
    gpio_free(GPIO_D11);
error_D10:
    gpio_free(GPIO_D10);
error_D9:
    gpio_free(GPIO_D9);
error_D8:
    gpio_free(GPIO_D8);
error_D7:
    gpio_free(GPIO_D7);
error_D6:
    gpio_free(GPIO_D6);
error_D5:
    gpio_free(GPIO_D5);
error_D4:
    gpio_free(GPIO_D4);
error_D3:
    gpio_free(GPIO_D3);
error_D2:
    gpio_free(GPIO_D2);
error_D1:
    gpio_free(GPIO_D1);
error_D0:
    gpio_free(GPIO_D0);
error_WRITE:
    gpio_free(GPIO_WRITE);
error_READ:
    gpio_free(GPIO_READ);
error_A7:
    gpio_free(GPIO_A0);
error_A6:
    gpio_free(GPIO_A0);
error_A5:
    gpio_free(GPIO_A0);
error_A4:
    gpio_free(GPIO_A0);
error_A3:
    gpio_free(GPIO_A0);
error_A2:
    gpio_free(GPIO_A0);
error_A1:
    gpio_free(GPIO_A0);
error_A0:
    gpio_free(GPIO_A0);
error:
    class_destroy(class);
errorClass:
    unregister_chrdev(major, DRIVER_NAME);
    return status;
}

//----------------------------------------------------------------------
/*! \brief Fonction de sortie du module
 */
static void __exit vpbus_exit(void)
{
    gpio_free(GPIO_D15);
    gpio_free(GPIO_D14);
    gpio_free(GPIO_D13);
    gpio_free(GPIO_D12);
    gpio_free(GPIO_D11);
    gpio_free(GPIO_D10);
    gpio_free(GPIO_D9);
    gpio_free(GPIO_D8);
    gpio_free(GPIO_D7);
    gpio_free(GPIO_D6);
    gpio_free(GPIO_D5);
    gpio_free(GPIO_D4);
    gpio_free(GPIO_D3);
    gpio_free(GPIO_D2);
    gpio_free(GPIO_D1);
    gpio_free(GPIO_D0);
    gpio_free(GPIO_WRITE);
    gpio_free(GPIO_READ);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);
    gpio_free(GPIO_A0);

    device_destroy(class, devt);
    class_destroy(class);
    unregister_chrdev(major, DRIVER_NAME);
    printk(KERN_INFO "[%s] Unloading module\n", DRIVER_NAME);
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ouvre notre fichier virtuel.
 */
static int device_open(struct inode *i, struct file *f)
{
    printk(KERN_INFO "[%s] Opening device\n", DRIVER_NAME);
    init_bus();
    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ferme notre fichier virtuel.
 */
static int device_release(struct inode *i, struct file *f)
{
    printk(KERN_INFO "[%s] Device release\n", DRIVER_NAME);
    deinit_bus();
    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur effectue un ioctl sur notre
 * fichier.
 */
static long device_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
    printk(KERN_INFO "[%s] ioctl not implemented !\n", DRIVER_NAME);
    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur effectue une lecture sur notre
 * fichier virtuel.
 *
 * Elle reçoit un buffer à remplir et une taille demandée.
 * Elle doit remplir le buffer et retourner la taille effective.
 */
static int device_read(struct file *f, char __user *data, size_t size, loff_t *l)
{
    //on doit gérer le cas de la première lecture qui est peut être non alignée
    //tout comme la dernière peut l'être aussi.
    //Dans le cas de la lecture c'est sous optimal mais non génant car on ne perturbe pas
    //le périphérique à faire un accès non aligné (en réalité on fait un accès aligné dont on ne garde qu'une partie)

    //Allocation mémoire pour la lecture
    char * tempData = kmalloc(size, GFP_KERNEL);
    uint16_t currentReadIndex = 0;
    uint16_t tempRead = 0;

    //Premier accès non aligné
    if(vpbus.currentAddress & 0x01)
    {
        tempRead = read_bus(vpbus.currentAddress - 1);
        tempData[currentReadIndex] = (tempRead >> 8);
        currentReadIndex++;
        vpbus.currentAddress++;
    }

    while(currentReadIndex < size-1)
    {
        tempRead = read_bus(vpbus.currentAddress);
        *(uint16_t*)(&tempData[currentReadIndex]) = tempRead;
        vpbus.currentAddress += 2;
        currentReadIndex += 2;
    }

    //dernier accès eventuellement non aligné
    if(currentReadIndex+1 < size)
    {
        tempRead = read_bus(vpbus.currentAddress);
        tempData[currentReadIndex] = tempRead & 0xFF;
        currentReadIndex++;
        vpbus.currentAddress++;
    }

    copy_to_user(tempData, data, size);
    kfree(tempData);
    return currentReadIndex;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur effectue une écriture sur
 * notre fichier virtuel.
 *
 * Elle reçoit un buffer contenant des données.
 * Elle doit utiliser les données et retourner le nombre de données
 * effectivement utilisées.
*/
static int device_write(struct file *f, const char __user *data, size_t size, loff_t *l)
{
    //Les premiers et dernier octets peuvent être des accès non alignés
    //Dans ce cas on ajoute un octet à 0 pour le rendre aligné
    //ce cas ne devrait pas arriver normalement
    //mais il faut le gérer au cas où

    uint16_t sizeWritten = 0;
    uint16_t currentWriteIndex = 0;
    uint16_t tempWrite = 0;
    uint8_t * dataToWrite = kmalloc(size, GFP_KERNEL);
    copy_from_user(dataToWrite, data, size);

    //Premier accès non aligné
    if(vpbus.currentAddress & 0x01)
    {
        tempWrite = (uint16_t)dataToWrite[currentWriteIndex] << 8;
        write_bus(vpbus.currentAddress - 1, tempWrite);
        sizeWritten += 2;
        currentWriteIndex++;
        vpbus.currentAddress++;
    }

    while(currentWriteIndex < size-1)
    {
        tempWrite = dataToWrite[currentWriteIndex] | ((uint16_t)dataToWrite[currentWriteIndex+1] << 8);
        write_bus(vpbus.currentAddress, tempWrite);
        sizeWritten += 2;
        vpbus.currentAddress += 2;
        currentWriteIndex += 2;
    }

    //dernier accès eventuellement non aligné
    if(currentWriteIndex+1 < size)
    {
        tempWrite = dataToWrite[currentWriteIndex];
        write_bus(vpbus.currentAddress, tempWrite);
        sizeWritten += 2;
        vpbus.currentAddress += 2;
        currentWriteIndex += 2;
    }

    kfree(dataToWrite);
    return sizeWritten;
}

//----------------------------------------------------------------------
/*! \brief Specifie l'adresse courante du peripherique
 */
static loff_t device_seek(struct file* f, loff_t offset, int from)
{
    loff_t newAddress = 0;
    switch(from)
    {
        case SeekFromStart:
            newAddress = offset;
            if(newAddress > MAX_BUS_ADDRESS)
            {
                newAddress = MAX_BUS_ADDRESS;
            }
            break;

        case SeekFromCurrentPos:
            newAddress = vpbus.currentAddress + offset;
            if(newAddress > MAX_BUS_ADDRESS)
            {
                newAddress = MAX_BUS_ADDRESS;
            }
            break;

        case SeekFromEnd:
            if(offset < MAX_BUS_ADDRESS)
            {
                newAddress = MAX_BUS_ADDRESS - offset;
            }
            else
            {
                newAddress = 0;
            }

        default:
            return vpbus.currentAddress;
    }

    vpbus.currentAddress = newAddress;
    return vpbus.currentAddress;
}

//----------------------------------------------------------------------
/*! \brief Initialise le bus
 */
static void init_bus(void)
{
    gpio_direction_output(GPIO_READ, 1);
    gpio_direction_output(GPIO_WRITE, 1);

    gpio_direction_output(GPIO_A0, 0);
    gpio_direction_output(GPIO_A1, 0);
    gpio_direction_output(GPIO_A2, 0);
    gpio_direction_output(GPIO_A3, 0);
    gpio_direction_output(GPIO_A4, 0);
    gpio_direction_output(GPIO_A5, 0);
    gpio_direction_output(GPIO_A6, 0);
    gpio_direction_output(GPIO_A7, 0);

    gpio_direction_input(GPIO_D0);
    gpio_direction_input(GPIO_D1);
    gpio_direction_input(GPIO_D2);
    gpio_direction_input(GPIO_D3);
    gpio_direction_input(GPIO_D4);
    gpio_direction_input(GPIO_D5);
    gpio_direction_input(GPIO_D6);
    gpio_direction_input(GPIO_D7);
    gpio_direction_input(GPIO_D8);
    gpio_direction_input(GPIO_D9);
    gpio_direction_input(GPIO_D10);
    gpio_direction_input(GPIO_D11);
    gpio_direction_input(GPIO_D12);
    gpio_direction_input(GPIO_D13);
    gpio_direction_input(GPIO_D14);
    gpio_direction_input(GPIO_D15);

    //Init variables internes
    vpbus.currentAddress = 0;
    set_bus_address(vpbus.currentAddress);
}

//----------------------------------------------------------------------
/*! \brief Relache le bus
 */
static void deinit_bus(void)
{
    //on remet toutes les pins en entrée
    gpio_direction_input(GPIO_D15);
    gpio_direction_input(GPIO_D14);
    gpio_direction_input(GPIO_D13);
    gpio_direction_input(GPIO_D12);
    gpio_direction_input(GPIO_D11);
    gpio_direction_input(GPIO_D10);
    gpio_direction_input(GPIO_D9);
    gpio_direction_input(GPIO_D8);
    gpio_direction_input(GPIO_D7);
    gpio_direction_input(GPIO_D6);
    gpio_direction_input(GPIO_D5);
    gpio_direction_input(GPIO_D4);
    gpio_direction_input(GPIO_D3);
    gpio_direction_input(GPIO_D2);
    gpio_direction_input(GPIO_D1);
    gpio_direction_input(GPIO_D0);
    gpio_direction_input(GPIO_WRITE);
    gpio_direction_input(GPIO_READ);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
    gpio_direction_input(GPIO_A0);
}

//----------------------------------------------------------------------
/*! \brief Configure l'adresse courante sur le bus
 */
static void set_bus_address(uint16_t address)
{
    //on supprime le dernier bit pour passer de l'adressage en octets
    //à l'adressage par mot de 16bits
    address = address >> 1;

    gpio_set_value(GPIO_A0, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A1, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A2, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A3, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A4, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A5, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A6, address & 0x01);
    address = address >> 1;
    gpio_set_value(GPIO_A7, address & 0x01);
}

//----------------------------------------------------------------------
/*! \brief Effectue une lecture sur le bus
 */
static uint16_t read_bus(uint16_t address)
{
    uint16_t dataRead;
    if(vpbus.directivity != BusRead)
    {
        vpbus.directivity = BusRead;
        gpio_direction_input(GPIO_D15);
        gpio_direction_input(GPIO_D14);
        gpio_direction_input(GPIO_D13);
        gpio_direction_input(GPIO_D12);
        gpio_direction_input(GPIO_D11);
        gpio_direction_input(GPIO_D10);
        gpio_direction_input(GPIO_D9);
        gpio_direction_input(GPIO_D8);
        gpio_direction_input(GPIO_D7);
        gpio_direction_input(GPIO_D6);
        gpio_direction_input(GPIO_D5);
        gpio_direction_input(GPIO_D4);
        gpio_direction_input(GPIO_D3);
        gpio_direction_input(GPIO_D2);
        gpio_direction_input(GPIO_D1);
        gpio_direction_input(GPIO_D0);
    }

    //Activation du read
    gpio_set_value(GPIO_READ, 0);

    dataRead = gpio_get_value(GPIO_D15) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D14) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D13) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D12) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D11) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D10) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D9) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D8) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D7) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D6) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D5) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D4) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D3) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D2) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D1) & 0x01;
    dataRead = dataRead << 1;
    dataRead |= gpio_get_value(GPIO_D0) & 0x01;

    //Désactivation du read
    gpio_set_value(GPIO_READ, 1);

    return dataRead;
}

//----------------------------------------------------------------------
/*! \brief Effectue une ecriture sur le bus
 */
static void write_bus(uint16_t address, uint16_t data)
{
    vpbus.directivity = BusWrite;

    gpio_direction_output(GPIO_D0, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D1, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D2, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D3, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D4, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D5, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D6, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D7, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D8, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D9, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D10, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D11, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D12, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D13, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D14, data & 0x01);
    data = data >> 1;
    gpio_direction_output(GPIO_D15, data & 0x01);

    //Activation du write
    gpio_set_value(GPIO_WRITE, 0);

    //Désactivation du write
    gpio_set_value(GPIO_WRITE, 1);
}


