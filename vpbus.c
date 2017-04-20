/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#include "vpbus.h"
#include <stdint.h>
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
#include <sys/ioctl.h>

MODULE_LICENSE("LGPLv3");
MODULE_AUTHOR("Marc Laval");
MODULE_DESCRIPTION("VirtuAiles Parallel bus driver");

//================================================================================
//                              Définitions
//================================================================================

#define DRIVER_NAME "vpbus"

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

enum BusDirectivity
{
    BusRead,
    BusWrite
};

enum SeekFrom
{
    SeekFromStart = 0,
    SeekFromCurrentPos = 1,
    SeekFromEnd = 2
};

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
static void init_bus();
static void deinit_bus();
static void set_bus_directivity(BusDirectivity dir);
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

volatile void * gpio0;
volatile void * gpio1;
volatile void * gpio2;
volatile void * gpio3;
volatile void * control;

struct
{
    uint16_t currentAddress;
    BusDirectivity directivity;
} vpbus;

//================================================================================
//                              Fonctions
//================================================================================

//----------------------------------------------------------------------
/*! \brief Fonction d'initialisation du module
 */
static int __init vpbus_init(void)
{
    int status;

    printk(KERN_INFO "[VPBUS] Initializing...\n");

    major = register_chrdev(0, DRIVER_NAME, &fops);
    if(major < 0)
    {
        printk(KERN_ERR "[VPBUS] Failed registering chrdev!\n");
        return major;
    }

    class = class_create(THIS_MODULE, DRIVER_NAME);
    if(IS_ERR(class))
    {
        printk(KERN_ERR "[VPBUS] Failed to create class!\n");
        status = PTR_ERR(class);
        goto errorClass;
    }

    devt = MKDEV(major, 0);
    dev = device_create(class, NULL, devt, NULL, DRIVER_NAME);
    status = IS_ERR(dev) ? PTR_ERR(dev) : 0;

    if(status != 0)
    {
        printk(KERN_ERR "[VPBUS] Failed to create device\n");
        goto error;
    }

    //mapping memoire des block de registre GPIO
    //MLa: Je suppose qu'il faut ici utiliser _nocache car on accède a des registres...
    gpio0 = ioremap_nocache(GPIO0_BASE_ADDR, GPIO_BLOCK_SIZE);
    gpio1 = ioremap_nocache(GPIO1_BASE_ADDR, GPIO_BLOCK_SIZE);
    gpio2 = ioremap_nocache(GPIO2_BASE_ADDR, GPIO_BLOCK_SIZE);
    gpio3 = ioremap_nocache(GPIO3_BASE_ADDR, GPIO_BLOCK_SIZE);
    control = ioremap_nocache(CONTROL_MODULE_BASE_ADDR, CONTROL_MODULE_BLOCK_SIZE);

    return 0;

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
    iounmap(gpio0);
    iounmap(gpio1);
    iounmap(gpio2);
    iounmap(gpio3);
    iounmap(control);

    device_destroy(class, devt);
    class_destroy(class);
    unregister_chrdev(major, DRIVER_NAME);
    printk(KERN_INFO "[VPBUS] Unloading module\n");
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ouvre notre fichier virtuel.
 */
static int device_open(struct inode *i, struct file *f)
{
    printk(KERN_INFO "[VPBUS] Opening device\n");
    init_bus();
    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ferme notre fichier virtuel.
 */
static int device_release(struct inode *i, struct file *f)
{
    printk(KERN_INFO "[VPBUS] Device release\n");
    deinit_bus();
    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur effectue un ioctl sur notre
 * fichier.
 */
static long device_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
    printk(KERN_INFO "[VPBUS] ioctl not implemented !\n");
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

    uint8_t * dataToWrite = kmalloc(size, GFP_KERNEL);
    copy_from_user(dataToWrite, data, size);

    uint16_t sizeWritten = 0;
    uint16_t currentWriteIndex = 0;
    uint16_t tempWrite = 0;

    //Premier accès non aligné
    if(vpbus.currentAddress & 0x01)
    {
        tempWrite = (uint16_t)dataToWrite[currentWriteIndex] << 8
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
static void init_bus()
{
    uint32_t gpio_oe = 0, gpio_output = 0;
    uint32_t pad_config = PAD_CONTROL_MUX_MODE_7 |
                          PAD_CONTROL_PULL_DISABLE |
                          PAD_CONTROL_RX_ACTIVE |
                          PAD_CONTROL_SLEW_FAST;

    //Configuration du multiplexeur interne pour que les GPIO soient cablées
    //sur les pates en sortie
    iowrite32(pad_config, control + PAD_CONTROL_SPI0_SCLK);
    iowrite32(pad_config, control + PAD_CONTROL_SPI0_D0);
    iowrite32(pad_config, control + PAD_CONTROL_SPI0_D1);
    iowrite32(pad_config, control + PAD_CONTROL_SPI0_CS0);
    iowrite32(pad_config, control + PAD_CONTROL_ECAP0_IN_PWM0_OUT);
    iowrite32(pad_config, control + PAD_CONTROL_UART1_CTSn);
    iowrite32(pad_config, control + PAD_CONTROL_UART1_RTSn);
    iowrite32(pad_config, control + PAD_CONTROL_UART1_RXD);
    iowrite32(pad_config, control + PAD_CONTROL_UART1_TXD);
    iowrite32(pad_config, control + PAD_CONTROL_XDMA_EVENT_INTR1);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_AD12);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_AD13);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_AD14);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_AD15);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_A0);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_A1);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_A2);
    iowrite32(pad_config, control + PAD_CONTROL_GPMC_A3);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_ACLKX);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_FSX);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_AXR0);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_AHCLKR);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_ACLKR);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_FSR);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_AXR1);
    iowrite32(pad_config, control + PAD_CONTROL_MCASP0_AHCLKX);

    //Init read et write
    //Init bus d'adresse

    //on calcule les pattes à mettre en sortie pour chaque port
    gpio_output = (1uL << 7); //Read P0.7
    gpio_output |= (1uL << 20); //Write P0.20
    gpio_output |= (0xFuL << 2); //A(0-3) sur P0.2 à P0.5
    gpio_output |= (0xFuL << 12); //A(4-7) sur P0.12 à P0.15

    //Faut il une section critique ou autre chose du même genre ?
    //ATTENTION dans registre OE, bit à 1 = pin en entrée
    gpio_oe = ioread32(gpio0 + GPIO_OE);
    gpio_oe &= ~gpio_output;
    iowrite32(gpio_oe, gpio0 + GPIO_OE);

    //Init variables internes
    vpbus.currentAddress = 0;
    set_bus_address(vpbus.currentAddress);

    //init bus de données par défaut en lecture
    set_bus_directivity(BusRead);
}

//----------------------------------------------------------------------
/*! \brief Relache le bus
 */
static void deinit_bus()
{

}

//----------------------------------------------------------------------
/*! \brief Change la directivité du bus
 */
static void set_bus_directivity(BusDirectivity dir)
{
    uint32_t gpio1_pins, gpio3_pins
    uint32_t gpio_oe;
    if(vpbus.directivity != dir)
    {
        vpbus.directivity = dir;

        //Selection des bits à commuter dans les registres OE
        gpio1_pins = (0xFFuL << 12); //D0 à D7 sur P1.12 à P1.19
        gpio3_pins = (0xFFuL << 14); //D8 à D15 sur P1.14 à P1.21

        //Faut il une section critique ou autre chose du même genre ?
        //ATTENTION dans registre OE, bit à 1 = pin en entrée
        gpio_oe = ioread32(gpio1 + GPIO_OE);
        if(dir == BusRead)
        {
           gpio_oe |= gpio1_pins;
        }
        else
        {
            gpio_oe &= ~gpio1_pins;
        }
        iowrite32(gpio_oe, gpio1 + GPIO_OE);

        gpio_oe = ioread32(gpio3 + GPIO_OE);
        if(dir == BusRead)
        {
           gpio_oe |= gpio3_pins;
        }
        else
        {
            gpio_oe &= ~gpio3_pins;
        }
        iowrite32(gpio_oe, gpio3 + GPIO_OE);
    }
}

//----------------------------------------------------------------------
/*! \brief Configure l'adresse courante sur le bus
 */
static void set_bus_address(uint16_t address)
{
    //TODO
}

//----------------------------------------------------------------------
/*! \brief Effectue une lecture sur le bus
 */
static uint16_t read_bus(uint16_t address)
{
    //TODO
}

//----------------------------------------------------------------------
/*! \brief Effectue une ecriture sur le bus
 */
static void write_bus(uint16_t address, uint16_t data)
{
    //TODO
}

