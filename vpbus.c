/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#include "vpbus.h"
#include <linux/types.h>
#include "am335x_gpio.h"
#include <linux/sched.h>	/* For current */
#include <linux/tty.h>		/* For the tty declarations */
#include <linux/version.h>	/* For LINUX_VERSION_CODE */
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
//static long device_ioctl(struct file*, unsigned int, unsigned long);
static int device_read(struct file* f, char __user *data, size_t size, loff_t *l);
static int device_write(struct file* f, const char __user *data, size_t size, loff_t *l);
static loff_t device_seek(struct file* f, loff_t offset, int from);

//Fonctions internes
static void init_bus(void);
static void deinit_bus(void);
static void set_bus_directivity(BusDirectivity dir);
static void set_address_and_latch(uint16_t address);
static uint16_t read_bus(void);
static void write_bus(uint16_t data);
static void write_data_on_bus(uint16_t data);


struct file_operations vpbus_fops =
{
    .owner = THIS_MODULE,
    .read = device_read,
    .write = device_write,
    //.unlocked_ioctl = device_ioctl,
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
static struct
{
    int major;
    struct device* device;
    struct class* class;
    dev_t devt;
    struct cdev cdev;
    void* bus_base_address;

    uint8_t is_opened;
    BusDirectivity directivity;

} vpbus;

volatile void * gpio1;
volatile void * gpio2;
volatile void * gpio3;

static const uint32_t GPIO2_PIN_MASK = ((1uL << READ_PIN_INDEX)   | //Read
                                       (1uL << WRITE_PIN_INDEX)   | //Write
                                       (1uL << ALE_PIN_INDEX));     //ALE

//On réserve une patte de chaque block GPIO pour que les horloges soient actives
//et qu'on puisse faire les accès via registres
static struct gpio gpio_to_enable_block[] = {
    {P1PIN, GPIOF_IN, "gpioP1"},
    {P2PIN, GPIOF_IN, "gpioP2"},
    {P3PIN, GPIOF_IN, "gpioP3"}};


//================================================================================
//                              Fonctions
//================================================================================

//----------------------------------------------------------------------
/*! \brief Fonction d'initialisation du module
 */
static int __init vpbus_init(void)
{
    int status;
    int result;

    //mapping memoire des block de registre GPIO
    gpio1 = ioremap_nocache(GPIO1_BASE_ADDR, GPIO_BLOCK_SIZE);
    gpio2 = ioremap_nocache(GPIO2_BASE_ADDR, GPIO_BLOCK_SIZE);
    gpio3 = ioremap_nocache(GPIO3_BASE_ADDR, GPIO_BLOCK_SIZE);

    result = gpio_request_array(gpio_to_enable_block, ARRAY_SIZE(gpio_to_enable_block));
    if(result == 0)
    {
        printk(KERN_INFO "GPIO reservation OK!");
    }
    else
    {
        printk(KERN_INFO "GPIO reservation fail %d !", result); 
        return result;
    }

    //On preconfigure READ et WRITE a l'état haut car c'est un signal inversé
    iowrite32(GPIO2_PIN_MASK, gpio2 + GPIO_SETDATAOUT);

    //Demander l'allocation d'un chrdev avec mineur start = 0, count = 1, pour DEVICE_NAME
    result = alloc_chrdev_region(&vpbus.devt, 0, 1, DEVICE_NAME);
    vpbus.major = MAJOR(vpbus.devt);

    if(result < 0)
    {
       printk(KERN_ERR "[%s] Failed registering chrdev! \n", DEVICE_NAME);
       return vpbus.major;
    }

    vpbus.class = class_create(THIS_MODULE, DEVICE_NAME);
    if(IS_ERR(vpbus.class))
    {
       printk(KERN_ERR "[%s] Failed to create class!\n", DEVICE_NAME);
       status = PTR_ERR(vpbus.class);
       goto errorClass;
    }

    vpbus.device = device_create(vpbus.class, NULL, vpbus.devt, NULL, DEVICE_NAME);
    status = IS_ERR(vpbus.device) ? PTR_ERR(vpbus.device) : 0;

    if(status != 0)
    {
       printk(KERN_ERR "[%s] Failed to create device\n", DEVICE_NAME);
       goto error;
    }

    cdev_init(&vpbus.cdev, &vpbus_fops);
    vpbus.cdev.owner = THIS_MODULE;
    vpbus.cdev.ops = &vpbus_fops;
    result = cdev_add(&vpbus.cdev, vpbus.devt, 1);
    if (result < 0)
    {
       printk(KERN_ERR "[%s] Failed to add c device\n", DEVICE_NAME);
       status = result;
    }
    else
    {
       printk(KERN_INFO "[%s] Init Ok\n", DEVICE_NAME);
       return 0;
    }

    error:
    class_destroy(vpbus.class);
    errorClass:
    unregister_chrdev_region(vpbus.devt, 1);
    gpio_free_array(gpio_to_enable_block, ARRAY_SIZE(gpio_to_enable_block));
    return status;
}

//----------------------------------------------------------------------
/*! \brief Fonction de sortie du module
 */
static void __exit vpbus_exit(void)
{
    device_destroy(vpbus.class, vpbus.devt);
    cdev_del(&vpbus.cdev);
    class_destroy(vpbus.class);
    unregister_chrdev_region(vpbus.devt, 1);

    iounmap(gpio1);
    iounmap(gpio2);
    iounmap(gpio3);

    gpio_free_array(gpio_to_enable_block, ARRAY_SIZE(gpio_to_enable_block));

    printk(KERN_INFO "[%s] Exit\n", DEVICE_NAME);
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ouvre notre fichier virtuel.
 */
static int device_open(struct inode *i, struct file *f)
{
    if(vpbus.is_opened)
    {
       //on n'autorise pas l'ouverture simultanée de ce périph
       return -EBUSY;
    }

    vpbus.is_opened = 1;

    printk(KERN_INFO "[%s] Opening device\n", DEVICE_NAME);

    init_bus();
    //incremente le compteur d'utilisation du module
    //si ce compte n'est pas à 0, le kernel n'autorisera pas le rmmod
    try_module_get(THIS_MODULE);

    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ferme notre fichier virtuel.
 */
static int device_release(struct inode *i, struct file *f)
{
    printk(KERN_INFO "[%s] Device release\n", DEVICE_NAME);
    if(vpbus.is_opened)
    {
       deinit_bus();
       module_put(THIS_MODULE);
    }

    vpbus.is_opened = 0;
    return 0;
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur effectue une lecture sur notre
 * fichier virtuel.
 *
 * Elle reçoit un buffer à remplir et une taille demandée.
 * Elle doit remplir le buffer et retourner la taille effective.
 */
static int device_read(struct file *f, char __user *data, size_t size, loff_t* off)
{
    //on doit gérer le cas de la première lecture qui est peut être non alignée
    //tout comme la dernière peut l'être aussi.
    //Dans le cas de la lecture c'est sous optimal mais non génant car on ne perturbe pas
    //le périphérique à faire un accès non aligné (en réalité on fait un accès aligné dont on ne garde qu'une partie)

    //Allocation mémoire pour la lecture
    char * tempData = NULL;
    uint16_t currentReadIndex = 0;
    uint16_t tempRead = 0;

    #ifdef WORD_ADDRESSING_ONLY
    if ((f->f_pos & 0x01) || (size & 0x01))
    {
       printk(KERN_ERR "[%s] Read transfer must be 16bits aligned ! \n", DEVICE_NAME);
       return -EFAULT;
    }
    #endif

    //Vérification qu'on ne déborde pas de la plage d'adresse du bus
    if(f->f_pos + size > BUS_SIZE)
    {
       size = BUS_SIZE - f->f_pos;
    }

    tempData = kmalloc(size, GFP_KERNEL);

    set_address_and_latch(f->f_pos);

    while(currentReadIndex <= size-1)
    {
        tempRead = read_bus();
        *(uint16_t*)(&tempData[currentReadIndex]) = tempRead;
        *off += 2;
        currentReadIndex += 2;
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
static int device_write(struct file *f, const char __user *data, size_t size, loff_t* off)
{
    //Les premiers et dernier octets peuvent être des accès non alignés
    //Dans ce cas on ajoute un octet à 0 pour le rendre aligné
    //ce cas ne devrait pas arriver normalement
    //mais il faut le gérer au cas où

    uint16_t currentWriteIndex = 0;
    uint16_t tempWrite = 0;
    uint8_t * dataToWrite = kmalloc(size, GFP_KERNEL);
    copy_from_user(dataToWrite, data, size);

    #ifdef WORD_ADDRESSING_ONLY
    if ((f->f_pos & 0x01) || (size & 0x01))
    {
       printk(KERN_ERR "[%s] : Write transfer must be 16bits aligned ! \n", DEVICE_NAME);
       return -EFAULT;
    }
    #endif

    if(f->f_pos + size > BUS_SIZE)
    {
       size = BUS_SIZE - f->f_pos;
    }

    //On divise l'adresse par 2 pour passer de l'adressage à l'octet à l'adressage au mot.
    set_address_and_latch(f->f_pos >> 1);
    *off = f->f_pos;
    while(currentWriteIndex < size-1)
    {
        tempWrite = *(uint16_t*)(&dataToWrite[currentWriteIndex]);
        write_bus(tempWrite);
        *off += 2;
        currentWriteIndex += 2;
    }

    kfree(dataToWrite);
    return currentWriteIndex;
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
            if(newAddress > MAX_ADDRESS)
            {
                newAddress = MAX_ADDRESS;
            }
            break;

        case SeekFromCurrentPos:
            newAddress = f->f_pos + offset;
            if(newAddress > MAX_ADDRESS)
            {
                newAddress = MAX_ADDRESS;
            }
            break;

        case SeekFromEnd:
            if(offset < MAX_ADDRESS)
            {
                newAddress = MAX_ADDRESS - offset;
            }
            else
            {
                newAddress = 0;
            }

        default:
            return f->f_pos;
    }

    f->f_pos = newAddress;
    return f->f_pos;
}

//----------------------------------------------------------------------
/*! \brief Initialise le bus
 */
static void init_bus(void)
{
    uint32_t gpio_oe = 0;
//    uint32_t pad_config = PAD_CONTROL_MUX_MODE_7 |
//                          PAD_CONTROL_PULL_DISABLE |
//                          PAD_CONTROL_RX_ACTIVE |
//                          PAD_CONTROL_SLEW_FAST;

    //ATTENTION dans registre OE, bit à 1 = pin en entrée

    printk(KERN_ERR "[%s] Init bus \n", DEVICE_NAME);
    //On preconfigure READ et WRITE a l'état haut car c'est un signal inversé
    //iowrite32(GPIO2_PIN_MASK, gpio2 + GPIO_SETDATAOUT);

    printk(KERN_ERR "[%s] Init bus R/W/ALE directivity \n", DEVICE_NAME);
    //Protection de la séquence Read-Modify-Write
    //preempt_disable();
    gpio_oe = ioread32(gpio2 + GPIO_OE);
    gpio_oe &= ~GPIO2_PIN_MASK;
    iowrite32(gpio_oe, gpio2 + GPIO_OE);
    //preempt_enable();

    //init bus de données par défaut en lecture
    printk(KERN_ERR "[%s] Init bus directivity \n", DEVICE_NAME);
    set_bus_directivity(BusRead);
}

//----------------------------------------------------------------------
/*! \brief Relache le bus
 */
static void deinit_bus(void)
{
    uint32_t gpio_oe = 0;
    //Protection de la séquence Read-Modify-Write
    //preempt_disable();
    //gpio_oe = ioread32(gpio2 + GPIO_OE);
    //gpio_oe |= GPIO2_PIN_MASK;
    //iowrite32(gpio_oe, gpio2 + GPIO_OE);
    //preempt_enable();

    //on remet toutes les pins en entrée
    set_bus_directivity(BusRead);
}

//----------------------------------------------------------------------
/*! \brief Change la directivité du bus
 */
static void set_bus_directivity(BusDirectivity dir)
{
    uint32_t gpio1_pins, gpio3_pins;
    uint32_t gpio_oe;
    if(vpbus.directivity != dir)
    {
//        printk(KERN_INFO "[%s] Set bus directivity at %d \n", DEVICE_NAME, dir);
        vpbus.directivity = dir;

        //Selection des bits à commuter dans les registres OE
        gpio1_pins = (0xFFuL << AD0_PIN_INDEX); //AD0 à AD7 sur P1.12 à P1.19
        gpio3_pins = (0xFFuL << AD8_PIN_INDEX); //AD8 à AD15 sur P3.14 à P3.21

        //Protection de la séquence Read-Modify-Write
        //preempt_disable();
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
        //preempt_enable();
    }
}

//----------------------------------------------------------------------
/*! \brief Ecrit une donnée sur le bus
*/
static void write_data_on_bus(uint16_t data)
{                               
    uint32_t gpio_set;
    uint32_t gpio_clr;
    volatile int i = 0;

    set_bus_directivity(BusWrite);

    //poids faible
    gpio_set = ((uint32_t)(data & 0xFF) << AD0_PIN_INDEX);
    gpio_clr = (~gpio_set & GPIO1_AD_PIN_MASK);
    iowrite32(gpio_set, gpio1 + GPIO_SETDATAOUT);
    iowrite32(gpio_clr, gpio1 + GPIO_CLEARDATAOUT);

    //poids fort
    gpio_set = ((uint32_t)(data & 0xFF00) << (AD8_PIN_INDEX - 8));
    gpio_clr = (~gpio_set & GPIO3_AD_PIN_MASK);
    iowrite32(gpio_set, gpio3 + GPIO_SETDATAOUT);
    iowrite32(gpio_clr, gpio3 + GPIO_CLEARDATAOUT);

    for(i = 0; i < 2000; i++)
    {
        gpio_clr = i;
    }
}

//----------------------------------------------------------------------
/*! \brief Ecrit l'adresse sur le bus et la valide
 */
static void set_address_and_latch(uint16_t address)
{
    //Activation du ALE
    iowrite32((uint32_t)(1uL << ALE_PIN_INDEX), gpio2 + GPIO_CLEARDATAOUT);

    write_data_on_bus(address);

    //Désactivation du ALE
    iowrite32((uint32_t)(1uL << ALE_PIN_INDEX), gpio2 + GPIO_SETDATAOUT);
}

//----------------------------------------------------------------------
/*! \brief Effectue une lecture sur le bus
 */
static uint16_t read_bus(void)
{
    uint32_t gpio_in;
    uint16_t dataRead;
    set_bus_directivity(BusRead);

    //Activation du read
    iowrite32((uint32_t)(1uL << READ_PIN_INDEX), gpio2 + GPIO_CLEARDATAOUT);

    //Besoin d'ajouter une attente?
    gpio_in = ioread32(gpio1 + GPIO_DATAIN);
    dataRead = (gpio_in >> AD0_PIN_INDEX) & 0xFF;
    gpio_in = ioread32(gpio3 + GPIO_DATAIN);
    dataRead |= (gpio_in >> (AD8_PIN_INDEX - 8)) & 0xFF00;

    //Désactivation du read
    iowrite32((uint32_t)(1uL << READ_PIN_INDEX), gpio2 + GPIO_SETDATAOUT);

    return dataRead;
}

//----------------------------------------------------------------------
/*! \brief Effectue une ecriture sur le bus
 */
static void write_bus(uint16_t data)
{
    //Activation du write
    iowrite32((uint32_t)(1uL << WRITE_PIN_INDEX), gpio2 + GPIO_CLEARDATAOUT);

    write_data_on_bus(data);

    //Désactivation du write
    iowrite32((uint32_t)(1uL << WRITE_PIN_INDEX), gpio2 + GPIO_SETDATAOUT);
}

