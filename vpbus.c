/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 */

#include "vpbus.h"
#include <linux/types.h>
#include <linux/cdev.h>

#include <linux/sched.h>	/* For current */
#include <linux/tty.h>		/* For the tty declarations */
#include <linux/version.h>	/* For LINUX_VERSION_CODE */
#include <asm/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "dma.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marc Laval");
MODULE_DESCRIPTION("VirtuAiles Parallel bus driver");

//================================================================================
//                              Définitions
//================================================================================

typedef enum
{
    SeekFromStart = 0,
    SeekFromCurrentPos = 1,
    SeekFromEnd = 2
} SeekFrom;

//On indique à linux que tout les périph compatibles avec "vpbus" sont gérés par
//ce driver
//Note MLa: je ne suis pas certain qu'avec un build out of tree ce soit utile...
static const struct of_device_id driver_of_match[] = {
    { .compatible = DEVICE_NAME, },
    { },
};

MODULE_DEVICE_TABLE(of, driver_of_match);

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
static int init_bus(void);
static void deinit_bus(void);

//Debug
#ifdef PROFILE

static struct timespec start_ts, end_ts;//profile timer

static inline void start_profile();
static inline void stop_profile();
static inline void print_bandwidth(unsigned int nb_byte);

#endif

struct file_operations vpbus_fops =
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
static struct
{
    int major;
    device* dev;
    struct class* class;
    dev_t devt;
    struct cdev cdev;
    void* bus_base_address;

    uint8_t is_opened;
    dma_channel_t dma;
} vpbus;

#ifdef PROFILE
static struct timespec start_ts, end_ts;//profile timer
#endif

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

    //Demander l'allocation d'un chrdev avec mineur start = 0, count = 1, pour DEVICE_NAME
    result = alloc_chrdev_region(&vpbus.devt, 0, 1, DEVICE_NAME);
    vpbus.major = MAJOR(dev)

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

    vpbus.dev = device_create(vpbus.class, NULL, vpbus.devt, NULL, DEVICE_NAME);
    status = IS_ERR(vpbus.dev) ? PTR_ERR(vpbus.dev) : 0;

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
    class_destroy(class);
errorClass:
    unregister_chrdev_region(vpbus.devt, 1);
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
    printk(KERN_INFO "[%s] Exit\n", DEVICE_NAME);
}

//----------------------------------------------------------------------
/*! \brief Cette fonction est appelée quand l'utilisateur ouvre notre fichier virtuel.
 */
static int device_open(struct inode *i, struct file *f)
{
    int result = 0;

    if(vpbus.is_opened)
    {
        //on n'autorise pas l'ouverture simultanée de ce périph
        return -EBUSY;
    }

    vpbus.is_opened = 1;

    printk(KERN_INFO "[%s] Opening device\n", DEVICE_NAME);

    result = init_bus();
    if(result >= 0)
    {
        //incremente le compteur d'utilisation du module
        //si ce compte n'est pas à 0, le kernel n'autorisera pas le rmmod
        try_module_get(THIS_MODULE);
    }

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
/*! \brief Cette fonction est appelée quand l'utilisateur effectue un ioctl sur notre
 * fichier.
 */
static long device_ioctl (struct file *f, unsigned int cmd, unsigned long arg)
{
    printk(KERN_INFO "[%s] ioctl not implemented !\n", DEVICE_NAME);
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
    unsigned short int transfer_size;
    ssize_t transferred = 0;
    unsigned long src_addr, trgt_addr;

    struct drvr_mem * mem_to_read = &(((struct drvr_device *) filp->private_data)->data);

    #ifdef USE_WORD_ADDRESSING
        if (count % 2 != 0) {
            DBG_LOG("read: Transfer must be 16bits aligned\n");

            return -EFAULT;
        }
    #endif

        if (count < MAX_DMA_TRANSFER_IN_BYTES)
        {
            transfer_size = count;
        } else
        {
            transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
        }

    #ifdef USE_WORD_ADDRESSING
        src_addr = (unsigned long) &(mem_to_read->base_addr[(*f_pos) / 2]);
    #else
        src_addr = (unsigned long) &(mem_to_read->base_addr[(*f_pos)]);
    #endif

        trgt_addr = (unsigned long) dmaphysbuf;

        while (transferred < count) {

    #ifdef PROFILE
            DBG_LOG("Read\n");
            start_profile();
    #endif

            if(transfer_size <= 256) {
                memcpy(mem_to_read->dma.buf, (void *) &(mem_to_read->virt_addr[(*f_pos)+transferred/2]), transfer_size);
            }
            else {
                int res = logi_dma_copy(mem_to_read, trgt_addr, src_addr, transfer_size);

                if (res < 0) {
                    DBG_LOG("read: Failed to trigger EDMA transfer\n");

                    return res;
                }
            }

            if (copy_to_user(&buf[transferred], mem_to_read->dma.buf, transfer_size)) {
                return -EFAULT;
            }

    #ifdef PROFILE
            stop_profile();
            compute_bandwidth(transfer_size);
    #endif

            src_addr += transfer_size;
            transferred += transfer_size;

            if ((count - transferred) < MAX_DMA_TRANSFER_IN_BYTES) {
                transfer_size = (count - transferred);
            } else {
                transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
            }
        }

        return transferred;
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
    unsigned short int transfer_size;
    ssize_t transferred = 0;
    unsigned long src_addr, trgt_addr;
    struct drvr_mem * mem_to_write = &(((struct drvr_device *) filp->private_data)->data);

#ifdef USE_WORD_ADDRESSING
    if (count % 2 != 0) {
        DBG_LOG("write: Transfer must be 16bits aligned\n");

        return -EFAULT;
    }
#endif

    if (count < MAX_DMA_TRANSFER_IN_BYTES) {
        transfer_size = count;
    } else {
        transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
    }

#ifdef USE_WORD_ADDRESSING
    trgt_addr = (unsigned long) &(mem_to_write->base_addr[(*f_pos) / 2]);
#else
    trgt_addr = (unsigned long) &(mem_to_write->base_addr[(*f_pos)]);
#endif

    src_addr = (unsigned long) dmaphysbuf;

    if (copy_from_user(mem_to_write->dma.buf, buf, transfer_size)) {
        return -EFAULT;
    }

    if(transfer_size <= 256){
        memcpy((void *) &(mem_to_write->virt_addr[(*f_pos)]), mem_to_write->dma.buf,transfer_size);

        return transfer_size ;
    }

    while (transferred < count) {
        int res;

#ifdef PROFILE
        DBG_LOG("Write\n");
        start_profile();
#endif

        res = logi_dma_copy(mem_to_write, trgt_addr, src_addr,
                    transfer_size);
        if (res < 0) {
            DBG_LOG("write: Failed to trigger EDMA transfer\n");

            return res;
        }

#ifdef PROFILE
        stop_profile();
        compute_bandwidth(transfer_size);
#endif

        trgt_addr += transfer_size;
        transferred += transfer_size;

        if ((count - transferred) < MAX_DMA_TRANSFER_IN_BYTES) {
            transfer_size = count - transferred;
        } else {
            transfer_size = MAX_DMA_TRANSFER_IN_BYTES;
        }

        if (copy_from_user(mem_to_write->dma.buf, &buf[transferred], transfer_size)) {
            return -EFAULT;
        }
    }

    return transferred;
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

    printk(KERN_INFO "[%s] Set address at %d \n", DEVICE_NAME, vpbus.currentAddress);

    f->f_pos = newAddress;
    return f->f_pos;
}

//----------------------------------------------------------------------
/*! \brief Initialise le bus
 */
static int init_bus(void)
{
    int result = 0;
    //On reserve la plage mémoire pour l'accès au CPLD
    if(request_mem_region(BUS_BASE_ADDRESS, BUS_SIZE, DEVICE_NAME) == NULL)
    {
        printk(KERN_INFO "[%s] Failed to request I/O memory region ! \n", DEVICE_NAME);
        return -ENOMEM;
    }

    vpbus.bus_base_address = ioremap_nocache(BUS_BASE_ADDRESS, BUS_SIZE);

    if(vpbus.bus_base_address == NULL)
    {
        printk(KERN_INFO "[%s] Failed to remap I/O memory region ! \n", DEVICE_NAME);
        return -ENOMEM;
    }

    result = dma_open(vpbus.dma);

    return result;
}

//----------------------------------------------------------------------
/*! \brief Relache le bus
 */
static void deinit_bus(void)
{
    dma_release(vpbus.dma);
}

#ifdef PROFILE
//----------------------------------------------------------------------
/*! \brief Commence la mesure de temps
 */
static void start_profile()
{
    getnstimeofday(&start_ts);
}

//----------------------------------------------------------------------
/*! \brief Termine la mesure de temps
 */
static void stop_profile()
{
    getnstimeofday(&end_ts);
}

//-----------------------------------------------------------------------------------
/*! \brief Affiche le temps et la bande passante
 */
static void print_bandwidth(const unsigned int nb_byte)
{
    struct timespec dt = timespec_sub(end_ts, start_ts);
    double elapsed_us_time = dt.tv_sec * 1000000 + dt.tv_nsec / 1000;

    printk("[%s] %d bytes in %f us => %f kBytes/s", DEVICE_NAME, nb_byte, elapsed_ms_time, 1000000*(nb_byte>>10)/elapsed_us_time)
}
#endif
