/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 * Inspired from ValentFx Logi-Bone source: https://github.com/fpga-logi/logi-kernel
 */

#include "dma.h"

//================================================================================
//                              Fonctions
//================================================================================

//----------------------------------------------------------------------
/*! \brief Fonction de callback lorsque l'action est terminée
 */
#ifdef USE_DMA_ENGINE
static void dma_callback(void* param)
{
    dma_channel_t* dma_channel = (dma_channel_t*) param;

    switch(dma_async_is_tx_complete(dma_channel->chan, dma_channel->cookie, NULL, NULL))
    {
        case DMA_COMPLETE:
            dma_channel->result = 1;
            break;

        case DMA_ERROR:
            dma_channel->result = -1;
            break;

        default:
            dma_channel->result = -1;
            break;
    }

    complete(&dma_channel->completion);
}
#else
static void dma_callback(unsigned lch, u16 ch_status, void* data)
{
    dma_channel_t* dma_channel = (dma_channel_t*) data;

    switch (ch_status)
    {
        case EDMA_DMA_COMPLETE:
            dma_channel->result = 1;
            break;

        case EDMA_DMA_CC_ERROR:
            dma_channel->result = -1;
            break;

        default:
            dma_channel->result = -1;
            break;
    }

    complete(&dma_channel->completion);
}
#endif /* USE_DMA_ENGINE */

//----------------------------------------------------------------------
/*! \brief Fonction d'initialisation du dma
 */
int dma_open(dma_channel_t* dma_channel)
{
#ifdef USE_DMA_ENGINE
    struct dma_slave_config	conf;
    dma_cap_mask_t mask;
#endif

    /* Allocate DMA buffer */
    dma_channel->buffer = dma_alloc_coherent(NULL,
                                             MAX_DMA_TRANSFER_IN_BYTES,
                                             &dma_channel->physbuf,
                                             0);

    if(!dma_channel->buffer)
    {
        printk("Failed to allocate DMA buffer\n");
        return -ENOMEM;
    }

#ifdef USE_DMA_ENGINE
    /* Allocate DMA channel */
    dma_cap_zero(mask);
    dma_cap_set(DMA_MEMCPY, mask);
    dma_channel->chan = dma_request_channel(mask, NULL, NULL);

    if (!dma_channel->chan)
    {
        return -ENODEV;
    }

    /* Configure DMA channel */
    conf.direction = DMA_MEM_TO_MEM;
    /*conf.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;*/
    dmaengine_slave_config(dma_channel->chan, &conf);

    printk("Using Linux DMA Engine API");
#else
    dma_channel->id = edma_alloc_channel(EDMA_CHANNEL_ANY, dma_callback, dma_channel, EVENTQ_0);

    if(dma_channel->id < 0)
    {
        printk("edma_alloc_channel failed for dma_ch, error: %d\n", dma_channel->id);
        return dma_channel->id;
    }

    printk("Using EDMA/DMA Engine");
    printk("EDMA channel %d reserved\n", dma_channel->id);
#endif /* USE_DMA_ENGINE */

    init_completion(&dma_channel->completion);

    return 0;
}

//----------------------------------------------------------------------
/*! \brief Relache le dma utilisé
 */
void dma_release(dma_channel_t* dma_channel)
{
    #ifdef USE_DMA_ENGINE
    dma_release_channel(dma_channel->chan);
    #else
    edma_free_channel(dma_channel->id);
    #endif /* USE_DMA_ENGINE */
    dma_free_coherent(NULL, MAX_DMA_TRANSFER_IN_BYTES, dma_channel->buffer, dma_channel->physbuf);
}

//----------------------------------------------------------------------
/*! \brief Fonction d'initialisation du dma
 */
int dma_copy(dma_channel_t* dma_channel, unsigned long dest_addr, unsigned long src_addr, int count)
{
    int result = 0;

#ifdef USE_DMA_ENGINE
    struct dma_device* dev;
    struct dma_async_tx_descriptor* tx;
    unsigned long flags;

    dev = dma_channel->chan->device;
    flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
    tx = dev->device_prep_dma_memcpy(dma_channel->chan, dest_addr, src_addr, count, flags);

    if (!tx)
    {
        printk("device_prep_dma_memcpy failed\n");
        return -ENODEV;
    }

    dma_channel->result = 0u;
    dma_channel->.done = 0;
    /* set the callback and submit the transaction */
    tx->callback = dma_callback;
    tx->callback_param = dma_channel;
    cookie = dmaengine_submit(tx);
    dma_async_issue_pending(chan);
#else
    struct edmacc_param param_set;

    edma_set_src(dma_channel->id, src_addr, INCR, W256BIT);
    edma_set_dest(dma_channel->id, dest_addr, INCR, W256BIT);
    edma_set_src_index(dma_channel->id, 1, 1);
    edma_set_dest_index(dma_channel->id, 1, 1);
    /* A Sync Transfer Mode */
    edma_set_transfer_params(dma_channel->id, count, 1, 1, 1, ASYNC);//one block of one frame of one array of count bytes

    /* Enable the Interrupts on Channel 1 */
    edma_read_slot(dma_channel->id, &param_set);
    param_set.opt |= ITCINTEN;
    param_set.opt |= TCINTEN;
    param_set.opt |= EDMA_TCC(EDMA_CHAN_SLOT(dma_channel->id));
    edma_write_slot(dma_channel->id, &param_set);
    dma_channel->result = 0u;
    dma_channel->completion.done = 0;
    result = edma_start(dma_channel->id);

    if (result != 0)
    {
        printk("edma copy failed\n");
        return result;
    }

#endif /* USE_DMA_ENGINE */

    wait_for_completion(&dma_channel->completion);

    /* Check the status of the completed transfer */

    if (dma_channel->result < 0)
    {
        printk("edma copy: Event Miss Occured!!!\n");
#ifdef USE_DMA_ENGINE
        dmaengine_terminate_all(dma_channel->chan);
#else
        edma_stop(dma_channel->id);
#endif /* USE_DMA_ENGINE */
        result = -EAGAIN;
    }

    return result;
}
