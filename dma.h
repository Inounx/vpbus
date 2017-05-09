/*
 * Copyrigth © 2017 Virtu'Ailes
 * Author: Marc Laval
 * inounx@gmail.com
 * Inspired from ValentFx Logi-Bone source: https://github.com/fpga-logi/logi-kernel
 */

#ifndef DMA_H
#define DMA_H

#include <linux/types.h>
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/edma.h>
#include <linux/version.h>

#define MAX_DMA_TRANSFER_IN_BYTES   (32768)

//auto detect Linux DMA Engine API (Kernel 4.4+)
#ifndef EDMA_DMA_COMPLETE
#define USE_DMA_ENGINE
#endif

typedef struct
{
    void * buffer;
    dma_addr_t physbuf;
    volatile int result;
    struct completion completion;

    #ifdef USE_DMA_ENGINE
    struct dma_chan* chan;
    dma_cookie_t cookie;
    #else
    int id;
    #endif
} dma_channel_t;

int  dma_open(dma_channel_t* dma_channel);
void dma_release(dma_channel_t* dma_channel);
int  dma_copy(dma_channel_t* dma_channel, unsigned long dest_addr, unsigned long src_addr, int count);

#endif
