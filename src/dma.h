//
// Created by jeremy on 7/5/20.
//

#ifndef POLYGLOT_TURTLE_XIAO_PROJ_DMA_H
#define POLYGLOT_TURTLE_XIAO_PROJ_DMA_H

typedef void (*dma_callback_t)(void);

void dma_set_callback(uint8_t channel, dma_callback_t callback);
DmacDescriptor *dma_get_descriptor_for_channel(uint8_t channel);
DmacDescriptor *dma_get_writeback_descriptor_for_channel(uint8_t channel);
void dma_start_channel(uint8_t channel);
void dma_configure_channel(uint8_t channel, hri_dmac_chctrlb_reg_t trigact, hri_dmac_chctrlb_reg_t trigsrc);
void dma_init();

#endif //POLYGLOT_TURTLE_XIAO_PROJ_DMA_H
