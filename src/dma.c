#include "sam.h"
#include "compiler.h"
#include "utils.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include <string.h>
#include "SEGGER_RTT.h"

#include "dma.h"

COMPILER_ALIGNED(16)
static DmacDescriptor _descriptor_section[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;

COMPILER_ALIGNED(16)
static DmacDescriptor _descriptor_writeback_section[DMAC_CH_NUM] SECTION_DMAC_DESCRIPTOR;

static dma_callback_t callbacks[DMAC_CH_NUM];

void dma_set_callback(uint8_t channel, dma_callback_t callback) {
    configASSERT(channel < DMAC_CH_NUM);
    portENTER_CRITICAL();
    callbacks[channel] = callback;
    portEXIT_CRITICAL();
}

DmacDescriptor *dma_get_descriptor_for_channel(uint8_t channel) {
    configASSERT(channel < DMAC_CH_NUM);
    return &_descriptor_section[channel];
}

DmacDescriptor *dma_get_writeback_descriptor_for_channel(uint8_t channel) {
    configASSERT(channel < DMAC_CH_NUM);
    return &_descriptor_writeback_section[channel];
}

void dma_start_channel(uint8_t channel) {
    configASSERT(channel < DMAC_CH_NUM);
    portENTER_CRITICAL();
    hri_dmac_write_CHID_reg(DMAC, channel);
    hri_dmac_set_CHCTRLA_ENABLE_bit(DMAC);
    portEXIT_CRITICAL();
}

void dma_configure_channel(uint8_t channel, hri_dmac_chctrlb_reg_t trigact, hri_dmac_chctrlb_reg_t trigsrc) {
    configASSERT(channel < DMAC_CH_NUM);
    portENTER_CRITICAL();
    hri_dmac_write_CHID_reg(DMAC, channel);

    hri_dmac_write_CHCTRLB_TRIGACT_bf(DMAC, trigact);
    hri_dmac_write_CHCTRLB_TRIGSRC_bf(DMAC, trigsrc);

    if (callbacks[channel] != NULL) {
        hri_dmac_write_CHINTEN_TCMPL_bit(DMAC, true); // enable DMA complete interrupt
    } else {
        hri_dmac_write_CHINTEN_TCMPL_bit(DMAC, false); // disable DMA complete interrupt
    }

    portEXIT_CRITICAL();
}

void dma_init() {
    // clear all DMA descriptors and callbacks
    memset(_descriptor_section, 0, sizeof(_descriptor_section));
    memset(_descriptor_writeback_section, 0, sizeof(_descriptor_writeback_section));
    memset(callbacks, 0, sizeof(callbacks));

    // reset DMA
    hri_dmac_clear_CTRL_DMAENABLE_bit(DMAC);
    hri_dmac_clear_CTRL_CRCENABLE_bit(DMAC);
    hri_dmac_set_CHCTRLA_SWRST_bit(DMAC);

    // enable all priorities
    hri_dmac_set_CTRL_LVLEN0_bit(DMAC);
    hri_dmac_set_CTRL_LVLEN1_bit(DMAC);
    hri_dmac_set_CTRL_LVLEN2_bit(DMAC);
    hri_dmac_set_CTRL_LVLEN3_bit(DMAC);

    // set up DMA descriptor addresses
    hri_dmac_write_BASEADDR_reg(DMAC, (uint32_t)_descriptor_section);
    hri_dmac_write_WRBADDR_reg(DMAC, (uint32_t)_descriptor_writeback_section);

    NVIC_EnableIRQ(DMAC_IRQn);

    // start DMA
    hri_dmac_set_CTRL_DMAENABLE_bit(DMAC);
}

static void dma_run_callbacks(void *unused, uint32_t callbacks_to_activate) {
    for (size_t i=0; i<DMAC_CH_NUM; i++) {
        if ((callbacks_to_activate & (1 << i)) && callbacks[i] != NULL) {
            (*callbacks[i])();
        }
    }
}

// DMA handler
void DMAC_Handler() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t intstatus = hri_dmac_get_INTSTATUS_reg(DMAC, DMAC_INTSTATUS_MASK);

    xTimerPendFunctionCallFromISR(dma_run_callbacks, NULL, intstatus, &xHigherPriorityTaskWoken);

    for (int i=0; i<DMAC_CH_NUM; i++) {
        if (intstatus & (1 << i)) {
            hri_dmac_write_CHID_reg(DMAC, i);
            hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC);
        }
    }
}