#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "nvic.h"
#include "io_impl.h"
#include "exti.h"

// #ifdef USE_EXTI

typedef struct {
    extiCallbackRec_t* handler;
} extiChannelRec_t;

extiChannelRec_t extiChannelRecs[16];

// IRQ gouping, same on 103 and 303
#define EXTI_IRQ_GROUPS 7
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static const uint8_t extiGroups[16] = { 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6 };
static uint8_t extiGroupPriority[EXTI_IRQ_GROUPS];

static const uint8_t extiGroupIRQn[EXTI_IRQ_GROUPS] = {
    EXTI0_IRQn,
    EXTI1_IRQn,
    EXTI2_TS_IRQn,
    EXTI3_IRQn,
    EXTI4_IRQn,
    EXTI9_5_IRQn,
    EXTI15_10_IRQn
};


void EXTIInit(void)
{
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    memset(extiChannelRecs, 0, sizeof(extiChannelRecs));
    memset(extiGroupPriority, 0xff, sizeof(extiGroupPriority));
}

void EXTIHandlerInit(extiCallbackRec_t *self, extiHandlerCallback *fn)
{
    self->fn = fn;
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, EXTITrigger_TypeDef trigger)
{
    int chIdx;
    chIdx = IO_GPIOPinIdx(io);
    if(chIdx < 0)
        return;
    extiChannelRec_t *rec = &extiChannelRecs[chIdx];
    int group = extiGroups[chIdx];

    rec->handler = cb;

    SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io), IO_EXTI_PinSource(io));

    uint32_t extiLine = IO_EXTI_Line(io);

    EXTI_ClearITPendingBit(extiLine);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = extiLine;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = trigger;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    if(extiGroupPriority[group] > irqPriority) {
        extiGroupPriority[group] = irqPriority;

        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = extiGroupIRQn[group];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
}

void EXTIRelease(IO_t io)
{
    // don't forget to match cleanup with config
    EXTIEnable(io, false);

    int chIdx;
    chIdx = IO_GPIOPinIdx(io);
    if(chIdx < 0)
        return;
    extiChannelRec_t *rec = &extiChannelRecs[chIdx];
    rec->handler = NULL;
}

void EXTIEnable(IO_t io, bool enable)
{
    int extiLine = IO_EXTI_Line(io);
    if(extiLine < 0)
        return;
    // assume extiLine < 32 (valid for all EXTI pins)
    if(enable)
        EXTI->IMR |= 1 << extiLine;
    else
        EXTI->IMR &= ~(1 << extiLine);
}

void EXTI_IRQHandler(void)
{
    uint32_t exti_active = EXTI->IMR & EXTI->PR;

    while(exti_active) {
        unsigned idx = 31 - __builtin_clz(exti_active);
        uint32_t mask = 1 << idx;
        extiChannelRecs[idx].handler->fn(extiChannelRecs[idx].handler);
        EXTI->PR = mask;  // clear pending mask (by writing 1)
        exti_active &= ~mask;
    }
}

#define _EXTI_IRQ_HANDLER(name)                 \
    void name(void) {                           \
        EXTI_IRQHandler();                      \
    }                                           \
    struct dummy                                \
    /**/


_EXTI_IRQ_HANDLER(EXTI0_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI1_IRQHandler);

_EXTI_IRQ_HANDLER(EXTI2_TS_IRQHandler);

_EXTI_IRQ_HANDLER(EXTI3_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI4_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI9_5_IRQHandler);
_EXTI_IRQ_HANDLER(EXTI15_10_IRQHandler);

// #endif
