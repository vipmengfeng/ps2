#include <stm32f1xx.h>

#define    DWT_CYCCNT    *(volatile uint32_t *)0xE0001004
#define    DWT_CONTROL   *(volatile uint32_t *)0xE0001000
#define    SCB_DEMCR     *(volatile uint32_t *)0xE000EDFC

extern void  ticks_timer_enable();
extern  uint32_t get_ticks();

extern  uint32_t ticks_elapsed(uint32_t start);
extern  uint32_t ticks_between(uint32_t start, uint32_t end);
extern  uint32_t time_elapsed(uint32_t start);
extern  uint32_t time_between(uint32_t start, uint32_t end);
