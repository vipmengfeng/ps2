#include "timer.h"


inline void  ticks_timer_enable() {
//http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0439c/BABJFFGJ.html
 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Разрешаем TRACE SCB_DEMCR  |= 0x01000000;
 DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Разрешаем счетчик тактов DWT_CONTROL|= 1; // enable the counter
 DWT_CYCCNT  = 0; // Обнуляем счетчик
}

inline uint32_t get_ticks() {
  return DWT_CYCCNT;
}

inline uint32_t ticks_between(uint32_t start, uint32_t end) {
  return (uint32_t)(end - start);
}

inline uint32_t ticks_elapsed(uint32_t start) {
  return (uint32_t)(get_ticks() - start);
}

//TODO redefine as a MCU frequency parameter
#define TICKS_MULT (8)
#define TICKS_TO_MS_SHIFTER (3)
// 8 MHz == 1/8us per 1 tick
// делим на 8 ( == сдвигаем на 3 бита)
inline uint32_t time_elapsed(uint32_t start) {
  return ticks_elapsed(start)>>TICKS_TO_MS_SHIFTER;
}

inline uint32_t time_between(uint32_t start, uint32_t end) {
  return ticks_between(start, end)>>TICKS_TO_MS_SHIFTER;
}
