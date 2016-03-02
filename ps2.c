// vim: ai:ts=2

//TODO rename:
//     ps2_clk_line_set
//					ps2_clk_line_EXTI_enable
//					ps2_clk_line_EXTI_disable

#include <stm32f1xx.h>
#include "ps2.h"
#include "timer.h"

#define WATCHDOG_TIMEOUT (5000UL)
//#define WATCHDOG_TIMEOUT (0xFFFFFFFFul)
#define TIMEOUT_60uS (600UL)
#define TIMEOUT_10mS (0x0FFFFFFFUL)
//#define RECEIVE_ACK_DELAY (10000UL)

//http://embeddedgurus.com/stack-overflow/2009/02/effective-c-tips-2-defining-buffer-sizes/
#define RX_BUF_SIZE (4)
#define RX_BUF_MASK  (RX_BUF_SIZE - 1)
#if ( RX_BUF_SIZE & RX_BUF_MASK )
#error Rx buffer size is not a power of 2
#endif


uint8_t volatile ps2_rx_buf[RX_BUF_SIZE];
uint8_t volatile ps2_rx_buf_head;
uint8_t volatile ps2_rx_buf_len;


void ps2_clk_line_as_open_drain();
void ps2_data_line_as_open_drain();
void ps2_clk_enable_EXTI();
void ps2_clk_disable_EXTI();

void ps2_clk_line(uint8_t);
void ps2_set_data_line(uint8_t data);
uint8_t ps2_get_data_line();

/*------------------------------------------------------------*/
/*                                                            */
/*                Circular buffer routines                    */
/*                                                            */
/* http://embeddedgurus.com/stack-overflow/2009/02/effective-c-tips-2-defining-buffer-sizes */
/*------------------------------------------------------------*/

uint8_t PS2_buf_get_last() {
				uint8_t last = ps2_rx_buf[ (ps2_rx_buf_head + ps2_rx_buf_len) & RX_BUF_MASK];
				ps2_rx_buf_head--;
				ps2_rx_buf_len--;
				ps2_rx_buf_head &= RX_BUF_MASK;
				return last;
}

uint8_t PS2_buf_get_first() {
				uint8_t first = ps2_rx_buf[ (RX_BUF_SIZE + ps2_rx_buf_head - ps2_rx_buf_len) & RX_BUF_MASK ];
				ps2_rx_buf_len--;
				return first;
}

uint8_t PS2_buf_get_len() {
				return ps2_rx_buf_len;
}

/*------------------------------------------------------------*/
/*                                                            */
/*                Initialization                              */
/*                                                            */
/*------------------------------------------------------------*/

volatile uint8_t  ps2_flags;
volatile uint16_t ps2_rx_data = 0xFFFFu; // 0xFF - Keyboard scan code of error or buffer overflow
volatile uint16_t  ps2_tx_data;
volatile uint16_t ps2_rx_data2;
ps2_states volatile ps2_state;
volatile uint32_t rx_watchdog;
volatile uint32_t tx_watchdog;
volatile uint8_t  rx_parity;
volatile uint8_t  rx_counter;
volatile uint8_t ps2_get_data_line();


void PS2_Init(void)
{
				ps2_state = PS2_IDLE;
				ps2_rx_data = 0xFFFFu;
				ps2_tx_data = 0;

				ps2_rx_buf_head = 0;
				ps2_rx_buf_len = 0;

				ps2_flags &= ~(PS2_TX_DONE | PS2_RX_DONE);

				ps2_clk_line_as_open_drain();
				ps2_clk_line(1);
				ps2_clk_enable_EXTI();

				ps2_data_line_as_open_drain();
				ps2_set_data_line(1);

				rx_watchdog = 0;
				tx_watchdog = 0;

				ticks_timer_enable();
}

/*------------------------------------------------------------*/
/*                                                            */
/*                PS/2 protocol state machine                 */
/*                                                            */
/*------------------------------------------------------------*/


static inline void ps2_rx_isr() {

				if (time_elapsed(rx_watchdog) > WATCHDOG_TIMEOUT) {
								// Probably there had been a time out and/or we are recieving a new byte now, so resetting
								ps2_rx_data = 0xFFFFu;
								rx_parity = 0;
								ps2_flags &= ~PS2_RX_DONE;
				}

				ps2_state = PS2_RX_BUSY;
				uint8_t dataline = ps2_get_data_line();
				ps2_rx_data = (ps2_rx_data << 1 ) | dataline;
				rx_parity ^= dataline;
				rx_watchdog = get_ticks();
				// _____________________________________________
				//|STRT| 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | P |STOP|  // rx_data
				//|----+---+---+---+---+---+---+---+---+---+----|
				//| 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0  |  // 1 << x
				//+---------------------------------------------+
				if ( ~ps2_rx_data & PS2_START_BIT ) { // Inverting rx_data, as the initial value is 0xFF and START_BIT is eq. to 0
								//if ( !!(ps2_rx_data & PS2_PARITY_BIT) ^ (rx_parity & 1u))
								{
												ps2_flags |= PS2_RX_DONE;
												ps2_state = PS2_IDLE;//PS2_RX_START_ACK;

												if (ps2_rx_buf_len <= RX_BUF_SIZE) {
																ps2_rx_buf_head &= RX_BUF_MASK;
																ps2_rx_buf[ps2_rx_buf_head] = ps2_rx_data>>2;
																ps2_rx_buf_head++;
																ps2_rx_buf_len++;
																ps2_rx_data = 0xFFFFu;
																rx_parity = 0;
												}
								}
				}

				//return;
}

/*
// this function is only needed when we use push-pull configuration. in open drain it isn't needed.
void ps2_rx() {
				switch (ps2_state) {
								case PS2_RX_START_ACK:
												ps2_clk_line(0);
												rx_watchdog = get_ticks();
												ps2_state = PS2_RX_ACKING;
												break;
								case PS2_RX_ACKING:
												if (time_elapsed(rx_watchdog) > TIMEOUT_10mS) {
																//ps2_clk_line_Z_Pullup();
																ps2_clk_line(1);
																ps2_state = PS2_IDLE;
												}
												break;
								default:
												break;
				}
}
*/

uint8_t volatile tx_counter = 0;

inline void ps2_isr() {
				switch (ps2_state) {
								case PS2_TX_TRANSMITTING:
												ps2_set_data_line(ps2_tx_data & 1u);
												ps2_tx_data >>= 1u;

												if (ps2_tx_data == 0) {// Everyting has been sent (including STOP bit eq. to 1)
																//ps2_set_data_line(1u); // Already released while sending "STOP" bit  // "Releasing" data line (==High)
																ps2_state = PS2_TX_WAITING_ACK;
												}
												break;
								case PS2_TX_WAITING_ACK:
												//if (ps2_get_data_line() == 0UL) // ACK ==0, everything is Ok
												{
																ps2_flags |= PS2_TX_DONE;
																ps2_state = PS2_IDLE;
												}
												break;
								default:
												ps2_rx_isr();
												break;
				}
				//return;
}

static void ps2_tx () {
				switch (ps2_state) {

								case PS2_TX_REQUEST:
												ps2_flags &= ~(PS2_RX_DONE | PS2_TX_DONE);

												ps2_clk_disable_EXTI();
												ps2_clk_line(0);
												ps2_set_data_line(0);	// TX START bit = 0

												ps2_state = PS2_TX_REQ_DELAY;
												tx_watchdog = get_ticks();
												tx_counter = 0;
												break;

								case PS2_TX_REQ_DELAY:
												if (time_elapsed(tx_watchdog) < TIMEOUT_60uS) {
																//Holding CLK line Low for at least 60uS
																break;
												}

												ps2_clk_line(1);
												ps2_state = PS2_TX_TRANSMITTING;
												tx_watchdog = get_ticks();

												ps2_clk_enable_EXTI();
												break;

								case PS2_TX_TRANSMITTING:
												if (time_elapsed(tx_watchdog) > TIMEOUT_10mS) {
																//ERROR! The keyboard has not been sening CLK for more than 10ms
																ps2_set_data_line(1);
																ps2_state = PS2_IDLE;
																break;
												}
												break;

								default:
												break;
				}
}


void ps2_state_machine() {
				ps2_tx();
				//  ps2_rx();
}


void ps2_send_command(uint8_t cmd) {
				uint16_t parity = 0;
				for (int i=0; i<8; i++) {
								parity ^= (cmd>>i);
				}
				ps2_tx_data = cmd | ((~parity&1)<<8) | (1<<9) ;
				ps2_state = PS2_TX_REQUEST;
}
/*------------------------------------------------------------*/
/*                                                            */
/*                DATA line routines                          */
/*                                                            */
/*------------------------------------------------------------*/
static inline void ps2_set_data_line(uint8_t data) { GPIOB->BSRR = (data)? GPIO_BSRR_BS4: GPIO_BSRR_BR4; }

static inline uint8_t ps2_get_data_line() {return !!(GPIOB->IDR & GPIO_IDR_IDR4); }

static void ps2_data_line_as_open_drain()
{
				GPIOB->CRL &= ~(GPIO_CRL_CNF4); // CNF = 00 -- push-pull output
				GPIOB->CRL |=  (GPIO_CRL_CNF4_0); // CNF = 01 -- open drain 
				GPIOB->CRL &= ~(GPIO_CRL_MODE4); //  Mode = 00
				GPIOB->CRL |=  (GPIO_CRL_MODE4_1); //  Mode = 10 -- 2MHz
}

/*------------------------------------------------------------+
	|                                                             |
	|                 KBD line routines                           |
	|                                                             |
	+------------------------------------------------------------*/

static inline void ps2_clk_line(uint8_t data)  { GPIOB->BSRR = (data)? GPIO_BSRR_BS3: GPIO_BSRR_BR3; }

static void ps2_clk_disable_EXTI() {
				NVIC_DisableIRQ(EXTI2_IRQn);
				// Disabling the interrupts
				EXTI->IMR &= ~(EXTI_IMR_MR3);
				// отвязываем прерывание по падающему фронту
				EXTI->FTSR &= ~EXTI_FTSR_TR3;
				EXTI->RTSR &= ~EXTI_RTSR_TR3;
}

static void ps2_clk_enable_EXTI() {
				// Включаем EXTI3
				// Настраиваем внешние прерывания EXTI3 на PB3
				AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PB;
				// Прерывание по падению/фронту уровня на ноге 0 порта привязанного к EXTI
				EXTI->FTSR |= EXTI_FTSR_TR3;
				//EXTI->RTSR |= EXTI_RTSR_TR3;

				// Разрешаем прерывания в периферии
				EXTI->IMR |=(EXTI_IMR_MR3);
				// Функции CMSIS разрешающие прерывания в NVIC
				NVIC_EnableIRQ(EXTI3_IRQn);
}

static void ps2_clk_line_as_open_drain(void) {
				GPIOB->CRL &= ~(GPIO_CRL_CNF3); // CNF = 00 -- push-pull output
				GPIOB->CRL |=  (GPIO_CRL_CNF3_0); // CNF = 01 -- open drain 
				GPIOB->CRL &= ~(GPIO_CRL_MODE3); //  Mode = 00
				GPIOB->CRL |=  (GPIO_CRL_MODE3_1); //  Mode = 10 -- 2MHz
}




