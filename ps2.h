// Circular buffer routines
uint8_t PS2_buf_get_last();
uint8_t PS2_buf_get_first();
uint8_t PS2_buf_get_len();



#define PS2_RX_DONE (1<<0)
#define PS2_TX_DONE (1<<1)

#define PS2_START_BIT  (1U<<10U)
#define PS2_PARITY_BIT (1U<<1U)



typedef enum {PS2_IDLE, PS2_RX_BUSY,  PS2_RX_START_ACK, PS2_RX_ACKING, \
														PS2_TX_REQUEST, PS2_TX_REQ_DELAY, PS2_TX_TRANSMITTING,PS2_TX_WAITING_ACK} ps2_states;

extern volatile ps2_states ps2_state;
extern volatile uint8_t  ps2_flags;

void PS2_Init(void);
void ps2_isr();

void ps2_state_machine();
void ps2_send_command(uint8_t cmd);
