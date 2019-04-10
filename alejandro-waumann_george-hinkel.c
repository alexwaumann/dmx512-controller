#include "tm4c123gh6pm.h"

#include <stdint.h>

#define PC5_MASK 32
#define DMX_TX (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define DMX_DE (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))

int dmx512_state;
int dmx512_max = 512;
char dmx512_data[513];

void init_hw( void );

void init_uart_coms( uint32_t sys_clock, uint32_t baud_rate );
void init_uart_dmx( void );

void init_hw( void )
{
}

void init_uart_coms( uint32_t sys_clock, uint32_t baud_rate )
{
    // setup UART0 for COMS PORT
    // setup interrupts
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;    // enable UART0 module
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;    // enable PORTA module
    GPIO_PORTA_AFSEL_R |= 2;                    // enable peripheral control for PA1 (TX)
                                                // default: 2ma drive
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX;    // UART TX on PA1

    float    brd  = (float) sys_clock / (16.0f * (float) baud_rate);
    uint16_t bri  = (int) brd;
    uint8_t  brf  = (int) ((brd - (float) bri) * 64.0f + 0.5f);
             brf &= 0x3F;                       // only need bottom 6 bits

    UART0_CTL_R  &= ~UART_CTL_UARTEN;           // disable UART0 for configuration
    UART0_IBRD_R |= bri;                        // baud rate divisor integer part
    UART0_FBRD_R |= brf;                        // baud rate divisor fractional part
    UART0_LCRH_R |= UART_LCRH_WLEN_8;           // 8-bit data frame
    UART0_CC_R    = UART_CC_CS_SYSCLK;          // use system clock
    UART0_CTL_R  |= UART_CTL_UARTEN             // enable UART0 and TX
                 |  UART_CTL_TXE;
}

void dmx_prime(void)
{
    //Break
    GPIO_PORTC_AFSEL_R  &= ~32;                 // clear 6th bit to disable peripheral control for PC5
    TIMER1_TAILR_R = 7040;                      // set Timer1A ILR to appropriate value for 176 us
    DMX_DE = 1;                                 // drive DMX_DE high to enable transmission
    DMX_TX = 0;                                 // drive DMX_TX low
    TIMER1_CTL_R |= TIMER_CTL_TAEN;             // turn on timer 1
}

void init_uart_dmx( void )
{
    dmx512_state = 0;

    //GPIO PORTC :: for DMX-512
    SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R2;   // enable PORTC module
    GPIO_PORTC_DIR_R    |= 64 | 32;             // set PC5 direction as output
    GPIO_PORTC_DEN_R    |= 64 | 32 | 16;        // set PC5/DMX_TX and PC4 as digital I/O
    GPIO_PORTC_ODR_R    &= ~112;                // set PC5 and PC4 ODR to 0

    //UART1 :: for DMX-512 Transmission
    float    brd  = (float) 40000000 / (16.0f * (float) 250000);
    uint16_t bri  = (int) brd;
    uint8_t  brf  = (int) ((brd - (float) bri) * 64.0f + 0.5f);
             brf &= 0x3F;                       // only need bottom 6 bits

    SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R1;   // enable UART1 module
    UART1_CTL_R  &= ~UART_CTL_UARTEN;           // disable UART1 for configuration
    UART1_IBRD_R |= bri;                        // baud rate divisor integer part
    UART1_FBRD_R |= brf;                        // baud rate divisor fractional part
    UART1_LCRH_R |= UART_LCRH_WLEN_8            // 8-bit data frame
                 |  UART_LCRH_STP2              // 2 stop-bits
                 |  UART_LCRH_FEN;              // enable transmit and receive FIFOs
    UART1_CC_R    = UART_CC_CS_SYSCLK;          // use system clock
    UART1_IFLS_R |= UART_IFLS_TX1_8;            // set transmit interrupt to go off when transmit FIFO is 1/8th full
    NVIC_EN0_R |= 1 << (INT_UART1 - 16);        // turn on interrupt 22 (UART1)

    //TIMER1 :: for DMX-512 Break and MAB
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  // turn on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            // turn off timer before configuring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;      // configure as 32-bit timer (concatenated)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;     // configure in one-shot mode (count down)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;            // turn on timer1 interrupt
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);      // turn on interrupt 37 (TIMER1A)

    //Break
    dmx_prime();
}

void Uart1ISR(void){
    for(int i=0;i<14;i++){
        UART1_DR_R = dmx512_data[(dmx512_state++) - 2];
        if((dmx512_state - 2) >= dmx512_max ){
            dmx_prime();
            return;
        }
    }
}

void Timer1ISR(void){
    if(dmx512_state == 0){
        //TODO set TAILR_R to value for 12 us
        dmx512_state = 1;                           // advance dmx512 state
        TIMER1_TAILR_R = 480;                       // set Timer1A ILR to appropriate value for 12 us
        DMX_TX = 1;                                 // drive DMX_TX high
        TIMER1_CTL_R |= TIMER_CTL_TAEN;             // turn on timer 1
    }
    else if(dmx512_state == 1){
        dmx512_state = 2;                           // advance dmx512 state
        GPIO_PORTC_AFSEL_R |= 32 | 16;              // set 5th and 6th bits to enable peripheral control for PC5 and PC4
        GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC5_U1TX    // UART1 TX ON PC5
                           |  GPIO_PCTL_PC4_U1RX;   // UART1 RX on PC4
        UART1_CTL_R  |= UART_CTL_UARTEN             // enable UART1
                     |  UART_CTL_TXE;               // enable UART1 transmission
        //UART1_DR_R = dmx512_data[(dmx512_state++) - 2]; //maybe the interrupt will occur any way
    }
}

int main(void)
{
    init_hw();

    while( 1 );
}
