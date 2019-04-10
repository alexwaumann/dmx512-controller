#include "tm4c123gh6pm"

#include <stdint.h>

int dmx512_state;

void init_hw( void );

void init_uart_coms( uint32_t sys_clock, uint32_t baud_rate );
void init_uart_dmx( void );

int main(void)
{
    init_hw();

    while( 1 );
}

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

void init_uart_dmx( void )
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;    // enable PORTC module
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;    // enable UART1 module
    GPIO_PORTC_AFSEL_R |=

    //TIMER1 :: for DMX-512 Break and MAB
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  //turn on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            //turn off timer before configuring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;      //configure as 32-bit timer (concatenated)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;     //configure in one-shot mode (count down)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;            //turn on timer1 interrupt
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);      //turn on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;             //turn on timer
    dmx512_state = 0;
    // setup UART1 using ports PC4-5
    // setup interrupts
}

void Timer1ISR(void){
    if(dmx512_state == 0){

    }else if(dmx512_state == 1){

    }

}
