#include "tm4c123gh6pm"

#include <stdint.h>

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
    // setup UART1 using ports PC4-5
    // setup interrupts
}
