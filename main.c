/*
#include "tm4c123gh6pm.h"

#include <stdint.h>
#define DMX_TX (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))

int dmx512_phase = 0;

void init_hw( void );

void init_uart_coms( uint32_t sys_clock, uint32_t baud_rate );
void init_uart_dmx( void );

int main(void)
{
    init_hw();

    while( 1 ){


    }
}

void init_hw( void )
{
    //set PC6 as digital output
    SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R2;   // enable PORTC module
    GPIO_PORTC_DIR_R    |= 32;                  // set PC5 direction as output
    GPIO_PORTC_DEN_R    |= 32 | 16;             // set PC5/DMX_TX and PC4 as digital I/O
    GPIO_PORTC_ODR_R    &= ~48;                 // set PC5 and PC4 ODR to 0
    DMX_TX = 1;
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

    //UART0 :: for
    UART0_CTL_R  &= ~UART_CTL_UARTEN;           // disable UART0 for configuration
    UART0_IBRD_R |= bri;                        // baud rate divisor integer part
    UART0_FBRD_R |= brf;                        // baud rate divisor fractional part
    UART0_LCRH_R |= UART_LCRH_WLEN_8;           // 8-bit data frame
    UART0_CC_R    = UART_CC_CS_SYSCLK;          // use system clock
    UART0_CTL_R  |= UART_CTL_UARTEN             // enable UART0 and TX
                 |  UART_CTL_TXE;

    //TIMER1 :: for DMX-512 Break and MAB
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  //turn on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            //turn off timer before configuring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;      //configure as 32-bit timer (concatenated)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;     //configure in one-shot mode (count down)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;            //turn on timer1 interrupt
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);      //turn on interrupt 37 (TIMER1A)
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;             //turn on timer
       
}

void init_uart_dmx( void )
{
    // setup UART1 using ports PC4-5
    // setup interrupts
    TIMER1_TAILR_R = 5682;                      //set timer for 176 us
    //drive pc6 to 0
    TIMER1_CTL_R |= TIMER_CTL_TAEN;             //turn on timer

}

Timer1ISR(){


}
*/
