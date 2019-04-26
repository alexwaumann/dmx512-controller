/**/
#include "tm4c123gh6pm.h"

#include <stdint.h>
#include <string.h>

#define PC5_MASK 32
#define DMX_TX (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define DMX_DE (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define MODE_EEPROM_BLOCK 0
#define MODE_EEPROM_WORD 0
#define ADDR_EEPROM_BLOCK 1
#define ADDR_EEPROM_WORD 0
#define EEPROM_STAT_BLOCK 2
#define EEPROM_STAT_WORD 0

//general globals
char MODE = 'c';                            //tracks whether in controller or device mode
char coms_cmd[128];                         //buffer for virtual COMS port
unsigned char coms_index = 0;               //index for virtual COMS port buffer
unint32_t EEPROM_STAT = 0;              //if 0 EEPROM was never written, else the MODE and ADDR have been stored

//controller mode globals
unsigned char dmx512_data[513];             //stores values of DMX512 data to be transmitted
unsigned int dmx512_max = 512;              //stores DMX max value
unsigned int dmx512_state = 0;              //global to control state of DMX512 Transmission algorithm

//device mode globals
unsigned int dmx512_rx_index = 0;           //index of DMX512 receive buffer
uint32_t ADDR = 0;                      //stores dmx listening address
unsigned char dmx512_rx_data = 0;           //stores dmx data received at listening address
unsigned char dmx512_rx_buffer[513];        //stores all received dmx data

void init_hw( void );

void init_uart_coms( uint32_t sys_clock, uint32_t baud_rate );
void init_uart_dmx( void );

void init_hw( void )
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and C peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC;

    init_uart_coms(40000000,115200);
    init_uart_dmx();
}

//function that configures UART for virtual COMS port
void init_uart_coms( uint32_t sys_clock, uint32_t baud_rate )
{
    // setup UART0 for COMS PORT
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;    // enable UART0 module
    //SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;    // enable PORTA module
    GPIO_PORTA_DEN_R |= 3;
    GPIO_PORTA_AFSEL_R |= 3;                    // enable peripheral control for PA0 and PA1
                                                // default: 2ma drive
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;   // UART TX on PA1


    float    brd  = (float) sys_clock / (16.0f * (float) baud_rate);
    uint16_t bri  = (int) brd;
    uint8_t  brf  = (int) ((brd - (float) bri) * 64.0f + 0.5f);
             brf &= 0x3F;                       // only need bottom 6 bits

    UART0_CTL_R   = 0;                          // disable UART0 for configuration
    UART0_CC_R    = UART_CC_CS_SYSCLK;          // use system clock
    UART0_IBRD_R  = bri;                        // baud rate divisor integer part
    UART0_FBRD_R  = brf;                        // baud rate divisor fractional part
    UART0_LCRH_R  = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // 8-bit data frame with fifo
    UART0_CTL_R   = UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE; // enable UART0 and TX

}

//function that configures the UART, GPIO, and TIMER for DMX512
void init_uart_dmx( void )
{
    //GPIO PORTC :: for DMX-512
    SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R2;   // enable PORTC module
    GPIO_PORTC_DIR_R    |= 64 | 32;             // set PC5 direction as output
    GPIO_PORTC_DEN_R    |= 64 | 32 | 16;        // set PC5/DMX_TX and PC4 as digital I/O
    GPIO_PORTC_ODR_R    &= ~112;                // set PC5 and PC4 ODR to 0

    //UART1 :: for DMX-512
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
    UART1_IFLS_R |= UART_IFLS_TX1_8             // set TX interrupt to go off when TX FIFO is 1/8th full
                 |  UART_IFLS_RX7_8;            // set RX interrupt to go off when RX FIFO is 7/8th full
    NVIC_EN0_R |= 1 << (INT_UART1 - 16);        // turn on interrupt 22 (UART1)

    //TIMER1 :: for DMX-512 Break and MAB
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;  // turn on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;            // turn off timer before configuring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;      // configure as 32-bit timer (concatenated)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;     // configure in one-shot mode (count down)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;            // turn on timer1 interrupt
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);      // turn on interrupt 37 (TIMER1A)
}

//function that "primes the pump" for dmx transmission
void dmx_prime(void)
{
    //Break
    dmx512_state = 0;
    GPIO_PORTC_AFSEL_R  &= ~32;                 // clear 6th bit to disable peripheral control for PC5
    TIMER1_TAILR_R = 7040;                      // set Timer1A ILR to appropriate value for 176 us
    DMX_DE = 0;                                 // drive DMX_DE low to enable transmission
    DMX_TX = 0;                                 // drive DMX_TX low to create break condition
    TIMER1_CTL_R |= TIMER_CTL_TAEN;             // turn on timer 1
}

//function that starts transmitting dmx data
void init_dmx_tx(void)
{
    UART1_IM_R |= UART_IM_TXIM;                 //enable the TX interrupt mask to keep the tx buffer full
    UART1_IM_R &= ~UART_IM_RXIM                 //disable the RX interrupt mask for controller mode
               &  ~UART_IM_BEIM;                //disable the Break Error interrupt mask for controller mode
    dmx_prime();
}

//function that starts receiving dmx data
void init_dmx_rx(void)
{
    UART1_IM_R |= UART_IM_RXIM                  //enable the RX interrupt mask to keep the RX buffer not full
               |  UART_IM_BEIM;                 //enable the Break Error interrupt mask to see breaks
    UART1_IM_R &= ~UART_IM_TXIM;                //disable the TX interrupt mask for Device mode
    DMX_DE = 1;                                 //drive DMX_DE high to disable transmission
}

//UART1 interrupt service routine, manages TX and RX fifos as well as break conditions in device mode
void Uart1ISR(void) //TODO: add handling for UART error conditions
{
    //TX INTERRUPT -> fill TX fifo
    if(UART1_MIS_R & UART_MIS_TXMIS)
    {
        while(!(UART1_FR_R & UART_FR_TXFF))//while TX fifo not full
        {
            UART1_DR_R = dmx512_data[(dmx512_state++) - 2];
            if((dmx512_state - 2) >= dmx512_max )
            {
                while(UART1_FR_R & UART_FR_BUSY);//wait until transmission completes
                UART1_ICR_R |= UART_ICR_TXIC;   // clear the TX interrupt
                dmx_prime();
                return;
            }
        }
        UART1_ICR_R |= UART_ICR_TXIC;   // clear the TX interrupt flag
    }

    //RX INTERRUPT -> empty RX fifo
    else if(UART1_MIS_R & UART_MIS_RXMIS)
    {
        while(!(UART1_FR_R & UART_FR_RXFE))//while RX fifo not empty
        {
            dmx512_rx_buffer[dmx512_rx_index++] = UART1_DR_R & 0xFF;
        }
        UART1_ICR_R |= UART_ICR_RXIC;   // clear the RX interrupt flag
    }

    //BE INTERRUPT -> empty RX fifo and clear receive buffer
    if(UART1_MIS_R & UART_MIS_BEMIS)
    {
        while(!(UART1_FR_R & UART_FR_RXFE))//while RX fifo not empty
        {
            dmx512_rx_buffer[dmx512_rx_index++] = UART1_DR_R & 0xFF;
        }
        dmx512_rx_index = 0;
        dmx512_rx_data = dmx512_rx_buffer[ADDR];
        UART1_ICR_R |= UART_ICR_BEIC;   // clear the BE interrupt flag
    }
}

//TIMER1 interrupt service routine, handles priming the pump and starting uart for DMX transmission
void Timer1ISR(void)
{
    if(dmx512_state == 0)
    {
        dmx512_state = 1;                           // advance dmx512 state
        TIMER1_TAILR_R = 480;                       // set Timer1A ILR to appropriate value for 12 us
        DMX_TX = 1;                                 // drive DMX_TX high
        TIMER1_CTL_R |= TIMER_CTL_TAEN;             // turn on timer 1
    }
    else if(dmx512_state == 1)
    {
        dmx512_state = 2;                           // advance dmx512 state
        GPIO_PORTC_AFSEL_R |= 32 | 16;              // set 5th and 6th bits to enable peripheral control for PC5 and PC4
        GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC5_U1TX    // UART1 TX ON PC5
                           |  GPIO_PCTL_PC4_U1RX;   // UART1 RX on PC4
        UART1_CTL_R  |= UART_CTL_UARTEN             // enable UART1
                     |  UART_CTL_TXE;               // enable UART1 transmission
        //UART1_DR_R = dmx512_data[(dmx512_state++) - 2]; //maybe the interrupt will occur any way
    }
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    int i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

int parse(char cmd[128])
{
    int i = 0;
    for(i=0;i<coms_index;i++){
        switch(i){

        }
    }
    coms_index = 0;
    return 0;
}

//TODO: add init_eeprom function and put in init_hw

//function that loads MODE and ADDR vars from EEPROM if they were saved
void recover_from_reset(void)
{
    EEPROM_EEBLOCK_M = EEPROM_STAT_BLOCK;
    EEPROM_EEOFFSET_M = EEPROM_STAT_WORD;
    EEPROM_STAT = EEPROM_EERDWR_M;
    if(EEPROM_STAT)
    {
        EEPROM_EEBLOCK_M = MODE_EEPROM_BLOCK;
        EEPROM_EEOFFSET_M = MODE_EEPROM_WORD;
        MODE = EEPROM_EERDWR_M & 0xFF;
        EEPROM_EEBLOCK_M = ADDR_EEPROM_BLOCK;
        EEPROM_EEOFFSET_M = ADDR_EEPROM_WORD;
        ADDR = EEPROM_EERDWR_M;
    }
}

//function that saves MODE and ADDR to EEPROM and sets the EEPROM_STAT flag and saves it too
void save_mode_and_addr(void)
{
    EEPROM_EEBLOCK_M = MODE_EEPROM_BLOCK;
    EEPROM_EEOFFSET_M = MODE_EEPROM_WORD;
    EEPROM_EERDWR_M = MODE;
    EEPROM_EEBLOCK_M = ADDR_EEPROM_BLOCK;
    EEPROM_EEOFFSET_M = ADDR_EEPROM_WORD;
    EEPROM_EERDWR_M = ADDR;
    EEPROM_STAT = 1;
    EEPROM_EEBLOCK_M = EEPROM_STAT_BLOCK;
    EEPROM_EEOFFSET_M = EEPROM_STAT_WORD;
    EEPROM_EERDWR_M = EEPROM_STAT;
}

int main(void)
{
    init_hw();
    //recover_from_reset();

    if(MODE == 'c') //if controller mode start transmitting dmx data
        init_dmx_tx();
    else if(MODE == 'd') //if device mode start receiving dmx data
        init_dmx_rx();

    while( 1 )
    {
        char c = UART0_DR_R & 0xFF;
        if(c)
        {
            coms_cmd[coms_index++] = c;
            if(c == '\r')
                parse(coms_cmd);
        }
    }
}

