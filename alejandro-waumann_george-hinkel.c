#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "tm4c123gh6pm.h"

#define MAX_CMD_SIZE    20
#define MAX_WORD_SIZE   11
#define MAX_ARG_COUNT   3

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define PC5_MASK 32
#define DMX_TX (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define DMX_DE (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define MODE_EEPROM_BLOCK 0
#define MODE_EEPROM_WORD 0
#define ADDR_EEPROM_BLOCK 1
#define ADDR_EEPROM_WORD 0
#define EEPROM_STAT_BLOCK 2
#define EEPROM_STAT_WORD 0

/*
 * FUNCTION DECLARATIONS
 */

void init_hw( void );
void init_leds( void );
void init_eeprom( void );
void init_uart_coms( void );
void init_uart_dmx( void );

void uart_putc( char c );
void uart_putstr( char *str );
char uart_getc( void );

uint8_t get_cmd( char cmd[MAX_CMD_SIZE] );
uint8_t parse_cmd( char cmd[MAX_CMD_SIZE], char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void    run_cmd( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );

void cmd_device();
void cmd_controller();
void cmd_clear();
void cmd_set( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_get( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_on();
void cmd_off();
void cmd_max( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_address( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );

void wait_us( uint32_t us );

/*
 * GLOBAL VARIABLES
 */

char MODE = 'c';                            //tracks whether in controller or device mode
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

/*
 * MAIN PROGRAM
 */

int main( void )
{
    init_hw();

    RED_LED = 0;
    GREEN_LED = 1;

    while( true )
    {
        char cmd_str[MAX_CMD_SIZE] = {'\0'};
        char argv[MAX_ARG_COUNT][MAX_WORD_SIZE];
        uint8_t argc;

        while( get_cmd( cmd_str ) == 0 );
        argc = parse_cmd( cmd_str, argv );

        run_cmd( argc, argv );
    }
}

/*
 */
void init_hw( void )
{
    // 16 mhz xtal, pll enable, 40 mhz sysclock
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    SYSCTL_GPIOHBCTL_R = 0;

    init_leds();
    init_eeprom();
    init_uart_coms();
}

/*
 */
void init_leds( void )
{
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;

    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK;
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK;
}

/*
 */
void init_eeprom(void)
{
    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;

    wait_us( 1 );
    while( EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING );
    if( EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY) );

    SYSCTL_SREEPROM_R |= SYSCTL_SREEPROM_R0;
    SYSCTL_SREEPROM_R &= ~SYSCTL_SREEPROM_R0;

    wait_us( 1 );
    while( EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING );
    if( EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY | EEPROM_EESUPP_ERETRY) );
}

/*
 * setup and configure serial port communication
 *
 * U0Rx -> PA0
 * U0Tx -> PA1
 *
 * baud rate    9600
 * data frame   8-bit
 * fifa         enabled
 * receive      enabled
 * transmit     enabled
 */
void init_uart_coms( void )
{
    // enable UART0 and PORTA modules
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;

    // configure PA0 and PA1
    GPIO_PORTA_AFSEL_R |= 0x3;
    GPIO_PORTA_DEN_R   |= 0x3;
    GPIO_PORTA_DIR_R   |= 0x2;
    GPIO_PORTA_PCTL_R  &= ~( GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M );
    GPIO_PORTA_PCTL_R  |= ( GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX );

    UART0_CTL_R &= ~UART_CTL_UARTEN;

    // configure for 9600 baud
    UART0_IBRD_R = 260;
    UART0_FBRD_R = 27;

    UART0_LCRH_R |= UART_LCRH_WLEN_8 | UART_LCRH_FEN;

    UART0_CC_R    = UART_CC_CS_SYSCLK;

    UART0_CTL_R |= UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE;
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

/*
 * print character to the serial console
 */
void uart_putc(char c)
{
    while( UART0_FR_R & UART_FR_TXFF );

    UART0_DR_R = c;
}

/*
 * print string to the serial console
 *
 * note: blocking
 */
void uart_putstr( char *str )
{
    uint8_t i;

    for( i = 0; i < strlen( str ); ++i )
      uart_putc( str[i] );
}

/*
 * get character from serial receive line
 *
 * note: blocking
 */
char uart_getc()
{
    while( UART0_FR_R & UART_FR_RXFE );

    return UART0_DR_R & 0xFF;
}

/*
 * return 0 on empty command
 */
uint8_t get_cmd( char cmd[MAX_CMD_SIZE] )
{
    char ch;
    uint8_t cmd_idx = 0;
    uint8_t word_idx = 0;

    uart_putstr( "$ " );
    // wait for user to enter a command
    while( (ch = uart_getc()) != '\r' )
    {
        // handle backspace
        if( (ch == '\b' || ch == 127) && cmd_idx > 0 )
        {
            uart_putstr( "\b \b" );
            --cmd_idx;

            if( word_idx != 0 )
                --word_idx;
        }
        // handle cmd exceeding MAX_CMD_SIZE
        else if( cmd_idx == MAX_CMD_SIZE-1 )
        {
            continue;
        }
        // handle argument delimiters
        else if( ch == ' ' || ch == ',' )
        {
            uart_putc( ch );
            cmd[cmd_idx++] = ' ';
            word_idx = 0;
        }
        // handle alphanumeric chars
        else if( (word_idx != MAX_WORD_SIZE-1) && isalnum( ch ) )
        {
            uart_putc( ch );
            cmd[cmd_idx++] = ch;
            ++word_idx;
        }
    }

    uart_putstr( "\n\r" );
    cmd[cmd_idx] = '\0';

    return cmd_idx;
}

/*
 * cmd only has alphanumerics and spaces (delimiter)
 */
uint8_t parse_cmd( char cmd[MAX_CMD_SIZE], char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    uint8_t n = strlen( cmd );
    uint8_t argc = 0;
    uint8_t cmd_idx = 0;
    uint8_t word_idx = 0;

    for( cmd_idx = 0; cmd_idx < n && argc < MAX_ARG_COUNT; ++cmd_idx )
    {
        char ch = cmd[cmd_idx];

        if( ch != ' ' && word_idx < MAX_WORD_SIZE-1 )
        {
            if( cmd_idx+1 == n )
            {
                argv[argc][word_idx++] = ch;
                argv[argc++][word_idx] = '\0';
                word_idx = 0;
            }
            else
            {
                argv[argc][word_idx++] = ch;
            }

        }
        else if( word_idx != 0 )
        {
            argv[argc++][word_idx] = '\0';
            word_idx = 0;

            if( ch != ' ' )
                argv[argc][word_idx++] = ch;
        }
    }

    return argc;
}

/*
 */
void run_cmd( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    char *cmd = argv[0];

    if( strcmp( cmd, "device" ) == 0 )
        // change into device mode
        cmd_device();
    else if( strcmp( cmd, "controller" ) == 0 )
        // change into controller mode
        cmd_controller();
    else if( strcmp( cmd, "clear" ) == 0 )
        // set values of all addresses to 0
        cmd_clear();
    else if( strcmp( cmd, "set" ) == 0 )
        // set address A to value V
        cmd_set( argc, argv );
    else if( strcmp( cmd, "get" ) == 0 )
        // get value at address A
        cmd_get( argc, argv );
    else if( strcmp( cmd, "on" ) == 0 )
        // turn on dmx transmission
        cmd_on();
    else if( strcmp( cmd, "off" ) == 0 )
        // turn off dmx transmission
        cmd_off();
    else if( strcmp( cmd, "max" ) == 0 )
        // only M addresses will be sent
        cmd_max( argc, argv );
    else if( strcmp( cmd, "address" ) == 0 )
        // device address to use after next next break
        cmd_address( argc, argv );
    else
    {
    }
}

void cmd_device()
{
}

void cmd_controller()
{
}

void cmd_clear()
{
    uint16_t i;

    for( i = 1; i <= max_address; ++i )
        dmx_data[i] = 0;
}

void cmd_set( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
}

void cmd_get( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
}

void cmd_on()
{
}

void cmd_off()
{
}

void cmd_max( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
}

void cmd_address( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
}

void wait_us( uint32_t us )
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
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

//TODO: add init_eeprom function and put in init_hw

//function that loads MODE and ADDR vars from EEPROM if they were saved
void recover_from_reset(void)
{
    EEPROM_EEBLOCK_R = EEPROM_STAT_BLOCK;
    EEPROM_EEOFFSET_R = EEPROM_STAT_WORD;
    EEPROM_STAT = EEPROM_EERDWR_R;
    if(EEPROM_STAT)
    {
        EEPROM_EEBLOCK_R = MODE_EEPROM_BLOCK;
        EEPROM_EEOFFSET_R = MODE_EEPROM_WORD;
        MODE = EEPROM_EERDWR_R & 0xFF;
        EEPROM_EEBLOCK_R = ADDR_EEPROM_BLOCK;
        EEPROM_EEOFFSET_R = ADDR_EEPROM_WORD;
        ADDR = EEPROM_EERDWR_R;
    }
}

//function that saves MODE and ADDR to EEPROM and sets the EEPROM_STAT flag and saves it too
void save_mode_and_addr(void)
{
    EEPROM_EEBLOCK_R = MODE_EEPROM_BLOCK;
    EEPROM_EEOFFSET_R = MODE_EEPROM_WORD;
    EEPROM_EERDWR_R = MODE;
    EEPROM_EEBLOCK_R = ADDR_EEPROM_BLOCK;
    EEPROM_EEOFFSET_R = ADDR_EEPROM_WORD;
    EEPROM_EERDWR_R = ADDR;
    EEPROM_STAT = 1;
    EEPROM_EEBLOCK_R = EEPROM_STAT_BLOCK;
    EEPROM_EEOFFSET_R = EEPROM_STAT_WORD;
    EEPROM_EERDWR_R = EEPROM_STAT;
}

