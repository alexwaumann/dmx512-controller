#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "tm4c123gh6pm.h"

#define CLSCR "\033[2J\033[1;1H"

#define MAX_CMD_SIZE    20
#define MAX_WORD_SIZE   11
#define MAX_ARG_COUNT   3

#define RED_LED_MASK    2
#define GREEN_LED_MASK  8
#define BLUE_LED_MASK   4

#define CONTROLLER_MODE 1
#define DEVICE_MODE     2

#define DMX_STATE_BREAK 0
#define DMX_STATE_MAB   1

#define RED_LED   (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define DMX_TX    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define DMX_DE    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))

/*
 * FUNCTION DECLARATIONS
 */

void init_hw( void );
void init_leds( void );
void init_eeprom( void );
void init_uart_coms( void );
void init_uart_dmx( void );
void init_timer_dmx( void );
void init_timer_led_blink( void );

void uart_putc( char c );
void uart_putui( uint16_t i );
void uart_putstr( char *str );
char uart_getc( void );

uint8_t get_cmd( char cmd[MAX_CMD_SIZE] );
uint8_t parse_cmd( char cmd[MAX_CMD_SIZE], char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void    run_cmd( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );

void cmd_default( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_help();
void cmd_device();
void cmd_controller();
void cmd_clear();
void cmd_set( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_get( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_on();
void cmd_off();
void cmd_max( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );
void cmd_address( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] );

void set_dmx_mode( uint8_t mode );
void dmx_transmit( void );
void dmx_prime( void );

void     recover_from_reset( void );
uint8_t  eeprom_read_mode( void );
void     eeprom_save_mode( uint8_t mode );
uint16_t eeprom_read_addr( void );
void     eeprom_save_addr( uint16_t addr );

void wait_us( uint32_t us );

/*
 * GLOBAL VARIABLES
 */

uint8_t dmx_mode = 0;
uint32_t timeout = 0;

// controller mode
uint8_t  dmx_data[513]      = {0};
uint16_t dmx_transmit_index = 0;
uint8_t  dmx_transmit_state = DMX_STATE_BREAK;
bool     dmx_transmit_on    = false;
uint16_t dmx_max_addr       = 512;

// device mode
uint16_t dmx_current_addr = 0;
uint16_t dmx_receive_index = 0;
uint8_t  dmx_receive_data[514];
uint8_t  dmx_value        = 0;
bool rx_state = false;

/*
 * MAIN PROGRAM
 */

int main( void )
{
    init_hw();
    recover_from_reset();
    while( true )
    {
        char    cmd_str[MAX_CMD_SIZE] = {'\0'};
        char    argv[MAX_ARG_COUNT][MAX_WORD_SIZE];
        uint8_t argc;

        while( get_cmd( cmd_str ) == 0 );
        argc = parse_cmd( cmd_str, argv );
        run_cmd( argc, argv );
    }
}

/*
 * FUNCTION DEFINITIONS
 */

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
    init_uart_dmx();
    init_timer_dmx();
    init_timer_led_blink();
}

/*
 */
void init_leds( void )
{
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;

    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK | BLUE_LED_MASK;

    RED_LED   = 0;
    GREEN_LED = 0;
    BLUE_LED  = 0;
}

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
    //GPIO_PORTA_PCTL_R  &= ~( GPIO_PCTL_PA0_M | GPIO_PCTL_PA1_M );
    GPIO_PORTA_PCTL_R  |= ( GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX );

    UART0_CTL_R = 0;

    // configure for 9600 baud
    UART0_IBRD_R = 260;
    UART0_FBRD_R = 27;

    UART0_LCRH_R  = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
    UART0_CC_R    = UART_CC_CS_SYSCLK;

    UART0_CTL_R = UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE;
}

void init_uart_dmx( void )
{
    // enable UART1 and PORTC modules
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC;

    // configure PC4, PC5, and PC6
    GPIO_PORTC_AFSEL_R |= 0x30;
    GPIO_PORTC_DEN_R   |= 0x70;
    GPIO_PORTC_DIR_R   |= 0x60;
    //GPIO_PORTC_PCTL_R  &= ~( GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M );
    GPIO_PORTC_PCTL_R  |= ( GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX );

    UART1_CTL_R = 0;

    // configure for 250,000 baud
    UART1_IBRD_R = 10;
    UART1_FBRD_R = 0;

    UART1_LCRH_R  = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_STP2;
    UART1_CC_R    = UART_CC_CS_SYSCLK;

    //UART1_CTL_R  |= UART_CTL_UARTEN | UART_CTL_RXE | UART_CTL_TXE;

    NVIC_EN0_R |= 1 << (INT_UART1 - 16);
}

void init_timer_dmx( void )
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    TIMER1_CTL_R = 0;

    // 32-bit one-shot timer
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;

    // turn on interrupts
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);
}

void init_timer_led_blink( void )
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;  // turn on timer 2
    TIMER2_CTL_R = 0;            // turn off timer before configuring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;      // configure as 32-bit timer (concatenated)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;     // configure in periodic mode (count down)
    TIMER2_IMR_R = TIMER_IMR_TATOIM;            // turn on timer2 interrupt
    TIMER2_TAILR_R = 10000000;                  // set TIMER2 ILR to appropriate value for 250ms = 10,000,000
    NVIC_EN0_R |= 1 << (INT_TIMER2A - 16);      // turn on interrupt 39 (TIMER2A)

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;  // turn on timer 3
    TIMER3_CTL_R = 0;            // turn off timer before configuring
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;      // configure as 32-bit timer (concatenated)
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;     // configure in one-shot mode (count down)
    TIMER3_IMR_R = TIMER_IMR_TATOIM;            // turn on timer3 interrupt
    TIMER3_TAILR_R = 80000000;                  // set TIMER3 ILR to appropriate value for 2s = 80,000,000
    NVIC_EN1_R |= 1 << (INT_TIMER3A - 48);      // turn on interrupt 51 (TIMER3A)
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
 * print unsigned int to the serial console
 */
void uart_putui( uint16_t ui )
{
    uint8_t i;
    char str[6];
    uint8_t str_index = 0;
    uint16_t divisor = 10000;
    char c;

    while( UART0_FR_R & UART_FR_TXFF );

    for( i = 0; i < 5; ++i )
    {
        if( (c = ui / divisor) != 0 )
        {
            str[str_index++] = c + 48;
            ui -= c * divisor;
        }
        else if( i == 4 )
        {
            str[str_index++] = '0';
        }
        divisor /= 10;
    }
    str[str_index] = '\0';

    uart_putstr( str );
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
    if(dmx_mode == CONTROLLER_MODE)
    {
        GREEN_LED = 1;
        TIMER2_CTL_R |= TIMER_CTL_TAEN;
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
 * switch over commands and call designated cmd function
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
        // invalid command
        cmd_default( argc, argv );
}

/*
 * print "command not understood" on serial output
 * print help text
 */
void cmd_default( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    uart_putstr( "error: command '" );
    uart_putstr( argv[0] );
    uart_putstr( "' not understood\n\r" );
}

/*
 * help text
 */
void cmd_help()
{
}

/*
 * change into device mode
 * save to eeprom
 */
void cmd_device()
{
    //UART1_CTL_R &= ~UART_CTL_UARTEN;
    dmx_transmit_on = false;
    uart_putstr( "current mode: " );

    if( dmx_mode == DEVICE_MODE )
    {
        uart_putstr( "device\n\r" );
        uart_putstr( "no action taken\n\r" );
        set_dmx_mode( DEVICE_MODE );
    }
    else
    {
        uart_putstr( "controller\n\r" );
        set_dmx_mode( DEVICE_MODE );
        uart_putstr( "new mode: device\n\r" );
    }
}

/*
 * change into controller mode
 * save to eeprom
 */
void cmd_controller( void )
{
    //UART1_CTL_R &= ~UART_CTL_UARTEN;
    dmx_transmit_on = false;
    uart_putstr( "current mode: " );

    if( dmx_mode == CONTROLLER_MODE )
    {
        uart_putstr( "controller\n\r" );
        uart_putstr( "no action taken\n\r" );
        set_dmx_mode( CONTROLLER_MODE );
    }
    else
    {
        uart_putstr( "device\n\r" );
        set_dmx_mode( CONTROLLER_MODE );
        uart_putstr( "new mode: controller\n\r" );
    }
}

/*
 * set all dmx data to 0
 */
void cmd_clear( void )
{
    uint16_t i;

    if( dmx_mode != CONTROLLER_MODE )
    {
        uart_putstr( "error: not in controller mode\n\r" );
        return;
    }

    for( i = 1; i <= dmx_max_addr; ++i )
        dmx_data[i] = 0;
}

/*
 * set the dmx data value to value in arg at the address in arg
 */
void cmd_set( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    uint32_t address;
    uint32_t value;

    if( dmx_mode != CONTROLLER_MODE )
    {
        uart_putstr( "error: not in controller mode\n\r" );
        return;
    }

    if( argc != 3 )
    {
        uart_putstr( "error: expected 3 arguments, got " );
        uart_putui( argc );
        uart_putstr( "\n\r" );
        return;
    }

    address = atoi( argv[1] );
    value   = atoi( argv[2] );

    if( address > 512 || address < 1 )
    {
        uart_putstr( "error: address must be in the range [1, 512]\n\r" );
        return;
    }

    if( value > 255 )
    {
        uart_putstr( "error: value must be in the range [0, 255]\n\r" );
        return;
    }

    dmx_data[address] = (uint8_t)value;

    uart_putstr( "set address " );
    uart_putui( address );
    uart_putstr( " to " );
    uart_putui( value );
    uart_putstr( "\n\r" );
}

/*
 * return the dmx data value at address in arg
 */
void cmd_get( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    uint32_t address;
    uint8_t  value;

    if( dmx_mode != CONTROLLER_MODE )
    {
        uart_putstr( "error: not in controller mode\n\r" );
        return;
    }

    if( argc != 2 )
    {
        uart_putstr( "error: expected 2 arguments, got " );
        uart_putui( argc );
        uart_putstr( "\n\r" );
        return;
    }

    address = atoi( argv[1] );

    if( address > 512 || address < 1 )
    {
        uart_putstr( "error: address must be in the range [1, 512]\n\r" );
        return;
    }

    value = dmx_data[address];
    uart_putstr( "value at address " );
    uart_putui( address );
    uart_putstr( ": " );
    uart_putui( value );
    uart_putstr( "\n\r" );
}

/*
 * turn on dmx transmission
 */
void cmd_on()
{
    if( dmx_mode != CONTROLLER_MODE )
    {
        uart_putstr( "error: not in controller mode\n\r" );
        return;
    }

    uart_putstr( "current dmx transmit state: " );

    if( dmx_transmit_on )
    {
        dmx_transmit_on = true;
        dmx_prime();
        uart_putstr( "on\n\r" );
        uart_putstr( "no action taken\n\r" );
    }
    else
    {
        uart_putstr( "off\n\r" );
        dmx_transmit_on = true;
        dmx_prime();
        uart_putstr( "new dmx transmit state: on\n\r" );
    }
}

/*
 * turn off dmx transmission
 */
void cmd_off()
{
    dmx_transmit_on = false;
    if( dmx_mode != CONTROLLER_MODE )
    {
        uart_putstr( "error: not in controller mode\n\r" );
        return;
    }

    uart_putstr( "current dmx transmit state: " );

    if( !dmx_transmit_on )
    {
        uart_putstr( "off\n\r" );
        uart_putstr( "no action taken\n\r" );
    }
    else
    {
        uart_putstr( "on\n\r" );
        dmx_transmit_on = false;
        uart_putstr( "new dmx transmit state: off\n\r" );
    }
}

/*
 * set the max addresses to send dmx data to
 */
void cmd_max( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    uint32_t max_addr;

    if( dmx_mode != CONTROLLER_MODE )
    {
        uart_putstr( "error: not in controller mode\n\r" );
        return;
    }

    if( argc != 2 )
    {
        uart_putstr( "error: expected 2 arguments, got " );
        uart_putui( argc );
        uart_putstr( "\n\r" );
        return;
    }

    max_addr = atoi( argv[1] );

    if( max_addr > 512 || max_addr < 1 )
    {
        uart_putstr( "error: max address must be in the range [1, 512]\n\r" );
        return;
    }

    uart_putstr( "current max address: " );
    uart_putui(dmx_max_addr);
    uart_putstr( "\n\r" );
    dmx_max_addr = (uint16_t)max_addr;
    uart_putstr( "new max address: " );
    uart_putui(dmx_max_addr);
    uart_putstr( "\n\r" );
}

/*
 * set the address of device
 * save to eeprom
 */
void cmd_address( uint8_t argc, char argv[MAX_ARG_COUNT][MAX_WORD_SIZE] )
{
    uint16_t addr;

    if( dmx_mode != DEVICE_MODE )
    {
        uart_putstr( "error: not in device mode\n\r" );
        return;
    }

    if( argc != 2 )
    {
        uart_putstr( "error: expected 2 arguments, got " );
        uart_putui( argc );
        uart_putstr( "\n\r" );
        return;
    }

    addr = atoi( argv[1] );

    if( addr > 512 || addr == 0 )
    {
        uart_putstr( "error: device address must be in the range [1, 512]\n\r" );
        return;
    }

    uart_putstr( "current device address: " );
    uart_putui( dmx_current_addr );
    uart_putstr( "\n\r" );

    if( addr == dmx_current_addr )
    {
        uart_putstr( "no action taken\n\r" );
    }
    else
    {
        eeprom_save_addr( addr );
        uart_putstr( "new device address: " );
        uart_putui( dmx_current_addr );
        uart_putstr( "\n\r" );
        dmx_value = dmx_receive_data[dmx_current_addr];
        if(dmx_value)
            BLUE_LED = 1;
        else
            BLUE_LED = 0;
    }
}

/*
 * set up UART interrupts for transmitting
 */
void dmx_transmit( void )
{
    UART1_CTL_R = 0;

    while(UART1_FR_R & UART_FR_BUSY);

    UART1_LCRH_R &= ~UART_LCRH_FEN;
    UART1_LCRH_R |= UART_LCRH_FEN;

    // turn on interrupts for tx and off for rx
    UART1_IM_R = UART_IM_TXIM;
}

/*
 * "prime the pump" for dmx protocol, reset everything for starting transmission again
 */
void dmx_prime( void )
{
    RED_LED = 0;
    dmx_transmit_index = 0;
    dmx_transmit_state = DMX_STATE_BREAK;
    // manually drive DMX_TX (PC5)
    GPIO_PORTC_AFSEL_R &= ~0x20;
    // 176 us
    TIMER1_TAILR_R = 7040;
    // break condition
    DMX_TX = 0;
    if( dmx_transmit_on )
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

/*
 * Set up UART interrupts for receiving and enable RX
 */
void dmx_receive( void )
{
    uart_putstr("\n\rdmx receive\n\r");
    UART1_CTL_R = 0;

    while(UART1_FR_R & UART_FR_BUSY);

    UART1_LCRH_R &= ~UART_LCRH_FEN;
    UART1_LCRH_R |= UART_LCRH_FEN;

    // turn on interrupts for rx and off for tx
    UART1_IM_R = UART_IM_RXIM | UART_IM_BEIM;
    UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;
    TIMER3_CTL_R |= TIMER_CTL_TAEN;
}

/*
 * save new mode to global, save to EEPROM
 * perform mode switching operations to ensure proper function of LEDs and DMX UART
 */
void set_dmx_mode( uint8_t mode )
{
    eeprom_save_mode( mode );

    if( dmx_mode == CONTROLLER_MODE )
    {
        DMX_DE = 1;
        rx_state = true;

        dmx_transmit();
    }
    else
    {
        rx_state = false;
        DMX_DE = 0;
        RED_LED = 0;
        dmx_receive();
    }
}

/*
 * read the EEPROM for dmx_current_addr and dmx_mode
 * revert to defaults if invalid
 * print opening screen the mode and addr displayed
 */
void recover_from_reset( void )
{
    dmx_mode = eeprom_read_mode();
    if( dmx_mode != CONTROLLER_MODE && dmx_mode != DEVICE_MODE )
        dmx_mode = CONTROLLER_MODE;
    set_dmx_mode( dmx_mode );
    dmx_current_addr = eeprom_read_addr();
    if( dmx_current_addr > 512 || dmx_current_addr == 0 )
    {
        eeprom_save_addr( 1 );
        dmx_current_addr = 1;
    }
    uart_putstr(CLSCR);
    uart_putstr("   Mode: ");
    if( dmx_mode == CONTROLLER_MODE )
        uart_putstr("Controller\n\r");
    else if( dmx_mode == DEVICE_MODE )
        uart_putstr("Device\n\r");
    else
        uart_putstr("Error\n\r");
    uart_putstr("Address: ");
    uart_putui(dmx_current_addr);
    uart_putstr("\n\r");
}

/*
 * read EEPROM where dmx_mode should be stored then return
 */
uint8_t eeprom_read_mode( void )
{
    EEPROM_EEBLOCK_R = 0;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEOFFSET_R = 0;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    uint8_t temp = (uint8_t)EEPROM_EERDWR_R;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    return temp;
}

/*
 * update dmx_mode to mode, then save to EEPROM
 */
void eeprom_save_mode( uint8_t mode )
{
    dmx_mode = mode;
    EEPROM_EEBLOCK_R = 0;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEOFFSET_R = 0;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EERDWR_R = (uint32_t)dmx_mode;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

/*
 * read EEPROM where dmx_current_addr should be stored then return
 */
uint16_t eeprom_read_addr( void )
{
    EEPROM_EEBLOCK_R = 1;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEOFFSET_R = 0;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    uint16_t temp = (uint16_t)EEPROM_EERDWR_R & 0x3FF;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    return temp;
}

/*
 * update dmx_current_addr to addr, then save to EEPROM
 */
void eeprom_save_addr( uint16_t addr )
{
    dmx_current_addr = addr;
    EEPROM_EEBLOCK_R = 1;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EEOFFSET_R = 0;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    EEPROM_EERDWR_R = (uint32_t)dmx_current_addr;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
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

/*
 * Uart1ISR
 *
 * If TX interrupt    -> fill TX fifo from buffer while checking for max, if max restart transmission and break
 * If Break Interrupt -> empty RX fifo, discard break data, update at listening addr, restart break watch timer
 * If RX interrupt    -> empty RX fifo, keep GREEN_LED on to indicate receiving
 */
void Uart1ISR( void )
{

    if( UART1_MIS_R & UART_MIS_TXMIS )
    {
        while( !(UART1_FR_R & UART_FR_TXFF) )
        {
            // check if done
            if( dmx_transmit_index > dmx_max_addr )
            {
                // wait until transmission complete
                while( UART1_FR_R & UART_FR_BUSY );

                UART1_ICR_R |= UART_ICR_TXIC;

                // start transmission again
                dmx_prime();
                return;
            }else //send data
                UART1_DR_R = dmx_data[dmx_transmit_index++];

        }

        UART1_ICR_R |= UART_ICR_TXIC;
    }
    else if( UART1_MIS_R & UART_MIS_BEMIS )
    {
        // empty the buffer in case it still has data, discard the break error byte
        uint16_t temp;
        while( !(UART1_FR_R & UART_FR_RXFE) )
        {
            temp = UART1_DR_R & 0xFFF;
            if( !(temp & 0x400) )
            {
                dmx_receive_data[dmx_receive_index] = temp & 0xFF;
                if(dmx_receive_data[0]==0)
                    dmx_receive_index++;
            }
        }

        // update the dmx_value and update the controlled device
        dmx_value = dmx_receive_data[dmx_current_addr];
        if( dmx_value )
            BLUE_LED = 1;
        else
            BLUE_LED = 0;

        // reset receive index and set rx_state to false to trigger the led when break watch timer goes off
        dmx_receive_index = 0;
        rx_state = false;

        // reset the break watching timer since a break has been received
        TIMER3_CTL_R &= ~TIMER_CTL_TAEN;
        TIMER3_TAILR_R = 80000000;
        TIMER3_CTL_R |= TIMER_CTL_TAEN;
    }
    else if( UART1_MIS_R & UART_MIS_RXMIS )
    {
        // set rx_state to true and turn on GREEN_LED since receiving
        rx_state = true;
        GREEN_LED = 1;

        // empty the receive fifo into the buffer, only fill buffer after start code received
        while( !(UART1_FR_R & UART_FR_RXFE) )
        {
            dmx_receive_data[dmx_receive_index] = (UART1_DR_R & 0xFF);
            if(dmx_receive_data[0]==0)
                dmx_receive_index++;
        }
    }
    UART1_ICR_R = UART_ICR_RXIC | UART_ICR_BEIC | UART_ICR_TXIC;
}

/*
 * Timer1ISR
 *
 * if DMX in Break state -> drive TX high and advance to Mark After Break state
 * if DMX in MAB state   -> turn over GPIO control to UART, enable UART and TX, and fill TX fifo from buffer
 */
void Timer1ISR( void )
{
    if( dmx_transmit_state == DMX_STATE_BREAK )
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

        dmx_transmit_state = DMX_STATE_MAB;
        // 12 us
        TIMER1_TAILR_R = 480;
        // mark after break
        DMX_TX = 1;

        TIMER1_CTL_R |= TIMER_CTL_TAEN;
    }
    else if( dmx_transmit_state == DMX_STATE_MAB )
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

        dmx_transmit_state = 2;

        // uart1 drives DMX_TCX (PC5)
        GPIO_PORTC_AFSEL_R |= 0x20;
        GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC5_U1TX;

        RED_LED = 1;

        UART1_CTL_R  |= UART_CTL_UARTEN | UART_CTL_TXE;

        while( !(UART1_FR_R & UART_FR_TXFF) ){

            if( dmx_transmit_index > dmx_max_addr )
            {
                // wait until transmission complete
                while( UART1_FR_R & UART_FR_BUSY );

                TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
                // start transmission again
                dmx_prime();
                return;
            }else
                UART1_DR_R = dmx_data[dmx_transmit_index++];
        }
    }
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
}

/*
 * Timer2ISR
 *
 * Turn off Green LED on trigger to complete a 250 ms flash
 */
Timer2ISR( void )
{
    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;
    GREEN_LED = 0;
}

/*
 * Timer3ISR
 *
 * if rx_state == true, stop flashing GREEN_LED and reset period to 2s
 * if rx_state == false, start toggling GREEN_LED every 1s
 */
Timer3ISR( void )
{
    if(rx_state){
        TIMER3_TAILR_R = 80000000;      // set TIMER3 ILR to appropriate value for 2s = 80,000,000
        GREEN_LED = 0;
    }
    else
    {
        TIMER3_TAILR_R = 40000000;      // set TIMER3 ILR to appropriate value for 1s = 40,000,000
        GREEN_LED = !(GREEN_LED);       // toggle the Green LED
        TIMER3_CTL_R |= TIMER_CTL_TAEN; // turn on timer 3
    }
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT; // clear timer interrupt flag
}
