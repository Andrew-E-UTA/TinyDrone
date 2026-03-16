// Shell functions
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "uart0.h"
#include "shell.h"

//-----------------------------------------------------------------------------
// SHELL Defines and SHELL Variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//   Subroutines
//-----------------------------------------------------------------------------

void _printArgs(const char str[], UartArgs uArgs)
{
    switch(uArgs.fg) {
        case White: { putsUart0(SET_FG_W); }break;
        case Black: { putsUart0(SET_FG_BL); }break;
        case Red: { putsUart0(SET_FG_R); }break;
        case Green: { putsUart0(SET_FG_G); }break;
        case Blue: { putsUart0(SET_FG_B); }break;
        case Yellow: { putsUart0(SET_FG_Y); }break;
    }

    switch(uArgs.bg) {
        case White: { putsUart0(SET_BG_W); }break;
        case Black: { putsUart0(SET_BG_BL); }break;
        case Red: { putsUart0(SET_BG_R); }break;
        case Green: { putsUart0(SET_BG_G); }break;
        case Blue: { putsUart0(SET_BG_B); }break;
        case Yellow: { putsUart0(SET_BG_Y); }break;
    }

    putsUart0(str);

    putcUart0(uArgs.end);

    putsUart0(CLEAR_COLOR);
}

void strcpy(char* to, const char* from)
{
    uint8_t c;
    for(c = 0; from[c]; c++)
        to[c] = from[c];
    to[c] = '\0';
}


void strncpy(char* to, const char* from, uint8_t n)
{
    uint8_t c;
    for(c = 0; from[c] && c < n; c++)
        to[c] = from[c];
    to[c] = '\0';
}

void strcpyFill(char* to, const char* from, uint8_t len, char letter)
{
    uint8_t c;
    bool fill = false;
    for(c = 0; (c < len); c++)
    {
        if(!from[c]) fill = true;
        if(!fill)
            to[c] = from[c];
        else
            to[c] = letter;
    }

    to[len-1] = '\0';
}

uint32_t strlen(const char *str){
    uint32_t len  = 0;
    while(*(str + len++));
    return len;
}
bool strcmp(const char* s1, const char* s2)
{
    uint8_t i;
    for(i = 0; i < MAX_CHARS; i++)
    {
        if(s1[i] != s2[i])
            return false;
        if(s1[i] == '\0')
            return true;
    }
    return true;
}

void strnappnd(char* to, const char* from, uint8_t n)
{
    uint8_t i, j;
    i = strlen(to)-1; //index of null terminator of to
    for(j = 0; from[j] && j < n; ++j)
        to[i+j] = from[j];
    to[i+j] = '\0';
}

uint16_t pow(int base, int power)
{
    uint8_t i = 0;
    uint16_t result = base;
    for(i = 0; i < power; i++)
    {
        result *= base;
    }
    return result;
}

    //We reached the end of the buffer and it didnt exit false
    //  therefore all the letters matched and we can assume strings match

int32_t atoi32(const char* str)
{
    char c = 0;
    int32_t sum = 0;
    uint8_t i = 0, base = 0;
    bool n = false;

    //determine the base of the integer
    if(*(str+0)=='0' && *(str+1)=='x')
    {
        base = 16;
        i = 2;
    }
    else if(*(str+0)=='0' && *(str+1)=='b')
    {
        base = 2;
        i = 2;
    }
    else
    {
        base = 10;
        if(n = (*str == '-')) i = 1;
        else i = 0;
    }

    while((c = str[i++]) != '\0')
    {
        sum *= base;
        if(c >= '0' && c <= '9')
            sum += c - '0';
        else if('a' <= c && c <= 'z')
            sum += c - 'a' + 10;
        else if('A' <= c && c <= 'Z')
            sum += c - 'A' + 10;
        else return -1;
    }
    if(n) sum *= -1;
    return sum;
}

void htoa(uint32_t num, char str[MAX_DIG_U32])
{
    uint8_t i = 0, j = 0, temp = 0;
    char buff[MAX_DIG_U32] = {};
    str[0] = '0';
    str[1] = 'x';

    while(num)
    {
        temp = num % 16;
        if(temp >= 10)
            buff[i++] = temp-10 + 'a';
        else
            buff[i++] = temp + '0';
        num /= 16;
    }
    for(j = 0; j < i; j++)
        str[j+2] = buff[(i-1) - j];
    str[i+2] = '\0';
}

void itoa32(uint32_t num, char str[MAX_DIG_U32])
{
    uint8_t i, j, remainder, last = 0;
    int16_t k;
    char temp;

    if(!num)
    {
        str[0] = 48;
        str[1] = 0;
        return;
    }

    for(i = 0; i < MAX_DIG_U32; i++)
    {
        remainder = num % 10;
        if(!remainder)
            str[i] = 0;
        else
        {
            str[i] = remainder + 48;
            last = i;
        }
        num /= 10;
    }
    for(k = last; k >= 0; k--)
    {
        //null present before the last digit
        if(!str[k])
            str[k] = 48;
    }

    for(j = 0; j < (last+1)/2; j++)
    {
        temp = str[j];
        str[j] = str[last-j];
        str[last-j] = temp;
    }
}

inline int min(int l, int r)
{
    bool lt = (l <= r);
    return ( lt*l + (!lt)*r );
}
