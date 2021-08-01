/*****************************************************************************/
/**** SimCore/RISC-V since 2018-07-05                ArchLab. TokyoTech   ****/
/*****************************************************************************/

#include "simrv.h"

volatile int *TOHOST_ADDR = (int *)0x40008000;

void simrv_exit () {
    *TOHOST_ADDR = CMD_POWER_OFF << 16;
}

void simrv_putc (char c) {
    volatile int i = 0;
    while(i < 100) i++;
    *TOHOST_ADDR = CMD_PRINT_CHAR << 16 | c;
}

void simrv_puts (char *str) {
    for (char *c = str; *c != '\0'; c++) {
        simrv_putc(*c);
    }
}

void simrv_puth (unsigned int n) {
    char str[8];
    unsigned int  hexn = 0;
    unsigned int  size = 0;

    do {
        hexn = n & 0xf;
        if(hexn < 10) str[size++] = (char) (hexn + '0');
        else          str[size++] = (char) (hexn - 10 + 'A');
        n = n >> 4;
    } while (n != 0);

    for (int i = size-1; i >= 0; i--) {
        simrv_putc(str[i]);
    }
}
