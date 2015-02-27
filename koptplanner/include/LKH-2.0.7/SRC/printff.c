#include <stdio.h>
#include <stdarg.h>

void printff(char *fmt, ...);

/* 
 * The printff function prints a message and flushes stdout.
 */

void printff(char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
}
