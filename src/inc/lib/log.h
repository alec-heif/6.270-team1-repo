#ifndef LOG_H
#define LOG_H

void log_init(size_t size);

int log_putchar(char c, FILE *f);
int log_getchar(FILE * f);


void log_dump();

int log_printf(const char *fmt, ...);
    







#endif
