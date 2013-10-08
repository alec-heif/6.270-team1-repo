#include <joyos.h>
#include <lib/log.h>
struct lock log_lock;

FILE logfile = FDEV_SETUP_STREAM(log_putchar, NULL, _FDEV_SETUP_RW);


volatile char * log_ptr;
char * log_end = 0;
char * log_start = 0;
size_t log_size;

void log_init(size_t size){
    init_lock(&log_lock, "log lock");
    if ( (log_ptr = (volatile char * ) malloc(size)) < 0 )
        panic("Malloc fail\n");
    log_start = log_ptr;
    log_end = log_ptr + size;
}


int log_putchar(char c, FILE *f){
    if (log_ptr < log_end){
        *log_ptr++ = c; 
    }
    return 0;
}

int log_getchar(FILE * f){

    return 0;
}

void log_dump(){
    printf("Dumping log...\n");
    log_end = log_ptr;
    log_ptr = log_start;
    while (log_ptr <= log_end){
        putchar(*log_ptr++);
    }
}

int log_vprintf(const char *fmt, va_list ap) {
    int count;
    acquire(&log_lock);
    count = vfprintf(&logfile, fmt, ap);
    release(&log_lock);

    return count;
}


int log_printf(const char *fmt, ...) {
    va_list ap;
    int count;

    va_start(ap, fmt);
	#ifndef SIMULATE
    count = log_vprintf(fmt, ap);
	#else
	count = vprintf(fmt, ap);
	#endif
    va_end(ap);

    return count;
}


int log_vprintf_P(const char *fmt, va_list ap) {
    int count; 
    acquire(&log_lock);
    count = vfprintf_P(&logfile, fmt, ap);
    release(&log_lock);

    return count;
}

int log_printf_P(const char *fmt, ...) {
    va_list ap;
    int count;

    va_start(ap, fmt);
    count = log_vprintf_P(fmt, ap);
    va_end(ap);

    return count;
}
