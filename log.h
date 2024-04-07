#pragma once 


static FILE * log_file = NULL;

static void nes_set_logfile(char * filename)
{
    log_file = fopen(filename, "w");
}

static inline void nes_log(char * f, ...)
{
    if (!log_file) 
        return;
    va_list args;
    va_start(args, f);
    vfprintf(log_file, f, args);
    fflush(log_file);
    va_end(args);
}

static inline void nes_log_close()
{
    fclose(log_file);
}
