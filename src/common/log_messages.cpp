/* log_messages.cpp */
#include "log_messages.h"

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

void
sts_msg(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    printf("\n");
    va_end(args);
}

void
dbg_msg(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    printf("%s‹dbg›%s ", KGRN, KNRM);
    vprintf(format, args);
    printf("\n");
    va_end(args);
}

void
wrn_msg(const char* format, ...)
{
    static unsigned long wrn_msg_cnt = 0;
    va_list args;
    va_start(args, format);
    printf("%sWARNING %ld: %s", KYEL, ++wrn_msg_cnt, KNRM);
    vprintf(format, args);
    printf("\n");
    va_end(args);
}

void
err_msg(const char* file, unsigned int line, const char* format, ...)
{
    static unsigned long err_msg_cnt = 0;
    va_list args;
    va_start(args, format);
    printf("%sERROR %ld:%s ", KRED, ++err_msg_cnt, KNRM);
    vprintf(format, args);
    printf("%s File %s in line %d%s\n", KWHT, file, line, KNRM);
    va_end(args);
    exit(EXIT_FAILURE);
}

