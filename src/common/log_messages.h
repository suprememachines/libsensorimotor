#ifndef LOG_MESSAGES_H
#define LOG_MESSAGES_H

#include <cstdlib>
#include <cstdio>
#include <cstdarg>

void sts_msg(const char* format, ...);
void dbg_msg(const char* format, ...);
void wrn_msg(const char* format, ...);
void err_msg(const char* file, unsigned int line, const char* format, ...);
void promise(bool condition, const char* file, unsigned int line, const char* format, ...);

#define assertion(COND, FMT, ...)                               \
promise(COND, __FILE__, __LINE__, #COND " " FMT, ##__VA_ARGS__) \


#endif // LOG_MESSAGES_H
