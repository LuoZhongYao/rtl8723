#ifndef __ZDEBUG_H__
#define __ZDEBUG_H__

//#define DEBUG
#define TREM_COLOR

#ifdef  TREM_COLOR
#define WARNING_COLOR   "\e[31m"
#define MESSAGE_COLOR   "\e[32m"
#define DEFAULT_COLOR   "\e[0m"
#define MAKE_COLOR(color,message)   color  message  DEFAULT_COLOR
#else
#define WARNING_COLOR
#define MESSAGE_COLOR
#define MAKE_COLOR(color,message)   message
#endif

#ifdef  DEBUG
#define dprint(fmt,...) printk("[ RTLBT] : %4d "fmt" (%s)\n",__LINE__,##__VA_ARGS__,__func__)
#else
#define dprint(...)
#endif

#define warning(fmt,...) dprint(MAKE_COLOR(WARNING_COLOR,"*WARNING* ")fmt,##__VA_ARGS__)
#define message(fmt,...) dprint(MAKE_COLOR(MESSAGE_COLOR,"*MESSAGE* ")fmt,##__VA_ARGS__)

#endif
