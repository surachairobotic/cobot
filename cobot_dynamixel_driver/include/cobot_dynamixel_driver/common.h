#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <sstream>
#include <stdarg.h>


template <typename T> std::string tostr(const T& t) {
  std::ostringstream os;
  os<<t;
  return os.str();
}

/*
inline void mythrow(const std::string &str){
  throw str + "\n " +  __FILE__ + ":" + tostr(__LINE__);
}*/

#define mythrow(str) throw std::string(str) + "\n " +  __FILE__ + ":" + tostr(__LINE__)
/*
inline void mythrow(const char* format, ...)
{
    char dest[1024];
    va_list argptr;
    va_start(argptr, format);
    vsprintf(dest, format, argptr);
    va_end(argptr);
    mythrow(std::string(dest));
}*/


#endif
