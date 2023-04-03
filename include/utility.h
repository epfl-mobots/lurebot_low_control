#ifndef UTILITY_H
#define UTILITY_H

#include <parameters.h>

template <typename arg>
void LOG(arg v)
{
#ifdef ENABLE_LOGGING
    Serial.print(v);
    Serial.println();
#endif
}

template <typename arg, typename... args>
void LOG(const arg& v, const args... vs)
{
#ifdef ENABLE_LOGGING
    Serial.print(v);
    Serial.print(" ");
    LOG(vs...);
#endif
}

#endif