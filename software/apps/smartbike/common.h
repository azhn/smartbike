/* data pair for quadrature encoder */
#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

typedef struct DataPair {
   uint32_t count;
   uint32_t time;
} DataPair;

DataPair* DataPair_new(uint32_t count, uint32_t time);

#endif // COMMON_H_
