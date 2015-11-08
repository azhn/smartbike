#include "common.h"
#include <stdlib.h>

DataPair* DataPair_new(uint32_t count, uint32_t time) {
   DataPair* d = (DataPair*)malloc(sizeof(DataPair));
   d->count = count;
   d->time = time;
   return d;
}
