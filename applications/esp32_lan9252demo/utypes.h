#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   uint32_t Encoder1;
   uint32_t Encoder2;
   uint32_t IMU;

   /* Outputs */

   uint8_t LED;

} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
