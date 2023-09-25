

#ifndef __PACKED_H__
#define __PACKED_H__

#include "stdint.h"

#if  defined ( __GNUC__ )
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
  #ifndef __packed
    #define __packed __attribute__((__packed__))
  #endif /* __packed */
#elif defined(__CC_ARM)
#else
// unknown compiler
#define __weak
#endif /* __GNUC__ */

#define WEAK     __weak

#if defined   (__GNUC__)        /* GNU Compiler */
  #define __ALIGN_END    __attribute__ ((aligned (4)))
  #define __ALIGN_BEGIN
#else
  #define __ALIGN_END
  #if defined   (__CC_ARM)      /* ARM Compiler */
    #define __ALIGN_BEGIN    __align(4)
  #elif defined (__ICCARM__)    /* IAR Compiler */
    #define __ALIGN_BEGIN
  #elif defined  (__TASKING__)  /* TASKING Compiler */
    #define __ALIGN_BEGIN    __align(4)
  #else
    #define __ALIGN_BEGIN
  #endif /* __CC_ARM */
#endif /* __GNUC__ */

#if defined   (__GNUC__)
#define __PACK_BEGIN
#define __PACK_END    __packed
#elif defined(__CC_ARM)
#define __PACK_BEGIN  __packed
#define __PACK_END
#else
#define __PACK_BEGIN
#define __PACK_END
#endif

// Some device not support un-aligned access
#define EP_MPS(ep_desc)  (\
    ((uint16_t)(((uint8_t*)(&((ep_desc)->wMaxPacketSize)))[0]) << 0) \
  | ((uint16_t)(((uint8_t*)(&((ep_desc)->wMaxPacketSize)))[1]) << 8))

#endif
