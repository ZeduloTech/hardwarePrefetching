#ifndef __SIM_H__
#define __SIM_H__

#include <errno.h>
#include <stdio.h>
#include <linux/types.h>   /* __u32 / __u64, same header the kernel build uses */

#define pr_info  printf
#define pr_err(...)  fprintf(stderr, __VA_ARGS__)

__u64 ktime_get_ns(void);

#include "kernel_common.h"
#include "kernel_primitive.h"


#endif /* __SIM_H__ */