#pragma once
#include <stdio.h>
#include <iostream>

/* level 0: only errors */
/* level 1: + app messages */
/* level 2: + warnings */
/* level 3: + infos */
#define PHYSICS_PRINT_DEBUG_LEVEL 1

#define PRINT_ERR(x) \
  do {               \
    std::cout << x;  \
  } while (0)

#if PHYSICS_PRINT_DEBUG_LEVEL > 0
#define PRINT_APP(x) \
  do {               \
    std::cout << x;  \
  } while (0)
#else
#define PRINT_APP(x)
#endif
#if PHYSICS_PRINT_DEBUG_LEVEL > 1
#define PRINT_WARN(x) \
  do {                \
    std::cout << x;   \
  } while (0)
#else
#define PRINT_WARN(x)
#endif
#if PHYSICS_PRINT_DEBUG_LEVEL > 2
#define PRINT_INFO(x) \
  do {                \
    std::cout << x;   \
  } while (0)
#else
#define PRINT_INFO(x)
#endif

#define DEBUG_PRINT PRINT_INFO