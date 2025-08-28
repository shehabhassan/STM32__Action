#include <stdlib.h>

// Required by some printf implementations
void abort(void) {
  while (1);
}