/* syscalls.c - Working debug output for STM32F407G-DISC1 */
#include <_ansi.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "core_cm4.h"

int _write(int file, char *ptr, int len) {
  if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
    return -1;
  }
  
  for (int i = 0; i < len; i++) {
    // Use CMSIS-provided ITM_SendChar
    if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0) && 
        ((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0) &&
        ((ITM->TER & 1) != 0)) {
      while (ITM->PORT[0].u32 == 0);
      ITM->PORT[0].u8 = (uint8_t)*ptr++;
    }
  }
  
  return len;
}

// Required standard functions (minimal implementation)
caddr_t _sbrk(int incr) {
  extern char end __asm__("end");
  static char *heap_end;
  char *prev_heap_end;
  
  if (heap_end == 0) {
    heap_end = &end;
  }
  
  prev_heap_end = heap_end;
  heap_end += incr;
  
  return (caddr_t)prev_heap_end;
}

void _exit(int status) {
  (void)status;
  while (1);
}

int _open(const char *name, int flags, int mode) {
  (void)name;
  (void)flags;
  (void)mode;
  return -1;
}

int _close(int file) {
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st) {
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}