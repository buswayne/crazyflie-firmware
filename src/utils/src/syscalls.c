/**
 * syscalls.c: Minimal newlib syscall stubs.
 *
 * Pulled in transitively (via rand() -> impure.o -> findfp.o -> mallocr.o/sbrkr.o
 * and friends) as soon as any reentrant libc state is touched, even though this
 * firmware never uses newlib's stdio or heap (malloc/free are redirected to
 * FreeRTOS's heap in malloc.c). Under -nostdlib none of the usual stubs
 * (nosys.specs) get linked, so provide just enough to satisfy the link; none of
 * these are expected to actually run.
 */

#include <errno.h>
#include <sys/stat.h>

void *_sbrk(int incr)
{
  (void)incr;
  errno = ENOMEM;
  return (void *)-1;
}

int _write(int file, const char *ptr, int len)
{
  (void)file;
  (void)ptr;
  return len;
}

int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _close(int file)
{
  (void)file;
  return -1;
}

int _lseek(int file, int offset, int whence)
{
  (void)file;
  (void)offset;
  (void)whence;
  return 0;
}

int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}
