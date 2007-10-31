PHDRS
{
  text PT_LOAD FILEHDR PHDRS FLAGS (7);
  data PT_LOAD FLAGS (7);
}
#ifdef CONFIG_SUN3
#include "vmlinux-sun3.lds"
#elif CONFIG_COLDFIRE
#include "../coldfire/vmlinux-cf.lds"
#else
#include "vmlinux-std.lds"
#endif
