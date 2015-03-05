#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <asm/sizes.h>

#define UL(x) _AC(x, UL)

#define PLAT_PHYS_OFFSET	UL(0x00000000)

#define PHYS_OFFSET		PLAT_PHYS_OFFSET

#ifdef CONFIG_SPARSEMEM

#define MAX_PHYSMEM_BITS    32 /* The top bit is set for virtual space so
                                  you need all 32-bits for an address */

#define SECTION_SIZE_BITS   28 /* log2(256M) = 28 */

#ifndef __ASSEMBLY__

#define __phys_to_virt(phys) ({                                 \
        unsigned long virt = 0;                                 \
        if (((phys) >= 0x20000000UL))                           \
                virt = (phys) - 0x10000000UL + PAGE_OFFSET;     \
        else                                                    \
                virt = (phys) + PAGE_OFFSET;                    \
        virt;                                                   \
})


#define __virt_to_phys(virt) ({                                 \
        unsigned long phys = 0;                                 \
        if (((virt) >= PAGE_OFFSET + 0x10000000UL))             \
                phys = (virt) - PAGE_OFFSET + 0x10000000UL;     \
        else if ((virt) >= PAGE_OFFSET)                         \
                phys = (virt) - PAGE_OFFSET;                    \
        phys;                                                   \
})

#endif

#endif
#endif /* __ASM_MACH_MEMORY_H__ */
