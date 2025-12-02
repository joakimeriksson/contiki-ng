/*
 * ESP32-C6 Minimal Loader (based on Zephyr approach)
 * This code runs from IRAM and sets up memory mapping before jumping to main app
 */

#include <stdint.h>

/* Memory addresses from ESP32-C6 TRM */
#define SOC_DRAM_LOW    0x40800000
#define SOC_DRAM_HIGH   0x40880000
#define SOC_IRAM_LOW    0x40800000
#define SOC_IRAM_HIGH   0x40880000
#define SOC_IROM_LOW    0x42000000
#define SOC_IROM_HIGH   0x42800000
#define SOC_DROM_LOW    0x42800000
#define SOC_DROM_HIGH   0x43000000

/* Cache and MMU hardware registers for ESP32-C6 */
#define CACHE_BASE      0x600C0000
#define MMU_BASE        0x600C5000

#define DR_REG_EXTMEM_BASE  0x600C0000
#define CACHE_L1_CACHE_CTRL_REG (DR_REG_EXTMEM_BASE + 0x04)
#define CACHE_L1_CACHE_CTRL_SHUTDOWN_BIT (1 << 0)
#define CACHE_L1_CACHE_CTRL_INVALIDATE_BIT (1 << 1)

/* MMU page size */
#define MMU_PAGE_SIZE   0x10000  /* 64KB pages */

/* Entry point symbol */
extern void __start(void);

/* Metadata structure (matches linker script) */
struct metadata {
    uint32_t magic;           /* 0xace637d3 */
    void (*entry_point)(void);

    /* IRAM */
    uint32_t iram_vaddr;
    uint32_t iram_laddr;
    uint32_t iram_size;

    /* DRAM */
    uint32_t dram_vaddr;
    uint32_t dram_laddr;
    uint32_t dram_size;

    /* RTC (unused) */
    uint32_t rtc_reserved[6];

    /* IROM */
    uint32_t irom_vaddr;
    uint32_t irom_laddr;
    uint32_t irom_size;

    /* DROM */
    uint32_t drom_vaddr;
    uint32_t drom_laddr;
    uint32_t drom_size;
};

/* Metadata is at the start of flash */
extern struct metadata _metadata_start;

/* Linker symbols */
extern uint32_t _image_irom_start, _image_irom_size, _image_irom_vaddr;
extern uint32_t _image_drom_start, _image_drom_size, _image_drom_vaddr;

/* Entry address marker for metadata */
__attribute__((section(".entry_addr"), used))
static void (*_entry_point)(void) = &__start;

/* Simple cache disable function */
static inline void cache_disable(void)
{
    volatile uint32_t *ctrl_reg = (volatile uint32_t *)CACHE_L1_CACHE_CTRL_REG;
    *ctrl_reg |= CACHE_L1_CACHE_CTRL_SHUTDOWN_BIT;
}

/* Simple cache invalidate and enable */
static inline void cache_enable(void)
{
    volatile uint32_t *ctrl_reg = (volatile uint32_t *)CACHE_L1_CACHE_CTRL_REG;

    /* Invalidate cache */
    *ctrl_reg |= CACHE_L1_CACHE_CTRL_INVALIDATE_BIT;
    while (*ctrl_reg & CACHE_L1_CACHE_CTRL_INVALIDATE_BIT);

    /* Enable cache */
    *ctrl_reg &= ~CACHE_L1_CACHE_CTRL_SHUTDOWN_BIT;
}

/* Simplified MMU mapping - maps flash region to virtual address
 * For production, this needs full ESP-IDF HAL implementation
 */
static void mmu_map_region(uint32_t vaddr, uint32_t paddr, uint32_t size)
{
    /* ESP32-C6 MMU table base */
    volatile uint32_t *mmu_table = (volatile uint32_t *)MMU_BASE;

    /* Calculate MMU entry index from virtual address */
    uint32_t mmu_entry = (vaddr - SOC_IROM_LOW) / MMU_PAGE_SIZE;

    /* Calculate physical page number */
    uint32_t phys_page = paddr / MMU_PAGE_SIZE;

    /* Calculate number of pages to map */
    uint32_t num_pages = (size + MMU_PAGE_SIZE - 1) / MMU_PAGE_SIZE;

    /* Map pages */
    for (uint32_t i = 0; i < num_pages; i++) {
        /* MMU entry format: [valid][physical_page] */
        /* For ESP32-C6, valid bit is bit 14, physical page in lower bits */
        mmu_table[mmu_entry + i] = (1 << 14) | (phys_page + i);
    }
}

/* Main loader function - called from startup */
void loader_main(void)
{
    struct metadata *meta = &_metadata_start;

    /* Verify metadata magic */
    if (meta->magic != 0xace637d3) {
        /* Metadata invalid - halt */
        while(1);
    }

    /* Disable cache before MMU changes */
    cache_disable();

    /* Map IROM region (code in flash) */
    if (meta->irom_size > 0) {
        mmu_map_region(meta->irom_vaddr, meta->irom_laddr, meta->irom_size);
    }

    /* Map DROM region (rodata in flash) */
    if (meta->drom_size > 0) {
        mmu_map_region(meta->drom_vaddr, meta->drom_laddr, meta->drom_size);
    }

    /* Enable cache */
    cache_enable();

    /* Jump to application entry point */
    if (meta->entry_point) {
        meta->entry_point();
    }

    /* Should never reach here */
    while(1);
}
