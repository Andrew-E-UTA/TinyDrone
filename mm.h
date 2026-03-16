// Memory manager functions
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef MM_H_
#define MM_H_

#include <stdint.h>
#include <stdbool.h>

#define NUM_SRAM_REGIONS 8
#define nullptr (void*)0x0

//base addresses
#define FLASH_BASE_ADDR         0x00000000
#define SRAM_BASE_ADDR          0x20000000
#define HEAP_BASE_ADDR          0x20001000
#define R0_BASE_ADDR            SRAM_BASE_ADDR+0x1000
#define R1_BASE_ADDR            SRAM_BASE_ADDR+0x2000
#define R2_BASE_ADDR            SRAM_BASE_ADDR+0x4000
#define R3_BASE_ADDR            SRAM_BASE_ADDR+0x5000
#define R4_BASE_ADDR            SRAM_BASE_ADDR+0x6000
#define PERIPHERAL_BASE_ADDR    0x40000000

//mpu attr&size
#define FULL_SIZE               0xC
#define HALF_SIZE               0xB
#define FLASH_SIZE              0x11
#define SRAM_SIZE               0xE
#define PERIPHERAL_SIZE         0x1C

#define RWRW                    0x3
#define RWRO                    0x2
#define RWXX                    0x1
#define FLASH_SCB               0x2
#define SRAM_SCB                0x6
#define PERIPHERAL_SCB          0x5
#define ALL_SRD_UNSET           0x00
#define ALL_SRD_SET             0xFF

#define AP_SHIFT                0x18
#define SCB_SHIFT               0x10
#define SRD_SHIFT               0x8
#define SIZE_SHIFT              0x1

typedef struct _heap_enrty{
    uint8_t length;
    uint8_t start;
    uint8_t task_idx;
}HeapEntry;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void allowFlashAccess(void);
void allowPeripheralAccess(void);
void setupSramAccess(void);
inline uint64_t createNoSramAccessMask(void);
void addSramAccessWindow(uint64_t *srdBitMask, uint32_t *baseAdd, uint32_t size_in_bytes);
void removeSramAccessWindow(uint64_t *srdBitMask, uint32_t *baseAdd, uint32_t size_in_bytes);
void applySramAccessMask(uint64_t srdBitMask);
void enableMpu(void);
void initMpu(void);


uint8_t srNum(uint32_t base);
uint16_t srLen(uint8_t start, uint8_t len);

void freeHeap(void *address_from_malloc);
void findSubRegions(uint8_t* start, uint8_t* len, uint16_t size);
void setFreeField(uint8_t sr, bool on);
void addHeapEntry(uint8_t start, uint8_t len);
bool findHeapEntry(uint8_t task, uint32_t* base, uint32_t* size);
bool getHeapEntry(uint8_t ofs, uint8_t* task, uint32_t* base, uint32_t* size);
void removeHeapEntry(uint8_t task);
void initHeap();

void* mallocHeap(uint32_t size_in_bytes);
void* srBase(uint8_t sr);

inline uint16_t ceil512(uint16_t size_bytes);
inline uint16_t srSize(uint8_t sr);
inline bool isFree(uint8_t sr);
inline bool islocked(uint8_t sr);
inline bool stateFlag(uint8_t state, uint8_t sr);
inline bool nextState(uint8_t* state);

#endif
