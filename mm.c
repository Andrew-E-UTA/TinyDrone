// Memory manager functions
// J Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "mm.h"
#include "kernel.h"

//-----------------------------------------------------------------------------
// MMU Defines and MMU Variables
//-----------------------------------------------------------------------------

uint64_t HeapFreeField = ~0;
uint64_t HeapLockedField = 0x0000000180018180;
HeapEntry HeapTable[12] = {};

extern uint8_t getTaskCurrent();
extern uint64_t* getCurrentSrd();

#define NUM_SR  40
#define MAX_TASKS 12

#define SMALL   5
#define LARGE   10
#define CONTIG1 15
#define CONTIG2 20

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

/*
 *  Layout:
 *                     SR       BaseAddr
 *       __________
 *      |____8k____|__39:32____0x2000.6000_____Border Regions
 *      |____4k____|__31:24___0x2000.5000   |
 *      |____4k____|__23:16__0x2000.4000____|
 *      |____8k____|__15:8__0x2000.2000_____|
 *      |____4k____|__7:0__0x2000.1000
 */

void * mallocHeap(uint32_t size_in_bytes)
{
    uint8_t sr_curr, sr_start, sr_len, i;
    uint8_t lockedSmall[3] = {7, 16, 31};
    uint16_t size_remaining;
    bool up = true;

    if(!size_in_bytes)         return nullptr;
    size_in_bytes = ceil512(size_in_bytes);

    //Would be better optimized using a border region
    if(((size_in_bytes - 512) % 1024) == 0 && size_in_bytes > 512)
    {
        for(i = 0; i < 3; ++i)
        {
            sr_curr = lockedSmall[i];
            size_remaining = size_in_bytes;
            while(isFree(sr_curr))
            {
                if((size_remaining -= srSize(up?sr_curr++:sr_curr--)) == 0)
                {
                    sr_start = up?lockedSmall[i]:(sr_curr-1);
                    sr_len = up?((sr_curr)-lockedSmall[i]):(lockedSmall[i]-(sr_curr-1));
                    addHeapEntry(sr_start, sr_len);
                    return srBase(sr_start);
                }
            }
            up = !up;
        }
    }
    else
    {
        findSubRegions(&sr_start, &sr_len, size_in_bytes);
        if(sr_start == NUM_SR)
            return nullptr;
        addHeapEntry(sr_start, sr_len);
        return srBase(sr_start);
    }
    return nullptr;
}

void freeHeap(void *address_from_malloc)
{
    uint8_t task;

    for(task = 0; task < MAX_TASKS; ++task)
        if(HeapTable[task].start == srNum((uint32_t)address_from_malloc))
            if(HeapTable[task].task_idx == getTaskCurrent())
            {
                removeHeapEntry(task);
                removeSramAccessWindow(getCurrentSrd(), srBase(HeapTable[task].start), srLen(HeapTable[task].start, HeapTable[task].length));
                return;
            }
            else
                _kill(getTaskCurrent());
}

//First finds best fit if cannot, then it finds contiguous blocks first ignoring locked regions then including them
void findSubRegions(uint8_t* start, uint8_t* len, uint16_t size)
{
    uint8_t sr_curr, ofs, state;
    uint16_t size_remaining;

    if(size == 512) state = SMALL;
    else if (size == 1024) state = LARGE;
    else state = CONTIG1;

    while(true)
    {
        for(sr_curr = 0; sr_curr < 40; ++sr_curr)
        {
            ofs = 0;
            size_remaining = size;
            while(isFree(sr_curr + ofs) && stateFlag(state, sr_curr + ofs))
            {
                if((size_remaining -= srSize(sr_curr+ofs++)) == 0)
                {
                    *start = sr_curr;
                    *len = ofs;
                    return;
                }
            }
        }
        if(nextState(&state)) break;
    }
    *start = NUM_SR;
    *len = 0;
    return;
}

//finds open entry and inserts start len and getTaskCurrent()
void addHeapEntry(uint8_t start, uint8_t len)
{
    uint8_t ofs = 0;
    while(HeapTable[ofs++].task_idx != MAX_TASKS);
    HeapTable[ofs-1].length = len;
    HeapTable[ofs-1].start = start;
    HeapTable[ofs-1].task_idx = getTaskCurrent();
    for(ofs = start; ofs < (start+len); ++ofs)
        setFreeField(ofs, 0);
}

void removeHeapEntry(uint8_t task)
{
    uint8_t sr;
    for(sr = HeapTable[task].start; sr < (HeapTable[task].start + HeapTable[task].length); ++sr)
        setFreeField(sr, 1);
    HeapTable[task].length = 0;
    HeapTable[task].start = NUM_SR;
    HeapTable[task].task_idx = MAX_TASKS;
}

bool findHeapEntry(uint8_t task, uint32_t* base, uint32_t* size)
{
    uint8_t ofs = 0;
    while(HeapTable[ofs++].task_idx != task && ofs < MAX_TASKS);
    ofs--;
    if(ofs < MAX_TASKS && HeapTable[ofs].task_idx != MAX_TASKS)
    {
        if(base) *base = (uint32_t)srBase(HeapTable[ofs].start);
        if(size) *size = srLen(HeapTable[ofs].start, HeapTable[ofs].length);
        return true;
    }
    else return false;
}

bool getHeapEntry(uint8_t ofs, uint8_t* task, uint32_t* base, uint32_t* size)
{
        if(base) *base = (uint32_t)srBase(HeapTable[ofs].start);
        if(size) *size = srLen(HeapTable[ofs].start, HeapTable[ofs].length);
        if(task) *task = HeapTable[ofs].task_idx;
        return (*task != MAX_TASKS);
}

inline uint16_t ceil512(uint16_t size_bytes)
{
    return (size_bytes/512)*512 + ((bool)(size_bytes%512))*512;
}

inline bool isFree(uint8_t sr)
{
    return HeapFreeField & (1ull<<sr);
}

inline bool islocked(uint8_t sr)
{
    return HeapLockedField & (1ull<<sr);
}

void setFreeField(uint8_t bit, bool on)
{
    if(on)
        HeapFreeField |= (1ull << bit);
    else
        HeapFreeField &= ~(1ull << bit);
}

inline uint16_t srSize(uint8_t sr)
{
    bool small =  ((16 <= sr) && (sr <= 31)) || (sr <= 7);
    return small * 512 + !small * 1024;
}

//given sr number, returns the base addr
void* srBase(uint8_t sr)
{
    uint8_t i;
    uint32_t addr = 0x20001000;
    for(i = 0; i < sr; ++i)
        addr += srSize(i);
    return (void*)addr;
}

//given base addr, returns the associated sr number
uint8_t srNum(uint32_t base)
{
    uint8_t i;
    uint32_t addr = 0x20001000;
    for(i = 0; i <= NUM_SR; ++i)
        if(addr == base) break;
        else addr += srSize(i);
    return i;
}

//given sr start and len, returns the size in bytes of that area
uint16_t srLen(uint8_t start, uint8_t len)
{
    uint8_t i;
    uint16_t size = 0;
    for(i = start; i < start+len; i++)
        size += srSize(i);
    return size;
}

//Switches to next state and returns true if reached the end
inline bool nextState(uint8_t* state)
{
    switch(*state)
    {
    case SMALL:     *state = LARGE;   return false;
    case LARGE:     *state = CONTIG1; return false;
    case CONTIG1:   *state = CONTIG2; return false;
    case CONTIG2:   *state = CONTIG2; return true;
    }
    return true;
}

//based on the state we want to check for different criteria
inline bool stateFlag(uint8_t state, uint8_t sr)
{
    switch(state)
    {
    case SMALL:     return (!islocked(sr) && srSize(sr) == 512);
    case LARGE:     return (!islocked(sr) && srSize(sr) == 1024);
    case CONTIG1:   return (!islocked(sr));
    case CONTIG2:   return true;
    }
    return false;
}

void initHeap()
{
    uint8_t i;
    for(i = 0; i < MAX_TASKS; ++i)
    {
        HeapTable[i].length = 0;
        HeapTable[i].start = NUM_SR;
        HeapTable[i].task_idx = MAX_TASKS;
    }
}
// REQUIRED: initialize MPU here
void initMpu(void)
{
    allowFlashAccess();
    allowPeripheralAccess();
    setupSramAccess();
    enableMpu();
}

void allowFlashAccess(void)
{
    NVIC_MPU_NUMBER_R = 0x5;
    NVIC_MPU_BASE_R  |= FLASH_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = RWRO << AP_SHIFT             |
                        FLASH_SCB << SCB_SHIFT       |
                        ALL_SRD_UNSET << SRD_SHIFT   |
                        FLASH_SIZE << SIZE_SHIFT     |
                        NVIC_MPU_ATTR_ENABLE;
}

void allowPeripheralAccess(void)
{
    NVIC_MPU_NUMBER_R = 0x6;
    NVIC_MPU_BASE_R  |= PERIPHERAL_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = NVIC_MPU_ATTR_XN                  |
                        RWRW << AP_SHIFT                  |
                        ALL_SRD_UNSET << SRD_SHIFT        |
                        PERIPHERAL_SCB << SCB_SHIFT       |
                        PERIPHERAL_SIZE << SIZE_SHIFT     |
                        NVIC_MPU_ATTR_ENABLE;
}

void setupSramAccess(void)
{
    NVIC_MPU_NUMBER_R = 0x0;
    NVIC_MPU_BASE_R  |= R0_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = NVIC_MPU_ATTR_XN            |
                        RWRW << AP_SHIFT            |
                        SRAM_SCB << SCB_SHIFT       |
                        ALL_SRD_SET << SRD_SHIFT    |
                        HALF_SIZE << SIZE_SHIFT|
                        NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 0x1;
    NVIC_MPU_BASE_R  |= R1_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = NVIC_MPU_ATTR_XN            |
                        RWRW << AP_SHIFT            |
                        SRAM_SCB << SCB_SHIFT       |
                        ALL_SRD_SET << SRD_SHIFT    |
                        FULL_SIZE << SIZE_SHIFT     |
                        NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 0x2;
    NVIC_MPU_BASE_R  |= R2_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = NVIC_MPU_ATTR_XN            |
                        RWRW << AP_SHIFT            |
                        SRAM_SCB << SCB_SHIFT       |
                        ALL_SRD_SET << SRD_SHIFT    |
                        HALF_SIZE << SIZE_SHIFT|
                        NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 0x3;
    NVIC_MPU_BASE_R  |= R3_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = NVIC_MPU_ATTR_XN            |
                        RWRW << AP_SHIFT            |
                        SRAM_SCB << SCB_SHIFT       |
                        ALL_SRD_SET << SRD_SHIFT    |
                        HALF_SIZE << SIZE_SHIFT|
                        NVIC_MPU_ATTR_ENABLE;

    NVIC_MPU_NUMBER_R = 0x4;
    NVIC_MPU_BASE_R  |= R4_BASE_ADDR & NVIC_MPU_BASE_ADDR_M;
    NVIC_MPU_ATTR_R   = NVIC_MPU_ATTR_XN            |
                        RWRW << AP_SHIFT            |
                        ALL_SRD_SET << SRD_SHIFT    |
                        SRAM_SCB << SCB_SHIFT       |
                        FULL_SIZE << SIZE_SHIFT|
                        NVIC_MPU_ATTR_ENABLE;
}

inline uint64_t createNoSramAccessMask(void)
{
    return 0xFFFFFFFFFFFFFFFF;
}

void removeSramAccessWindow(uint64_t *srdBitMask, uint32_t *baseAdd, uint32_t size_in_bytes)
{
    uint8_t sr = srNum((uint32_t)baseAdd);
    uint32_t sumSize = 0;
    do
    {
        *srdBitMask |= (1ull<<(sr++));
    }while((sumSize += srSize(sr)) < size_in_bytes);
}

void addSramAccessWindow(uint64_t *srdBitMask, uint32_t *baseAdd, uint32_t size_in_bytes)
{
    uint8_t sr = srNum((uint32_t)baseAdd);
    uint32_t sumSize = 0;
    do
    {
        *srdBitMask &= ~(1ull<<(sr++));
    }while((sumSize += srSize(sr)) < size_in_bytes);
}

void applySramAccessMask(uint64_t srdBitMask)
{
    uint8_t region;
    uint8_t SRD;
    for(region = 0; region < 5; region++)
    {
        SRD = 0x00;
        NVIC_MPU_NUMBER_R = region;
        NVIC_MPU_ATTR_R  &= ~NVIC_MPU_ATTR_SRD_M;
        SRD = (uint8_t)(srdBitMask >> (region*8));
        NVIC_MPU_ATTR_R  |= SRD<<SRD_SHIFT;
    }
}

void enableMpu(void)
{

    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM | NVIC_SYS_HND_CTRL_BUS
                         | NVIC_SYS_HND_CTRL_USAGE;
    NVIC_CFG_CTRL_R |= NVIC_CFG_CTRL_DIV0;
    NVIC_MPU_CTRL_R = NVIC_MPU_CTRL_PRIVDEFEN | NVIC_MPU_CTRL_ENABLE;
}
