#ifndef POLYGLOT_TURTLE_XIAO_PROJ_DELAY_H
#define POLYGLOT_TURTLE_XIAO_PROJ_DELAY_H

#define _CyclesPerDelayLoop		6
#define _LoopCount_ms(ms)		(uint32_t)(((F_CPU / 1000UL) * (ms)) / _CyclesPerDelayLoop)
#define _LoopCount_us(us)		(uint32_t)(((F_CPU / 1000000UL) * (us)) / _CyclesPerDelayLoop)
#define ddDelay_ms(ms)			_ddCycleDelay(_LoopCount_ms(ms))
#define ddDelay_us(us)			_ddCycleDelay(_LoopCount_us(us))

inline void _ddCycleDelay(volatile uint32_t count) {
    asm volatile (
            "mov r0, %0 \n"		// Get the loop count
            "loop%=:    \n"		// Loop start label
            "DMB        \n"		// Data Memory Barrier (3 cycles)
            "sub r0, #1 \n"		// Decrement count     (1 cycle)
            "bne loop%= \n"		// Branch back to loop (2 cycles)
            :					// Nothing output from assembly
            : "r" (count)		// Register input for count
            : "r0", "cc"		// Clobber list
            );
}

#endif