.thumb

.globl _start
_start:
    .word 0x20010000 // init stack pointer at end of sram1
    .word reset

    .thumb_func
    reset:
        bl main
        b .

    .globl dummy
    .thumb_func
    dummy:
        bx lr
    
