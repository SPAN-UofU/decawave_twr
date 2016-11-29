.origin 0
.entrypoint START

#include "SPI.hp"

#define SPI1_CLKCTRL    0x44E00050
#define SYSCONFIG       0x481A0110
#define SYSSTATUS       0x481a0114
#define IRQSTATUS       0x481A0118
#define IRQENABLE       0x481A011C
#define SYST            0x481A0124
#define MODULCTRL       0x481A0128
#define CHCONF          0x481a012c 
#define CHSTAT          0x481a0130
#define CHCTRL          0x481a0134
#define TX              0x481a0138
#define RX              0x481a013c

START:
    // Enable OCP master port
    LBCO r0, CONST_PRUCFG, 4, 4
    CLR  r0, r0, 4
    SBCO r0, CONST_PRUCFG, 4, 4

    // C28 will point to 0x00010000 (PRU shared RAM)
    MOV  r0, 0x00000100
    MOV  r1, CTPPR_0
    ST32 r0, r1
  
//    // Enable CLKSPIREF and CLK
//    MOV  r1, SPI1_CLKCTRL
//    MOV  r2, 0x00000002
//    SBBO r2, r1, 0, 4
//    
//    // Reset SPI
//    MOV  r1, SYSCONFIG
//    LBBO r2, r1, 0, 4
//    SET  r2.t1
//    SBBO r2, r1, 0, 4
//    
//    // Wait for RESET
//RESET:
//    MOV  r1, SYSSTATUS
//    LBBO r2, r1, 0, 4
//    QBBC RESET, r2.t0
//    
//    // Config MODULCTRL
//    MOV  r1, MODULCTRL
//    MOV  r2, 0x00000001
//    SBBO r2, r1, 0, 4
// 
//    // Config SYSCONFIG
//    MOV  r1, SYSCONFIG
//    MOV  r2, 0x00000311
//    SBBO r2, r1, 0, 4
//    
//    // Reset interrupt status bits
//    MOV  r1, IRQSTATUS
//    MOV  r2, 0xFFFFFFFF
//    SBBO r2, r1, 0, 4
//    
//    // Disable interupts
//    MOV  r1, IRQENABLE
//    MOV  r2, 0x00000000
//    SBBO r2, r1, 0, 4
// 
//    // Set MCSPI_SYST
//    MOV  r1, SYST
//    MOV  r2, 0x00000000
//    SET  r2.t9
//    SBBO r2, r1, 0, 4
    
    // Disable channel 0
    CALL DISABLE_CH0
    
    // Start
    //SET r30.t7

    // Initialize sampling counter
    LBCO r5, CONST_PRUSHAREDRAM, 0, 4
    MOV  r6, 0
SMP:
    // Initialize word counter
    MOV  r3, 0

    // Wait until sample is ready
    WBS r31.t16

    // Configure channel 0 of MCSPI1
    CALL SET_SPIEN

// ******************** BEGIN ********************
SPI_TX:    
    // Enable channel 0
    CALL ENABLE_CH0
    
    // Make sure TX reg is cleared
    CALL CHECKTX0

    // Write word to TX register
    MOV  r1, TX
    //LBCO r4, CONST_PRUSHAREDRAM, r3, 1 //byte to transmit
    // Start case
    MOV r4, 0xef
    QBEQ ELSE, r3, 0 
    MOV r4, 0x81
    QBEQ ELSE, r3, 1
    MOV r4, 0x00
ELSE:
    SBBO r4, r1, 0, 4

    // Check if RX done
    CALL CHECKRX0

SPI_RX:
    // Get data from RX Reg
    MOV  r1, RX
    LBBO r2, r1, 0, 4

    // Store
    SBCO r2, CONST_PRUSHAREDRAM, r6, 1 //byte to receive

    // Increment
    ADD  r6, r6, 1

    // Check channel 0
    MOV  r1, CHSTAT
    LBBO r2, r1, 0, 4
    QBBS SPI_RX, r2.t0

END_TX:
    // Disable channel 0
    CALL DISABLE_CH0

    // Increment counter 
    ADD r3, r3, 1
    QBNE SPI_TX, r3, 7
// ******************** END ********************

END_LOOP: 
    // Configure channel 0 of MCSPI1
    CALL CLR_SPIEN

    // Increment counter
    SUB r5, r5, 1
    QBEQ EXIT, r5, 0
    JMP SMP

//////////////////////////////////////
CHECKTX0:
    // Check channel 0
    MOV  r1, CHSTAT
    LBBO r2, r1, 0, 4
    QBBC CHECKTX0, r2.t1
    RET

CHECKRX0:
    // Check channel 0
    MOV  r1, CHSTAT
    LBBO r2, r1, 0, 4
    QBBC CHECKRX0, r2.t0
    RET

CHECKEOT0:
    // Check channel 0
    MOV  r1, CHSTAT
    LBBO r2, r1, 0, 4
    QBBC CHECKEOT0, r2.t2
    RET

DISABLE_CH0:
    //Disable channel 0
    MOV  r1, CHCTRL
    MOV  r2, 0x00000000
    SBBO r2, r1, 0, 4
    RET

ENABLE_CH0:
    // Enable channel 0
    MOV  r1, CHCTRL
    MOV  r2, 0x00000001
    SBBO r2, r1, 0, 4
    RET

CLR_SPIEN:
    // Configure channel 0 of MCSPI1
    MOV  r1, CHCONF 
    MOV  r2, 0x000103CC
    CLR  r2.t20 
    SBBO r2, r1, 0, 4
    RET

SET_SPIEN:
    // Configure channel 0 of MCSPI1
    MOV  r1, CHCONF 
    MOV  r2, 0x000103CC
    SET  r2.t20 
    SBBO r2, r1, 0, 4
    RET

//////////////////////////////////////
EXIT:
    // End
    //CLR r30.t7

    MOV R31.b0, PRU0_ARM_INTERRUPT+16
    HALT

