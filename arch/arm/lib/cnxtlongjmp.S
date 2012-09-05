/******************************************************************************/
/*                   CONEXANT PROPRIETARY AND CONFIDENTIAL                    */
/*                        SOFTWARE FILE/MODULE HEADER                         */
/*                 Copyright Conexant Systems Inc. 2007                       */
/*                                 Hyderabad                                  */
/*                            All Rights Reserved                             */
/******************************************************************************/
/*
 * Filename:        cnxtlongjmp.S
 *
 *
 * Description:     ARM Assembly file for longjmp.
 *
 *
 * Author:          Kaushik Bar
 *
 ******************************************************************************/
/* $Id$
 ******************************************************************************/
.global longjmp
.align 2

longjmp:
     mov       ip, r0       /* save jmp_buf pointer */
     movs      r0, r1       /* get the return value in place */
     moveq     r0, #1       /* can't let longjmp return zero! */
     ldmia     ip!,  {v1-v6, sl, fp, sp, lr}
     #ifdef CONFIG_FRAME_POINTER
        bx     lr
     #else
        mov    pc, lr
     #endif

/*******************************************************************************
 * Modifications:
 * $Log:
 *
 ******************************************************************************/
 
