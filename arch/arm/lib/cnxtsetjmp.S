/******************************************************************************/
/*                   CONEXANT PROPRIETARY AND CONFIDENTIAL                    */
/*                        SOFTWARE FILE/MODULE HEADER                         */
/*                 Copyright Conexant Systems Inc. 2007                       */
/*                                 Hyderabad                                  */
/*                            All Rights Reserved                             */
/******************************************************************************/
/*
 * Filename:        cnxtsetjmp.S
 *
 *
 * Description:     ARM Assembly file for setjmp.
 *
 *
 * Author:          Kaushik Bar
 *
 ******************************************************************************/
/* $Id$
 ******************************************************************************/
.global _setjmp
.align 2

_setjmp:
     mov     ip, r0
     /* Save registers */
     stmia   ip!, {v1-v6, sl, fp, sp, lr}
     mov     r0, #0                        /* make setjmp return zero */
     #ifdef CONFIG_FRAME_POINTER
        bx   lr
     #else
        mov  pc, lr
     #endif

/*******************************************************************************
 * Modifications:
 * $Log:
 *
 ******************************************************************************/

