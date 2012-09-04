/* linux/include/asm-arm/arm11jmp.h
 *
 * prototypes for setjmp and longjump on Trident CX2450x (ARM11)
 *
 * Copyright (C) 2010 CoolStream International Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __ARM11JMP__
#define __ARM11JMP__

typedef int    __jmp_buf[64];
typedef struct __jmp_buf_tag
{
   __jmp_buf __jmpbuf;
} jmp_buf[1];

int  _setjmp (jmp_buf __env);
void longjmp (jmp_buf __env, int __val);

#endif /* __ARM11JMP__ */

