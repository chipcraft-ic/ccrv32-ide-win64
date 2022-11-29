/*H*****************************************************************************
*
* Copyright (c) 2018 ChipCraft Sp. z o.o. All rights reserved
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ChipCraft Sp. z o.o. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* ******************************************************************************
* File Name : libc_multicore.c
* Author    : Rafal Harabien
* ******************************************************************************
* $Date: 2022-01-19 09:38:48 +0100 (śro, 19 sty 2022) $
* $Revision: 814 $
*H*****************************************************************************/

#include <sys/reent.h>
#include <sys/lock.h>
#include <ccrv32.h>
#include <ccrv32-csr.h>
#include <ccrv32-mcore.h>
#include <ccrv32-utils.h>

static struct _reent * g_spramImpureData_array;

extern void *_sbrk_r(struct _reent *r, ptrdiff_t incr);

struct _reent * __getreent(void)
{
    return &g_spramImpureData_array[0];
}

void __libc_impure_alloc(void)
{
    g_spramImpureData_array = (struct _reent *)((int)(_sbrk_r(0, (1 * sizeof(struct _reent))+3)+3)&(~3));
}

void __libc_impure_init(void)
{
    _REENT_INIT_PTR(__getreent());
}

__attribute__((weak)) void __libc_lock_release_recursive(_LOCK_RECURSIVE_T *lock){};
__attribute__((weak)) void __libc_lock_acquire_recursive(_LOCK_RECURSIVE_T *lock){};
__attribute__((weak)) void __libc_lock_close_recursive(_LOCK_RECURSIVE_T *lock){};
__attribute__((weak)) void __libc_lock_init_recursive(_LOCK_RECURSIVE_T *lock){};
