/*
 * Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of the Kustaa Nyholm or SpareTimeLabs nor the names of its
 * contributors may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

#include <array>
#include <cstdarg>
#include <cstdint>

#include "printf.h"


// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,cert-dcl50-cpp)

// print bf, padded from left to at least n characters.
// padding is zero ('0') if z!=0, space (' ') otherwise
static int putchw(void* handle, putcFnPtr putFn, int n, char z, char *bf)
{
    int written = 0;
    const char fc = z ? '0' : ' ';
    char *p = bf;
    while (*p++ && n > 0) {
        n--;
    }
    while (n-- > 0) {
        putFn(handle, fc);
        ++written;
    }
    char ch = *bf++;
    while (ch) {
        putFn(handle, ch);
        ch = *bf++;
        ++written;
    }
    return written;
}

#if defined(LIBRARY_BLACKBOX_PRINTF_LONG_SUPPORT)
static void uli2a(unsigned long int num, unsigned int base, int uc, char *bf) // NOLINT(google-runtime-int)
{
    unsigned int d = 1;
    while (num / d >= base) {
        d *= base;
    }
    while (d != 0) {
        const int dgt = static_cast<int>(num / d);
        *bf++ = static_cast<char>(dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10));
        // Next digit
        num %= d;
        d /= base;
    }
    *bf = 0;
}

static void li2a(long num, char *bf) // NOLINT(google-runtime-int)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    uli2a(num, 10, 0, bf);
}
#endif

static void ui2a(unsigned int num, unsigned int base, int uc, char *bf)
{
    unsigned int d = 1;
    while (num / d >= base) {
        d *= base;
    }
    while (d != 0) {
        const unsigned int dgt = num / d;
        *bf++ = static_cast<char>(dgt + (dgt < 10 ? '0' : (uc ? 'A' : 'a') - 10));
        // Next digit
        num %= d;
        d /= base;
    }
    *bf = 0;
}

static void i2a(int num, char *bf)
{
    if (num < 0) {
        num = -num;
        *bf++ = '-';
    }
    ui2a(num, 10, 0, bf);
}

static int a2d(char ch)
{
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    }
    if (ch >= 'a' && ch <= 'f') {
        return ch - 'a' + 10;
    }
    if (ch >= 'A' && ch <= 'F') {
        return ch - 'A' + 10;
    }
    return -1;
}

static char a2i(char ch, const char **src, int base, int *nump)
{
    const char *p = *src;
    int num = 0;
    int digit = a2d(ch);
    while (digit >= 0) {
        if (digit > base) {
            break;
        }
        num = num * base + digit;
        ch = *p++;
        digit = a2d(ch);
    }
    *src = p;
    *nump = num;
    return ch;
}

// retrun number of bytes written
size_t tfp_format(void *handle, putcFnPtr putFn, const char *fmt, va_list va) // NOLINT(readability-non-const-parameter,readability-function-cognitive-complexity)
{
    std::array<char, 12> bf;
    size_t written = 0;
    char ch;

    while ((ch = *(fmt++))) {
        if (ch != '%') {
            putFn(handle, ch);
            ++written;
        } else {
            char lz = 0;
#if defined(LIBRARY_BLACKBOX_PRINTF_LONG_SUPPORT)
            char lng = 0;
#endif
            int w = 0;
            ch = *(fmt++);
            if (ch == '0') {
                ch = *(fmt++);
                lz = 1;
            }
            if (ch >= '0' && ch <= '9') {
                ch = a2i(ch, &fmt, 10, &w);
            }
#if defined(LIBRARY_BLACKBOX_PRINTF_LONG_SUPPORT)
            if (ch == 'l') { ch = *(fmt++); lng = 1; }
#endif
            switch (ch) {
            case 0:
                return written;
            case 'u':{
#if defined(LIBRARY_BLACKBOX_PRINTF_LONG_SUPPORT)
                if (lng) { uli2a(va_arg(va, unsigned long int), 10, 0, &bf[0]); } else
#endif
                { ui2a(va_arg(va, unsigned int), 10, 0, &bf[0]); }
                written += putchw(handle, putFn, w, lz, &bf[0]);
                break;
            }
            case 'd':{
#if defined(LIBRARY_BLACKBOX_PRINTF_LONG_SUPPORT)
                if (lng) { li2a(va_arg(va, unsigned long int), &bf[0]); } else
#endif
                { i2a(va_arg(va, int), &bf[0]); }
                written += putchw(handle, putFn, w, lz, &bf[0]);
                break;
            }
            case 'x':
            case 'X':
#if defined(LIBRARY_BLACKBOX_PRINTF_LONG_SUPPORT)
                if (lng) { uli2a(va_arg(va, unsigned long int), 16, (ch == 'X'), &bf[0]); } else
#endif
                { ui2a(va_arg(va, unsigned int), 16, (ch == 'X'), &bf[0]); }
                written += putchw(handle, putFn, w, lz, &bf[0]);
                break;
            case 'c':
                putFn(handle, static_cast<char>(va_arg(va, int)));
                ++written;
                break;
            case 's':
                written += putchw(handle, putFn, w, 0, va_arg(va, char *));
                break;
            case '%':
                putFn(handle, ch);;
                ++written;
                break;
            case 'n':
                *va_arg(va, int*) = static_cast<int>(written);
                break;
            default:
                break;
            }
        }
    }
    return written;
}
// NOLINTEND(cppcoreguidelines-pro-type-vararg,hicpp-vararg,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers,cppcoreguidelines-pro-bounds-pointer-arithmetic,cert-dcl50-cpp)
