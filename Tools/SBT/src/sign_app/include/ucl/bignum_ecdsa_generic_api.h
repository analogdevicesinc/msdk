#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */
//#include <stdio.h>
//#include <stdlib.h>
#include "ucl/ucl_config.h"
#include "ucl/ucl_types.h"

#ifdef WORD32
/*#define DOUBLE_DIGIT unsigned long int
#define DIGIT unsigned short int
#define MAX_DIGIT 0xFFFF
#define HALF_DIGIT 0xFF
#define MAX_DIGITS 20
#define DIGIT_BITS 16*/
#define DOUBLE_DIGIT unsigned long long int
#define DIGIT u32
#define MAX_DIGIT 0xFFFFFFFF
#define HALF_DIGIT 0xFFFF
//move from 16 to 17 then 33 for dsa use of bignum_mod
#define MAX_DIGITS (UCL_RSA_KEY_MAXSIZE/4+1)
#define DIGIT_BITS 32
#endif//WORD32
#ifdef WORD16
#define DOUBLE_DIGIT unsigned long int
#define DIGIT unsigned short int
#define MAX_DIGIT 0xFFFF
#define HALF_DIGIT 0xFF
#define MAX_DIGITS 16
#define DIGIT_BITS 16
#endif//WORD16

/* Length of digit in bytes */
#define DIGIT_LEN (DIGIT_BITS / 8)
/* int min(int a, int b); */
  void bignum_mod_same_size(DIGIT *b,DIGIT *c,DIGIT *d,DIGIT dDigits);
int bignum_modsquare(DIGIT *r,DIGIT *a,DIGIT *m,DIGIT k);
int bignum_modmult(DIGIT *r,DIGIT *a,DIGIT *b,DIGIT *m,DIGIT k);
void bignum_d2us(u8 *a,DIGIT  len,DIGIT *b,DIGIT digits);
void bignum_us2d(DIGIT *a,DIGIT digits,u8 *b,DIGIT len);
void bignum_truncate(u32 *a,u32 bitsize,u32 wordsize);
int bignum_cmp(DIGIT *a,DIGIT *b,DIGIT s);
int bignum_cmp_zero(DIGIT *a,DIGIT s);
int bignum_cmp_zero_slow(DIGIT *a,DIGIT s);
int bignum_cmp_slow(DIGIT *a,DIGIT *b,DIGIT s);
int bignum_isnul(DIGIT *A,DIGIT tA);
int bignum_isnul_slow(DIGIT *A,DIGIT tA);
DIGIT bignum_digits(DIGIT *N,DIGIT tn);
DIGIT bignum_digitbits(DIGIT a);
void bignum_copydigit(DIGIT *E,DIGIT F,DIGIT tE);
void bignum_copyzero(DIGIT *E,DIGIT tE);
void bignum_copy(DIGIT *E,DIGIT *F,DIGIT tE);

DIGIT bignum_add(DIGIT *w,DIGIT *x,DIGIT *y,DIGIT digits);
DIGIT bignum_sub(DIGIT *w,DIGIT *x,DIGIT *y,DIGIT digits);
void bignum_mult(DIGIT *t,DIGIT *a,DIGIT *b,DIGIT n);
void bignum_mult_operand_scanning_form(DIGIT *t,DIGIT *a,DIGIT *b,DIGIT n);
void bignum_multopt(DIGIT *t,DIGIT *a,DIGIT *b,DIGIT n);
void bignum_multlimited(DIGIT *t,DIGIT *a,DIGIT *b,DIGIT n);
void bignum_multscalar(DIGIT *t,DIGIT a,DIGIT *b,DIGIT n);
void bignum_mult2sizes(DIGIT *t,DIGIT *a,DIGIT na,DIGIT *b,DIGIT nb);
void bignum_square(DIGIT *a,DIGIT *b,DIGIT digits);
DIGIT bignum_leftshift(DIGIT *a,DIGIT *b,DIGIT c,DIGIT digits);
DIGIT bignum_rightshift(DIGIT *a,DIGIT *b,DIGIT c,DIGIT digits);

/** <b>big numbers inverse computation</b>.
 * compute x such as x.a0=1 modulo b0, a0, b0 being digits-words numbers
 * using a software implementation
 * @param[out]  x   The pointer to the inverse
 * @param[int]  a0  The pointer to the number to be inverted
 * @param[in]   b0   The pointer to the modulus
 * @param[in]   digits   The precision, i.e. the size in words of a0 and b0
 *
 * @return no returned value
 *
 */

void bignum_modinv(DIGIT *x,DIGIT *a0,DIGIT *b0,DIGIT digits);

/** <b>big numbers inverse computation using the MAA</b>.
 * compute x such as x.a0=1 modulo b0, a0, b0 being digits-words numbers
 * using the MAA exponentiation
 * in fact, it uses the fermat theorem which says that a^(n-1)=1 mod n
 * it implies a.a^(n-2)=1 mod n, which means that the inverse of a is a^(n-2)
 * because of the MAA, computing a^(n-2) is more efficient than a software inversion
 * 
 * @param[out]  x   The pointer to the inverse
 * @param[int]  a0  The pointer to the number to be inverted
 * @param[in]   b0   The pointer to the modulus, a prime number
 * @param[in]   digits   The precision, i.e. the size in words of a0 and b0
 *
 * @return no returned value
 *
 * note: the modulus_set variable (extern int modulus_set) has to be cleared first (modulus_set=0)
 */
void bignum_modinv_subs(DIGIT *x,DIGIT *a0,DIGIT *b0,DIGIT digits);



/** <b>big numbers modular addition</b>.
 * compute r such as r=a+b modulo n, a,b and n being digits-words numbers
 * using the MAA
 *
 * @param[out]  r   The pointer to the inverse
 * @param[int]  a  The pointer to the number to be inverted
 * @param[in]   b   The pointer to the modulus, a prime number
 * @param[in]   digits   The precision, i.e. the size in words of a0 and b0
 *
 * @return no returned value
 *
 */
int bignum_modadd(DIGIT *r,DIGIT *a,DIGIT *b,DIGIT *m,DIGIT k);
void bignum_modinv3(DIGIT *x,DIGIT *a0,DIGIT *b0,DIGIT digits);
void bignum_modinv4(DIGIT *x,DIGIT *a0,DIGIT *b0,DIGIT digits);
void bignum_modexp(DIGIT *r,DIGIT *a,DIGIT *b,DIGIT *m,DIGIT k);
void bignum_mod(DIGIT *b,DIGIT *c,DIGIT cDigits,DIGIT *d,DIGIT dDigits);
void bignum_div(DIGIT *rmd,DIGIT *b,DIGIT *c,DIGIT cDigits,DIGIT *d,DIGIT dDigits);
void bignum_modmult_maa(u32 segR, u32 segA,u32 segB,DIGIT k);
void bignum_modsquare_maa(u32 segR, u32 segB,DIGIT k);
void bignum_modsquaremult_maa(u32 segR, u32 segA,u32 segB,DIGIT k);
void bignum_modadd_maa(u32 segR, u32 segA,u32 segB,DIGIT k);
int bignum_modsub_maa(DIGIT *r,DIGIT *a,DIGIT *b,DIGIT *m,DIGIT k);
void load_maa(u32 seg,DIGIT *a,DIGIT k);
void load_modulus_maa(DIGIT *m,DIGIT k);
void unload_maa(u32 seg,DIGIT *r,DIGIT k);

void my_memcpy_maa(DIGIT *dst, const DIGIT *src, int n);
void my_memcpy_maa1(DIGIT *dst, const DIGIT *src, int n);
void my_memcpy_maa4(DIGIT *dst, const DIGIT *src, int n);
void my_memset_maa4(DIGIT *dst, const DIGIT *src, int n);
void my_memset_maa(DIGIT *dst,DIGIT value, int n);

unsigned long fastmul16(unsigned short a, unsigned short b);
unsigned short fastshr16(unsigned long);
#ifdef __cplusplus
}
#endif /* __cplusplus  */

