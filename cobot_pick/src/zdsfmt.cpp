/*
zdsfmt.c �������C�u�����{��
coded by isaku@pb4.so-net.ne.jp
*/

#include <math.h>
#if _MSC_VER >= 1400
#include <intrin.h>
#endif
#include "cobot_pick/zdsfmt.h"

#ifndef MSEQ
#define MSEQ 1
#endif

struct _state_t DefaultMt[MSEQ];

volatile int32_t DefaultSeed=-1;

#define N128 ((int)((MEXP-128)/104)+1)
#define N64 (N128*2)
#define N32 (N128*4)
#define S32 (N32+4)
#define N16 (N128*8)
#define LAG (S32>=623?11:S32>=68?7:S32>=39?5:3)
#define MID ((S32-LAG)/2)
#define MLG (MID+LAG)

#if MEXP == 521L

#define POS1  3
#define SL1   25
#define MSK1  0x000fbfefff77efff
#define MSK2  0x000ffeebfbdfbfdf
#define MSK_0 0xff77efffLU
#define MSK_1 0x000fbfefLU
#define MSK_2 0xfbdfbfdfLU
#define MSK_3 0x000ffeebLU
#define FIX1  0xcfb393d661638469
#define FIX2  0xc166867883ae2adb
#define FIX_0 0x61638469LU
#define FIX_1 0xcfb393d6LU
#define FIX_2 0x83ae2adbLU
#define FIX_3 0xc1668678LU
#define PCV1  0xccaa588000000000
#define PCV2  0x0000000000000001
#define PCV_0 0x00000000LU
#define PCV_1 0xccaa5880LU
#define PCV_2 0x00000001LU
#define PCV_3 0x00000000LU
#define IDSTR "dSFMT2-521:3-25:fbfefff77efff-ffeebfbdfbfdf"

#elif MEXP == 1279L

#define POS1  9
#define SL1   19
#define MSK1  0x000efff7ffddffeeU
#define MSK2  0x000fbffffff77fffU
#define MSK_0 0xffddffeeLU
#define MSK_1 0x000efff7LU
#define MSK_2 0xfff77fffLU
#define MSK_3 0x000fbfffLU
#define FIX1  0xb66627623d1a31beU
#define FIX2  0x04b6c51147b6109bU
#define FIX_0 0x3d1a31beLU
#define FIX_1 0xb6662762LU
#define FIX_2 0x47b6109bLU
#define FIX_3 0x04b6c511LU
#define PCV1  0x7049f2da382a6aebU
#define PCV2  0xde4ca84a40000001U
#define PCV_0 0x382a6aebLU
#define PCV_1 0x7049f2daLU
#define PCV_2 0x40000001LU
#define PCV_3 0xde4ca84aLU
#define IDSTR "dSFMT2-1279:9-19:efff7ffddffee-fbffffff77fff"

#elif MEXP == 2203L

#define POS1  7
#define SL1   19
#define MSK1  0x000fdffff5edbfffU
#define MSK2  0x000f77fffffffbfeU
#define MSK_0 0xf5edbfffLU
#define MSK_1 0x000fdfffLU
#define MSK_2 0xfffffbfeLU
#define MSK_3 0x000f77ffLU
#define FIX1  0xb14e907a39338485U
#define FIX2  0xf98f0735c637ef90U
#define FIX_0 0x39338485LU
#define FIX_1 0xb14e907aLU
#define FIX_2 0xc637ef90LU
#define FIX_3 0xf98f0735LU
#define PCV1  0x8000000000000000U
#define PCV2  0x0000000000000001U
#define PCV_0 0x00000000LU
#define PCV_1 0x80000000LU
#define PCV_2 0x00000001LU
#define PCV_3 0x00000000LU
#define IDSTR "dSFMT2-2203:7-19:fdffff5edbfff-f77fffffffbfe"

#elif MEXP == 4253L

#define POS1  19
#define SL1   19
#define MSK1  0x0007b7fffef5feffU
#define MSK2  0x000ffdffeffefbfcU
#define MSK_0 0xfef5feffLU
#define MSK_1 0x0007b7ffLU
#define MSK_2 0xeffefbfcLU
#define MSK_3 0x000ffdffLU
#define FIX1  0x80901b5fd7a11c65U
#define FIX2  0x5a63ff0e7cb0ba74U
#define FIX_0 0xd7a11c65LU
#define FIX_1 0x80901b5fLU
#define FIX_2 0x7cb0ba74LU
#define FIX_3 0x5a63ff0eLU
#define PCV1  0x1ad277be12000000U
#define PCV2  0x0000000000000001U
#define PCV_0 0x12000000LU
#define PCV_1 0x1ad277beLU
#define PCV_2 0x00000001LU
#define PCV_3 0x00000000LU
#define IDSTR "dSFMT2-4253:19-19:7b7fffef5feff-ffdffeffefbfc"

#elif MEXP == 11213L

#define POS1  37
#define SL1   19
#define MSK1  0x000ffffffdf7fffdU
#define MSK2  0x000dfffffff6bfffU
#define MSK_0 0xfdf7fffdLU
#define MSK_1 0x000fffffLU
#define MSK_2 0xfff6bfffLU
#define MSK_3 0x000dffffLU
#define FIX1  0xd0ef7b7c75b06793U
#define FIX2  0x9c50ff4caae0a641U
#define FIX_0 0x75b06793LU
#define FIX_1 0xd0ef7b7cLU
#define FIX_2 0xaae0a641LU
#define FIX_3 0x9c50ff4cLU
#define PCV1  0x8234c51207c80000U
#define PCV2  0x0000000000000001U
#define PCV_0 0x07c80000LU
#define PCV_1 0x8234c512LU
#define PCV_2 0x00000001LU
#define PCV_3 0x00000000LU
#define IDSTR "dSFMT2-11213:37-19:ffffffdf7fffd-dfffffff6bfff"

#elif MEXP == 19937L

#define POS1  117
#define SL1   19
#define MSK1  0x000ffafffffffb3fU
#define MSK2  0x000ffdfffc90fffdU
#define MSK_0 0xfffffb3fLU
#define MSK_1 0x000ffaffLU
#define MSK_2 0xfc90fffdLU
#define MSK_3 0x000ffdffLU
#define FIX1  0x90014964b32f4329U
#define FIX2  0x3b8d12ac548a7c7aU
#define FIX_0 0xb32f4329LU
#define FIX_1 0x90014964LU
#define FIX_2 0x548a7c7aLU
#define FIX_3 0x3b8d12acLU
#define PCV1  0x3d84e1ac0dc82880U
#define PCV2  0x0000000000000001U
#define PCV_0 0x0dc82880LU
#define PCV_1 0x3d84e1acLU
#define PCV_2 0x00000001LU
#define PCV_3 0x00000000LU
#define IDSTR "dSFMT2-19937:117-19:ffafffffffb3f-ffdfffc90fffd"

#else

#error MEXP �̒l���s��

#endif

char*IdStringsTbl=(char*)IDSTR;

/*+--------------------------------+
  |SSE2�e�N�m���W�T�|�[�g�̃`�F�b�N|
  +--------------------------------+*/
#if defined(HAVE_SSE2)
#if defined(_MSC_VER)
#if _MSC_VER >= 1400
static int IsSSE2SFMT(void)
{
    int reg[4];
    __cpuid(reg,4);
    if (reg[3]&0x04000000) return 1;
    return 0;
}

#else

static int IsSSE2SFMT(void)
{
    _asm {
        push    ebx
        push    ecx
        push    edx
        pushfd
        pop     eax
        xor     eax, 00200000h
        push    eax
        popfd
        pushfd
        pop     edx
        cmp     eax, edx
        jnz     no_sse2
        mov     eax, 1
        cpuid
        and     edx, 04000000h
        jnz     yes_sse2
    no_sse2:
        xor     eax,eax
        jmp     exit_sse2
    yes_sse2:
        mov     eax,1
    exit_sse2:
        pop     edx
        pop     ecx
        pop     ebx
    }
}
#endif
#elif defined(__GNUC__)
int IsSSE2SFMT(void) {
    int a,b,c,d;
  __asm__ (
    "xchg{l}\t{%%}ebx, %1\n\t"
    "cpuid\n\t"
    "xchg{l}\t{%%}ebx, %1\n\t"
    : "=a" (a), "=r" (b), "=c" (c), "=d" (d)
    : "0" (1));
    if (d&0x04000000) return 1;
    return 0;
}
#else
static int IsSSE2SFMT(void) { return 1; }
#endif
#endif

static void zinitial_mask(state_p mt) {
    int i;
    for (i=BE1(1);i<N32;i+=2)
        mt->u.x[i]=(mt->u.x[i]&0xFFFFFUL)|0x3FF00000UL;
}

/*+------------------+
�@|�������̉������֐�|
�@+------------------+*/
static void zperiod_certification(state_p mt) {
    uint32_t pcv[]={PCV_0,PCV_1,PCV_2,PCV_3};
    uint32_t tmp[4],inner0,inner1; int i;
#if (PCV_2 & 1) != 1
    uint32_t work0,work1,j;
#endif

    mt->index=N64; mt->normal_sw=0; mt->coin_bits=0; mt->byte_pos=0;
#ifdef HAVE_SSE2
    mt->flag=IsSSE2SFMT();
#else
    mt->flag=0;
#endif
    tmp[0] =(mt->u.x[BE1(N32+0)]^FIX_0);
    tmp[1] =(mt->u.x[BE1(N32+1)]^FIX_1);
    tmp[2] =(mt->u.x[BE1(N32+2)]^FIX_2);
    tmp[3] =(mt->u.x[BE1(N32+3)]^FIX_3);
    inner0 =tmp[0]&pcv[0];
    inner1 =tmp[1]&pcv[1]; 
    inner0^=tmp[2]&pcv[2];
    inner1^=tmp[3]&pcv[3];
    inner0^=inner1;
    for (i=16;i>0;i>>=1)
    { inner0^=(inner0>>i)|(inner1<<(32-i)); inner1^=inner1>>i; }
    inner0 &= 1;
    if (inner0==1) return;
    /* check NG, and modification */
#if (PCV_2 & 1) == 1
    mt->u.x[BE1(N32+2)] ^= 1;
#else
    for (i=1;i>=0;i--) {
        work0=1; work1=0;
        for (j=0;j<64;j++) {
            if ((work0&pcv[i*2])!=0||(work1&pcv[i*2+1])) {
                mt->u.x[BE1(N32+i*2)]^=work0;
                mt->u.x[BE1(N32+i*2+1)]^=work1;
                return;
            }
            work1=(work1<<1)|(work0>>31); work0<<=1;
        }
    }
#endif
}
/*+------------+
�@|�̈�����|
�@+------------+*/
void InitMt_r(state_p mt,uint32_t s) {
    unsigned i;
    for (mt->u.x[BE1(0)]=s,i=1;i<N32+4;i++)
        mt->u.x[BE1(i)]=s=1812433253UL*(s^(s>>30))+i;
    zinitial_mask(mt);
    zperiod_certification(mt);
}

/*+------------------+
�@|�z���g���ď�����|
�@+------------------+*/
#define BX1(X) x[BE1(X)]
void InitMtEx_r(state_p mt,uint32_t*init_key,unsigned key_len)
{
    uint32_t r,*x=mt->u.x; unsigned i,j,c;

    for (i=0;i<S32;i++) BX1(i)=0x8b8b8b8bUL;
    if (key_len+1>S32) c=key_len+1; else c=S32;
    r=BX1(0)^BX1(MID)^BX1(S32-1); r=(r^(r>>27))*1664525UL;
    BX1(MID)+=r; r+=key_len; BX1(MLG)+=r; BX1(0)=r; c--;
    for (i=1,j=0;j<c&&j<key_len;j++) {
        r=BX1(i)^BX1((i+MID)%S32)^BX1((i+S32-1)%S32);
        r=(r^(r>>27))*1664525UL; BX1((i+MID)%S32)+=r;
        r+=init_key[j]+i; BX1((i+MLG)%S32)+=r;
        BX1(i)=r; i=(i+1)%S32;
    }
    for (;j<c;j++) {
        r=BX1(i)^BX1((i+MID)%S32)^BX1((i+S32-1)%S32);
        r=(r^(r>>27))*1664525UL; BX1((i+MID)%S32)+=r;
        r+=i; BX1((i+MLG)%S32)+=r; BX1(i)=r; i=(i+1)%S32;
    }
    for (j=0;j<S32;j++) {
        r=BX1(i)+BX1((i+MID)%S32)+BX1((i+S32-1)%S32);
        r=(r^(r>>27))*1566083941UL; BX1((i+MID)%S32)^=r;
        r-=i; BX1((i+MLG)%S32)^=r; BX1(i)=r; i=(i+1)%S32;
    }
    zinitial_mask(mt); zperiod_certification(mt);
}

/*+--------------+
�@|�R���X�g���N�^|
�@+--------------+*/
#if defined(__cplusplus) && !defined(__GNUC__)
_state_t::_state_t()
{
    if (this==DefaultMt) InitMt_r(this,1);
    else InitMt_r(this,DefaultSeed--);
}
#endif

#if INT_MAX <= 32767

#if SL1 <= 16 || SL1 >=32
#error '16 < SL1 < 32' �̏ꍇ�̂ݑΉ����Ă��܂�
#endif

#define MS0 ((unsigned short)(MSK_0&0xffffU))
#define MS1 ((unsigned short)(MSK_0>>16))
#define MS2 ((unsigned short)(MSK_1&0xffffU))
#define MS3 ((unsigned short)(MSK_1>>16))
#define MS4 ((unsigned short)(MSK_2&0xffffU))
#define MS5 ((unsigned short)(MSK_2>>16))
#define MS6 ((unsigned short)(MSK_3&0xffffU))
#define MS7 ((unsigned short)(MSK_3>>16))

#define GEN_FOUR()                                             \
  t0=a[BE3(0)]; t1=a[BE3(1)]; t2=a[BE3(2)]; t3=a[BE3(3)];      \
  t4=a[BE3(4)]; t5=a[BE3(5)]; t6=a[BE3(6)]; t7=a[BE3(7)];      \
  L0=L[BE3(0)]; L1=L[BE3(1)]; L2=L[BE3(2)]; L3=L[BE3(3)];      \
  L4=L[BE3(4)]; L5=L[BE3(5)]; L6=L[BE3(6)]; L7=L[BE3(7)];      \
  L[BE3(0)]=                              L6^b[BE3(0)];        \
  L[BE3(1)]=(t0<<(SL1-16))               ^L7^b[BE3(1)];        \
  L[BE3(2)]=(t1<<(SL1-16))^(t0>>(32-SL1))^L4^b[BE3(2)];        \
  L[BE3(3)]=(t2<<(SL1-16))^(t1>>(32-SL1))^L5^b[BE3(3)];        \
  L[BE3(4)]=                              L2^b[BE3(4)];        \
  L[BE3(5)]=(t4<<(SL1-16))               ^L3^b[BE3(5)];        \
  L[BE3(6)]=(t5<<(SL1-16))^(t4>>(32-SL1))^L0^b[BE3(6)];        \
  L[BE3(7)]=(t6<<(SL1-16))^(t5>>(32-SL1))^L1^b[BE3(7)];        \
  a[BE3(0)]=(L[BE3(0)]>>12)^(L[BE3(1)]<<4)^(L[BE3(0)]&MS0)^t0; \
  a[BE3(1)]=(L[BE3(1)]>>12)^(L[BE3(2)]<<4)^(L[BE3(1)]&MS1)^t1; \
  a[BE3(2)]=(L[BE3(2)]>>12)^(L[BE3(3)]<<4)^(L[BE3(2)]&MS2)^t2; \
  a[BE3(3)]=(L[BE3(3)]>>12)               ^(L[BE3(3)]&MS3)^t3; \
  a[BE3(4)]=(L[BE3(4)]>>12)^(L[BE3(5)]<<4)^(L[BE3(4)]&MS4)^t4; \
  a[BE3(5)]=(L[BE3(5)]>>12)^(L[BE3(6)]<<4)^(L[BE3(5)]&MS5)^t5; \
  a[BE3(6)]=(L[BE3(6)]>>12)^(L[BE3(7)]<<4)^(L[BE3(6)]&MS6)^t6; \
  a[BE3(7)]=(L[BE3(7)]>>12)               ^(L[BE3(7)]&MS7)^t7

#ifdef STABLE_SPEED

void GenNormal(state_p mt) {
    int i=mt->index*4;
    unsigned short*a=((unsigned short*)mt->u.x)+i;
    unsigned short t0,t1,t2,t3,t4,t5,t6,t7,L0,L1,L2,L3,L4,L5,L6,L7;
    unsigned short*b=i<N16-POS1*8?a+POS1*8:a+POS1*8-N16;
    unsigned short*L=((unsigned short*)mt->u.x)+N16;
    GEN_FOUR();
}

#else

void GenNormal(state_p mt)
{
    unsigned short*a=(unsigned short*)mt->u.x,*b=a+POS1*8,*L=a+N16;
    unsigned short t0,t1,t2,t3,t4,t5,t6,t7,L0,L1,L2,L3,L4,L5,L6,L7;
    do {
        GEN_FOUR(); a+=8; b+=8;
        if (b==L) b-=N16;
    } while (a!=L);
}

#endif
#elif defined(WORD64)

/*+--------------------------------+
�@|��{�֐��̉������֐�(64�r�b�g��)|
�@+--------------------------------+*/
#define GEN_FOUR()                                     \
  t0=a[0]; t1=a[1];                                    \
  L0=(((uint64_t)LL[BE1(2)])<<32)|LL[BE1(3)];          \
  L1=(((uint64_t)LL[BE1(0)])<<32)|LL[BE1(1)];          \
  L[0]=L0=(t0<<SL1)^L0^b[0];                           \
  L[1]=L1=(t1<<SL1)^L1^b[1];                           \
  a[0]=(L0>>12)^(L0&(((uint64_t)MSK_1)<<32|MSK_0))^t0; \
  a[1]=(L1>>12)^(L1&(((uint64_t)MSK_3)<<32|MSK_2))^t1

#ifdef STABLE_SPEED

void GenNormal(state_p mt) {
    int i=mt->index;
    uint64_t*a=mt->u.y+i,*L=mt->u.y+N32/2,t0,t1,L0,L1;
    uint64_t*b=i<N32/2-POS1*2?a+POS1*2:a+POS1*2-N32/2;
    uint32_t*LL=(uint32_t*)L;
    GEN_FOUR();
}

#else

void GenNormal(state_p mt)
{
    uint64_t*a=mt->u.y,*b=a+POS1*2,*L=a+N32/2,t0,t1,L0,L1;
    uint32_t*LL=(uint32_t*)L;
    do {
        GEN_FOUR(); a+=2; b+=2;
        if (b==L) b-=N32/2;
    } while (a!=L);
}

#endif

#else

/*+--------------------------------+
�@|��{�֐��̉������֐�(32�r�b�g��)|
�@+--------------------------------+*/
#if SL1 <= 0 || SL1 >=32
#error '0 < SL1 < 32' �̏ꍇ�̂ݑΉ����Ă��܂�
#endif

#define GEN_FOUR()                                                \
  t0=a[BE1(0)]; t1=a[BE1(1)]; t2=a[BE1(2)]; t3=a[BE1(3)];         \
  L0=L[BE1(0)]; L1=L[BE1(1)]; L2=L[BE1(2)]; L3=L[BE1(3)];         \
  L[BE1(0)]=(t0<<SL1)               ^L3^b[BE1(0)];                \
  L[BE1(1)]=(t1<<SL1)^(t0>>(32-SL1))^L2^b[BE1(1)];                \
  L[BE1(2)]=(t2<<SL1)               ^L1^b[BE1(2)];                \
  L[BE1(3)]=(t3<<SL1)^(t2>>(32-SL1))^L0^b[BE1(3)];                \
  a[BE1(0)]=(L[BE1(0)]>>12)^(L[BE1(1)]<<20)^(L[BE1(0)]&MSK_0)^t0; \
  a[BE1(1)]=(L[BE1(1)]>>12)                ^(L[BE1(1)]&MSK_1)^t1; \
  a[BE1(2)]=(L[BE1(2)]>>12)^(L[BE1(3)]<<20)^(L[BE1(2)]&MSK_2)^t2; \
  a[BE1(3)]=(L[BE1(3)]>>12)                ^(L[BE1(3)]&MSK_3)^t3

#ifdef STABLE_SPEED

void GenNormal(state_p mt) {
    int i=mt->index*2;
    uint32_t*a=mt->u.x+i,t0,t1,t2,t3,L0,L1,L2,L3;
    uint32_t*b=i<N32-POS1*4?a+POS1*4:a+POS1*4-N32;
    uint32_t*L=mt->u.x+N32;
    GEN_FOUR();
}

#else

void GenNormal(state_p mt)
{
    uint32_t*a=mt->u.x,*b=a+POS1*4,*L=a+N32;
    uint32_t t0,t1,t2,t3,L0,L1,L2,L3;
    do {
        GEN_FOUR(); a+=4; b+=4;
        if (b==L) b-=N32;
    } while (a!=L);
}

#endif
#endif

/*+----------------------------+
�@|��{�֐��̉������֐�(SSE2��)|
�@+----------------------------+*/
#ifdef HAVE_SSE2

#ifdef STABLE_SPEED

void GenSSE2(state_p mt)
{
    static union { unsigned x[4]; __m128i m; } u =
    { {MSK_0,MSK_1,MSK_2,MSK_3} };
    int i=mt->index>>1; __m128i*t=mt->u.m,v,w,x,y,z;

    x=t[i]; z=_mm_slli_epi64(x,SL1);
    y=_mm_shuffle_epi32(t[N128],0x1b);
    z=_mm_xor_si128(z,i<N128-POS1?t[i+POS1]:t[i+POS1-N128]);
    y=_mm_xor_si128(y,z);
    v=_mm_srli_epi64(y,12);
    w=_mm_and_si128(y,u.m);
    v=_mm_xor_si128(v,x);
    v=_mm_xor_si128(v,w);
    t[i]=v; t[N128]=y;
}

#else

#define GEN_RECUR(P)                 \
    x=t[i]; z=_mm_slli_epi64(x,SL1); \
    y=_mm_shuffle_epi32(y,0x1b);     \
    z=_mm_xor_si128(z,t[i+(P)]);     \
    y=_mm_xor_si128(y,z);            \
    v=_mm_srli_epi64(y,12);          \
    w=_mm_and_si128(y,u.m);          \
    v=_mm_xor_si128(v,x);            \
    v=_mm_xor_si128(v,w); t[i]=v

void GenSSE2(state_p mt)
{
    static union { unsigned x[4]; __m128i m; } u =
    { {MSK_0,MSK_1,MSK_2,MSK_3} };
    int i; __m128i*t=mt->u.m,v,w,x,y,z;

    y=t[N128];
    for (i=0;i<N128-POS1;i++) { GEN_RECUR(POS1); }
    for (;i<N128;i++) { GEN_RECUR(POS1-N128); }
    t[N128]=y;
}

#endif

#endif

/*+------------------------------------------------------+
�@|�ۂߌ덷�̂Ȃ��O�ȏ�range�����̐��������@�@�@�@�@�@�@ |
�@+------------------------------------------------------+*/
int32_t NextIntEx_r(state_p mt,int32_t range)
{
    uint32_t y,base,remain; int shift;

    if (range<=0) return 0;
    if (range!=mt->range) {
        mt->range=base=range;
        for (shift=0;base<=(1UL<<30);shift++) base<<=1;
        mt->base=base; mt->shift=shift;
    }
    for (;;) {
        y=NextMt_r(mt)>>1;
        if (y<mt->base) return(int32_t)(y>>mt->shift);
        base=mt->base; shift=mt->shift; y-=base;
        remain=(1UL<<31)-base;
        for (;remain>=(uint32_t)range;remain-=base) {
            for (;base>remain;base>>=1) shift--;
            if (y<base) return(int32_t)(y>>shift); else y-=base;
        }
    }
}

/*+----------------------------+
�@|���R�x�˂̃J�C�Q�敪�z p.27 |
�@+----------------------------+*/
double NextChisq_r(state_p mt,double n)
{
    return 2*NextGamma_r(mt,0.5*n);
}

/*+------------------------------+
�@|�p�����[�^���̃K���}���z p.31 |
�@+------------------------------+*/
double NextGamma_r(state_p mt,double a)
{
    double t,u,x,y;
    if (a>1) {
        t=sqrt(2*a-1);
        do {
            do {
                do {
                    x=1-NextUnif_r(mt);
                    y=2*NextUnif_r(mt)-1;
                } while (x*x+y*y>1);
                y/=x; x=t*y+a-1;
            } while (x<=0);
            u=(a-1)*log(x/(a-1))-t*y;
        } while (u<-50||NextUnif_r(mt)>(1+y*y)*exp(u));
    } else {
        t=2.718281828459045235/(a+2.718281828459045235);
        do {
            if (NextUnif_r(mt)<t) {
                x=pow(NextUnif_r(mt),1/a); y=exp(-x);
            } else {
                x=1-log(1-NextUnif_r(mt)); y=pow(x,a-1);
            }
        } while (NextUnif_r(mt)>=y);
    }
    return x;
}

/*+----------------------+
�@|�m���o�̊􉽕��z p.34 |
�@+----------------------+*/
int NextGeometric_r(state_p mt,double p)
{ return(int)ceil(log(1.0-NextUnif_r(mt))/log(1-p)); }

/*+--------------+
�@|�O�p���z p.89 |
�@+--------------+*/
double NextTriangle_r(state_p mt)
{ double a=NextUnif_r(mt),b=NextUnif_r(mt); return a-b; }

/*+----------------------+
�@|���ςP�̎w�����z p.106|
�@+----------------------+*/
double NextExp_r(state_p mt)
{ return-log(1-NextUnif_r(mt)); }

/*+----------------------------------+
�@|�W�����K���z(�ő�6.660437��) p.133|
�@+----------------------------------+*/
double NextNormal_r(state_p mt)
{
    if (mt->normal_sw==0) {
        double t=sqrt(-2*log(1.0-NextUnif_r(mt)));
        double u=3.141592653589793*2*NextUnif_r(mt);
        mt->normal_save=t*sin(u); mt->normal_sw=1; return t*cos(u);
    }else{ mt->normal_sw=0; return mt->normal_save; }
}

/*+----------------------------------+
�@|�m�����̃����_���P�ʃx�N�g�� p.185|
�@+----------------------------------+*/
void NextUnitVect_r(state_p mt,double*v,int n)
{
    int i; double r=0;
    for (i=0;i<n;i++) { v[i]=NextNormal_r(mt); r+=v[i]*v[i]; }
    if (r==0.0) r=1.0;
    r=sqrt(r);
    for (i=0;i<n;i++) v[i]/=r;
}

/*+--------------------------------+
�@|�p�����[�^�m,�o�̂Q�����z p.203 |
�@+--------------------------------+*/
int NextBinomial_r(state_p mt,int n,double p)
{
    int i,r=0;
    for (i=0;i<n;i++) if (NextUnif_r(mt)<p) r++;
    return r;
}

/*+--------------------------------+
�@|���֌W���q�̂Q�ϗʐ��K���z p.211|
�@+--------------------------------+*/
void NextBinormal_r(state_p mt,double r,double*x,double*y)
{
    double r1,r2,s;
    do {
        r1=2*NextUnif_r(mt)-1;
        r2=2*NextUnif_r(mt)-1;
        s=r1*r1+r2*r2;
    } while (s>1||s==0);
    s= -log(s)/s; r1=sqrt((1+r)*s)*r1;
    r2=sqrt((1-r)*s)*r2; *x=r1+r2; *y=r1-r2;
}

/*+----------------------------------+
�@|�p�����[�^�`,�a�̃x�[�^���z p.257 |
�@+----------------------------------+*/
double NextBeta_r(state_p mt,double a,double b)
{
    double temp=NextGamma_r(mt,a);
    return temp/(temp+NextGamma_r(mt,b));
}

/*+----------------------------+
�@|�p�����[�^�m�̗ݏ敪�z p.305|
�@+----------------------------+*/
double NextPower_r(state_p mt,double n)
{ return pow(NextUnif_r(mt),1.0/(n+1)); }

/*+------------------------+
�@|���W�X�e�B�b�N���z p.313|
�@+------------------------+*/
double NextLogistic_r(state_p mt)
{
    double r;
    do r=NextUnif_r(mt); while (r==0);
    return log(r/(1-r));
}

/*+------------------+
�@|�R�[�V�[���z p.331|
�@+------------------+*/
double NextCauchy_r(state_p mt)
{
    double x,y;
    do { x=1-NextUnif_r(mt); y=2*NextUnif_r(mt)-1; }
    while (x*x+y*y>1);
    return y/x;
}

/*+--------------------------+
�@|���R�x�`,�a�̂e���z p.344 |
�@+--------------------------+*/
double NextFDist_r(state_p mt,double n1,double n2)
{
    double nc1=NextChisq_r(mt,n1),nc2=NextChisq_r(mt,n2);
    return (nc1*n2)/(nc2*n1);
}

/*+--------------------------+
�@|���σɂ̃|�A�\�����z p.412|
�@+--------------------------+*/
int NextPoisson_r(state_p mt,double lambda)
{
    int k; lambda=exp(lambda)*NextUnif_r(mt);
    for (k=0;lambda>1;k++) lambda*=NextUnif_r(mt);
    return k;
}

/*+----------------------+
�@|���R�x�m�̂����z p.428|
�@+----------------------+*/
double NextTDist_r(state_p mt,double n)
{
    double a,b,c;
    if (n<=2) {
        do a=NextChisq_r(mt,n); while (a==0);
        return NextNormal_r(mt)/sqrt(a/n);
    }
    do {
        a=NextNormal_r(mt); b=a*a/(n-2);
        c=log(1-NextUnif_r(mt))/(1-0.5*n);
    } while (exp(-b-c)>1-b);
    return a/sqrt((1-2.0/n)*(1-b));
}

/*+--------------------------------+
�@|�p�����[�^���̃��C�u�����z p.431|
�@+--------------------------------+*/
double NextWeibull_r(state_p mt,double alpha)
{ return pow(-log(1-NextUnif_r(mt)),1/alpha); }

