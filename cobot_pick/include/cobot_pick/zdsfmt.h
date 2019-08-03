

#ifndef __zdsfmt_h
#define __zdsfmt_h

#if defined(WORD64) && defined(BIG_E)
#error WORD64 と BIG_E は同時に指定できません。
#endif

#if defined(_MSC_VER) && _MSC_VER <= 1200
#pragma warning(disable : 4710)
#pragma warning(disable : 4514)
#endif

#include <limits.h>
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
#  include <inttypes.h>
#elif INT_MAX == 32767
#  define uint32_t unsigned long
#  define int32_t  long
#else
#  define uint32_t unsigned
#  define int32_t  int
#if defined(__GNUC__)
#  define uint64_t unsigned long long
#  define int64_t long long
#else
#  define uint64_t unsigned __int64
#  define int64_t __int64
#endif
#endif

#ifdef HAVE_SSE2
#include <emmintrin.h>
#endif

#ifndef MEXP
#define MEXP 19937L
#endif

#ifdef __GNUC__
#define __inline __inline__
#endif

#if INT_MAX == 32767
#define __inline
#endif

#ifdef BIG_E
#define BE1(X) ((X)^1)
#define BE3(X) ((X)^3)
#else
#define BE1(X) X
#define BE3(X) X
#endif

struct _state_t {
    unsigned index;       /* インデックス */
    int      flag;        /* SSE2 が使用可能であれば 1 */
    int      coin_bits;   /* NextBit での残りのビット */
#if INT_MAX == 32767
#if defined(BIG_E)
    union {
        uint32_t z; 
        struct { short hi,lo; } s;
    } c;
#else
    union {
        uint32_t z; 
        struct { short lo,hi; } s;
    } c;
#endif
#else
    uint32_t coin_save;     /* NextBit での値保持 */
#endif
    int      byte_pos;      /* NextByte で使用したバイト数 */
#if INT_MAX == 32767
#if defined(BIG_E)
    union {
        uint32_t z;
        struct { unsigned char u3,u2,u1,u0; } s;
    } b;                    /* NextByte での値保持 */
#else
    union {
        uint32_t z;
        struct { unsigned char u0,u1,u2,u3; } s;
    } b;                    /* NextByte での値保持 */
#endif
#else
    uint32_t byte_save;     /* NextByte での値保持 */
#endif
    int32_t  range;       /* NextIntEx で前回の範囲 */
    uint32_t base;        /* NextIntEx で前回の基準値 */
    int      shift;       /* NextIntEx で前回のシフト数 */
    int      normal_sw;   /* NextNormal で残りを持っている */
    double   normal_save; /* NextNormal の残りの値 */
    union {
#ifdef HAVE_SSE2
        __m128i  m[(int)((MEXP-128)/104+2)];
#endif
#ifdef WORD64
        uint64_t y[(int)((MEXP-128)/104+2)*2];
#endif
        uint32_t x[(int)((MEXP-128)/104+2)*4];
        double   d[(int)((MEXP-128)/104+2)*2];
    } u;
#ifdef __cplusplus
#ifdef __GNUC__
    _state_t() { index=0; }
#else
    _state_t();
#endif
#endif
};

typedef struct _state_t state_t[1];
#ifdef __cplusplus
#define DeclMt(N) state_t N
#else
#define DeclMt(N) state_t N={{0}}
#endif

typedef struct _state_t*state_p;
extern struct _state_t DefaultMt[];
extern volatile int32_t DefaultSeed;
extern char*IdStringsTbl;
extern void     GenNormal    (state_p);
#ifdef HAVE_SSE2
extern void     GenSSE2      (state_p);
#endif
extern void     InitMt_r     (state_p,uint32_t);
extern void     InitMtEx_r   (state_p,uint32_t*,unsigned);
extern int32_t  NextIntEx_r  (state_p,int32_t);
extern double NextChisq_r    (state_p,double);
extern double NextGamma_r    (state_p,double);
extern int    NextGeometric_r(state_p,double);
extern double NextTriangle_r (state_p);
extern double NextExp_r      (state_p);
extern double NextNormal_r   (state_p);
extern void   NextUnitVect_r (state_p,double*,int);
extern int    NextBinomial_r (state_p,int,double);
extern void   NextBinormal_r (state_p,double,double*,double*);
extern double NextBeta_r     (state_p,double,double);
extern double NextPower_r    (state_p,double);
extern double NextLogistic_r (state_p);
extern double NextCauchy_r   (state_p);
extern double NextFDist_r    (state_p,double,double);
extern int    NextPoisson_r  (state_p,double);
extern double NextTDist_r    (state_p,double);
extern double NextWeibull_r  (state_p,double);

#define IdString_m(M)  IdStringsTbl
#define IdString       IdString_m(0)
#define NextInt_r(M,N) ((int32_t)((N)*NextUnif_r(M)))

#define InitMt(S)      InitMt_r    (DefaultMt,S)
#define InitMtEx(K,L)  InitMtEx_r  (DefaultMt,K,L)
#define NextInt(N)     NextInt_r   (DefaultMt,N)
#define NextIntEx(N)   NextIntEx_r (DefaultMt,N)

#if INT_MAX == 32767
#define NextBit_r(M) ((--(M)->coin_bits==-1? \
 ((M)->coin_bits=31,((M)->c.z=NextMt_r(M))): \
 (M)->coin_bits==15?((M)->c.s.lo=(M)->c.s.hi): \
 ((M)->c.s.lo>>=1))&1)
#define NextBit() ((--DefaultMt->coin_bits==-1? \
 (DefaultMt->coin_bits=31,(DefaultMt->c.z=NextMt())): \
 DefaultMt->coin_bits==15?(DefaultMt->c.s.lo=DefaultMt->c.s.hi): \
 (DefaultMt->c.s.lo>>=1))&1)
#else
#define NextBit_r(M) ((--(M)->coin_bits==-1?                      \
    (int)((M)->coin_bits=31,(M)->coin_save=NextMt_r(M)):          \
    (int)((M)->coin_save>>=1))&1)
#define NextBit()   ((--DefaultMt->coin_bits==-1?                 \
    (int)(DefaultMt->coin_bits=31,DefaultMt->coin_save=NextMt()): \
    (int)(DefaultMt->coin_save>>=1))&1)
#endif

#if INT_MAX == 32767

#define NextByte_r(M) (--(M)->byte_pos==-1?          \
 ((M)->byte_pos=3,(int)((M)->b.z=NextMt_r(M))&255):  \
 (M)->byte_pos==2?(int)(M)->b.s.u1:(M)->byte_pos==1? \
 (int)(M)->b.s.u2:(int)(M)->b.s.u3)
#define NextByte() (--DefaultMt->byte_pos==-1?               \
 (DefaultMt->byte_pos=3,(int)(DefaultMt->b.z=NextMt())&255): \
 DefaultMt->byte_pos==2?(int)DefaultMt->b.s.u1:              \
 DefaultMt->byte_pos==1?(int)DefaultMt->b.s.u2:              \
 (int)DefaultMt->b.s.u3)

#else

#define NextByte_r(M) ((--(M)->byte_pos==-1?        \
 (int)((M)->byte_pos=3,(M)->byte_save=NextMt_r(M)): \
 (int)((M)->byte_save>>=8))&255)
#define NextByte()   ((--DefaultMt->byte_pos==-1?            \
 (int)(DefaultMt->byte_pos=3,DefaultMt->byte_save=NextMt()): \
 (int)(DefaultMt->byte_save>>=8))&255)

#endif

#define NextChisq(N)        NextChisq_r    (DefaultMt,N)
#define NextGamma(A)        NextGamma_r    (DefaultMt,A)
#define NextGeometric(P)    NextGeometric_r(DefaultMt,P)
#define NextTriangle()      NextTriangle_r (DefaultMt)
#define NextExp()           NextExp_r      (DefaultMt)
#define NextNormal()        NextNormal_r   (DefaultMt)
#define NextUnitVect(V,N)   NextUnitVect_r (DefaultMt,V,N)
#define NextBinomial(N,P)   NextBinomial_r (DefaultMt,N,P)
#define NextBinormal(R,X,Y) NextBinormal_r (DefaultMt,R,X,Y)
#define NextBeta(A,B)       NextBeta_r     (DefaultMt,A,B)
#define NextPower(N)        NextPower_r    (DefaultMt,N)
#define NextLogistic()      NextLogistic_r (DefaultMt)
#define NextCauchy()        NextCauchy_r   (DefaultMt)
#define NextFDist(A,B)      NextFDist_r    (DefaultMt,A,B)
#define NextPoisson(L)      NextPoisson_r  (DefaultMt,L)
#define NextTDist(N)        NextTDist_r    (DefaultMt,N)
#define NextWeibull(A)      NextWeibull_r  (DefaultMt,A)

#define InitMt_m(M,S)      InitMt_r    (DefaultMt+(M),S)
#define InitMtEx_m(M,K,L)  InitMtEx_r  (DefaultMt+(M),K,L)
#define NextMt_m(M)        NextMt_r    (DefaultMt+(M))
#define NextUnif_m(M)      NextUnif_r  (DefaultMt+(M))
#define NextInt_m(M,N)     NextInt_r   (DefaultMt+(M),N)
#define NextIntEx_m(M,N)   NextIntEx_r (DefaultMt+(M),N)
#define NextBit_m(M)       NextBit_r   (DefaultMt+(M))
#define NextByte_m(M)      NextByte_r     (DefaultMt+(M))

#define NextChisq_m(M,N)        NextChisq_r    (DefaultMt+(M),N)
#define NextGamma_m(M,A)        NextGamma_r    (DefaultMt+(M),A)
#define NextGeometric_m(M,P)    NextGeometric_r(DefaultMt+(M),P)
#define NextTriangle_m(M)       NextTriangle_r (DefaultMt+(M))
#define NextExp_m(M)            NextExp_r      (DefaultMt+(M))
#define NextNormal_m(M)         NextNormal_r   (DefaultMt+(M))
#define NextUnitVect_m(M,V,N)   NextUnitVect_r (DefaultMt+(M),V,N)
#define NextBinomial_m(M,N,P)   NextBinomial_r (DefaultMt+(M),N,P)
#define NextBinormal_m(M,R,X,Y) NextBinormal_r (DefaultMt+(M),R,X,Y)
#define NextBeta_m(M,A,B)       NextBeta_r     (DefaultMt+(M),A,B)
#define NextPower_m(M,N)        NextPower_r    (DefaultMt+(M),N)
#define NextLogistic_m(M)       NextLogistic_r (DefaultMt+(M))
#define NextCauchy_m(M)         NextCauchy_r   (DefaultMt+(M))
#define NextFDist_m(M,A,B)      NextFDist_r    (DefaultMt+(M),A,B)
#define NextPoisson_m(M,L)      NextPoisson_r  (DefaultMt+(M),L)
#define NextTDist_m(M,N)        NextTDist_r    (DefaultMt+(M),N)
#define NextWeibull_m(M,A)      NextWeibull_r  (DefaultMt+(M),A)

#ifdef STABLE_SPEED

static __inline double NextUnif_r(state_p mt)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) {
        if (mt==DefaultMt) InitMt_r(mt,1);
        else InitMt_r(mt,DefaultSeed--);
    }
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) mt->index=0;
    if ((mt->index&1)==0) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
    }
    return mt->u.d[mt->index++]-1.0;
}

static __inline uint32_t NextMt_r(state_p mt)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) {
        if (mt==DefaultMt) InitMt_r(mt,1);
        else InitMt_r(mt,DefaultSeed--);
    }
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) mt->index=0;
    if ((mt->index&1)==0) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
    }
#ifdef BIG_E
    return mt->u.x[(mt->index++<<1)^1];
#else
    return mt->u.x[mt->index++<<1];
#endif
}

#else

static __inline double NextUnif_r(state_p mt)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) {
        if (mt==DefaultMt) InitMt_r(mt,1);
        else InitMt_r(mt,DefaultSeed--);
    }
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
        mt->index=0;
    }
    return mt->u.d[mt->index++]-1.0;
}

static __inline uint32_t NextMt_r(state_p mt)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) {
        if (mt==DefaultMt) InitMt_r(mt,1);
        else InitMt_r(mt,DefaultSeed--);
    }
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
        mt->index=0;
    }
#ifdef BIG_E
    return mt->u.x[(mt->index++<<1)^1];
#else
    return mt->u.x[mt->index++<<1];
#endif
}

#endif

#endif

#define mt DefaultMt

#ifdef STABLE_SPEED

static __inline double NextUnif(void)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) InitMt_r(mt,1);
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) mt->index=0;
    if ((mt->index&1)==0) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
    }
    return mt->u.d[mt->index++]-1.0;
}

static __inline uint32_t NextMt(void)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) InitMt_r(mt,1);
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) mt->index=0;
    if ((mt->index&1)==0) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
    }
#ifdef BIG_E
    return mt->u.x[(mt->index++<<1)^1];
#else
    return mt->u.x[mt->index++<<1];
#endif
}

#else

static __inline double NextUnif(void)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) InitMt_r(mt,1);
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
        mt->index=0;
    }
    return mt->u.d[mt->index++]-1.0;
}

static __inline uint32_t NextMt(void)
{
#if !defined(__cplusplus) || defined(__GNUC__)
    if (mt->index==0) InitMt_r(mt,1);
#endif
    if (mt->index==((int)((MEXP-128)/104)+1)*2) {
#ifdef HAVE_SSE2
        if (mt->flag) GenSSE2(mt); else
#endif
        GenNormal(mt);
        mt->index=0;
    }
#ifdef BIG_E
    return mt->u.x[(mt->index++<<1)^1];
#else
    return mt->u.x[mt->index++<<1];
#endif
}

#undef mt

#endif
