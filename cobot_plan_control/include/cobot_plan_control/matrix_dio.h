//////////////////////////////////////////////////////////////////////
// awl.h: definition for ADLINK WMI library.
//
//////////////////////////////////////////////////////////////////////

#if !defined(MATRIX_DIO_H)
#define MATRIX_DIO_H

#if defined(__cplusplus)
extern "C"
{
#endif

typedef unsigned char  U8;
typedef short          I16;
typedef unsigned short U16;
typedef long           I32;
typedef unsigned long  U32;
typedef float          F32;
typedef double         F64;
typedef unsigned char Byte;

/*
 * Error Number
 */
#define NoError                     0
#define ErrorOpenDriverFailed      -13
#define ErrorDeviceIoctl           -200
#define NotSupport				   -1

int AwlLEDCount();// return with BIT0 as LED1's Status, BIT1 as LED2's Status, BIT2 as LED3's Status
int AwlLEDSetValue(int Index, int Value);//index count from 1; value 0 is off, 1 is on; return 1 is success

I16 GPIO_Init();
I16 GPI_Read(U16* pwState);
I16 GPO_Write(U16 wState);
I16 GPO_Read(U16* pwState);
#if defined(__cplusplus)
}
#endif

#endif	// !defined(AWL_H)