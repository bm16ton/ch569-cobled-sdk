// 2003.09.08, 2003.12.28
//****************************************
//**  Copyright  (C)  W.ch  1999-2005   **
//**  Web:  http://www.winchiphead.com  **
//****************************************
//**  DLL for USB interface chip CH375  **
//**  C, VC5.0                          **
//****************************************
//
// USB���߽ӿ�оƬCH375�����ݿ���Գ��� V1.0
// �Ͼ��ߺ�������޹�˾  ����: W.ch 2003.12
// CH375-BLK  V1.0
// ���л���: Windows 98/ME, Windows 2000/XP
// support USB chip: CH372/CH375
//

#include	<windows.h>
#include	<stdlib.h>
#include	<stdio.h>
#include	<conio.h>
#include	<winioctl.h>

#include	"CH375DLL.H"			
#pragma comment(lib,"CH375DLL")
#define		TEST_DATA_LEN		0x400000
#define		TEST_NUM     		1000
unsigned char	mReadBuf[TEST_DATA_LEN];
unsigned char	mWriteBuf[TEST_DATA_LEN];
//�������
void main (int argc,char **argv )
{
	unsigned long mLength, mTestCount, mErrCnt,mArg,mFirstTick,mLastTick;
	long long mTotal=0;
	double          speed;
	USHORT          mCount = 0;
	printf( "\nCH372/CH375 Bulk Data Test Program V1.1 ,   Copyright (C) W.ch 2004.12\n" );
	printf( "test data correctness \n" );
	mArg = TEST_DATA_LEN;

// ��Ҫʹ��DLL����Ҫ�ȼ���,û�д˾�����Զ�����
	printf( "*** CH375OpenDevice: 0# \n" );
	if ( CH375OpenDevice( 0 ) == INVALID_HANDLE_VALUE ) return;  /* ʹ��֮ǰ������豸 */

	memset(mWriteBuf, 0xFF, sizeof(mWriteBuf));
	
	mErrCnt=0;

	printf( "*** CH375ReadData: 1000 times 4M Byte ***\n" );

	mTotal = 0.0;
	for ( mTestCount=0; mTestCount < TEST_NUM; ++mTestCount )  // ѭ������
	{
		if(mTestCount == 0)
		{
			mFirstTick=GetTickCount();
		}
		mLength = mArg;
		if (CH375ReadEndP(0, 2, mReadBuf, &mLength))  // ���ճɹ�
		{
			mTotal += mLength;
			if(mLength == 0 )
			{
				Sleep(0);  //������ǰ�̵߳�ʱ��Ƭ����ֹCPU����100%���
			} 
		}
		else 
		{
			
			printf( "S1-T%0ld-C%ld CH375ReadData return error, length=%d\n", mTestCount, mTestCount, mTotal );
		}

		mLength = mArg;
		if (CH375WriteEndP(0, 2, mWriteBuf, &mLength))  // д��ɹ�
		{
			mTotal += mLength;
			if (mLength == 0)
			{
				Sleep(0);  //������ǰ�̵߳�ʱ��Ƭ����ֹCPU����100%���
			}
		}
		else 
		{  // д����ʧ��
			printf("S1-T%0ld-C%ld CH375WriteEndP return error, length=%d\n", mTestCount, mTestCount, mTotal);
		}
		
	}
	
	mLastTick =GetTickCount();
	mLastTick = mLastTick - mFirstTick;
	speed=1000;
	speed=speed*mTotal/mLastTick;
	printf( "*** average speed = %7.1f MBytes/Sec, total=%lld bytes\n", speed/1000/1000, mTotal);
	
	CH375CloseDevice( 0 );

	printf( "\nExit.\n" );
	_getch();
	
}
