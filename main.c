/*******************************************************************************
 * 24B�����λ�����
 *
 * debug1.00  20201218  1207
 * �������
 * 
 * debug1.01-1.02  20201222  1207 home
 * CAN IP ͨ�Ų������ʵ��
 *
 * debug1.03  20201222  1207
 * �ȿز��ֹ���ʵ��
 *
*******************************************************************************/

#include "math.h"
#include ".\drivers\mss_gpio\mss_gpio.h"
#include ".\drivers\mss_timer\mss_timer.h"

#define TRUE 0xAA
#define FALSE 0x55

#define ID_BC1 0x3F // �㲥1  GPS�����ʱ�㲥
#define ID_BC2 0x6F // �㲥2  ����ʱ��㲥
#define ID_BC3 0xAF // �㲥3  GPS��λ�㲥���� �Կع㲥����
#define ID_BC4 0x9F // �㲥4  �˿������㲥
#define ID_BC5 0x8F // �㲥5  �˿����ݹ㲥��  �����ڹ㲥
#define ID_BC6 0xA3 // �㲥6  ��̬��Ԫ��Ԥ���㲥
#define ID_BC7 0x4D // �㲥7  �������зַ�����   ????
#define ID_MU1 0x8B // ����ID1  ң��ָ��
#define ID_MU2 0x4B // ����ID2  ���ָ����ݿ�

#define WK_uLIMIT 0x0073 //
#define WK_lLIMIT 0x07CF //
#define WK_PERIOD 4		 // main_period*WK_PERIOD)

#define HEAT_ADDR *((volatile unsigned short *)0x301AAAA0)		// �ȿص�ַ
#define CAN_RESET_HARD *((volatile unsigned short *)0x301AB000) // CAN����Ӳ��λ
#define RESET_COUNT *((volatile unsigned short *)0x3017F7F0)	// �ȸ�λ����

#define RXD422_Ctrl_InitialA *((volatile unsigned short *)0x301AA110)	//422-A
#define RXD422_Buffer_ADDRESSA *((volatile unsigned short *)0x301AA000) //422
#define TXD422_Buffer_ADDRESSA *((volatile unsigned short *)0x301AA000) //422

#define RXD422_Ctrl_InitialB *((volatile unsigned short *)0x301BB110)	//422-B
#define RXD422_Buffer_ADDRESSB *((volatile unsigned short *)0x301BB000) //422
#define TXD422_Buffer_ADDRESSB *((volatile unsigned short *)0x301BB000) //422

#define RXD422_BTRSetA *((volatile unsigned short *)0x301AA440) //422
#define RXD422_BTRSetB *((volatile unsigned short *)0x301BB440) //422

#define VERSION 104 // �ڲ��汾��

const uint32_t CAN_A_ADDRESS = 0x3019A000u; // CAN-A address
const uint32_t CAN_B_ADDRESS = 0x3019B000u; // CAN-B address

uint8_t fg40ms;	   // �������� 40ms
uint8_t count40ms; // ��ʱ������ 40ms

uint32_t timeS;	 // ϵͳʱ�� -- ��
uint16_t timeMS; // ϵͳʱ�� -- ����

uint8_t fgSwitch;		// ��֡��־
uint8_t canErrorCount1; // CAN��ЧͨѶ����1
uint8_t canErrorCount2; // CAN��ЧͨѶ����2

uint8_t resetCount;			// �ȸ�λ����
uint8_t camOnoffTransCount; // ���ת��ָ�����

uint16_t anData[30];	  // ģ�����ɼ�
uint8_t stateWord[2];	  // �����λ��״̬��
uint8_t camPowerState;	  // ����ӵ�״̬
uint8_t heatModeState[2]; // ����ģʽ��
uint8_t heatState;		  // ���¼���״̬
uint8_t heatCheckState;	  // ���¼����Լ�״̬
uint8_t camMotorState;	  // ����˶�״̬

uint8_t camMotorLocate; // ���λ����Ϣ
uint8_t taskNumber[4];	// ������

uint8_t onoffBuffer[48][4]; // ָ��� 48�� ����
uint8_t wrOnoffIndex;		// ָ��д����
uint8_t rdOnoffIndex;		// ָ�������

uint8_t onoffCount;	 // ָ��ִ�м���
uint8_t onoffCountB; // ָ��ִ�м��� B
uint8_t onoffCountC; // ָ��ִ�м��� C

uint8_t onoffRecCode;  // ָ����һ����¼���
uint8_t onoffRecCodeB; // ָ����һ����¼��� B
uint8_t onoffRecCodeC; // ָ����һ����¼��� C

uint8_t onoffErrorCount;  // ָ��������
uint8_t onoffErrorCountB; // ָ�������� B
uint8_t onoffErrorCountC; // ָ�������� C

uint8_t encoderOverTimeCount;  // ���̳�ʱ����
uint8_t encoderOverTimeCountB; // ���̳�ʱ����B
uint8_t encoderOverTimeCountC; // ���̳�ʱ����C

uint8_t encoderRsErrorCount;  // ����ͨ�Ŵ������
uint8_t encoderRsErrorCountB; // ����ͨ�Ŵ������B
uint8_t encoderRsErrorCountC; // ����ͨ�Ŵ������C

uint8_t camPowerOverTimeCount;	// ����ϵ糬ʱ����
uint8_t camPowerOverTimeCountB; // ����ϵ糬ʱ����B
uint8_t camPowerOverTimeCountC; // ����ϵ糬ʱ����C

uint8_t canResetCountA;	  // CAN-A���߸�λ����
uint8_t canResetCountA_B; // CAN-A���߸�λ���� B
uint8_t canResetCountA_C; // CAN-A���߸�λ���� C

uint8_t canResetCountB;	  // CAN-B���߸�λ����
uint8_t canResetCountB_B; // CAN-A���߸�λ���� B
uint8_t canResetCountB_C; // CAN-A���߸�λ���� C

uint8_t canReceiveCountA;		// CAN���߽��ռ���  A����
uint8_t canReceiveFrameA;		// CAN���߽���֡����  A����
uint8_t canReceiveBufferA[256]; // CAN���߽��ջ�����  A����

uint8_t canReceiveCountB;		// CAN���߽��ռ���  B����
uint8_t canReceiveFrameB;		// CAN���߽���֡����  B����
uint8_t canReceiveBufferB[256]; // CAN���߽��ջ�����  B����

uint8_t muRsFrame1A[180]; // �����Ԫ�ٱ�ң�ⷢ�ͻ�����	18֡ -A
uint8_t muRsFrame1B[180]; // �����Ԫ�ٱ�ң�ⷢ�ͻ�����	18֡ -B
uint8_t muRsFrame2A[110]; // �����Ԫ����ң�ⷢ�ͻ�����	11֡ -A
uint8_t muRsFrame2B[110]; // �����Ԫ����ң�ⷢ�ͻ�����	11֡ -B

uint8_t muData9[9];		// �����Ԫ���ݿ�9�ֽ�
uint8_t muData57[57];	// �����Ԫ���ݿ�57�ֽ�
uint8_t muData121[121]; // �����Ԫ���ݿ�121�ֽ�
uint8_t muData249[249]; // �����Ԫ���ݿ�249�ֽ�

uint8_t timeSMUBcData[8];	 // ����㲥ʱ�仺��
uint8_t timeGPSBcData[8];	 // GPS�㲥ʱ�仺��
uint8_t gpsLocateBcData[99]; // GPS��λ�㲥���ݻ���
uint8_t gpsPoiseBcData[100]; // GPS��̬�㲥���ݻ���
uint8_t starBcData[233];	 // �˿������㲥���ݻ���
uint8_t topBcData[104];		 // �˿����ݹ㲥���ݻ���
uint8_t periodBcData[168];	 // �����ڹ㲥���ݻ���
uint8_t muAssistData[48];	 // ����������ݻ���

uint8_t fgAssistDataRenew;		   // �������ݸ��±�־
uint8_t assistDataSendBuffer[600]; // �������ݷ��ͻ���

uint8_t fgMuBusUse;	 // ��� CAN����ʹ�ñ�־
uint8_t fgCanBusUse; // ��� 422����ʹ�ñ�־

uint8_t cam1RsData[43]; // ���1ң�����ݻ���
uint8_t cam2RsData[43]; // ���2ң�����ݻ���
uint8_t cam3RsData[43]; // ���3ң�����ݻ���

uint8_t cam1RsErrorCount; // ���1ң��������
uint8_t cam2RsErrorCount; // ���2ң��������
uint8_t cam3RsErrorCount; // ���3ң��������

struct _WK_Parameter
{
	uint8_t Mode; // ����ģʽ
	uint16_t WD;  // ���µ�
};

const struct _WK_Parameter wk_CODEparameter[8] = //
	{
		//
		{0x02, 0x68C}, // No.0: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.1: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.2: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.3: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.4: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.5: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.6: ���»�·Ĭ��ֵ
		{0x02, 0x68C}, // No.7: ���»�·Ĭ��ֵ
};

struct _WK_Parameter wk_parameter[8];  // ���²������� - A
struct _WK_Parameter wk_parameterB[8]; // - B
struct _WK_Parameter wk_parameterC[8]; // - C

uint8_t WKS_Buffer; // ����״̬��
uint8_t wk_period;	// �������ڱ���

struct _WK_Time
{
	uint8_t Last; //
	uint8_t Now;  //
};
struct _WK_Time wk_time[8]; // ���²�������

uint16_t wk_tLast[8]; // ����ʱ�����
uint16_t wk_rData[8];

void SysInit(void);	   // ϵͳ��ʼ������
void GPIO_Init(void);  // GPIO��ʼ��
void DataInit(void);   // ���ݳ�ʼ��
void Timer1Init(void); // ��ʱ��1��ʼ��
void WatchDog(void);   // ι��

void AnCollect(void);	   // ģ�����ɼ�����
void SaveData(void);	   // ��֡����
void BuildStateWord(void); // ��֡����
void OnoffHook(void);	   // ָ�����

void HeatControl(void);							   // ���ȿ���
void WK_OUTPUT(uint8_t index, uint8_t onoffstate); // �ȿ����
void WK_Time14(uint8_t Index, uint32_t r);		   // ����ʽ����ģ��

void CanResume(void); // CAN�ָ�����

void DataPortect(void);									 // ���ݱ���ģ��
uint8_t Get2_3_F(float *a, float *b, float *c);			 // ������3ȡ2����
uint8_t Get2_3_I(uint16_t *a, uint16_t *b, uint16_t *c); // 16bit3ȡ2����
uint8_t Get2_3_U(uint8_t *a, uint8_t *b, uint8_t *c);	 // 8bit3ȡ2����

void CORECanInit_BASIC(uint32_t address); // IP��CAN��ʼ��

void CanReceiveA(void);											// SJA1000 CAN���߽��� -A
void timeGPSBcDataCanReceiveA(uint8_t id);						// GPSʱ��㲥���ݽ��պ���   A����   ����
void timeSMUBcDataCanReceiveA(uint8_t id);						// ����ʱ��㲥���ݽ��պ���   A����  ����
void GeneralCanReceiveA(uint8_t id1, uint8_t id2, uint8_t dlc); // һ���������ݽ��մ���  A����

void CanSendRs1A(void);		 // �ٱ�ң�ⷢ�� - A����
void CanSendRs2A(void);		 // ����ң�ⷢ�� - A����
void CanSendOnoffAckA(void); // ���ָ��Ӧ�� - A����
void CanSendDataAckA(void);	 // ���ݿ�ָ��Ӧ�� - A����

void CanReceiveB(void);											// SJA1000 CAN���߽��� -B
void timeGPSBcDataCanReceiveB(uint8_t id);						// GPSʱ��㲥���ݽ��պ���   B����   ����
void timeSMUBcDataCanReceiveB(uint8_t id);						// ����ʱ��㲥���ݽ��պ���   B����  ����
void GeneralCanReceiveB(uint8_t id1, uint8_t id2, uint8_t dlc); // һ���������ݽ��մ��� B����

void CanSendRs1B(void);		 // �ٱ�ң�ⷢ�� - B����
void CanSendRs2B(void);		 // ����ң�ⷢ�� - B����
void CanSendOnoffAckB(void); // ���ָ��Ӧ�� - B����
void CanSendDataAckB(void);	 // ���ݿ�ָ��Ӧ�� - B����

void Control(void);			// ��������ģ��
void AssistDataTrans(void); // ��������ת��ģ��
void AssistDataSave(void);	// ����������֡ģ��
void CamTask(void);			// �ڹ���������滮

void CamRsSend1(void); // ���1��ѯָ���
void CamRsSend2(void); // ���2��ѯָ���
void CamRsSend3(void); // ���3��ѯָ���

void CamRsReceive1(void); // ���1��ѯ���ݽ���
void CamRsReceive2(void); // ���2��ѯ���ݽ���
void CamRsReceive3(void); // ���3��ѯ���ݽ���

void RXD422_Init(void); // 422��ʼ��

void GPIO9_IRQHandler(void);  // CAN-A GPIO�ж�
void GPIO11_IRQHandler(void); // CAN-B GPIO�ж�
void Timer1_IRQHandler(void); // ��ʱ��1�жϷ�����

/***************************************************************************************************/
int main()
{
	SysInit(); // ϵͳ��ʼ��

	while (1)
	{
		Control();	 // ���ڹ����л�
		OnoffHook(); // ָ���

		do // ת�����ݴ���
		{
			//			TransData();		  // ת�����ݡ�����ʱ���
			CamTask();			  // ����ڹ�����滮
		} while (fg40ms != TRUE); // 40ms���ڽ���
		count40ms++;			  // 40ms ���ڼ�������
		fg40ms = FALSE;

		WatchDog(); // ι��	40msι��һ��
	}
}
/****************************************************************************************************************************************/
void Control(void)
{
	//	unsigned int i;

	switch (count40ms) // 40ms ���ڿ���
	{
	case 1:			   // step 1 -->
		DataPortect(); // ���ݱ���ģ��
		break;

	case 2: // step 2 -->  ���1��ѯָ���
		CamRsSend1();
		break;

	case 3: // step 3 --> ���1��ѯ���ݽ��գ���ѯ��ʽ��
		CamRsReceive1();
		break;

	case 4: // step 4 -->  ���2��ѯָ���
		CamRsSend2();
		break;

	case 5: // step 5 --> ���2��ѯ���ݽ��գ���ѯ��ʽ��
		CamRsReceive2();
		break;

	case 6: // step 6 --> ���3��ѯָ���
		CamRsSend3();
		break;

	case 7: // step 7 --> ���3��ѯ���ݽ��գ���ѯ��ʽ��
		CamRsReceive3();
		break;

	case 8: // step 8 --> ģ�����ɼ�ģ��
		AnCollect();
		break;

	case 9: // step 9 --> �ȿ����
		HeatControl();
		break;

	case 10: // step 10 -->

		break;

	case 11: // step 11 -->

		break;

	case 12:		 // step 12 -->
		CanResume(); // CAN�����ݴ�ģ��
		break;

	case 13: // step 13 -->

		break;

	case 14:			  // step 14 -->
		BuildStateWord(); // ��״̬��
		SaveData();		  // ��֡����
		break;

	case 15: // step 15 --> ����������֡ģ��
		AssistDataSave();
		break;

	case 16: // step 16 -->	��������ת��   ����1s
		AssistDataTrans();
		break;

	case 17: // step 17 -->	
	case 18: // step 18 -->	
	case 19: // step 19 -->	
	case 20: // step 20 -->	
	case 21: // step 21 -->	
	case 22: // step 22 -->	
	case 23: // step 23 -->	
	case 24: // step 24 -->	
		break;
		
	case 25:		   // step 25  -- 1s
		count40ms = 0; // ���������ڸ�λ

		break;

	default:
		count40ms = 0;		// ��ֹ�ڹ��쳣��ת ��ȫ�����
		break;
	}
}

/****************************************************************************************************************************************/
void CamTask(void)
{
}

/****************************************************************************************************************************************/
void CamRsSend1(void)
{
	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		RXD422_Ctrl_InitialA = 0x48;   // ������ջ����� ????
		TXD422_Buffer_ADDRESSA = 0xEB; // W0   ֡ͷ
		TXD422_Buffer_ADDRESSA = 0x90; // W1
		TXD422_Buffer_ADDRESSA = 0x01; // W2   ͨ����
		TXD422_Buffer_ADDRESSA = 0xEE; // W3   ��ѯ����
		TXD422_Buffer_ADDRESSA = 0xEF; // W4   SUM
	}
	else
	{
		RXD422_Ctrl_InitialB = 0x48;   // ������ջ����� ????
		TXD422_Buffer_ADDRESSB = 0xEB; // W0   ֡ͷ
		TXD422_Buffer_ADDRESSB = 0x90; // W1
		TXD422_Buffer_ADDRESSB = 0x01; // W2   ͨ����
		TXD422_Buffer_ADDRESSB = 0xEE; // W3   ��ѯ����
		TXD422_Buffer_ADDRESSB = 0xEF; // W4   SUM
	}
}

/****************************************************************************************************************************************/
void CamRsReceive1(void)
{
	uint8_t i;
	uint8_t sum;

	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		for (i = 0; i < 43; i++) // ��ȡԤ�ڵķ�������
		{
			cam1RsData[i] = RXD422_Buffer_ADDRESSA;
		}
	}
	else
	{
		for (i = 0; i < 43; i++) // ��ȡԤ�ڵķ�������
		{
			cam1RsData[i] = RXD422_Buffer_ADDRESSB;
		}
	}

	for (i = 2; i < 42; i++) // ����У���
	{
		sum = sum + cam1RsData[i];
	}

	if (sum != cam1RsData[42]) // У��ʹ���   ң���������  0xE1
	{
		cam1RsErrorCount++;		 // ���ң��������+1
		for (i = 3; i < 42; i++) // ������ʱ���
		{
			cam1RsData[i] = 0xE1;
		}
	}
}

/****************************************************************************************************************************************/
void CamRsSend2(void)
{
	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		RXD422_Ctrl_InitialA = 0x48;   // ������ջ����� ????
		TXD422_Buffer_ADDRESSA = 0xEB; // W0   ֡ͷ
		TXD422_Buffer_ADDRESSA = 0x90; // W1
		TXD422_Buffer_ADDRESSA = 0x02; // W2   ͨ����
		TXD422_Buffer_ADDRESSA = 0xEE; // W3   ��ѯ����
		TXD422_Buffer_ADDRESSA = 0xF0; // W4   SUM
	}
	else
	{
		RXD422_Ctrl_InitialB = 0x48;   // ������ջ����� ????
		TXD422_Buffer_ADDRESSB = 0xEB; // W0   ֡ͷ
		TXD422_Buffer_ADDRESSB = 0x90; // W1
		TXD422_Buffer_ADDRESSB = 0x02; // W2   ͨ����
		TXD422_Buffer_ADDRESSB = 0xEE; // W3   ��ѯ����
		TXD422_Buffer_ADDRESSB = 0xF0; // W4   SUM
	}
}

/****************************************************************************************************************************************/
void CamRsReceive2(void)
{
	uint8_t i;
	uint8_t sum;

	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		for (i = 0; i < 43; i++) // ��ȡԤ�ڵķ�������
		{
			cam2RsData[i] = RXD422_Buffer_ADDRESSA;
		}
	}
	else
	{
		for (i = 0; i < 43; i++) // ��ȡԤ�ڵķ�������
		{
			cam2RsData[i] = RXD422_Buffer_ADDRESSB;
		}
	}

	for (i = 2; i < 42; i++) // ����У���
	{
		sum = sum + cam2RsData[i];
	}

	if (sum != cam2RsData[42]) // У��ʹ���   ң���������  0xE1
	{
		cam2RsErrorCount++;		 // ���ң��������+1
		for (i = 3; i < 42; i++) // ������ʱ���
		{
			cam2RsData[i] = 0xE1;
		}
	}
}

/****************************************************************************************************************************************/
void CamRsSend3(void)
{
	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		RXD422_Ctrl_InitialA = 0x48;   // ������ջ����� ????
		TXD422_Buffer_ADDRESSA = 0xEB; // W0   ֡ͷ
		TXD422_Buffer_ADDRESSA = 0x90; // W1
		TXD422_Buffer_ADDRESSA = 0x03; // W2   ͨ����
		TXD422_Buffer_ADDRESSA = 0xEE; // W3   ��ѯ����
		TXD422_Buffer_ADDRESSA = 0xF1; // W4   SUM
	}
	else
	{
		RXD422_Ctrl_InitialB = 0x48;   // ������ջ����� ????
		TXD422_Buffer_ADDRESSB = 0xEB; // W0   ֡ͷ
		TXD422_Buffer_ADDRESSB = 0x90; // W1
		TXD422_Buffer_ADDRESSB = 0x03; // W2   ͨ����
		TXD422_Buffer_ADDRESSB = 0xEE; // W3   ��ѯ����
		TXD422_Buffer_ADDRESSB = 0xF1; // W4   SUM
	}
}

/****************************************************************************************************************************************/
void CamRsReceive3(void)
{
	uint8_t i;
	uint8_t sum;

	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		for (i = 0; i < 43; i++) // ��ȡԤ�ڵķ�������
		{
			cam3RsData[i] = RXD422_Buffer_ADDRESSA;
		}
	}
	else
	{
		for (i = 0; i < 43; i++) // ��ȡԤ�ڵķ�������
		{
			cam3RsData[i] = RXD422_Buffer_ADDRESSB;
		}
	}

	for (i = 2; i < 42; i++) // ����У���
	{
		sum = sum + cam3RsData[i];
	}

	if (sum != cam3RsData[42]) // У��ʹ���   ң���������  0xE1
	{
		cam3RsErrorCount++;		 // ���ң��������+1
		for (i = 3; i < 42; i++) // ������ʱ���
		{
			cam3RsData[i] = 0xE1;
		}
	}
}

/****************************************************************************************************************************************/
void AssistDataSave(void)
{
	uint8_t i;
	uint8_t *p;
	uint8_t sum;

	do
	{
		fgAssistDataRenew = FALSE;	  // ��ʼ������±�־  �ж����ݱ������
		sum = 0;					  // �ۼӺ�����
		p = &assistDataSendBuffer[0]; // ����ָ�������ֵ

		*p = 0xEB; // W0 ֡ͷ  EB 90
		p++;
		*p = 0x90; // W1
		p++;
		*p = 0x07;		// W2 ͨ��ѡ��	 111ȫ��
		sum = sum + *p; // �����ۼӺ�
		p++;

		for (i = 0; i < 5; i++) // GPS�����ʱ�㲥����	5�ֽ�
		{
			*p = timeGPSBcData[i]; // ��Ч���ݸ���
			sum = sum + *p;		   // �����ۼӺ�
			p++;
		}

		for (i = 0; i < 6; i++) // ����ʱ��㲥����	6�ֽ�
		{
			*p = timeSMUBcData[i]; // ��Ч���ݸ���
			sum = sum + *p;		   // �����ۼӺ�
			p++;
		}

		for (i = 0; i < 99; i++) // GPS��λ�㲥����	99�ֽ�
		{
			*p = gpsLocateBcData[i]; // ��Ч���ݸ���
			sum = sum + *p;			 // �����ۼӺ�
			p++;
		}

		for (i = 0; i < 100; i++) // �˿���̬�㲥����	100�ֽ�
		{
			*p = gpsPoiseBcData[i]; // ��Ч���ݸ���
			sum = sum + *p;			// �����ۼӺ�
			p++;
		}

		for (i = 0; i < 233; i++) // �˿������㲥	233�ֽ�
		{
			*p = starBcData[i]; // ��Ч���ݸ���
			sum = sum + *p;		// �����ۼӺ�
			p++;
		}

		for (i = 0; i < 104; i++) // �˿����ݹ㲥����	104�ֽ�
		{
			*p = topBcData[i]; // ��Ч���ݸ���
			sum = sum + *p;	   // �����ۼӺ�
			p++;
		}

		for (i = 0; i < 48; i++) // �ȿ���λ����������	48�ֽ�
		{
			*p = muAssistData[i]; // ��Ч���ݸ���
			sum = sum + *p;		  // �����ۼӺ�
			p++;
		}

		*p = sum; // sum ����

	} while (fgAssistDataRenew != FALSE); // ��֡�����з������ݸ���
}

/***************************************************************************************************/
void AssistDataTrans(void)
{
	uint16_t i;

	if (fgCanBusUse == 0xAA) // ������߱�־
	{
		for (i = 0; i < 600; i++) // ��������д�뻺����  A����
		{
			TXD422_Buffer_ADDRESSA = assistDataSendBuffer[i];
		}
	}
	else
	{
		for (i = 0; i < 600; i++) // ��������д�뻺����  B����
		{
			TXD422_Buffer_ADDRESSB = assistDataSendBuffer[i];
		}
	}
}

/***************************************************************************************************/
void SysInit(void)
{
	uint16_t i;
	uint32_t delay;

	for (delay = 0; delay < 50000; delay++)
		; // �ϵ���ʱ �ȴ�

	GPIO_Init();

	SYSREG->WDOG_CR = 0; // Turn off the watchdog

	for (i = 0; i < 50; i++)
	{
		WatchDog();
		for (delay = 0; delay < 5000; delay++)
			; // �ϵ���ʱ �ȴ�
	}

	// HEAT_ADDR = ( 1*256 + 0x0055 );											// ���»�·�ر�

	DataInit(); // ���ݳ�ʼ��

	AnCollect(); // ң�����ݳ�ʼ��䴦��
	BuildStateWord();
	SaveData();
	SaveData();

	Timer1Init();  // ��ʱ����ʼ������
	RXD422_Init(); // 422���߳�ʼ��

	CORECanInit_BASIC(CAN_A_ADDRESS); // CAN - A ��ʼ��
	CORECanInit_BASIC(CAN_B_ADDRESS); // CAN - B ��ʼ��		??? �建����������Ҫ�ԣ�����

	MSS_GPIO_enable_irq(MSS_GPIO_9);  // CAN �����ж�--A  ʹ��
	MSS_GPIO_enable_irq(MSS_GPIO_11); // CAN �����ж�--B  ʹ��

	// CAN_RESET_HARD = 0xAA;													// Ӳ��λ���
	// for( delay=0; delay<5000; delay++ );									// �ϵ���ʱ �ȴ�
}

/***************************************************************************************************/
void AnCollect(void)
{
	anData[0] = *((volatile unsigned short *)(0x30199000 | (0 << 4))); // ���µ�1�¶�ֵ
	anData[1] = *((volatile unsigned short *)(0x30199000 | (1 << 4))); // ���µ�2�¶�ֵ
	anData[2] = *((volatile unsigned short *)(0x30199000 | (2 << 4))); // ���µ�3�¶�ֵ
	anData[3] = *((volatile unsigned short *)(0x30199000 | (3 << 4))); // ���µ�4�¶�ֵ
	anData[4] = *((volatile unsigned short *)(0x30199000 | (4 << 4))); // ���µ�5�¶�ֵ
	anData[5] = *((volatile unsigned short *)(0x30199000 | (5 << 4))); // ���µ�6�¶�ֵ
	anData[6] = *((volatile unsigned short *)(0x30199000 | (6 << 4))); // ���µ�7�¶�ֵ
	anData[7] = *((volatile unsigned short *)(0x30199000 | (7 << 4))); // ���µ�8�¶�ֵ

	anData[8] = *((volatile unsigned short *)(0x30199000 | (8 << 4)));	 // ���������Դ12Vң��
	anData[9] = *((volatile unsigned short *)(0x30199000 | (9 << 4)));	 // ��׼��ѹԴ5Vң��
	anData[10] = *((volatile unsigned short *)(0x30199000 | (10 << 4))); // �����Դ12Vң��

	anData[11] = *((volatile unsigned short *)(0x30199000 | (11 << 4))); // ������2�¶�
	anData[12] = *((volatile unsigned short *)(0x30199000 | (12 << 4))); // ̫����1�¶�
	anData[13] = *((volatile unsigned short *)(0x30199000 | (13 << 4))); // ̫����2�¶�
	anData[14] = *((volatile unsigned short *)(0x30199000 | (14 << 4))); // �ŵ翪��״̬
	anData[15] = *((volatile unsigned short *)(0x30199000 | (15 << 4))); // ���ؼ���1״̬
	anData[16] = *((volatile unsigned short *)(0x30199000 | (16 << 4))); // ���ؼ���2״̬
	anData[17] = *((volatile unsigned short *)(0x30199000 | (17 << 4))); // �����ֿ���״̬
	anData[18] = *((volatile unsigned short *)(0x30199000 | (18 << 4))); // ����𵶿���״̬
	anData[19] = *((volatile unsigned short *)(0x30199000 | (19 << 4))); // ��췫����״̬
	anData[20] = *((volatile unsigned short *)(0x30199000 | (20 << 4))); // Xһ�������״̬
	anData[21] = *((volatile unsigned short *)(0x30199000 | (21 << 4))); // �Ӿ�ϵͳ����״̬
	anData[22] = *((volatile unsigned short *)(0x30199000 | (22 << 4))); // ��е��1����״̬
	anData[23] = *((volatile unsigned short *)(0x30199000 | (23 << 4))); // ��е��2����״̬
	anData[24] = *((volatile unsigned short *)(0x30199000 | (24 << 4))); // +5V��Դң��
}

/***************************************************************************************************/
void BuildStateWord(void)
{
	stateWord[0] = 0; // ״̬������
					  // bit7-6  BIT7~ BIT6	������ε�״̬	00B�����ӵ�  01B�����ӵ�  10B�������ӵ�  11B�������ϵ�

	// BIT5	��ǰ�����λ����ʶ	0B������1B������
	// BIT4	CAN����B״̬	0B������1B���쳣
	// BIT3	CAN����A״̬	0B������1B���쳣

	if (fgMuBusUse == 0xAA) // BIT2	��ǰʹ��CAN����	0B��A����1B��B����
	{
		stateWord[0] = (stateWord[0] | 0x04); // 1 A����
	}
	else
	{
		stateWord[0] = (stateWord[0] & 0x7B); // 0 B����
	}
	// BIT1~BIT0	ָ�����ִ��״̬  00B����ȷ��ִ��   01B����ȷ����ִ��

	stateWord[1] = 0; // ״̬������
					  // BIT7	�߹�ʱ��ת��״̬	0B��ʹ��1B������
					  // BIT6	����ʱʹ��״̬	0B��ʹ��1B������
					  // BIT5 ~ BIT4	��λ��ʱ��״̬  00B����ʧGPSʱ��  01B����ʧӲ��������  10��Уʱ  11��������ʱ
					  // BIT3 ~ BIT2	����ִ��״̬  00B��ִ��  01B��δִ��  02B������ʧ��

	camPowerState = 0;
	// BIT7~ BIT3	����
	// BIT2	����·ģ��POWER ON/OFF״̬��0-OFF��1-ON
	// BIT1	�ڶ�·ģ��POWER ON/OFF״̬��0-OFF��1-ON
	// BIT0	��һ·ģ��POWER ON/OFF״̬��0-OFF��1-ON

	heatModeState[0] = 0;
	heatModeState[1] = 0;
	heatModeState[0] = ((wk_parameter[3].Mode & 0x03) << 6) | ((wk_parameter[2].Mode & 0x03) << 4) | ((wk_parameter[1].Mode & 0x03) << 2) | (wk_parameter[0].Mode & 0x03);
	heatModeState[1] = ((wk_parameter[7].Mode & 0x03) << 6) | ((wk_parameter[6].Mode & 0x03) << 4) | ((wk_parameter[5].Mode & 0x03) << 2) | (wk_parameter[4].Mode & 0x03);
}

/***************************************************************************************************/
void OnoffHook(void)
{
	uint8_t i;

	if (rdOnoffIndex != wrOnoffIndex) // �ж�ָ�����������
	{
		switch (onoffBuffer[rdOnoffIndex][0]) // ��Ӧָ������
		{
		case 0x01: // ID - 01

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x02: // ID - 02

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x03: // ID - 03

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x04: // ID - 04

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x05: // ID - 05

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x06: // ID - 05

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x07: // ID - 07

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x08: // ID - 08

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x09: // ID - 09

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0A: // ID - 0A

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0B: // ID - 0B

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0C: // ID - 0C

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0D: // ID - 0D    �ȿز�����עָ��

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0E: // ID - 0E    ��ע������ű�������1������������ű���-ʹ�ܵ�ѹ��

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0F: // ID - 0F    ��ע������ű�������2������������ű���-�ŵ������

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x10: // ID - 10    ��ע������ű�������3�������������-������ѹ��

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x11: // ID - 11    ��ע�����������������4��﮵���¶ȣ�

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x12: // ID - 12    ��ע�����������������5��﮵�ص�ѹ��

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x13: // ID - 13    ��ע����������ʱ����6�����ص�ѹ��

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x14: // ID - 14    ��ע����������ʱ����7�����س�������

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		default:
			break;
		}

		for (i = 0; i < 4; i++)
		{
			onoffBuffer[rdOnoffIndex][i] = 0x00; // ��ȡָ��󻺴�����
		}

		rdOnoffIndex++; // ��ָ������++  ָ����һ��ָ��
		if (rdOnoffIndex > 47)
		{
			rdOnoffIndex = 0;
		}
	}
}

/***************************************************************************************************/
void HeatControl(void)
{
	uint8_t i;
	uint32_t r;

	wk_period++; //
	for (i = 0; i < 5; i++)
	{
		// wk_rData[i] = (anData[i + 8] >> 1); // 11bit			��ȡ��������ֵ
	}

	heatState = 0;

	for (i = 0; i < 8; i++) //
	{
		if (wk_parameter[i].Mode == 0x00) // �����ر�ģʽ
		{
			WK_OUTPUT((i + 1), 0);
		}

		if (wk_parameter[i].Mode == 0x01) // ��������ģʽ
		{
			WK_OUTPUT((i + 1), 1);
		}

		if (wk_parameter[i].Mode == 0x02) // ����ʽ����
		{
			r = wk_rData[i];						// ��������ֵ���£�ָ����ʽ��
			if ((r < WK_uLIMIT) || (r > WK_lLIMIT)) // ����ֵ�������ж�
			{
				wk_parameter[i].Mode = 0; // �쳣�����Ϊ�����ر�ģʽ
				wk_parameterB[i].Mode = 0;
				wk_parameterC[i].Mode = 0;
				WK_OUTPUT((i + 1), 0); // ��Ӧ���¹ر����
			}
			else // ���������ں�������
			{
				if ((wk_period == 1) || (wk_period == (WK_PERIOD + 1))) // ���ڲ�������
				{
					WK_Time14(i, r); //
				}

				if ((wk_time[i].Now > 0) && (wk_time[i].Now < (uint8_t)(WK_PERIOD + 1))) // ���������ڱȽ�??
				{																		 //
					wk_time[i].Now--;													 //
					WK_OUTPUT((i + 1), 1);												 // ���¿�������
				}
				else //
				{
					WK_OUTPUT((i + 1), 0); // ���¹رռ���
				}
			}
		}
	}

	if (wk_period > WK_PERIOD) // �������ڽ���
	{
		wk_period = 1; // ���ڼ�������ֵ
	}
}

/***************************************************************************************************/
void WK_OUTPUT(uint8_t index, uint8_t onoffstate)
{
	switch (index)
	{
	case 0x01:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1155;
			heatState = (heatState & 0xFE);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x11AA;
			heatState = (heatState | 0x01);
		}
		break;

	case 0x02:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1255;
			heatState = (heatState & 0xFD);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x12AA;
			heatState = (heatState | 0x02);
		}
		break;

	case 0x03:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1355;
			heatState = (heatState & 0xFB);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x13AA;
			heatState = (heatState | 0x04);
		}
		break;

	case 0x04:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1455;
			heatState = (heatState & 0xF7);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x14AA;
			heatState = (heatState | 0x08);
		}
		break;

	case 0x05:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1555;
			heatState = (heatState & 0xEF);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x15AA;
			heatState = (heatState | 0x10);
		}
		break;

	case 0x06:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1555;
			heatState = (heatState & 0xDF);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x15AA;
			heatState = (heatState | 0x20);
		}
		break;

	case 0x07:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1555;
			heatState = (heatState & 0xBF);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x15AA;
			heatState = (heatState | 0x40);
		}
		break;

	case 0x08:
		if (onoffstate == 0)
		{
			*((volatile unsigned short *)0x30144440) = 0x1555;
			heatState = (heatState & 0x7F);
		}

		else if (onoffstate == 0x01)
		{
			*((volatile unsigned short *)0x30144440) = 0x15AA;
			heatState = (heatState | 0x80);
		}
		break;

	default:
		break;
	}
}

/***************************************************************************************************/
void WK_Time14(uint8_t Index, uint32_t r)
{
	uint32_t b;

	b = (wk_parameter[Index].WD >> 1); //

	if (r > b) //
	{
		if (r >= wk_tLast[Index]) //
		{						  //

			wk_time[Index].Now = wk_time[Index].Last; //

			if (wk_time[Index].Now < WK_PERIOD) //
			{
				wk_time[Index].Now++; //
			}
			else
			{
				wk_time[Index].Now = WK_PERIOD;
			}

			wk_time[Index].Last = wk_time[Index].Now; //
			wk_tLast[Index] = r;					  //
		}
		else
		{
			wk_time[Index].Now = wk_time[Index].Last; //
			wk_tLast[Index] = r;					  //
		}
	}
	else //
	{
		if (r <= wk_tLast[Index])					  //
		{											  //
			wk_time[Index].Now = wk_time[Index].Last; //

			if (wk_time[Index].Now != 0) //
			{
				wk_time[Index].Now--; //
			}
			else
			{
				wk_time[Index].Now = 0;
			}

			wk_time[Index].Last = wk_time[Index].Now; //
			wk_tLast[Index] = r;					  //
		}
		else
		{
			wk_time[Index].Now = wk_time[Index].Last; //
			wk_tLast[Index] = r;					  //
		}
	}
}

/***************************************************************************************************/
void CanResume(void)
{
}

/***************************************************************************************************/
void SaveData(void)
{
	uint16_t i;				//
	uint16_t j;				//
	uint8_t *p1;			// ����ٱ��ָ��
	uint8_t *p2;			// ��������ָ��
	uint8_t *p3;			//	�������ƴ��ָ��
	uint8_t sum;			//
	uint8_t camBuffer[122]; // ����ٱ仺�� ƴ֡

	//----------------------���ƴ����֡---------------------------------------------------------------//
	p3 = &camBuffer[0]; // ָ���ֵ

	*p3 = 0x3C;
	p3++;
	*p3 = 0x22;
	p3++;

	for (i = 0; i < 40; i++)
	{
		*p3 = cam1RsData[2 + i];
		p3++;
	}

	for (i = 0; i < 40; i++)
	{
		*p3 = cam2RsData[2 + i];
		p3++;
	}

	for (i = 0; i < 40; i++)
	{
		*p3 = cam3RsData[2 + i];
		p3++;
	}

	//----------------------CANң����֡---------------------------------------------------------------//
	if (fgSwitch == TRUE) // �鎬�л���־
	{
		p1 = &muRsFrame1B[0]; // ָ�򻺴���B  �ٱ�
		p2 = &muRsFrame2B[0]; // ָ�򻺴���B  ����
	}
	else
	{
		p1 = &muRsFrame1A[0]; // ָ�򻺴���A  �ٱ�
		p2 = &muRsFrame2A[0]; // ָ�򻺴���A  ����
	}

	sum = 0;
	p3 = &camBuffer[0];
	//----------------------�ٱ����֡---------------------------------------------------------------//
	*p1 = 0x8B;
	p1++; // ��1֡
	*p1 = 0x68;
	p1++;
	*p1 = 0x00;
	p1++; // ֡����
	*p1 = 0x7A;
	p1++; // ����
	*p1 = 0x27;
	sum = sum + *p1;
	p1++; // title

	for (i = 0; i < 5; i++) // W0 - W4
	{
		*p1 = *p3;
		sum = sum + *p1;
		p1++; // W0
		p3++;
	}

	for (i = 0; i < 16; i++) // �ٱ���м�֡����16��
	{
		*p1 = 0x8B;
		p1++; // ��N֡
		*p1 = 0x68;
		p1++;
		*p1 = i + 1;
		p1++; // ֡����

		for (j = 0; j < 7; j++) // ��Ч���ݸ���  w5 - w116
		{
			*p1 = *p3;
			sum = sum + *p1;
			p1++; // WN
			p3++;
		}
	}

	*p1 = 0x8B; // ����֡
	p1++;		//
	*p1 = 0x67;
	p1++;		//
	*p1 = 0x11; // 18֡
	p1++;		//

	for (i = 0; i < 5; i++) //  ���½���֡��Ч����  W117 - W121
	{
		*p1 = *p3;
		sum = sum + *p1;
		p1++; // W0
		p3++;
	}

	*p1 = sum; // SUM

	sum = 0;
	//----------------------�������֡---------------------------------------------------------------//
	//----------------------�����--1֡---------------------------------------------------------------//
	*p2 = 0x8B;
	p2++; // ��1֡
	*p2 = 0x68;
	p2++;
	*p2 = 0x00; // ֡����
	p2++;
	*p2 = 0x46; // ����
	p2++;
	*p2 = 0x47; // title
	sum = sum + *p2;
	p2++;
	*p2 = timeS >> 24; // W0 �����ʱ - byte0   ��λ
	sum = sum + *p2;
	p2++;
	*p2 = timeS >> 16; // W1 �����ʱ - byte1
	sum = sum + *p2;
	p2++;
	*p2 = timeS >> 8; // W2	�����ʱ - byte2
	sum = sum + *p2;
	p2++;
	*p2 = timeS; // W3 �����ʱ - byte3
	sum = sum + *p2;
	p2++;

	*p2 = timeMS >> 8; // W4 �����ʱ - byte0  ��λ
	sum = sum + *p2;
	p2++;
	//----------------------�����--2֡---------------------------------------------------------------//
	*p2 = 0x8B;
	p2++; // ��2֡
	*p2 = 0x68;
	p2++;
	*p2 = 0x01; // ֡����
	p2++;
	*p2 = timeMS; // W5 �����ʱ - byte1
	sum = sum + *p2;
	p2++;

	*p2 = taskNumber[0]; // W6 ������ - byte0
	sum = sum + *p2;
	p2++;
	*p2 = taskNumber[1]; // W7 ������ - byte1;
	sum = sum + *p2;
	p2++;
	*p2 = taskNumber[2]; // W8 ������ - byte2;
	sum = sum + *p2;
	p2++;
	*p2 = taskNumber[3]; // W9 ������ - byte3;;
	sum = sum + *p2;
	p2++;

	*p2 = stateWord[0]; // W10 �����λ������״̬1
	sum = sum + *p2;
	p2++;
	*p2 = stateWord[1]; // W11 �����λ������״̬1
	sum = sum + *p2;
	p2++;

	//----------------------�����--3֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��3֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x02; // ֡����
	p2++;

	*p2 = resetCount; // W12	����������
	sum = sum + *p2;
	p2++;
	*p2 = camOnoffTransCount; // W13	����ģ��ָ�����
	sum = sum + *p2;
	p2++;
	*p2 = onoffCount; // W14	�ȿؽ���ָ���ܼ���
	sum = sum + *p2;
	p2++;
	*p2 = onoffErrorCount; // W15	�ȿش���ָ�����
	sum = sum + *p2;
	p2++;
	*p2 = onoffRecCode; // W16	���һ��ָ����
	sum = sum + *p2;
	p2++;
	*p2 = camPowerOverTimeCount; // W17	����ϵ糬ʱ����
	sum = sum + *p2;
	p2++;
	*p2 = canResetCountA << 4 + canResetCountB; // W18	CAN���߸�λ���� ����λ -A ����λ -B
	sum = sum + *p2;
	p2++;
	//----------------------�����--4֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��4֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x03; // ֡����
	p2++;
	*p2 = encoderOverTimeCount << 4 + encoderRsErrorCount; // W19	������״̬	����λ -��ʱ���� ����λ -�������
	sum = sum + *p2;
	p2++;
	*p2 = camMotorState; // W20	�����������˶�״̬
	sum = sum + *p2;
	p2++;
	*p2 = camMotorLocate; // W21	��ǰ������λ��
	sum = sum + *p2;
	p2++;
	*p2 = anData[8] >> 4; // W22	���������Դ12Vң��
	sum = sum + *p2;
	p2++;
	*p2 = anData[9] >> 4; // W23	��׼��ѹԴ5Vң��
	sum = sum + *p2;
	p2++;
	*p2 = anData[10] >> 4; // W24	�����Դ12Vң��
	sum = sum + *p2;
	p2++;
	*p2 = camPowerState; // W25	ģ��ӵ�ʹ��״̬
	sum = sum + *p2;
	p2++;
	//----------------------�����--5֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��5֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x04; // ֡����
	p2++;
	*p2 = 0xAA; // W26	����
	sum = sum + *p2;
	p2++;
	*p2 = heatModeState[0]; // W27	���ȴ�1��4����ģʽ
	sum = sum + *p2;
	p2++;
	*p2 = heatModeState[1]; // W28	���ȴ�5��8����ģʽ
	sum = sum + *p2;
	p2++;
	*p2 = heatState; // W29	���ȴ�0��8����״̬
	sum = sum + *p2;
	p2++;
	*p2 = heatCheckState; // W30	���ȴ�0��8�ؼ�״̬
	sum = sum + *p2;
	p2++;
	*p2 = anData[0] >> 8; // W31 ���µ�1�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[0]; // W32	���µ�1�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	//----------------------�����--6֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��6֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x05; // ֡����
	p2++;
	*p2 = anData[1] >> 8; // W33 ���µ�2�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[1]; // W34	���µ�2�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[2] >> 8; // W35 ���µ�3�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[2]; // W36	���µ�3�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[3] >> 8; // W37 ���µ�4�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[3]; // W38	���µ�4�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[4] >> 8; // W39 ���µ�5�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	//----------------------�����--7֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��7֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x06; // ֡����
	p2++;
	*p2 = anData[4]; // W40	���µ�5�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[5] >> 8; // W41 ���µ�6�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[5]; // W42	���µ�6�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[6] >> 8; // W43 ���µ�7�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[6]; // W44	���µ�7�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[7] >> 8; // W45 ���µ�8�¶�ֵ-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[7]; // W46	���µ�8�¶�ֵ-L
	sum = sum + *p2;
	p2++;
	//----------------------�����--8֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��8֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x07; // ֡����
	p2++;
	*p2 = wk_parameter[0].WD >> 8; // W47 ����ͨ��1Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[0].WD; // W48 ����ͨ��1Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[1].WD >> 8; // W49 ����ͨ��2Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[1].WD; // W50 ����ͨ��2Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[2].WD >> 8; // W51 ����ͨ��3Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[2].WD; // W52 ����ͨ��3Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[3].WD >> 8; // W53 ����ͨ��4Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	//----------------------�����--9֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��9֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x08; // ֡����
	p2++;
	*p2 = wk_parameter[3].WD; // W54 ����ͨ��4Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[4].WD >> 8; // W55 ����ͨ��5Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[4].WD; // W56 ����ͨ��5Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[5].WD >> 8; // W57 ����ͨ��6Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[5].WD; // W58 ����ͨ��6Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[6].WD >> 8; // W59 ����ͨ��7Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[6].WD; // W60 ����ͨ��7Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	//----------------------�����--10֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��10֡
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x09; // ֡����
	p2++;
	*p2 = wk_parameter[7].WD >> 8; // W61 ����ͨ��8Ŀ���¶�-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[7].WD; // W62 ����ͨ��8Ŀ���¶�-L
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W63	����
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W64	����
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W65	����
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W66	����
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W67	����
	sum = sum + *p2;
	p2++;
	//----------------------�����--11֡---------------------------------------------------------------//
	*p2 = 0x8B; // ��11֡
	p2++;
	*p2 = 0x64;
	p2++;
	*p2 = 0x0A; // ֡����
	p2++;
	*p2 = 0xAA; // W68	����
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W69	����
	sum = sum + *p2;
	p2++;

	*p2 = sum; // sum

	if (fgSwitch == TRUE) // �鎬�л���־
	{
		fgSwitch = FALSE; // �л���־
	}
	else
	{
		fgSwitch = TRUE; // �л���־
	}
}

/***************************************************************************************************/
void DataInit(void)
{
	uint16_t i;
	uint16_t j;

	fg40ms = FALSE;	  // 40ms��ʱ��־
	fgSwitch = FALSE; // Ĭ��δ������֡����

	fgCanBusUse = 0xAA; // ��� ��ʼĬ��A���� - CAN
	fgMuBusUse = 0xAA;	// �����λ�� ��ʼĬ��A���� - 422

	onoffCount = 0;	 // ָ��ִ�м��� ����
	onoffCountB = 0; // ָ��ִ�м��� ���� B
	onoffCountC = 0; // ָ��ִ�м��� ���� C

	canErrorCount1 = 0; // CAN���ߴ����������1
	canErrorCount2 = 0; // CAN���ߴ����������2

	wrOnoffIndex = 0;		 // ָ��д���� ONOFF
	rdOnoffIndex = 0;		 // ָ������� ONOFF
	for (i = 0; i < 48; i++) // ָ�������
	{
		for (j = 0; j < 4; j++)
		{
			onoffBuffer[i][j] = 0x00; // ��������
		}
	}

	for (i = 0; i < 8; i++) // ����GPSУʱ��������
	{
		timeSMUBcData[i] = 0x00;
		timeGPSBcData[i] = 0x00;
	}

	for (i = 0; i < 8; i++) // ���¿��²���  ����
	{
		wk_parameter[i].Mode = wk_CODEparameter[i].Mode;
		wk_parameter[i].WD = wk_CODEparameter[i].WD;
		wk_parameterB[i].Mode = wk_CODEparameter[i].Mode;
		wk_parameterB[i].WD = wk_CODEparameter[i].WD;
		wk_parameterC[i].Mode = wk_CODEparameter[i].Mode;
		wk_parameterC[i].WD = wk_CODEparameter[i].WD;
	}

	wk_period = 0;
	for (i = 0; i < 8; i++)
	{
		wk_time[i].Now = 0;
		wk_time[i].Last = 0;
		wk_tLast[i] = 0x07F8; // ?????  ��;
	}
}

/***************************************************************************************************/
void GPIO_Init(void)
{
	MSS_GPIO_init();
	MSS_GPIO_config(MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE); // �����ʽ

	MSS_GPIO_config(MSS_GPIO_9, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW);	// �͵�ƽ�жϷ�ʽ
	MSS_GPIO_config(MSS_GPIO_11, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW); // �͵�ƽ�жϷ�ʽ

	MSS_GPIO_clear_irq(MSS_GPIO_9);
	MSS_GPIO_clear_irq(MSS_GPIO_11);
}

/***************************************************************************************************/
void RXD422_Init(void)
{
	RXD422_Ctrl_InitialA = 0x88; // 422
	RXD422_Ctrl_InitialB = 0x88;

	RXD422_BTRSetA = 0x015B; // 115200
	RXD422_BTRSetB = 0x015B;
}

/***************************************************************************************************/
void CORECanInit_BASIC(uint32_t address)
{
	uint32_t delay;
	uint16_t temp;

	*((volatile unsigned short *)(address + (0x20 << 4))) = 0x84; //

	for (delay = 0; delay < 10000; delay++)
		; //

	*((volatile unsigned short *)(address + (0x26 << 4))) = 0x03; // prescaler register     500K - 40M OR 16M ???
	*((volatile unsigned short *)(address + (0x24 << 4))) = 0xF4; // bit timing register
	*((volatile unsigned short *)(address + (0x25 << 4))) = 0x1F; // additional register

	*((volatile unsigned short *)(address + (0x29 << 4))) = 0x00; // A1C28~A1C21		28 ~ 18  ->   ID10 ~ ID0   ��������Ĵ���
	*((volatile unsigned short *)(address + (0x2A << 4))) = 0xFF; // A1C20~A1C13

	*((volatile unsigned short *)(address + (0x2D << 4))) = 0x00; // A1M28~A1M21
	*((volatile unsigned short *)(address + (0x2E << 4))) = 0xFF; // A1M20~A1M13

	*((volatile unsigned short *)(address + (0x30 << 4))) = 0x00; // A2C28~A2C21
	*((volatile unsigned short *)(address + (0x31 << 4))) = 0xFF; // A2C20~A2C13

	*((volatile unsigned short *)(address + (0x34 << 4))) = 0x00; // A1M28~A1M21
	*((volatile unsigned short *)(address + (0x35 << 4))) = 0xFF; // A1M20~A1M13

	*((volatile unsigned short *)(address + (0x38 << 4))) = 0x00; // A1M28~A1M21
	*((volatile unsigned short *)(address + (0x39 << 4))) = 0xFF; // A1M20~A1M13

	*((volatile unsigned short *)(address + (0x3C << 4))) = 0x00; // A1M28~A1M21
	*((volatile unsigned short *)(address + (0x3D << 4))) = 0xFF; // A1M20~A1M13

	*((volatile unsigned short *)(address + (0x20 << 4))) = 0x44; // CR �����ж�ʹ�ܡ�����MODE��ֹ
	for (delay = 0; delay < 10000; delay++)
		;
	*((volatile unsigned short *)(address + (0x20 << 4))) = 0x44; // CR �����ж�ʹ�ܡ�����MODE��ֹ
	for (delay = 0; delay < 10000; delay++)
		;

	temp = *((volatile unsigned short *)(address + (0x22 << 4))); // SR
	temp = *((volatile unsigned short *)(address + (0x23 << 4))); // IR

	// *((volatile unsigned short *)(address + (0x21 << 4))) |= 0x10;
}

/***************************************************************************************************/
void CanReceiveA(void)
{
	uint8_t id10_3;
	uint8_t id2_0;
	uint8_t dlc;

	id10_3 = *((volatile unsigned short *)(CAN_A_ADDRESS));							 // ID10~ID3
	id2_0 = *((volatile unsigned short *)(CAN_A_ADDRESS + (0x01 << 4))) >> 5;		 // ID2~ID0
	dlc = (*((volatile unsigned short *)(CAN_A_ADDRESS + (0x01 << 4))) >> 1) & 0x0F; // DLC

	timeSMUBcDataCanReceiveA(id10_3);		// ����Уʱ�����㲥
	timeGPSBcDataCanReceiveA(id10_3);		// GPSУʱ�����㲥
	GeneralCanReceiveA(id10_3, id2_0, dlc); // һ�����ݽ��մ���
}

/***************************************************************************************************/
void timeSMUBcDataCanReceiveA(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC2) // ����Уʱ�㲥ID
	{
		sum = 0;
		for (i = 0; i < 8; i++) // ����Լ����ȡ��Ч����
		{
			buffer[i] = *((volatile unsigned short *)(CAN_A_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 7; i++) // ����У���
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0x80) // У����ȷ  TITLE��ȷ
		{

			for (i = 0; i < 6; i++) // ��ȡ��Ч6�ֽ�
			{
				timeSMUBcData[i] = buffer[1 + i];
			}
			fgAssistDataRenew = TRUE; // ���ø������ݸ��±�־
		}
	}
}

/***************************************************************************************************/
void timeGPSBcDataCanReceiveA(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC1) // GPSУʱ�㲥ID
	{
		sum = 0;
		for (i = 0; i < 7; i++) // ����Լ����ȡ��Ч����
		{
			buffer[i] = *((volatile unsigned short *)(CAN_A_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 6; i++) // ����У���
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0xEA) // У����ȷ  TITLE��ȷ
		{
			for (i = 0; i < 5; i++) // ��ȡ��Ч6�ֽ�
			{
				timeGPSBcData[i] = buffer[1 + i];
			}

			fgAssistDataRenew = TRUE; // ���ø������ݸ��±�־
		}
	}
}

/***************************************************************************************************/
void GeneralCanReceiveA(uint8_t id1, uint8_t id2, uint8_t dlc)
{
	uint8_t i;
	uint8_t sum;
	uint8_t length;
	uint8_t buffer[8];
	uint16_t idTitle;

	if ((id1 == ID_BC3) || (id1 == ID_BC4) || (id1 == ID_BC5) || (id1 == ID_BC6) || (id1 == ID_BC7) || (id1 == ID_MU1) || (id1 == ID_MU2)) // ID��Ч���ж�
	{
		if ((id2 & 0x20) == 0x00)
		{
			sum = 0;
			for (i = 0; i < dlc; i++) // ����Լ����ȡ��Ч����
			{
				buffer[i] = *((volatile unsigned short *)(CAN_A_ADDRESS + ((0x02 + i) << 4))); // ��ȡ��Ч����
			}

			for (i = 0; i < (dlc - 1); i++) // ����У���
			{
				sum = sum + buffer[i];
			}

			if (sum == buffer[dlc - 1]) // У����ȷ
			{
				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x01)) // �ٱ�ң��ָ��
				{
					fgMuBusUse = 0xAA; // A����
					CanSendRs1A();	   // �ٱ�ң��Ӧ��
				}

				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x02)) // �����ң��ָ��
				{
					fgMuBusUse = 0xAA; // A����
					CanSendRs2A();	   // ����ң��Ӧ��
				}

				if ((id1 == 0x4B) && (buffer[0] == 0x20)) // ���ָ��
				{
					fgMuBusUse = 0xAA;	// A����
					CanSendOnoffAckA(); // ���ָ��Ӧ��

					onoffBuffer[wrOnoffIndex][0] = buffer[1]; // ��ȡ��Чָ����1
					onoffBuffer[wrOnoffIndex][1] = buffer[2]; // ��ȡ��Чָ����2
					onoffBuffer[wrOnoffIndex][2] = buffer[3]; // ��ȡ��Чָ����3
					onoffBuffer[wrOnoffIndex][3] = buffer[4]; // ��ȡ��Чָ����4

					wrOnoffIndex++;
					if (wrOnoffIndex >= 48) // ָ��� 48��
					{
						wrOnoffIndex = 0;
					}
				}
			}
		}
		else
		{
			if (dlc < 8)
			{
				if (canReceiveFrameA == buffer[0]) // ֡����ж�
				{
					for (i = 0; i < dlc; i++) // ����֡
					{
						canReceiveBufferA[canReceiveCountA] = buffer[1 + i]; // ��ȡ��Ч����
						canReceiveCountA++;									 // ���ռ�������
					}

					if ((canReceiveCountA - 3) == canReceiveBufferA[0]) // ���ݽ��ճ����ж�
					{
						for (i = 0; i < (canReceiveCountA - 2); i++) // ���ƽ��ճ��� -1  (length /sum )
						{
							sum = sum + canReceiveBufferA[1]; // ����У��
						}

						if (sum == canReceiveBufferA[canReceiveCountA - 1]) // У����ȷ
						{
							idTitle = id1 * 256 + canReceiveBufferA[1]; // ID+TITLEѡ����
							switch (idTitle)							// ѡ���ж�
							{
							case 0x4B40: // ����������ݿ�
								CanSendDataAckA();

								switch (canReceiveBufferA[0]) // ���ݽ������ݳ����ж�
								{
								case 0x09: //  9 Byte ���ݳ���
									for (i = 0; i < 9; i++)
									{
										muData9[i] = canReceiveBufferA[i + 2];
									}
									break;

								case 0x39: //  57 Byte ���ݳ���
									for (i = 0; i < 57; i++)
									{
										muData57[i] = canReceiveBufferA[i + 2];
									}
									break;

								case 0x79: //  121 Byte ���ݳ���
									for (i = 0; i < 121; i++)
									{
										muData121[i] = canReceiveBufferA[i + 2];
									}
									break;

								case 0xF9: // 249 Byte ����ȡ
									for (i = 0; i < 249; i++)
									{
										muData249[i] = canReceiveBufferA[i + 2];
									}
									break;

								default:
									break;
								}
								break;

							case 0xAF6A: // GPS��λ�㲥����
								for (i = 0; i < 99; i++)
								{
									gpsLocateBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // ���ø������ݸ��±�־
								break;

							case 0xAF64: //  �˿ع㲥����
								for (i = 0; i < 100; i++)
								{
									gpsPoiseBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // ���ø������ݸ��±�־
								break;

							case 0x9FA4: // �˿������㲥
								for (i = 0; i < 233; i++)
								{
									starBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // ���ø������ݸ��±�־
								break;

							case 0x8FC4: // �˿����ݹ㲥
								for (i = 0; i < 104; i++)
								{
									topBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // ���ø������ݸ��±�־
								break;

							case 0x8F04: // �����ڹ㲥 - 04�˿�
								for (i = 0; i < 168; i++)
								{
									periodBcData[i] = canReceiveBufferA[i + 2];
								}
								break;

							default:
								break;
							}
						}
					}
				}

				canReceiveCountA = 0;
				canReceiveFrameA = 0;
			}
			else
			{
				if (buffer[0] == 0x00) // ��֡��������
				{
					canReceiveCountA = 0;
					canReceiveFrameA = 0;
				}

				if (canReceiveFrameA == buffer[0]) // ֡����ж�
				{
					for (i = 0; i < 7; i++) // ��ʼ֡  �м�֡
					{
						canReceiveBufferA[canReceiveCountA] = buffer[1 + i]; // ��ȡ��Ч����
						canReceiveCountA++;									 // ���ռ�������
					}
				}
			}
		}
	}
}

/***************************************************************************************************/
void CanSendRs1A(void)
{
	uint8_t i;
	uint8_t j;
	uint16_t delay;

	if (fgSwitch == FALSE) // B�����ݷ���
	{
		for (j = 0; j < 18; j++) // �ٱ������18֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A�����ݷ���
	{
		for (j = 0; j < 18; j++) // �ٱ������18֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
}

/***************************************************************************************************/
void CanSendRs2A(void)
{
	uint8_t i;
	uint8_t j;
	uint16_t delay;

	if (fgSwitch == FALSE) // B�����ݷ���
	{
		for (j = 0; j < 11; j++) // ���������11֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A�����ݷ���
	{
		for (j = 0; j < 11; j++) // �ٱ������11֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
}

/***************************************************************************************************/
void CanSendOnoffAckA(void)
{
	uint16_t delay;

	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // ָ��Ӧ����
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x20;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x26;

	*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // ��������

	delay = 0; // �жϷ������
	while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void CanSendDataAckA(void)
{
	uint16_t delay;

	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // ���ݿ�Ӧ����
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x40;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x46;

	*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // ��������

	delay = 0; // �жϷ������
	while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void CanReceiveB(void)
{
	uint8_t id10_3;
	uint8_t id2_0;
	uint8_t dlc;

	id10_3 = *((volatile unsigned short *)(CAN_B_ADDRESS));							 // ID10~ID3
	id2_0 = *((volatile unsigned short *)(CAN_B_ADDRESS + (0x01 << 4))) >> 5;		 // ID2~ID0
	dlc = (*((volatile unsigned short *)(CAN_B_ADDRESS + (0x01 << 4))) >> 1) & 0x0F; // DLC

	timeSMUBcDataCanReceiveB(id10_3);		// ����Уʱ�����㲥
	timeGPSBcDataCanReceiveB(id10_3);		// GPSУʱ�����㲥
	GeneralCanReceiveB(id10_3, id2_0, dlc); // һ�����ݽ��մ���
}

/***************************************************************************************************/
void timeSMUBcDataCanReceiveB(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC2) // ����Уʱ�㲥ID
	{
		sum = 0;
		for (i = 0; i < 8; i++) // ����Լ����ȡ��Ч����
		{
			buffer[i] = *((volatile unsigned short *)(CAN_B_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 7; i++) // ����У���
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0x80) // У����ȷ  TITLE��ȷ
		{
			for (i = 0; i < 6; i++) // ��ȡ��Ч6�ֽ�
			{
				timeSMUBcData[i] = buffer[1 + i];
			}
		}
	}
}

/***************************************************************************************************/
void timeGPSBcDataCanReceiveB(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC1) // GPSУʱ�㲥ID
	{
		sum = 0;
		for (i = 0; i < 7; i++) // ����Լ����ȡ��Ч����
		{
			buffer[i] = *((volatile unsigned short *)(CAN_B_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 6; i++) // ����У���
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0xEA) // У����ȷ  TITLE��ȷ
		{
			for (i = 0; i < 5; i++) // ��ȡ��Ч6�ֽ�
			{
				timeGPSBcData[i] = buffer[1 + i];
			}
		}
	}
}

/***************************************************************************************************/
void GeneralCanReceiveB(uint8_t id1, uint8_t id2, uint8_t dlc)
{
	uint8_t i;
	uint8_t sum;
	uint8_t length;
	uint8_t buffer[8];
	uint16_t idTitle;

	if ((id1 == ID_BC3) || (id1 == ID_BC4) || (id1 == ID_BC5) || (id1 == ID_BC6) || (id1 == ID_BC7) || (id1 == ID_MU1) || (id1 == ID_MU2)) // ID��Ч���ж�
	{
		if ((id2 & 0x20) == 0x00)
		{
			sum = 0;
			for (i = 0; i < dlc; i++) // ����Լ����ȡ��Ч����
			{
				buffer[i] = *((volatile unsigned short *)(CAN_B_ADDRESS + ((0x02 + i) << 4))); // ��ȡ��Ч����
			}

			for (i = 0; i < (dlc - 1); i++) // ����У���
			{
				sum = sum + buffer[i];
			}

			if (sum == buffer[dlc - 1]) // У����ȷ
			{
				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x01)) // �ٱ�ң��ָ��
				{
					CanSendRs1B(); // �ٱ�ң��Ӧ��
				}

				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x02)) // �����ң��ָ��
				{
					CanSendRs2B(); // ����ң��Ӧ��
				}

				if ((id1 == 0x4B) && (buffer[0] == 0x20)) // ���ָ��
				{
					CanSendOnoffAckB(); // ���ָ��Ӧ��

					onoffBuffer[wrOnoffIndex][0] = buffer[1]; // ��ȡ��Чָ����1
					onoffBuffer[wrOnoffIndex][1] = buffer[2]; // ��ȡ��Чָ����2
					onoffBuffer[wrOnoffIndex][2] = buffer[3]; // ��ȡ��Чָ����3
					onoffBuffer[wrOnoffIndex][3] = buffer[4]; // ��ȡ��Чָ����4

					wrOnoffIndex++;
					if (wrOnoffIndex >= 48) // ָ��� 48��
					{
						wrOnoffIndex = 0;
					}
				}
			}
		}
		else
		{
			if (dlc < 8)
			{
				if (canReceiveFrameB == buffer[0]) // ֡����ж�
				{
					for (i = 0; i < dlc; i++) // ����֡
					{
						canReceiveBufferB[canReceiveCountB] = buffer[1 + i]; // ��ȡ��Ч����
						canReceiveCountB++;									 // ���ռ�������
					}

					if ((canReceiveCountB - 3) == canReceiveBufferB[0]) // ���ݽ��ճ����ж�
					{
						for (i = 0; i < (canReceiveCountB - 2); i++) // ���ƽ��ճ��� -1  (length /sum )
						{
							sum = sum + canReceiveBufferB[1]; // ����У��
						}

						if (sum == canReceiveBufferB[canReceiveCountB - 1]) // У����ȷ
						{
							idTitle = id1 * 256 + canReceiveBufferB[1]; // ID+TITLEѡ����
							switch (idTitle)							// ѡ���ж�
							{
							case 0x4B40: // ����������ݿ�
								CanSendDataAckB();
								break;

							case 0xAF6A: // GPS��λ�㲥����
								break;

							case 0xAF64: //  �˿ع㲥����
								break;

							case 0x9FA4: // �˿������㲥
								break;

							case 0x8F68: // �˿����ݹ㲥
								break;

							case 0x8F04: // �����ڹ㲥 - 04�˿�
								break;

							default:
								break;
							}
						}
					}
				}

				canReceiveCountB = 0;
				canReceiveFrameB = 0;
			}
			else
			{
				if (buffer[0] == 0x00) // ��֡��������
				{
					canReceiveCountB = 0;
					canReceiveFrameB = 0;
				}

				if (canReceiveFrameB == buffer[0]) // ֡����ж�
				{
					for (i = 0; i < 7; i++) // ��ʼ֡  �м�֡
					{
						canReceiveBufferB[canReceiveCountB] = buffer[1 + i]; // ��ȡ��Ч����
						canReceiveCountB++;									 // ���ռ�������
					}
				}
			}
		}
	}
}

/***************************************************************************************************/
void CanSendRs1B(void)
{
	uint8_t i;
	uint8_t j;
	uint16_t delay;

	if (fgSwitch == FALSE) // B�����ݷ���
	{
		for (j = 0; j < 18; j++) // �ٱ������18֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A�����ݷ���
	{
		for (j = 0; j < 18; j++) // �ٱ������18֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
}

/***************************************************************************************************/
void CanSendRs2B(void)
{
	uint8_t i;
	uint8_t j;
	uint16_t delay;

	if (fgSwitch == FALSE) // B�����ݷ���
	{
		for (j = 0; j < 11; j++) // ���������11֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A�����ݷ���
	{
		for (j = 0; j < 11; j++) // �ٱ������11֡
		{
			for (i = 0; i < 10; i++) // д�뷢�ͻ�����
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // ��������

			delay = 0; // �жϷ������
			while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
}

/***************************************************************************************************/
void CanSendOnoffAckB(void)
{
	uint16_t delay;

	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // ָ��Ӧ����
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x20;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x26;

	*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // ��������

	delay = 0; // �жϷ������
	while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void CanSendDataAckB(void)
{
	uint16_t delay;

	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // ���ݿ�Ӧ����
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x40;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x46;

	*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // ��������

	delay = 0; // �жϷ������
	while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void GPIO9_IRQHandler(void) // CAN-A �����жϷ������
{
	uint8_t ir; // SJA1000 ״̬�Ĵ���

	ir = *((volatile unsigned short *)(CAN_A_ADDRESS + (0x23 << 4))); // ��ȡ�жϼĴ���  23h

	MSS_GPIO_clear_irq(MSS_GPIO_9);

	if ((ir & 0x80) == 0x80) // �����жϲ���
	{
		CanReceiveA(); // CAN RXD ISR  ���߽��մ���
	}

	*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x10; // ��ջ�����  Bit4 -- RRB
}

/***************************************************************************************************/
void GPIO11_IRQHandler(void) // CAN-B �����жϷ������
{
	uint8_t ir; // SJA1000 ״̬�Ĵ���

	ir = *((volatile unsigned short *)(CAN_B_ADDRESS + (0x23 << 4))); // ��ȡ�жϼĴ���  23h
	MSS_GPIO_clear_irq(MSS_GPIO_11);

	if ((ir & 0x80) == 0x80) // �����жϲ���
	{
		CanReceiveB(); // CAN RXD ISR  ���߽��մ���
	}

	*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x10; // ��ջ�����  Bit4 -- RRB
}

/***************************************************************************************************/
void WatchDog(void)
{
	uint32_t delay;

	MSS_GPIO_set_output(MSS_GPIO_0, 1); // �ߵ�ƽ
	for (delay = 0; delay < 1000; delay++)
		;								// ��ʱ
	MSS_GPIO_set_output(MSS_GPIO_0, 0); // �͵�ƽ
	for (delay = 0; delay < 1000; delay++)
		;								// �ϵ���ʱ �ȴ�  ���Թ�ҧʱ�临λ   Լ260us
	MSS_GPIO_set_output(MSS_GPIO_0, 1); // �ߵ�ƽ
}

/***************************************************************************************************/
void Timer1Init(void)
{
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE); // 40ms ��ʱ��ʼ��
	MSS_TIM1_load_immediate(640000);
	MSS_TIM1_start();
	MSS_TIM1_enable_irq();
}

/***************************************************************************************************/
void Timer1_IRQHandler(void) // 1s��ʱ�жϷ�����
{
	fg40ms = TRUE;
	MSS_TIM1_stop();

	MSS_TIM1_clear_irq();
	MSS_TIM1_load_immediate(640000); // 1600000000  ->  1s    40ms --> 640000
	MSS_TIM1_start();
}

/***************************************************************************************************/
void DataPortect(void)
{
	uint8_t result1;
	uint8_t result2;
	uint8_t result3;

	// result1 = Get2_3_I(&heatOpenValue[0], &heatOpenValueB[0], &heatOpenValueC[0]);
	// result2 = Get2_3_I(&heatCloseValue[0], &heatCloseValueB[0], &heatCloseValueC[0]);
	// result3 = Get2_3_U(&heatMode[0], &heatModeB[0], &heatModeC[0]);
	// if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE)) // ���޳����쳣  ģʽ
	// {
	// 	heatMode[0] = 0x51; // �ȿؿ���ģʽ       ����ѡ��1
	// 	heatModeB[0] = 0x51;
	// 	heatModeC[0] = 0x51;

	// 	heatOpenValue[0] = 909;	 // �ȿؿ��ƿ�����  		15��C - 18��C
	// 	heatOpenValueB[0] = 909; // �ȿؿ��ƿ�����B
	// 	heatOpenValueC[0] = 909; // �ȿؿ��ƿ�����B

	// 	heatCloseValue[0] = 836;  // �ȿؿ��ƹ�����
	// 	heatCloseValueB[0] = 836; // �ȿؿ��ƹ�����C
	// 	heatCloseValueC[0] = 836; // �ȿؿ��ƹ�����C
	// }

	// result1 = Get2_3_I(&heatOpenValue[1], &heatOpenValueB[1], &heatOpenValueC[1]);
	// result2 = Get2_3_I(&heatCloseValue[1], &heatCloseValueB[1], &heatCloseValueC[1]);
	// result3 = Get2_3_U(&heatMode[1], &heatModeB[1], &heatModeC[1]);
	// if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE)) // ���޳����쳣  ģʽ
	// {
	// 	heatMode[1] = 0x52; // �ȿؿ���ģʽ       ����ѡ��2
	// 	heatModeB[1] = 0x52;
	// 	heatModeC[1] = 0x52;

	// 	heatOpenValue[1] = 909;	 // �ȿؿ��ƿ�����  		15��C - 18��C
	// 	heatOpenValueB[1] = 909; // �ȿؿ��ƿ�����B
	// 	heatOpenValueC[1] = 909; // �ȿؿ��ƿ�����B

	// 	heatCloseValue[1] = 836;  // �ȿؿ��ƹ�����
	// 	heatCloseValueB[1] = 836; // �ȿؿ��ƹ�����C
	// 	heatCloseValueC[1] = 836; // �ȿؿ��ƹ�����C
	// }

	// result1 = Get2_3_U(&heatEn, &heatEnB, &heatEnC); // ����ʹ���о�
	// if (result1 == FALSE)
	// {
	// 	heatEn = TRUE;	// �ָ�Ĭ��ֵ  ����ʹ��
	// 	heatEnB = TRUE; // �ָ�Ĭ��ֵ  ����ʹ�� B
	// 	heatEnC = TRUE; // �ָ�Ĭ��ֵ  ����ʹ�� C
	// }

	result1 = Get2_3_U(&onoffCount, &onoffCountB, &onoffCountC); // ָ������о�
	if (result1 == FALSE)
	{
		onoffCount = 0;	 // �ָ�Ĭ������
		onoffCountB = 0; // �ָ�Ĭ������ B
		onoffCountC = 0; // �ָ�Ĭ������ C
	}
}

/***************************************************************************************************/
uint8_t Get2_3_F(float *a, float *b, float *c)
{
	float temp1;
	float temp2;
	float temp3;

	temp1 = fabs(*a - *b);
	temp2 = fabs(*a - *c);
	temp3 = fabs(*b - *c);

	if ((temp1 < 0.0001) && (temp2 < 0.0001) && (temp3 < 0.0001)) //	���߲�ֵ�Ƚ�
	{
		return (TRUE); //  ������ȷ
	}
	else
	{
		if (temp1 < 0.0001) //  ������ȸ�ֵ����
		{
			*c = *a;
			return (TRUE); //
		}
		else
		{
			if (temp2 < 0.0001)
			{
				*b = *a;
				return (TRUE);
			}
			else
			{
				if (temp3 < 0.0001)
				{
					*a = *b;
					return (TRUE);
				}
				else //
				{
					return (FALSE); //
				}
			}
		}
	}
}

/***************************************************************************************************/
uint8_t Get2_3_I(uint16_t *a, uint16_t *b, uint16_t *c)
{
	if ((*a == *b) && (*b == *c)) /* ǰ��������� */
	{
		return (TRUE); /* 3ȡ2�ж���ȷ */
	}
	else if (*a == *b)
	{
		*c = *b;	   /* ��ֵ���� */
		return (TRUE); /* 3ȡ2�ж���ȷ */
	}
	else
	{
		if (*a == *c) /* ����������� */
		{
			*b = *a;	   /* ��ֵ���� */
			return (TRUE); /* 3ȡ2�ж���ȷ */
		}
		else
		{
			if (*b == *c) /* ʣ����������� */
			{
				*a = *b;	   /* ��ֵ���� */
				return (TRUE); /* 3ȡ2�ж���ȷ */
			}
			else
			{
				return (FALSE); /* 3ȡ2�ж��쳣 */
			}
		}
	}
}

/***************************************************************************************************/
uint8_t Get2_3_U(uint8_t *a, uint8_t *b, uint8_t *c)
{
	if ((*a == *b) && (*b == *c)) /* ǰ��������� */
	{
		return (TRUE); /* 3ȡ2�ж���ȷ */
	}
	else if (*a == *b)
	{
		*c = *b;	   /* ��ֵ���� */
		return (TRUE); /* 3ȡ2�ж���ȷ */
	}
	else
	{
		if (*a == *c) /* ����������� */
		{
			*b = *a;	   /* ��ֵ���� */
			return (TRUE); /* 3ȡ2�ж���ȷ */
		}
		else
		{
			if (*b == *c) /* ʣ����������� */
			{
				*a = *b;	   /* ��ֵ���� */
				return (TRUE); /* 3ȡ2�ж���ȷ */
			}
			else
			{
				return (FALSE); /* 3ȡ2�ж��쳣 */
			}
		}
	}
}
