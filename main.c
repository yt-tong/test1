/*******************************************************************************
 * 24B相机下位机软件
 *
 * debug1.00  20201218  1207
 * 软件构架
 * 
 * debug1.01-1.02  20201222  1207 home
 * CAN IP 通信部分设计实现
 *
 * debug1.03  20201222  1207
 * 热控部分功能实现
 *
*******************************************************************************/

#include "math.h"
#include ".\drivers\mss_gpio\mss_gpio.h"
#include ".\drivers\mss_timer\mss_timer.h"

#define TRUE 0xAA
#define FALSE 0x55

#define ID_BC1 0x3F // 广播1  GPS整秒对时广播
#define ID_BC2 0x6F // 广播2  星务时间广播
#define ID_BC3 0xAF // 广播3  GPS定位广播数据 自控广播数据
#define ID_BC4 0x9F // 广播4  姿控星敏广播
#define ID_BC5 0x8F // 广播5  姿控陀螺广播数  行周期广播
#define ID_BC6 0xA3 // 广播6  姿态四元数预报广播
#define ID_BC7 0x4D // 广播7  高速上行分发数据   ????
#define ID_MU1 0x8B // 自身ID1  遥测指令
#define ID_MU2 0x4B // 自身ID2  间接指令、数据块

#define WK_uLIMIT 0x0073 //
#define WK_lLIMIT 0x07CF //
#define WK_PERIOD 4		 // main_period*WK_PERIOD)

#define HEAT_ADDR *((volatile unsigned short *)0x301AAAA0)		// 热控地址
#define CAN_RESET_HARD *((volatile unsigned short *)0x301AB000) // CAN总线硬复位
#define RESET_COUNT *((volatile unsigned short *)0x3017F7F0)	// 热复位计数

#define RXD422_Ctrl_InitialA *((volatile unsigned short *)0x301AA110)	//422-A
#define RXD422_Buffer_ADDRESSA *((volatile unsigned short *)0x301AA000) //422
#define TXD422_Buffer_ADDRESSA *((volatile unsigned short *)0x301AA000) //422

#define RXD422_Ctrl_InitialB *((volatile unsigned short *)0x301BB110)	//422-B
#define RXD422_Buffer_ADDRESSB *((volatile unsigned short *)0x301BB000) //422
#define TXD422_Buffer_ADDRESSB *((volatile unsigned short *)0x301BB000) //422

#define RXD422_BTRSetA *((volatile unsigned short *)0x301AA440) //422
#define RXD422_BTRSetB *((volatile unsigned short *)0x301BB440) //422

#define VERSION 104 // 内部版本号

const uint32_t CAN_A_ADDRESS = 0x3019A000u; // CAN-A address
const uint32_t CAN_B_ADDRESS = 0x3019B000u; // CAN-B address

uint8_t fg40ms;	   // 运行周期 40ms
uint8_t count40ms; // 定时器计数 40ms

uint32_t timeS;	 // 系统时间 -- 秒
uint16_t timeMS; // 系统时间 -- 毫秒

uint8_t fgSwitch;		// 组帧标志
uint8_t canErrorCount1; // CAN无效通讯计数1
uint8_t canErrorCount2; // CAN无效通讯计数2

uint8_t resetCount;			// 热复位计数
uint8_t camOnoffTransCount; // 相机转发指令计数

uint16_t anData[30];	  // 模拟量采集
uint8_t stateWord[2];	  // 相机下位机状态字
uint8_t camPowerState;	  // 相机加电状态
uint8_t heatModeState[2]; // 控温模式字
uint8_t heatState;		  // 控温加热状态
uint8_t heatCheckState;	  // 控温加热自检状态
uint8_t camMotorState;	  // 电机运动状态

uint8_t camMotorLocate; // 电机位置信息
uint8_t taskNumber[4];	// 任务编号

uint8_t onoffBuffer[48][4]; // 指令缓存 48条 能力
uint8_t wrOnoffIndex;		// 指令写索引
uint8_t rdOnoffIndex;		// 指令读索引

uint8_t onoffCount;	 // 指令执行计数
uint8_t onoffCountB; // 指令执行计数 B
uint8_t onoffCountC; // 指令执行计数 C

uint8_t onoffRecCode;  // 指令上一条记录编号
uint8_t onoffRecCodeB; // 指令上一条记录编号 B
uint8_t onoffRecCodeC; // 指令上一条记录编号 C

uint8_t onoffErrorCount;  // 指令错误计数
uint8_t onoffErrorCountB; // 指令错误计数 B
uint8_t onoffErrorCountC; // 指令错误计数 C

uint8_t encoderOverTimeCount;  // 码盘超时计数
uint8_t encoderOverTimeCountB; // 码盘超时计数B
uint8_t encoderOverTimeCountC; // 码盘超时计数C

uint8_t encoderRsErrorCount;  // 码盘通信错误计数
uint8_t encoderRsErrorCountB; // 码盘通信错误计数B
uint8_t encoderRsErrorCountC; // 码盘通信错误计数C

uint8_t camPowerOverTimeCount;	// 相机上电超时计数
uint8_t camPowerOverTimeCountB; // 相机上电超时计数B
uint8_t camPowerOverTimeCountC; // 相机上电超时计数C

uint8_t canResetCountA;	  // CAN-A总线复位计数
uint8_t canResetCountA_B; // CAN-A总线复位计数 B
uint8_t canResetCountA_C; // CAN-A总线复位计数 C

uint8_t canResetCountB;	  // CAN-B总线复位计数
uint8_t canResetCountB_B; // CAN-A总线复位计数 B
uint8_t canResetCountB_C; // CAN-A总线复位计数 C

uint8_t canReceiveCountA;		// CAN总线接收计数  A总线
uint8_t canReceiveFrameA;		// CAN总线接收帧计数  A总线
uint8_t canReceiveBufferA[256]; // CAN总线接收缓存区  A总线

uint8_t canReceiveCountB;		// CAN总线接收计数  B总线
uint8_t canReceiveFrameB;		// CAN总线接收帧计数  B总线
uint8_t canReceiveBufferB[256]; // CAN总线接收缓存区  B总线

uint8_t muRsFrame1A[180]; // 相机单元速变遥测发送缓冲区	18帧 -A
uint8_t muRsFrame1B[180]; // 相机单元速变遥测发送缓冲区	18帧 -B
uint8_t muRsFrame2A[110]; // 相机单元缓变遥测发送缓冲区	11帧 -A
uint8_t muRsFrame2B[110]; // 相机单元缓变遥测发送缓冲区	11帧 -B

uint8_t muData9[9];		// 相机单元数据块9字节
uint8_t muData57[57];	// 相机单元数据块57字节
uint8_t muData121[121]; // 相机单元数据块121字节
uint8_t muData249[249]; // 相机单元数据块249字节

uint8_t timeSMUBcData[8];	 // 星务广播时间缓存
uint8_t timeGPSBcData[8];	 // GPS广播时间缓存
uint8_t gpsLocateBcData[99]; // GPS定位广播数据缓存
uint8_t gpsPoiseBcData[100]; // GPS姿态广播数据缓存
uint8_t starBcData[233];	 // 姿控星敏广播数据缓存
uint8_t topBcData[104];		 // 姿控陀螺广播数据缓存
uint8_t periodBcData[168];	 // 行周期广播数据缓存
uint8_t muAssistData[48];	 // 相机辅助数据缓存

uint8_t fgAssistDataRenew;		   // 辅助数据更新标志
uint8_t assistDataSendBuffer[600]; // 辅助数据发送缓存

uint8_t fgMuBusUse;	 // 相机 CAN总线使用标志
uint8_t fgCanBusUse; // 相机 422总线使用标志

uint8_t cam1RsData[43]; // 相机1遥测数据缓存
uint8_t cam2RsData[43]; // 相机2遥测数据缓存
uint8_t cam3RsData[43]; // 相机3遥测数据缓存

uint8_t cam1RsErrorCount; // 相机1遥测错误计数
uint8_t cam2RsErrorCount; // 相机2遥测错误计数
uint8_t cam3RsErrorCount; // 相机3遥测错误计数

struct _WK_Parameter
{
	uint8_t Mode; // 控温模式
	uint16_t WD;  // 控温点
};

const struct _WK_Parameter wk_CODEparameter[8] = //
	{
		//
		{0x02, 0x68C}, // No.0: 控温回路默认值
		{0x02, 0x68C}, // No.1: 控温回路默认值
		{0x02, 0x68C}, // No.2: 控温回路默认值
		{0x02, 0x68C}, // No.3: 控温回路默认值
		{0x02, 0x68C}, // No.4: 控温回路默认值
		{0x02, 0x68C}, // No.5: 控温回路默认值
		{0x02, 0x68C}, // No.6: 控温回路默认值
		{0x02, 0x68C}, // No.7: 控温回路默认值
};

struct _WK_Parameter wk_parameter[8];  // 控温参数三区 - A
struct _WK_Parameter wk_parameterB[8]; // - B
struct _WK_Parameter wk_parameterC[8]; // - C

uint8_t WKS_Buffer; // 控温状态字
uint8_t wk_period;	// 控温周期变量

struct _WK_Time
{
	uint8_t Last; //
	uint8_t Now;  //
};
struct _WK_Time wk_time[8]; // 控温参数定义

uint16_t wk_tLast[8]; // 控温时间参数
uint16_t wk_rData[8];

void SysInit(void);	   // 系统初始化函数
void GPIO_Init(void);  // GPIO初始化
void DataInit(void);   // 数据初始化
void Timer1Init(void); // 定时器1初始化
void WatchDog(void);   // 喂狗

void AnCollect(void);	   // 模拟量采集函数
void SaveData(void);	   // 组帧函数
void BuildStateWord(void); // 组帧函数
void OnoffHook(void);	   // 指令输出

void HeatControl(void);							   // 加热控制
void WK_OUTPUT(uint8_t index, uint8_t onoffstate); // 热控输出
void WK_Time14(uint8_t Index, uint32_t r);		   // 步进式控温模块

void CanResume(void); // CAN恢复处理

void DataPortect(void);									 // 数据保护模块
uint8_t Get2_3_F(float *a, float *b, float *c);			 // 浮点数3取2保护
uint8_t Get2_3_I(uint16_t *a, uint16_t *b, uint16_t *c); // 16bit3取2保护
uint8_t Get2_3_U(uint8_t *a, uint8_t *b, uint8_t *c);	 // 8bit3取2保护

void CORECanInit_BASIC(uint32_t address); // IP核CAN初始化

void CanReceiveA(void);											// SJA1000 CAN总线接收 -A
void timeGPSBcDataCanReceiveA(uint8_t id);						// GPS时间广播数据接收函数   A总线   多主
void timeSMUBcDataCanReceiveA(uint8_t id);						// 星务时间广播数据接收函数   A总线  多主
void GeneralCanReceiveA(uint8_t id1, uint8_t id2, uint8_t dlc); // 一般总线数据接收处理  A总线

void CanSendRs1A(void);		 // 速变遥测发送 - A总线
void CanSendRs2A(void);		 // 缓变遥测发送 - A总线
void CanSendOnoffAckA(void); // 间接指令应答 - A总线
void CanSendDataAckA(void);	 // 数据块指令应答 - A总线

void CanReceiveB(void);											// SJA1000 CAN总线接收 -B
void timeGPSBcDataCanReceiveB(uint8_t id);						// GPS时间广播数据接收函数   B总线   多主
void timeSMUBcDataCanReceiveB(uint8_t id);						// 星务时间广播数据接收函数   B总线  多主
void GeneralCanReceiveB(uint8_t id1, uint8_t id2, uint8_t dlc); // 一般总线数据接收处理 B总线

void CanSendRs1B(void);		 // 速变遥测发送 - B总线
void CanSendRs2B(void);		 // 缓变遥测发送 - B总线
void CanSendOnoffAckB(void); // 间接指令应答 - B总线
void CanSendDataAckB(void);	 // 数据块指令应答 - B总线

void Control(void);			// 控制流程模块
void AssistDataTrans(void); // 辅助数据转发模块
void AssistDataSave(void);	// 辅助数据组帧模块
void CamTask(void);			// 在轨拍照任务规划

void CamRsSend1(void); // 相机1轮询指令发送
void CamRsSend2(void); // 相机2轮询指令发送
void CamRsSend3(void); // 相机3轮询指令发送

void CamRsReceive1(void); // 相机1轮询数据接收
void CamRsReceive2(void); // 相机2轮询数据接收
void CamRsReceive3(void); // 相机3轮询数据接收

void RXD422_Init(void); // 422初始化

void GPIO9_IRQHandler(void);  // CAN-A GPIO中断
void GPIO11_IRQHandler(void); // CAN-B GPIO中断
void Timer1_IRQHandler(void); // 定时器1中断服务函数

/***************************************************************************************************/
int main()
{
	SysInit(); // 系统初始化

	while (1)
	{
		Control();	 // 周期功能切换
		OnoffHook(); // 指令处理

		do // 转发数据处理
		{
			//			TransData();		  // 转发数据、积分时间等
			CamTask();			  // 相机在轨任务规划
		} while (fg40ms != TRUE); // 40ms周期结束
		count40ms++;			  // 40ms 周期计数增加
		fg40ms = FALSE;

		WatchDog(); // 喂狗	40ms喂狗一次
	}
}
/****************************************************************************************************************************************/
void Control(void)
{
	//	unsigned int i;

	switch (count40ms) // 40ms 周期控制
	{
	case 1:			   // step 1 -->
		DataPortect(); // 数据保护模块
		break;

	case 2: // step 2 -->  相机1轮询指令发送
		CamRsSend1();
		break;

	case 3: // step 3 --> 相机1轮询数据接收（查询方式）
		CamRsReceive1();
		break;

	case 4: // step 4 -->  相机2轮询指令发送
		CamRsSend2();
		break;

	case 5: // step 5 --> 相机2轮询数据接收（查询方式）
		CamRsReceive2();
		break;

	case 6: // step 6 --> 相机3轮询指令发送
		CamRsSend3();
		break;

	case 7: // step 7 --> 相机3轮询数据接收（查询方式）
		CamRsReceive3();
		break;

	case 8: // step 8 --> 模拟量采集模块
		AnCollect();
		break;

	case 9: // step 9 --> 热控输出
		HeatControl();
		break;

	case 10: // step 10 -->

		break;

	case 11: // step 11 -->

		break;

	case 12:		 // step 12 -->
		CanResume(); // CAN总线容错模块
		break;

	case 13: // step 13 -->

		break;

	case 14:			  // step 14 -->
		BuildStateWord(); // 组状态字
		SaveData();		  // 组帧函数
		break;

	case 15: // step 15 --> 辅助数据组帧模块
		AssistDataSave();
		break;

	case 16: // step 16 -->	辅助数据转发   周期1s
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
		count40ms = 0; // 控制器周期复位

		break;

	default:
		count40ms = 0;		// 防止在轨异常翻转 安全性设计
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
	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		RXD422_Ctrl_InitialA = 0x48;   // 清除接收缓存区 ????
		TXD422_Buffer_ADDRESSA = 0xEB; // W0   帧头
		TXD422_Buffer_ADDRESSA = 0x90; // W1
		TXD422_Buffer_ADDRESSA = 0x01; // W2   通道号
		TXD422_Buffer_ADDRESSA = 0xEE; // W3   轮询命令
		TXD422_Buffer_ADDRESSA = 0xEF; // W4   SUM
	}
	else
	{
		RXD422_Ctrl_InitialB = 0x48;   // 清除接收缓存区 ????
		TXD422_Buffer_ADDRESSB = 0xEB; // W0   帧头
		TXD422_Buffer_ADDRESSB = 0x90; // W1
		TXD422_Buffer_ADDRESSB = 0x01; // W2   通道号
		TXD422_Buffer_ADDRESSB = 0xEE; // W3   轮询命令
		TXD422_Buffer_ADDRESSB = 0xEF; // W4   SUM
	}
}

/****************************************************************************************************************************************/
void CamRsReceive1(void)
{
	uint8_t i;
	uint8_t sum;

	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		for (i = 0; i < 43; i++) // 读取预期的返回数据
		{
			cam1RsData[i] = RXD422_Buffer_ADDRESSA;
		}
	}
	else
	{
		for (i = 0; i < 43; i++) // 读取预期的返回数据
		{
			cam1RsData[i] = RXD422_Buffer_ADDRESSB;
		}
	}

	for (i = 2; i < 42; i++) // 计算校验和
	{
		sum = sum + cam1RsData[i];
	}

	if (sum != cam1RsData[42]) // 校验和错误   遥测数据填充  0xE1
	{
		cam1RsErrorCount++;		 // 相机遥测错误计数+1
		for (i = 3; i < 42; i++) // 数据临时填充
		{
			cam1RsData[i] = 0xE1;
		}
	}
}

/****************************************************************************************************************************************/
void CamRsSend2(void)
{
	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		RXD422_Ctrl_InitialA = 0x48;   // 清除接收缓存区 ????
		TXD422_Buffer_ADDRESSA = 0xEB; // W0   帧头
		TXD422_Buffer_ADDRESSA = 0x90; // W1
		TXD422_Buffer_ADDRESSA = 0x02; // W2   通道号
		TXD422_Buffer_ADDRESSA = 0xEE; // W3   轮询命令
		TXD422_Buffer_ADDRESSA = 0xF0; // W4   SUM
	}
	else
	{
		RXD422_Ctrl_InitialB = 0x48;   // 清除接收缓存区 ????
		TXD422_Buffer_ADDRESSB = 0xEB; // W0   帧头
		TXD422_Buffer_ADDRESSB = 0x90; // W1
		TXD422_Buffer_ADDRESSB = 0x02; // W2   通道号
		TXD422_Buffer_ADDRESSB = 0xEE; // W3   轮询命令
		TXD422_Buffer_ADDRESSB = 0xF0; // W4   SUM
	}
}

/****************************************************************************************************************************************/
void CamRsReceive2(void)
{
	uint8_t i;
	uint8_t sum;

	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		for (i = 0; i < 43; i++) // 读取预期的返回数据
		{
			cam2RsData[i] = RXD422_Buffer_ADDRESSA;
		}
	}
	else
	{
		for (i = 0; i < 43; i++) // 读取预期的返回数据
		{
			cam2RsData[i] = RXD422_Buffer_ADDRESSB;
		}
	}

	for (i = 2; i < 42; i++) // 计算校验和
	{
		sum = sum + cam2RsData[i];
	}

	if (sum != cam2RsData[42]) // 校验和错误   遥测数据填充  0xE1
	{
		cam2RsErrorCount++;		 // 相机遥测错误计数+1
		for (i = 3; i < 42; i++) // 数据临时填充
		{
			cam2RsData[i] = 0xE1;
		}
	}
}

/****************************************************************************************************************************************/
void CamRsSend3(void)
{
	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		RXD422_Ctrl_InitialA = 0x48;   // 清除接收缓存区 ????
		TXD422_Buffer_ADDRESSA = 0xEB; // W0   帧头
		TXD422_Buffer_ADDRESSA = 0x90; // W1
		TXD422_Buffer_ADDRESSA = 0x03; // W2   通道号
		TXD422_Buffer_ADDRESSA = 0xEE; // W3   轮询命令
		TXD422_Buffer_ADDRESSA = 0xF1; // W4   SUM
	}
	else
	{
		RXD422_Ctrl_InitialB = 0x48;   // 清除接收缓存区 ????
		TXD422_Buffer_ADDRESSB = 0xEB; // W0   帧头
		TXD422_Buffer_ADDRESSB = 0x90; // W1
		TXD422_Buffer_ADDRESSB = 0x03; // W2   通道号
		TXD422_Buffer_ADDRESSB = 0xEE; // W3   轮询命令
		TXD422_Buffer_ADDRESSB = 0xF1; // W4   SUM
	}
}

/****************************************************************************************************************************************/
void CamRsReceive3(void)
{
	uint8_t i;
	uint8_t sum;

	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		for (i = 0; i < 43; i++) // 读取预期的返回数据
		{
			cam3RsData[i] = RXD422_Buffer_ADDRESSA;
		}
	}
	else
	{
		for (i = 0; i < 43; i++) // 读取预期的返回数据
		{
			cam3RsData[i] = RXD422_Buffer_ADDRESSB;
		}
	}

	for (i = 2; i < 42; i++) // 计算校验和
	{
		sum = sum + cam3RsData[i];
	}

	if (sum != cam3RsData[42]) // 校验和错误   遥测数据填充  0xE1
	{
		cam3RsErrorCount++;		 // 相机遥测错误计数+1
		for (i = 3; i < 42; i++) // 数据临时填充
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
		fgAssistDataRenew = FALSE;	  // 初始清除更新标志  中断数据保护设计
		sum = 0;					  // 累加和清零
		p = &assistDataSendBuffer[0]; // 更新指令变量初值

		*p = 0xEB; // W0 帧头  EB 90
		p++;
		*p = 0x90; // W1
		p++;
		*p = 0x07;		// W2 通道选择	 111全部
		sum = sum + *p; // 计算累加和
		p++;

		for (i = 0; i < 5; i++) // GPS整秒对时广播数据	5字节
		{
			*p = timeGPSBcData[i]; // 有效数据更新
			sum = sum + *p;		   // 计算累加和
			p++;
		}

		for (i = 0; i < 6; i++) // 星务时间广播数据	6字节
		{
			*p = timeSMUBcData[i]; // 有效数据更新
			sum = sum + *p;		   // 计算累加和
			p++;
		}

		for (i = 0; i < 99; i++) // GPS定位广播数据	99字节
		{
			*p = gpsLocateBcData[i]; // 有效数据更新
			sum = sum + *p;			 // 计算累加和
			p++;
		}

		for (i = 0; i < 100; i++) // 姿控姿态广播数据	100字节
		{
			*p = gpsPoiseBcData[i]; // 有效数据更新
			sum = sum + *p;			// 计算累加和
			p++;
		}

		for (i = 0; i < 233; i++) // 姿控星敏广播	233字节
		{
			*p = starBcData[i]; // 有效数据更新
			sum = sum + *p;		// 计算累加和
			p++;
		}

		for (i = 0; i < 104; i++) // 姿控陀螺广播数据	104字节
		{
			*p = topBcData[i]; // 有效数据更新
			sum = sum + *p;	   // 计算累加和
			p++;
		}

		for (i = 0; i < 48; i++) // 热控下位机辅助数据	48字节
		{
			*p = muAssistData[i]; // 有效数据更新
			sum = sum + *p;		  // 计算累加和
			p++;
		}

		*p = sum; // sum 更新

	} while (fgAssistDataRenew != FALSE); // 组帧过程中发生数据更新
}

/***************************************************************************************************/
void AssistDataTrans(void)
{
	uint16_t i;

	if (fgCanBusUse == 0xAA) // 相机总线标志
	{
		for (i = 0; i < 600; i++) // 辅助数据写入缓存区  A总线
		{
			TXD422_Buffer_ADDRESSA = assistDataSendBuffer[i];
		}
	}
	else
	{
		for (i = 0; i < 600; i++) // 辅助数据写入缓存区  B总线
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
		; // 上电延时 等待

	GPIO_Init();

	SYSREG->WDOG_CR = 0; // Turn off the watchdog

	for (i = 0; i < 50; i++)
	{
		WatchDog();
		for (delay = 0; delay < 5000; delay++)
			; // 上电延时 等待
	}

	// HEAT_ADDR = ( 1*256 + 0x0055 );											// 控温回路关闭

	DataInit(); // 数据初始化

	AnCollect(); // 遥测数据初始填充处理
	BuildStateWord();
	SaveData();
	SaveData();

	Timer1Init();  // 定时器初始化函数
	RXD422_Init(); // 422总线初始化

	CORECanInit_BASIC(CAN_A_ADDRESS); // CAN - A 初始化
	CORECanInit_BASIC(CAN_B_ADDRESS); // CAN - B 初始化		??? 清缓冲区操作必要性？？？

	MSS_GPIO_enable_irq(MSS_GPIO_9);  // CAN 总线中断--A  使能
	MSS_GPIO_enable_irq(MSS_GPIO_11); // CAN 总线中断--B  使能

	// CAN_RESET_HARD = 0xAA;													// 硬复位输出
	// for( delay=0; delay<5000; delay++ );									// 上电延时 等待
}

/***************************************************************************************************/
void AnCollect(void)
{
	anData[0] = *((volatile unsigned short *)(0x30199000 | (0 << 4))); // 测温点1温度值
	anData[1] = *((volatile unsigned short *)(0x30199000 | (1 << 4))); // 测温点2温度值
	anData[2] = *((volatile unsigned short *)(0x30199000 | (2 << 4))); // 测温点3温度值
	anData[3] = *((volatile unsigned short *)(0x30199000 | (3 << 4))); // 测温点4温度值
	anData[4] = *((volatile unsigned short *)(0x30199000 | (4 << 4))); // 测温点5温度值
	anData[5] = *((volatile unsigned short *)(0x30199000 | (5 << 4))); // 测温点6温度值
	anData[6] = *((volatile unsigned short *)(0x30199000 | (6 << 4))); // 测温点7温度值
	anData[7] = *((volatile unsigned short *)(0x30199000 | (7 << 4))); // 测温点8温度值

	anData[8] = *((volatile unsigned short *)(0x30199000 | (8 << 4)));	 // 步进电机电源12V遥测
	anData[9] = *((volatile unsigned short *)(0x30199000 | (9 << 4)));	 // 基准稳压源5V遥测
	anData[10] = *((volatile unsigned short *)(0x30199000 | (10 << 4))); // 成像电源12V遥测

	anData[11] = *((volatile unsigned short *)(0x30199000 | (11 << 4))); // 蓄电池组2温度
	anData[12] = *((volatile unsigned short *)(0x30199000 | (12 << 4))); // 太阳阵1温度
	anData[13] = *((volatile unsigned short *)(0x30199000 | (13 << 4))); // 太阳阵2温度
	anData[14] = *((volatile unsigned short *)(0x30199000 | (14 << 4))); // 放电开关状态
	anData[15] = *((volatile unsigned short *)(0x30199000 | (15 << 4))); // 蓄电池加热1状态
	anData[16] = *((volatile unsigned short *)(0x30199000 | (16 << 4))); // 蓄电池加热2状态
	anData[17] = *((volatile unsigned short *)(0x30199000 | (17 << 4))); // 动量轮开关状态
	anData[18] = *((volatile unsigned short *)(0x30199000 | (18 << 4))); // 帆板火刀开关状态
	anData[19] = *((volatile unsigned short *)(0x30199000 | (19 << 4))); // 离轨帆开关状态
	anData[20] = *((volatile unsigned short *)(0x30199000 | (20 << 4))); // X一体机开关状态
	anData[21] = *((volatile unsigned short *)(0x30199000 | (21 << 4))); // 视觉系统开关状态
	anData[22] = *((volatile unsigned short *)(0x30199000 | (22 << 4))); // 机械臂1开关状态
	anData[23] = *((volatile unsigned short *)(0x30199000 | (23 << 4))); // 机械臂2开关状态
	anData[24] = *((volatile unsigned short *)(0x30199000 | (24 << 4))); // +5V电源遥测
}

/***************************************************************************************************/
void BuildStateWord(void)
{
	stateWord[0] = 0; // 状态字清零
					  // bit7-6  BIT7~ BIT6	成像二次电状态	00B：主加电  01B：备加电  10B：主备加电  11B：主备断电

	// BIT5	当前相机下位机标识	0B：主份1B：备份
	// BIT4	CAN总线B状态	0B：正常1B：异常
	// BIT3	CAN总线A状态	0B：正常1B：异常

	if (fgMuBusUse == 0xAA) // BIT2	当前使用CAN总线	0B：A总线1B：B总线
	{
		stateWord[0] = (stateWord[0] | 0x04); // 1 A总线
	}
	else
	{
		stateWord[0] = (stateWord[0] & 0x7B); // 0 B总线
	}
	// BIT1~BIT0	指令接收执行状态  00B：正确、执行   01B：正确、不执行

	stateWord[1] = 0; // 状态字清零
					  // BIT7	高光时间转发状态	0B：使能1B：禁能
					  // BIT6	成像超时使能状态	0B：使能1B：禁能
					  // BIT5 ~ BIT4	下位机时间状态  00B：丢失GPS时间  01B：丢失硬件秒脉冲  10：校时  11：自身守时
					  // BIT3 ~ BIT2	任务执行状态  00B：执行  01B：未执行  02B：任务失败

	camPowerState = 0;
	// BIT7~ BIT3	保留
	// BIT2	第三路模组POWER ON/OFF状态：0-OFF，1-ON
	// BIT1	第二路模组POWER ON/OFF状态：0-OFF，1-ON
	// BIT0	第一路模组POWER ON/OFF状态：0-OFF，1-ON

	heatModeState[0] = 0;
	heatModeState[1] = 0;
	heatModeState[0] = ((wk_parameter[3].Mode & 0x03) << 6) | ((wk_parameter[2].Mode & 0x03) << 4) | ((wk_parameter[1].Mode & 0x03) << 2) | (wk_parameter[0].Mode & 0x03);
	heatModeState[1] = ((wk_parameter[7].Mode & 0x03) << 6) | ((wk_parameter[6].Mode & 0x03) << 4) | ((wk_parameter[5].Mode & 0x03) << 2) | (wk_parameter[4].Mode & 0x03);
}

/***************************************************************************************************/
void OnoffHook(void)
{
	uint8_t i;

	if (rdOnoffIndex != wrOnoffIndex) // 判读指令队列中数据
	{
		switch (onoffBuffer[rdOnoffIndex][0]) // 对应指令索引
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

		case 0x0D: // ID - 0D    热控参数上注指令

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0E: // ID - 0E    上注软件过放保护参数1（蓄电池软件过放保护-使能电压）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x0F: // ID - 0F    上注软件过放保护参数2（蓄电池软件过放保护-放电电流）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x10: // ID - 10    上注软件过放保护参数3（蓄电池软件过放-保护电压）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x11: // ID - 11    上注蓄电池组自主接入参数4（锂电池温度）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x12: // ID - 12    上注蓄电池组自主接入参数5（锂电池电压）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x13: // ID - 13    上注蓄电池组满安时参数6（蓄电池电压）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		case 0x14: // ID - 14    上注蓄电池组满安时参数7（蓄电池充电电流）

			onoffCount++;
			onoffCountB++;
			onoffCountC++;
			break;

		default:
			break;
		}

		for (i = 0; i < 4; i++)
		{
			onoffBuffer[rdOnoffIndex][i] = 0x00; // 读取指令后缓存清零
		}

		rdOnoffIndex++; // 读指令索引++  指向下一条指令
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
		// wk_rData[i] = (anData[i + 8] >> 1); // 11bit			获取热敏电阻值
	}

	heatState = 0;

	for (i = 0; i < 8; i++) //
	{
		if (wk_parameter[i].Mode == 0x00) // 开环关闭模式
		{
			WK_OUTPUT((i + 1), 0);
		}

		if (wk_parameter[i].Mode == 0x01) // 开环加热模式
		{
			WK_OUTPUT((i + 1), 1);
		}

		if (wk_parameter[i].Mode == 0x02) // 步进式控温
		{
			r = wk_rData[i];						// 热敏电阻值更新，指定方式？
			if ((r < WK_uLIMIT) || (r > WK_lLIMIT)) // 电阻值合理性判读
			{
				wk_parameter[i].Mode = 0; // 异常则更新为开环关闭模式
				wk_parameterB[i].Mode = 0;
				wk_parameterC[i].Mode = 0;
				WK_OUTPUT((i + 1), 0); // 对应控温关闭输出
			}
			else // 热敏电阻在合理区间
			{
				if ((wk_period == 1) || (wk_period == (WK_PERIOD + 1))) // 周期步进控温
				{
					WK_Time14(i, r); //
				}

				if ((wk_time[i].Now > 0) && (wk_time[i].Now < (uint8_t)(WK_PERIOD + 1))) // 控温周期内比较??
				{																		 //
					wk_time[i].Now--;													 //
					WK_OUTPUT((i + 1), 1);												 // 控温开启加热
				}
				else //
				{
					WK_OUTPUT((i + 1), 0); // 控温关闭加热
				}
			}
		}
	}

	if (wk_period > WK_PERIOD) // 控制周期结束
	{
		wk_period = 1; // 周期计数付初值
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
	uint8_t *p1;			// 相机速变包指针
	uint8_t *p2;			// 相机缓变包指针
	uint8_t *p3;			//	相机缓存拼包指针
	uint8_t sum;			//
	uint8_t camBuffer[122]; // 相机速变缓存 拼帧

	//----------------------相机拼包组帧---------------------------------------------------------------//
	p3 = &camBuffer[0]; // 指针初值

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

	//----------------------CAN遥测组帧---------------------------------------------------------------//
	if (fgSwitch == TRUE) // 组幀切换标志
	{
		p1 = &muRsFrame1B[0]; // 指向缓存区B  速变
		p2 = &muRsFrame2B[0]; // 指向缓存区B  缓变
	}
	else
	{
		p1 = &muRsFrame1A[0]; // 指向缓存区A  速变
		p2 = &muRsFrame2A[0]; // 指向缓存区A  缓变
	}

	sum = 0;
	p3 = &camBuffer[0];
	//----------------------速变包组帧---------------------------------------------------------------//
	*p1 = 0x8B;
	p1++; // 第1帧
	*p1 = 0x68;
	p1++;
	*p1 = 0x00;
	p1++; // 帧计数
	*p1 = 0x7A;
	p1++; // 长度
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

	for (i = 0; i < 16; i++) // 速变包中间帧共计16包
	{
		*p1 = 0x8B;
		p1++; // 第N帧
		*p1 = 0x68;
		p1++;
		*p1 = i + 1;
		p1++; // 帧计数

		for (j = 0; j < 7; j++) // 有效数据更新  w5 - w116
		{
			*p1 = *p3;
			sum = sum + *p1;
			p1++; // WN
			p3++;
		}
	}

	*p1 = 0x8B; // 结束帧
	p1++;		//
	*p1 = 0x67;
	p1++;		//
	*p1 = 0x11; // 18帧
	p1++;		//

	for (i = 0; i < 5; i++) //  更新结束帧有效数据  W117 - W121
	{
		*p1 = *p3;
		sum = sum + *p1;
		p1++; // W0
		p3++;
	}

	*p1 = sum; // SUM

	sum = 0;
	//----------------------缓变包组帧---------------------------------------------------------------//
	//----------------------缓变包--1帧---------------------------------------------------------------//
	*p2 = 0x8B;
	p2++; // 第1帧
	*p2 = 0x68;
	p2++;
	*p2 = 0x00; // 帧计数
	p2++;
	*p2 = 0x46; // 长度
	p2++;
	*p2 = 0x47; // title
	sum = sum + *p2;
	p2++;
	*p2 = timeS >> 24; // W0 整秒计时 - byte0   高位
	sum = sum + *p2;
	p2++;
	*p2 = timeS >> 16; // W1 整秒计时 - byte1
	sum = sum + *p2;
	p2++;
	*p2 = timeS >> 8; // W2	整秒计时 - byte2
	sum = sum + *p2;
	p2++;
	*p2 = timeS; // W3 整秒计时 - byte3
	sum = sum + *p2;
	p2++;

	*p2 = timeMS >> 8; // W4 毫秒计时 - byte0  高位
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--2帧---------------------------------------------------------------//
	*p2 = 0x8B;
	p2++; // 第2帧
	*p2 = 0x68;
	p2++;
	*p2 = 0x01; // 帧计数
	p2++;
	*p2 = timeMS; // W5 毫秒计时 - byte1
	sum = sum + *p2;
	p2++;

	*p2 = taskNumber[0]; // W6 任务编号 - byte0
	sum = sum + *p2;
	p2++;
	*p2 = taskNumber[1]; // W7 任务编号 - byte1;
	sum = sum + *p2;
	p2++;
	*p2 = taskNumber[2]; // W8 任务编号 - byte2;
	sum = sum + *p2;
	p2++;
	*p2 = taskNumber[3]; // W9 任务编号 - byte3;;
	sum = sum + *p2;
	p2++;

	*p2 = stateWord[0]; // W10 相机下位机工作状态1
	sum = sum + *p2;
	p2++;
	*p2 = stateWord[1]; // W11 相机下位机工作状态1
	sum = sum + *p2;
	p2++;

	//----------------------缓变包--3帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第3帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x02; // 帧计数
	p2++;

	*p2 = resetCount; // W12	热启动计数
	sum = sum + *p2;
	p2++;
	*p2 = camOnoffTransCount; // W13	成像模组指令计数
	sum = sum + *p2;
	p2++;
	*p2 = onoffCount; // W14	热控接收指令总计数
	sum = sum + *p2;
	p2++;
	*p2 = onoffErrorCount; // W15	热控错误指令计数
	sum = sum + *p2;
	p2++;
	*p2 = onoffRecCode; // W16	最后一条指令编号
	sum = sum + *p2;
	p2++;
	*p2 = camPowerOverTimeCount; // W17	相机上电超时计数
	sum = sum + *p2;
	p2++;
	*p2 = canResetCountA << 4 + canResetCountB; // W18	CAN总线复位计数 高四位 -A 低四位 -B
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--4帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第4帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x03; // 帧计数
	p2++;
	*p2 = encoderOverTimeCount << 4 + encoderRsErrorCount; // W19	编码器状态	高四位 -超时计数 低四位 -错误计数
	sum = sum + *p2;
	p2++;
	*p2 = camMotorState; // W20	相机调焦电机运动状态
	sum = sum + *p2;
	p2++;
	*p2 = camMotorLocate; // W21	当前编码器位置
	sum = sum + *p2;
	p2++;
	*p2 = anData[8] >> 4; // W22	步进电机电源12V遥测
	sum = sum + *p2;
	p2++;
	*p2 = anData[9] >> 4; // W23	基准稳压源5V遥测
	sum = sum + *p2;
	p2++;
	*p2 = anData[10] >> 4; // W24	成像电源12V遥测
	sum = sum + *p2;
	p2++;
	*p2 = camPowerState; // W25	模组加电使能状态
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--5帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第5帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x04; // 帧计数
	p2++;
	*p2 = 0xAA; // W26	保留
	sum = sum + *p2;
	p2++;
	*p2 = heatModeState[0]; // W27	加热带1～4控温模式
	sum = sum + *p2;
	p2++;
	*p2 = heatModeState[1]; // W28	加热带5～8控温模式
	sum = sum + *p2;
	p2++;
	*p2 = heatState; // W29	加热带0～8控温状态
	sum = sum + *p2;
	p2++;
	*p2 = heatCheckState; // W30	加热带0～8回检状态
	sum = sum + *p2;
	p2++;
	*p2 = anData[0] >> 8; // W31 测温点1温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[0]; // W32	测温点1温度值-L
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--6帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第6帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x05; // 帧计数
	p2++;
	*p2 = anData[1] >> 8; // W33 测温点2温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[1]; // W34	测温点2温度值-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[2] >> 8; // W35 测温点3温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[2]; // W36	测温点3温度值-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[3] >> 8; // W37 测温点4温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[3]; // W38	测温点4温度值-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[4] >> 8; // W39 测温点5温度值-H
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--7帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第7帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x06; // 帧计数
	p2++;
	*p2 = anData[4]; // W40	测温点5温度值-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[5] >> 8; // W41 测温点6温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[5]; // W42	测温点6温度值-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[6] >> 8; // W43 测温点7温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[6]; // W44	测温点7温度值-L
	sum = sum + *p2;
	p2++;
	*p2 = anData[7] >> 8; // W45 测温点8温度值-H
	sum = sum + *p2;
	p2++;
	*p2 = anData[7]; // W46	测温点8温度值-L
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--8帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第8帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x07; // 帧计数
	p2++;
	*p2 = wk_parameter[0].WD >> 8; // W47 控温通道1目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[0].WD; // W48 控温通道1目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[1].WD >> 8; // W49 控温通道2目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[1].WD; // W50 控温通道2目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[2].WD >> 8; // W51 控温通道3目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[2].WD; // W52 控温通道3目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[3].WD >> 8; // W53 控温通道4目标温度-H
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--9帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第9帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x08; // 帧计数
	p2++;
	*p2 = wk_parameter[3].WD; // W54 控温通道4目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[4].WD >> 8; // W55 控温通道5目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[4].WD; // W56 控温通道5目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[5].WD >> 8; // W57 控温通道6目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[5].WD; // W58 控温通道6目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[6].WD >> 8; // W59 控温通道7目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[6].WD; // W60 控温通道7目标温度-L
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--10帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第10帧
	p2++;
	*p2 = 0x68;
	p2++;
	*p2 = 0x09; // 帧计数
	p2++;
	*p2 = wk_parameter[7].WD >> 8; // W61 控温通道8目标温度-H
	sum = sum + *p2;
	p2++;
	*p2 = wk_parameter[7].WD; // W62 控温通道8目标温度-L
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W63	备用
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W64	备用
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W65	备用
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W66	备用
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W67	备用
	sum = sum + *p2;
	p2++;
	//----------------------缓变包--11帧---------------------------------------------------------------//
	*p2 = 0x8B; // 第11帧
	p2++;
	*p2 = 0x64;
	p2++;
	*p2 = 0x0A; // 帧计数
	p2++;
	*p2 = 0xAA; // W68	备用
	sum = sum + *p2;
	p2++;
	*p2 = 0xAA; // W69	备用
	sum = sum + *p2;
	p2++;

	*p2 = sum; // sum

	if (fgSwitch == TRUE) // 组幀切换标志
	{
		fgSwitch = FALSE; // 切换标志
	}
	else
	{
		fgSwitch = TRUE; // 切换标志
	}
}

/***************************************************************************************************/
void DataInit(void)
{
	uint16_t i;
	uint16_t j;

	fg40ms = FALSE;	  // 40ms定时标志
	fgSwitch = FALSE; // 默认未进行组帧处理

	fgCanBusUse = 0xAA; // 相机 初始默认A总线 - CAN
	fgMuBusUse = 0xAA;	// 相机下位机 初始默认A总线 - 422

	onoffCount = 0;	 // 指令执行计数 清零
	onoffCountB = 0; // 指令执行计数 清零 B
	onoffCountC = 0; // 指令执行计数 清零 C

	canErrorCount1 = 0; // CAN总线错误计数清零1
	canErrorCount2 = 0; // CAN总线错误计数清零2

	wrOnoffIndex = 0;		 // 指令写索引 ONOFF
	rdOnoffIndex = 0;		 // 指令读索引 ONOFF
	for (i = 0; i < 48; i++) // 指令缓存清零
	{
		for (j = 0; j < 4; j++)
		{
			onoffBuffer[i][j] = 0x00; // 缓存清零
		}
	}

	for (i = 0; i < 8; i++) // 星务、GPS校时缓存清零
	{
		timeSMUBcData[i] = 0x00;
		timeGPSBcData[i] = 0x00;
	}

	for (i = 0; i < 8; i++) // 更新控温参数  三区
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
		wk_tLast[i] = 0x07F8; // ?????  用途
	}
}

/***************************************************************************************************/
void GPIO_Init(void)
{
	MSS_GPIO_init();
	MSS_GPIO_config(MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE); // 输出方式

	MSS_GPIO_config(MSS_GPIO_9, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW);	// 低电平中断方式
	MSS_GPIO_config(MSS_GPIO_11, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_LEVEL_LOW); // 低电平中断方式

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

	*((volatile unsigned short *)(address + (0x29 << 4))) = 0x00; // A1C28~A1C21		28 ~ 18  ->   ID10 ~ ID0   共计三组寄存器
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

	*((volatile unsigned short *)(address + (0x20 << 4))) = 0x44; // CR 接收中断使能、测试MODE禁止
	for (delay = 0; delay < 10000; delay++)
		;
	*((volatile unsigned short *)(address + (0x20 << 4))) = 0x44; // CR 接收中断使能、测试MODE禁止
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

	timeSMUBcDataCanReceiveA(id10_3);		// 星务校时多主广播
	timeGPSBcDataCanReceiveA(id10_3);		// GPS校时多主广播
	GeneralCanReceiveA(id10_3, id2_0, dlc); // 一般数据接收处理
}

/***************************************************************************************************/
void timeSMUBcDataCanReceiveA(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC2) // 星务校时广播ID
	{
		sum = 0;
		for (i = 0; i < 8; i++) // 依据约定读取有效数据
		{
			buffer[i] = *((volatile unsigned short *)(CAN_A_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 7; i++) // 计算校验和
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0x80) // 校验正确  TITLE正确
		{

			for (i = 0; i < 6; i++) // 读取有效6字节
			{
				timeSMUBcData[i] = buffer[1 + i];
			}
			fgAssistDataRenew = TRUE; // 设置辅助数据更新标志
		}
	}
}

/***************************************************************************************************/
void timeGPSBcDataCanReceiveA(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC1) // GPS校时广播ID
	{
		sum = 0;
		for (i = 0; i < 7; i++) // 依据约定读取有效数据
		{
			buffer[i] = *((volatile unsigned short *)(CAN_A_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 6; i++) // 计算校验和
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0xEA) // 校验正确  TITLE正确
		{
			for (i = 0; i < 5; i++) // 读取有效6字节
			{
				timeGPSBcData[i] = buffer[1 + i];
			}

			fgAssistDataRenew = TRUE; // 设置辅助数据更新标志
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

	if ((id1 == ID_BC3) || (id1 == ID_BC4) || (id1 == ID_BC5) || (id1 == ID_BC6) || (id1 == ID_BC7) || (id1 == ID_MU1) || (id1 == ID_MU2)) // ID有效性判读
	{
		if ((id2 & 0x20) == 0x00)
		{
			sum = 0;
			for (i = 0; i < dlc; i++) // 依据约定读取有效数据
			{
				buffer[i] = *((volatile unsigned short *)(CAN_A_ADDRESS + ((0x02 + i) << 4))); // 读取有效数据
			}

			for (i = 0; i < (dlc - 1); i++) // 计算校验和
			{
				sum = sum + buffer[i];
			}

			if (sum == buffer[dlc - 1]) // 校验正确
			{
				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x01)) // 速变遥测指令
				{
					fgMuBusUse = 0xAA; // A总线
					CanSendRs1A();	   // 速变遥测应答
				}

				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x02)) // 缓变变遥测指令
				{
					fgMuBusUse = 0xAA; // A总线
					CanSendRs2A();	   // 缓变遥测应答
				}

				if ((id1 == 0x4B) && (buffer[0] == 0x20)) // 间接指令
				{
					fgMuBusUse = 0xAA;	// A总线
					CanSendOnoffAckA(); // 间接指令应答

					onoffBuffer[wrOnoffIndex][0] = buffer[1]; // 读取有效指令码1
					onoffBuffer[wrOnoffIndex][1] = buffer[2]; // 读取有效指令码2
					onoffBuffer[wrOnoffIndex][2] = buffer[3]; // 读取有效指令码3
					onoffBuffer[wrOnoffIndex][3] = buffer[4]; // 读取有效指令码4

					wrOnoffIndex++;
					if (wrOnoffIndex >= 48) // 指令缓存 48条
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
				if (canReceiveFrameA == buffer[0]) // 帧序号判读
				{
					for (i = 0; i < dlc; i++) // 结束帧
					{
						canReceiveBufferA[canReceiveCountA] = buffer[1 + i]; // 读取有效数据
						canReceiveCountA++;									 // 接收计数增加
					}

					if ((canReceiveCountA - 3) == canReceiveBufferA[0]) // 数据接收长度判断
					{
						for (i = 0; i < (canReceiveCountA - 2); i++) // 共计接收长度 -1  (length /sum )
						{
							sum = sum + canReceiveBufferA[1]; // 计算校验
						}

						if (sum == canReceiveBufferA[canReceiveCountA - 1]) // 校验正确
						{
							idTitle = id1 * 256 + canReceiveBufferA[1]; // ID+TITLE选择处理
							switch (idTitle)							// 选择判读
							{
							case 0x4B40: // 相机上行数据块
								CanSendDataAckA();

								switch (canReceiveBufferA[0]) // 依据接收数据长度判读
								{
								case 0x09: //  9 Byte 数据长度
									for (i = 0; i < 9; i++)
									{
										muData9[i] = canReceiveBufferA[i + 2];
									}
									break;

								case 0x39: //  57 Byte 数据长度
									for (i = 0; i < 57; i++)
									{
										muData57[i] = canReceiveBufferA[i + 2];
									}
									break;

								case 0x79: //  121 Byte 数据长度
									for (i = 0; i < 121; i++)
									{
										muData121[i] = canReceiveBufferA[i + 2];
									}
									break;

								case 0xF9: // 249 Byte 长度取
									for (i = 0; i < 249; i++)
									{
										muData249[i] = canReceiveBufferA[i + 2];
									}
									break;

								default:
									break;
								}
								break;

							case 0xAF6A: // GPS定位广播数据
								for (i = 0; i < 99; i++)
								{
									gpsLocateBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // 设置辅助数据更新标志
								break;

							case 0xAF64: //  姿控广播数据
								for (i = 0; i < 100; i++)
								{
									gpsPoiseBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // 设置辅助数据更新标志
								break;

							case 0x9FA4: // 姿控星敏广播
								for (i = 0; i < 233; i++)
								{
									starBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // 设置辅助数据更新标志
								break;

							case 0x8FC4: // 姿控陀螺广播
								for (i = 0; i < 104; i++)
								{
									topBcData[i] = canReceiveBufferA[i + 2];
								}

								fgAssistDataRenew = TRUE; // 设置辅助数据更新标志
								break;

							case 0x8F04: // 行周期广播 - 04姿控
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
				if (buffer[0] == 0x00) // 首帧变量清零
				{
					canReceiveCountA = 0;
					canReceiveFrameA = 0;
				}

				if (canReceiveFrameA == buffer[0]) // 帧序号判读
				{
					for (i = 0; i < 7; i++) // 起始帧  中间帧
					{
						canReceiveBufferA[canReceiveCountA] = buffer[1 + i]; // 读取有效数据
						canReceiveCountA++;									 // 接收计数增加
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

	if (fgSwitch == FALSE) // B区数据发送
	{
		for (j = 0; j < 18; j++) // 速变包共计18帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
			while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A区数据发送
	{
		for (j = 0; j < 18; j++) // 速变包共计18帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
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

	if (fgSwitch == FALSE) // B区数据发送
	{
		for (j = 0; j < 11; j++) // 缓变包共计11帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
			while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A区数据发送
	{
		for (j = 0; j < 11; j++) // 速变包共计11帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
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

	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // 指令应答码
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x20;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x26;

	*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

	delay = 0; // 判断发送完成
	while (((*((volatile unsigned short *)(CAN_A_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void CanSendDataAckA(void)
{
	uint16_t delay;

	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // 数据块应答码
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x40;
	*((volatile unsigned short *)(CAN_A_ADDRESS + ((0x10 + 0) << 4))) = 0x46;

	*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

	delay = 0; // 判断发送完成
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

	timeSMUBcDataCanReceiveB(id10_3);		// 星务校时多主广播
	timeGPSBcDataCanReceiveB(id10_3);		// GPS校时多主广播
	GeneralCanReceiveB(id10_3, id2_0, dlc); // 一般数据接收处理
}

/***************************************************************************************************/
void timeSMUBcDataCanReceiveB(uint8_t id)
{
	uint8_t i;
	uint8_t sum;
	uint8_t buffer[8];

	if (id == ID_BC2) // 星务校时广播ID
	{
		sum = 0;
		for (i = 0; i < 8; i++) // 依据约定读取有效数据
		{
			buffer[i] = *((volatile unsigned short *)(CAN_B_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 7; i++) // 计算校验和
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0x80) // 校验正确  TITLE正确
		{
			for (i = 0; i < 6; i++) // 读取有效6字节
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

	if (id == ID_BC1) // GPS校时广播ID
	{
		sum = 0;
		for (i = 0; i < 7; i++) // 依据约定读取有效数据
		{
			buffer[i] = *((volatile unsigned short *)(CAN_B_ADDRESS + ((0x02 + i) << 4))); // 8bytes
		}

		for (i = 0; i < 6; i++) // 计算校验和
		{
			sum = sum + buffer[i];
		}

		if ((sum == buffer[i]) && buffer[0] == 0xEA) // 校验正确  TITLE正确
		{
			for (i = 0; i < 5; i++) // 读取有效6字节
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

	if ((id1 == ID_BC3) || (id1 == ID_BC4) || (id1 == ID_BC5) || (id1 == ID_BC6) || (id1 == ID_BC7) || (id1 == ID_MU1) || (id1 == ID_MU2)) // ID有效性判读
	{
		if ((id2 & 0x20) == 0x00)
		{
			sum = 0;
			for (i = 0; i < dlc; i++) // 依据约定读取有效数据
			{
				buffer[i] = *((volatile unsigned short *)(CAN_B_ADDRESS + ((0x02 + i) << 4))); // 读取有效数据
			}

			for (i = 0; i < (dlc - 1); i++) // 计算校验和
			{
				sum = sum + buffer[i];
			}

			if (sum == buffer[dlc - 1]) // 校验正确
			{
				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x01)) // 速变遥测指令
				{
					CanSendRs1B(); // 速变遥测应答
				}

				if ((id1 == 0x8B) && (buffer[0] == 0x00) && (buffer[1] == 0x10) && (buffer[2] == 0x02)) // 缓变变遥测指令
				{
					CanSendRs2B(); // 缓变遥测应答
				}

				if ((id1 == 0x4B) && (buffer[0] == 0x20)) // 间接指令
				{
					CanSendOnoffAckB(); // 间接指令应答

					onoffBuffer[wrOnoffIndex][0] = buffer[1]; // 读取有效指令码1
					onoffBuffer[wrOnoffIndex][1] = buffer[2]; // 读取有效指令码2
					onoffBuffer[wrOnoffIndex][2] = buffer[3]; // 读取有效指令码3
					onoffBuffer[wrOnoffIndex][3] = buffer[4]; // 读取有效指令码4

					wrOnoffIndex++;
					if (wrOnoffIndex >= 48) // 指令缓存 48条
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
				if (canReceiveFrameB == buffer[0]) // 帧序号判读
				{
					for (i = 0; i < dlc; i++) // 结束帧
					{
						canReceiveBufferB[canReceiveCountB] = buffer[1 + i]; // 读取有效数据
						canReceiveCountB++;									 // 接收计数增加
					}

					if ((canReceiveCountB - 3) == canReceiveBufferB[0]) // 数据接收长度判断
					{
						for (i = 0; i < (canReceiveCountB - 2); i++) // 共计接收长度 -1  (length /sum )
						{
							sum = sum + canReceiveBufferB[1]; // 计算校验
						}

						if (sum == canReceiveBufferB[canReceiveCountB - 1]) // 校验正确
						{
							idTitle = id1 * 256 + canReceiveBufferB[1]; // ID+TITLE选择处理
							switch (idTitle)							// 选择判读
							{
							case 0x4B40: // 相机上行数据块
								CanSendDataAckB();
								break;

							case 0xAF6A: // GPS定位广播数据
								break;

							case 0xAF64: //  姿控广播数据
								break;

							case 0x9FA4: // 姿控星敏广播
								break;

							case 0x8F68: // 姿控陀螺广播
								break;

							case 0x8F04: // 行周期广播 - 04姿控
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
				if (buffer[0] == 0x00) // 首帧变量清零
				{
					canReceiveCountB = 0;
					canReceiveFrameB = 0;
				}

				if (canReceiveFrameB == buffer[0]) // 帧序号判读
				{
					for (i = 0; i < 7; i++) // 起始帧  中间帧
					{
						canReceiveBufferB[canReceiveCountB] = buffer[1 + i]; // 读取有效数据
						canReceiveCountB++;									 // 接收计数增加
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

	if (fgSwitch == FALSE) // B区数据发送
	{
		for (j = 0; j < 18; j++) // 速变包共计18帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
			while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A区数据发送
	{
		for (j = 0; j < 18; j++) // 速变包共计18帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame1A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
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

	if (fgSwitch == FALSE) // B区数据发送
	{
		for (j = 0; j < 11; j++) // 缓变包共计11帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2B[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
			while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
			{
				delay++;
			}
		}
	}
	else // A区数据发送
	{
		for (j = 0; j < 11; j++) // 速变包共计11帧
		{
			for (i = 0; i < 10; i++) // 写入发送缓存区
			{
				*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + i) << 4))) = muRsFrame2A[(10 * j) + i];
			}

			*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

			delay = 0; // 判断发送完成
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

	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // 指令应答码
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x20;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x26;

	*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

	delay = 0; // 判断发送完成
	while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void CanSendDataAckB(void)
{
	uint16_t delay;

	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x4B; // 数据块应答码
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x44;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x07;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0xFF;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x40;
	*((volatile unsigned short *)(CAN_B_ADDRESS + ((0x10 + 0) << 4))) = 0x46;

	*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x40; // 启动发送

	delay = 0; // 判断发送完成
	while (((*((volatile unsigned short *)(CAN_B_ADDRESS + (0x22 << 4))) & 0x04) == 0x04) && (delay < 20000))
	{
		delay++;
	}
}

/***************************************************************************************************/
void GPIO9_IRQHandler(void) // CAN-A 总线中断服务程序
{
	uint8_t ir; // SJA1000 状态寄存器

	ir = *((volatile unsigned short *)(CAN_A_ADDRESS + (0x23 << 4))); // 读取中断寄存器  23h

	MSS_GPIO_clear_irq(MSS_GPIO_9);

	if ((ir & 0x80) == 0x80) // 接收中断产生
	{
		CanReceiveA(); // CAN RXD ISR  总线接收处理
	}

	*((volatile unsigned short *)(CAN_A_ADDRESS + (0x21 << 4))) = 0x10; // 清空缓存区  Bit4 -- RRB
}

/***************************************************************************************************/
void GPIO11_IRQHandler(void) // CAN-B 总线中断服务程序
{
	uint8_t ir; // SJA1000 状态寄存器

	ir = *((volatile unsigned short *)(CAN_B_ADDRESS + (0x23 << 4))); // 读取中断寄存器  23h
	MSS_GPIO_clear_irq(MSS_GPIO_11);

	if ((ir & 0x80) == 0x80) // 接收中断产生
	{
		CanReceiveB(); // CAN RXD ISR  总线接收处理
	}

	*((volatile unsigned short *)(CAN_B_ADDRESS + (0x21 << 4))) = 0x10; // 清空缓存区  Bit4 -- RRB
}

/***************************************************************************************************/
void WatchDog(void)
{
	uint32_t delay;

	MSS_GPIO_set_output(MSS_GPIO_0, 1); // 高电平
	for (delay = 0; delay < 1000; delay++)
		;								// 延时
	MSS_GPIO_set_output(MSS_GPIO_0, 0); // 低电平
	for (delay = 0; delay < 1000; delay++)
		;								// 上电延时 等待  调试狗咬时间复位   约260us
	MSS_GPIO_set_output(MSS_GPIO_0, 1); // 高电平
}

/***************************************************************************************************/
void Timer1Init(void)
{
	MSS_TIM1_init(MSS_TIMER_PERIODIC_MODE); // 40ms 定时初始化
	MSS_TIM1_load_immediate(640000);
	MSS_TIM1_start();
	MSS_TIM1_enable_irq();
}

/***************************************************************************************************/
void Timer1_IRQHandler(void) // 1s定时中断服务函数
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
	// if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE)) // 门限出现异常  模式
	// {
	// 	heatMode[0] = 0x51; // 热控控温模式       电阻选择1
	// 	heatModeB[0] = 0x51;
	// 	heatModeC[0] = 0x51;

	// 	heatOpenValue[0] = 909;	 // 热控控制开门限  		15°C - 18°C
	// 	heatOpenValueB[0] = 909; // 热控控制开门限B
	// 	heatOpenValueC[0] = 909; // 热控控制开门限B

	// 	heatCloseValue[0] = 836;  // 热控控制关门限
	// 	heatCloseValueB[0] = 836; // 热控控制关门限C
	// 	heatCloseValueC[0] = 836; // 热控控制关门限C
	// }

	// result1 = Get2_3_I(&heatOpenValue[1], &heatOpenValueB[1], &heatOpenValueC[1]);
	// result2 = Get2_3_I(&heatCloseValue[1], &heatCloseValueB[1], &heatCloseValueC[1]);
	// result3 = Get2_3_U(&heatMode[1], &heatModeB[1], &heatModeC[1]);
	// if ((result1 == FALSE) || (result2 == FALSE) || (result3 == FALSE)) // 门限出现异常  模式
	// {
	// 	heatMode[1] = 0x52; // 热控控温模式       电阻选择2
	// 	heatModeB[1] = 0x52;
	// 	heatModeC[1] = 0x52;

	// 	heatOpenValue[1] = 909;	 // 热控控制开门限  		15°C - 18°C
	// 	heatOpenValueB[1] = 909; // 热控控制开门限B
	// 	heatOpenValueC[1] = 909; // 热控控制开门限B

	// 	heatCloseValue[1] = 836;  // 热控控制关门限
	// 	heatCloseValueB[1] = 836; // 热控控制关门限C
	// 	heatCloseValueC[1] = 836; // 热控控制关门限C
	// }

	// result1 = Get2_3_U(&heatEn, &heatEnB, &heatEnC); // 控温使能判决
	// if (result1 == FALSE)
	// {
	// 	heatEn = TRUE;	// 恢复默认值  控温使能
	// 	heatEnB = TRUE; // 恢复默认值  控温使能 B
	// 	heatEnC = TRUE; // 恢复默认值  控温使能 C
	// }

	result1 = Get2_3_U(&onoffCount, &onoffCountB, &onoffCountC); // 指令计数判决
	if (result1 == FALSE)
	{
		onoffCount = 0;	 // 恢复默认清零
		onoffCountB = 0; // 恢复默认清零 B
		onoffCountC = 0; // 恢复默认清零 C
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

	if ((temp1 < 0.0001) && (temp2 < 0.0001) && (temp3 < 0.0001)) //	三者差值比较
	{
		return (TRUE); //  返回正确
	}
	else
	{
		if (temp1 < 0.0001) //  两者相比赋值操作
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
	if ((*a == *b) && (*b == *c)) /* 前两个数相等 */
	{
		return (TRUE); /* 3取2判读正确 */
	}
	else if (*a == *b)
	{
		*c = *b;	   /* 赋值操作 */
		return (TRUE); /* 3取2判读正确 */
	}
	else
	{
		if (*a == *c) /* 另两个数相等 */
		{
			*b = *a;	   /* 赋值操作 */
			return (TRUE); /* 3取2判读正确 */
		}
		else
		{
			if (*b == *c) /* 剩余两个数相等 */
			{
				*a = *b;	   /* 赋值操作 */
				return (TRUE); /* 3取2判读正确 */
			}
			else
			{
				return (FALSE); /* 3取2判读异常 */
			}
		}
	}
}

/***************************************************************************************************/
uint8_t Get2_3_U(uint8_t *a, uint8_t *b, uint8_t *c)
{
	if ((*a == *b) && (*b == *c)) /* 前两个数相等 */
	{
		return (TRUE); /* 3取2判读正确 */
	}
	else if (*a == *b)
	{
		*c = *b;	   /* 赋值操作 */
		return (TRUE); /* 3取2判读正确 */
	}
	else
	{
		if (*a == *c) /* 另两个数相等 */
		{
			*b = *a;	   /* 赋值操作 */
			return (TRUE); /* 3取2判读正确 */
		}
		else
		{
			if (*b == *c) /* 剩余两个数相等 */
			{
				*a = *b;	   /* 赋值操作 */
				return (TRUE); /* 3取2判读正确 */
			}
			else
			{
				return (FALSE); /* 3取2判读异常 */
			}
		}
	}
}
