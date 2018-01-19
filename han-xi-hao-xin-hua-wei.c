/*
深圳锦超科技有限公司
*/

#include "stm32f4xx_hal.h"
#include "types.h"
#include "lio.h"
#include "lmotor.h"
#include "sysparam.h"
#include "string.h"
#include "modbus.h"
#include "lbt.h"
#include "lflash.h"
#include "lmem.h"

//------------------------------------------------------------------------------

enum
{
	TYPE_EMPTY_MOVE = 0, // 空移
	TYPE_NODE_WELD, // 1,点焊
	TYPE_LINE_START, // 2,拖焊起点
	TYPE_LINE_WELD, // 3,拖焊
	TYPE_LINE_END, // 4,拖焊终点
	TYPE_MOTO_RESET, // 5,电机复位
	TYPE_WASH_GUN, // 6,清洗烙铁		
	TYPE_DELAY, // 7,延时暂停
	TYPE_WORK_OVER, // 8,加工结束
	TYPE_OUTPUT, // 9,端口输出
	TYPE_INPUT, // 10,等待输入
};

typedef struct MotorPara_st
{
	int ppr;
	float dpr;
	int dirPol;
	int resetDir;
	int org;
	float mdis;
	float pdis;
} MotorPara_st;

typedef struct
{
	s16 pin_number;//引脚号
	s16 pin_type;  //电平有效值  类型 常闭1 常开0
	s16 pin_fun;   //功能号(用于查列表) 
	s16 pin_state; //当前状态	
} IONode_st;

typedef struct //厂商参数
{
	int 	Password;		//密码
	int 	Set_password;	//是否启用密码
	char 	PowerDown;		//掉电保存
	char	StartAndPause;	//启动按钮加暂停功能
	char	ReadyToPos; // 1-使能Y2, 0-禁用Y2

	short	RZero_offset;	//R轴原点偏移，用作补偿与退偿
	
	int		reserve[10];	//预留参数
	
	MotorPara_st MotorPara[6];	//电机参数
	
	IONode_st input_io[34];	//下标为端口号
	IONode_st output_io[18];	//下标为端口号
	
} FactoryPara_st;

typedef struct //用户参数
{
	s16 cycle_piece;		//循环加工总次数
	s16 reset_piece;		//复位间隔
	s16 wash_piece;		//清洗间隔---加工时，加工多少遍清洗
	s16 cycle_file_num;	//循环加工文件数量
	s16 is_cycle;			//是否循环加工
	s16 is_reset_y2;		//是否复位Y2轴
	
	
	float cycle_time;		//循环加工间隔--左右工位切换的间隔时间
	float reset_time;		//开机延时复位时间
	float safe_height;		//Z轴安全高度
	float add_tin;			// 清洗时间
	int   max_weld_time;	//最大点焊次数;
	float plt_scale;		//PLT文件转化比例
	
	
	float pos[6];	//清洗位置

	int   Ip_Address;		//IP地址，最低位

	int   wash_time;		//待机时，隔多久清洗一次(s)
	float stop_pos[6];//停止位置
	int	  is_wash_y;		//清洗是否使能Y轴  
	int   is_goto_stop_pos;	//加工完是否回停止位置
	char  filelist[2][1][15];
	float speed1; // 补偿速度
}UserPara_st;

typedef struct SpeedPara_st //速度参数
{
	int speed; // 运行速度
	int acc; // 加速度
	int reserve[3]; /* 对于XYZA,reserve表示手动低速/手动高速/复位速度.对于C,reserve表示送锡速度/回锡速度/提前送锡速度 */
} SpeedPara_st;

typedef struct
{
	int ver; // 1
	u32 rand; // 接收到错误数据不再写入
	u32 CheckSum;	
	FactoryPara_st 	FactoryPara;
	UserPara_st   	UserPara;
	SpeedPara_st	SpeedPara[6];
} GlobalPara_st;

//工作状态信息
typedef struct
{
	long  xpulse;	//当前位置
	long  ypulse;
	long  zpulse;
	long  apulse;
	long  bpulse;
	long  cpulse;

	u16 total_node;	//总加工点数
	u16 cur_node;	//当前加工点号
	u16 finish_piece;//完成加工次数		

	u8 work_state;	//加工状态,
	u8 cur_work_pos;	//当前工位
	u8 is_cycle;		//循环
	u8 is_step;		//单步
	u8 is_send_tin;	//是否送锡

	int work_time;		//加工时间
	int weld_num;		//焊点次数
	int alarm; 			// bit0:无锡, bit1:卡锡, bit2:安全门

	int file_change[2];	//文件修改标志,扫描到BCD码之后进行刷新
	int file_BCD;		//文件号BCD码

	int	reserver[28];	//保留参数	
}work_st;

typedef struct //焊接特有参数
{
	float gun_height;		//枪高(mm)
	float send_tin[3];		//送锡(s)
	float back_tin[3];		//回锡(s)
	float close_tin;		//提前关锡(mm)
	float speed;			//送锡速度(mm/s)
	float ahead_speed;		//提前送锡速度(mm/s),补偿距离y

	float heat_gun;			//焊枪预热时间(s)
	float delay;			//停顿时间(s)

	float distance;			//偏移距离x(mm)
	float height;			//偏移高度(mm)
	int is_offset;			//是否退偿	0不退偿 1退偿
} weld_para;

typedef struct //IO操作特有参数
{
	s8 port;
	s8 value;
	float time;
} io_para;

typedef struct //电机复位特有参数
{
	char reset[4];	//方便采用循环
} reset_para;


typedef union
{
	weld_para	weld;
	io_para	  	io;
	reset_para	reset;
	float		delay;
} PUBLIC_1;

typedef struct //教导加工点
{
	char cmd_type;	//指令
	char group;		//组别	0表示不分组，只有焊接相关的类型才有组别，且同组的焊接参数相同
	float speed;	// 速度,%
	
	//各轴坐标
	float	pos[4];	//方便采用循环
	PUBLIC_1 public1;
} work_node;

typedef	struct
{
	u8		CmdType;
	u8		SpeedRate;
	float		PointMove;
	work_node	node;		//加工点定位信息	
} ManualCmd;

typedef struct
{
	char d_name[13];
	char ext_name[4];
	char FAT_DirAttr;
	int size;
} FILE_INFO_5600;

typedef struct // 文件参数
{
	float close_tin;	// 提前关锡距离
	float offset_speed;	// 补偿速度
	int wash_piece;	// 清洗次数
	int wash_time;	// 清洗延时时间

	// 视觉校正基准点,只需知道XY即可	
	float stdpoint[2];
	int para[8]; // 新增参数预留
} FilePara_st;

typedef struct
{
	int total; //当前教导加工点数0~N
	FilePara_st FilePara;	//文件参数保存在文件中,各个参数不同
	work_node work_file;
	int nodeId; // 当前加载的加工点的序号
} work_file;

//------------------------------------------------------------------------------

typedef struct
{
	u8 buf[256 + 8];
	int tick;
	int len;
} UARTA_RECV_BUF;

typedef struct
{
	u8 buf[256];
	int len;
	bool flag;
} UARTA_RECV_FRAME;

typedef struct
{
	u8 buf[256];
	int len;
	bool valid;
} UARTA_SEND_BUF;

typedef struct
{
	int len;
	bool valid; // true-正在发送
	u8 buf[256];
} UARTA_SEND_FRAME;

typedef struct
{
	UARTA_RECV_BUF recvBuf;
	UARTA_RECV_FRAME recvFrame;
	UARTA_SEND_FRAME sendFrame; // buf溢出时,刚好'sendBuf.buf'前面的空间已经腾出来了
	UARTA_SEND_BUF sendBuf;
} UARTA_DATA;

typedef struct
{
	float manualLo; // 手动低速
	float manualHi; // 手动高速
	float washTime; // 清洗时间(s)
} OTHER_PARAM; // 杂项参数

typedef struct
{
	int ver;
	int len; // 有效点数
	u32 crc;
	float dat[1][20];
} WORK_FILE_DATA;

typedef struct
{
	int startFlag; // b0-工位0启动键按下,b1-工位1启动键按下
	int status; // 状态机状态, 0-idle, 10-加工中, 11-判断是否要复位, 12-判断是否要清洗,
		// 13-判断是否要回停止位, 14-判断是否循环加工
	int curNode; // 当前加工点序号
	int curWorkSel; // 当前加工工位,0 or 1
	int workCount; // 加工计数,2个工位总共加工的板子数量
	int startNode; // 起始加工点
	bool pause; // 启动/暂停复用时,第1次按启动,执行启动功能,第2次按启动,执行暂停功能,
		// 但要等到一个加工点结束,才停下来,因此第2次按启动时,该参数为true,一个加工点结束时,
		// 停下来,pauseZ记录当z坐标,然后z抬起到0点
	float pauseZ; // 暂停时z坐标
	int workTickStart; // 加工启动时刻
	int workTickTotal; // 加工计时
} RunStatus;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static UARTA_DATA uartaData;
static UART_HandleTypeDef * sendUart = &huart2;
static char selectFile[2][50];
static int curManualSpeed = 0; // 0:低速 1:高速
static OTHER_PARAM otherParam;
static WORK_FILE_DATA * workFileData[2];
static GlobalPara_st globalParam;
static work_st workStatus;
static ManualCmd manualCmd;
static work_file * pWorkFile[2];
static const char MotionVer[3][2][30] = 
{
	{{""},	{""}},
	{{"运动控制器:"},	{"工控大师-407ZG/JC603X2"}},	
	{{"程序日期:"},	{__DATE__" "__TIME__}},	
};
static int workingTick; // 加工计时,正值表示加工起始时间(ms),负值表示加工耗时(ms)
static RunStatus runStatus;
static u32 stampRand; // 与手持盒通信的随机数
static u32 resetStartTick = 0xffffffff; // 到达这个时间,系统重启

const char * getCodeVersion(void);
void MX_USART1_UART_Init(int baud);
void MX_USART2_UART_Init(int baud);
//static void runNode(void);
static void uartaTxFinish(void);
static void uartaRxIsr(u8 ch);
static void uartaRecvScan(void);
static void userReset(void);

static void adtReloadWorkFile(int LorR)
{
	char * fileName;
	int fileLen;
	
	DBG((LorR == 0) || (LorR == 1));
	fileName = &selectFile[LorR][6];
	fileLen = readFile(fileName, 0, 0, 0);
	if(fileLen > 0)
	{
		if(pWorkFile[LorR] != NULL)
		{
			Lfree(pWorkFile[LorR]);
			pWorkFile[LorR] = NULL;
		}
		pWorkFile[LorR] = (work_file *)Lmalloc(sizeof(work_file));
		if(pWorkFile[LorR] != NULL)
		{
			readFile(fileName, 0, (u8 *)pWorkFile[LorR], sizeof(work_file) - 4);
			pWorkFile[LorR]->nodeId = 0;
		}
		else
		{
			DBG(0);
		}
	}
}

static void initSelectFile(void)
{
	readFile("selFile", 0, (u8 *)selectFile, sizeof(selectFile));
	adtReloadWorkFile(0);
	adtReloadWorkFile(1);
}

// i-众为兴序号, j-实际序号
static void initGlobalParam1(int i, int j)
{
	globalParam.FactoryPara.MotorPara[i].ppr = sysParam.motorParam[j].ppr;
	globalParam.FactoryPara.MotorPara[i].dpr = sysParam.motorParam[j].dpr;
	globalParam.FactoryPara.MotorPara[i].dirPol = sysParam.motorParam[j].dirPol;
	globalParam.FactoryPara.MotorPara[i].resetDir = sysParam.motorParam[j].resetDir;
	globalParam.FactoryPara.MotorPara[i].org = sysParam.motorParam[j].org;
	globalParam.FactoryPara.MotorPara[i].mdis = sysParam.motorParam[j].mdis;
	globalParam.FactoryPara.MotorPara[i].pdis = sysParam.motorParam[j].pdis;
	globalParam.SpeedPara[i].speed = sysParam.motorParam[j].speed;
	globalParam.SpeedPara[i].acc = sysParam.motorParam[j].acc;
	if(i == 0) // X
	{
		if(globalParam.SpeedPara[i].reserve[2] >= 1)
		{
			setUserParam(40, globalParam.SpeedPara[i].reserve[2]);
		}
		else
		{
			setUserParam(40, 60);
		}
	}
	else if(i == 1) // Y
	{
		if(globalParam.SpeedPara[i].reserve[2] >= 1)
		{
			setUserParam(41, globalParam.SpeedPara[i].reserve[2]);
		}
		else
		{
			setUserParam(41, 60);
		}
	}
	else if(i == 2) // Z
	{
		if(globalParam.SpeedPara[i].reserve[2] >= 1)
		{
			setUserParam(42, globalParam.SpeedPara[i].reserve[2]);
		}
		else
		{
			setUserParam(42, 60);
		}
	}
	else if(i == 4) // A
	{
		if(globalParam.SpeedPara[i].reserve[2] >= 1)
		{
			setUserParam(43, globalParam.SpeedPara[i].reserve[2]);
		}
		else
		{
			setUserParam(43, 60);
		}
	}
}

static void initGlobalParam(void)
{
	readFile("GPARA.INI", 0, (u8 *)&globalParam, sizeof(globalParam));
	// x
	if(globalParam.SpeedPara[0].reserve[0] < 1)
	{
		globalParam.SpeedPara[0].reserve[0] = 10;
	}
	if(globalParam.SpeedPara[0].reserve[1] < 1)
	{
		globalParam.SpeedPara[0].reserve[1] = 100;
	}
	// y
	if(globalParam.SpeedPara[1].reserve[0] < 1)
	{
		globalParam.SpeedPara[1].reserve[0] = 10;
		globalParam.SpeedPara[3].reserve[0] = 10;
	}
	if(globalParam.SpeedPara[1].reserve[1] < 1)
	{
		globalParam.SpeedPara[1].reserve[1] = 100;
		globalParam.SpeedPara[3].reserve[1] = 100;
	}
	// z
	if(globalParam.SpeedPara[2].reserve[0] < 1)
	{
		globalParam.SpeedPara[2].reserve[0] = 10;
	}
	if(globalParam.SpeedPara[2].reserve[1] < 1)
	{
		globalParam.SpeedPara[2].reserve[1] = 100;
	}
	// r
	if(globalParam.SpeedPara[4].reserve[0] < 1)
	{
		globalParam.SpeedPara[4].reserve[0] = 10;
	}
	if(globalParam.SpeedPara[4].reserve[1] < 1)
	{
		globalParam.SpeedPara[4].reserve[1] = 100;
	}
	// c
	if(globalParam.SpeedPara[5].reserve[0] < 1)
	{
		globalParam.SpeedPara[5].reserve[0] = 20;
	}
	if(globalParam.SpeedPara[5].reserve[1] < 1)
	{
		globalParam.SpeedPara[5].reserve[1] = 20;
	}
	initGlobalParam1(0, 0);
	initGlobalParam1(1, 1);
	initGlobalParam1(2, 2);
	initGlobalParam1(3, 1);
	initGlobalParam1(4, 3);
	initGlobalParam1(5, 5);
}

void init5600(void)
{
	extern void (* uart1TxFinish)(void);
	extern void (* uart2TxFinish)(void);
	extern void (* uart1RxIsr)(u8 ch);
	extern void (* uart2RxIsr)(u8 ch);
	extern void (* uart1Scan)(void);
	extern void (* uart2Scan)(void);
	extern RNG_HandleTypeDef hrng;
	static bool flag;

	if(!flag)
	{
		uart1TxFinish = NULL;
		uart2TxFinish = uartaTxFinish;
		uart1RxIsr = uartaRxIsr;
		uart2RxIsr = NULL;
		uart1Scan = uartaRecvScan;
		uart2Scan = NULL;
		otherParam.manualHi = 100;
		otherParam.manualLo = 10;
		initSelectFile();
		initGlobalParam();
		MX_USART1_UART_Init(115200);
		MX_USART2_UART_Init(115200);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		huart2.Instance->CR1 &= ~UART_MODE_RX;
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		workStatus.is_send_tin = 1;
		userReset();
		globalParam.ver = 0;
		HAL_RNG_GenerateRandomNumber(&hrng, &stampRand);
		globalParam.rand = stampRand;
		if(spiFlashSize() == 0)
		{
			alarmStatus[4 / 32] |= (1 << (4 % 32)); // 脚本中报警代码一定要有
		}
		flag = true;
	}
}

static void uartaTxFinish(void)
{
	uartaData.sendFrame.valid = false;
}

static void uartaSend(void)
{
	u8 hi = 0xff;
	u8 lo = 0xff;

	crc16(uartaData.sendBuf.buf, uartaData.sendBuf.len, &hi, &lo);
	uartaData.sendBuf.buf[uartaData.sendBuf.len++] = lo;
	uartaData.sendBuf.buf[uartaData.sendBuf.len++] = hi;
	memcpy(uartaData.sendFrame.buf, uartaData.sendBuf.buf, uartaData.sendBuf.len);
	uartaData.sendFrame.len = uartaData.sendBuf.len;
	uartaData.sendFrame.valid = true;
	HAL_UART_Transmit_IT(sendUart, uartaData.sendFrame.buf, uartaData.sendFrame.len);
}

static void uartaRxIsr(u8 ch)
{
	if(uartaData.sendFrame.valid)
	{
		return;
	}
	if(getTick() - uartaData.recvBuf.tick > 2)
	{
		uartaData.recvBuf.len = 0;
	}
	if(uartaData.recvBuf.len == 0)
	{
		if(ch == 0xbb)
		{
			uartaData.recvBuf.buf[uartaData.recvBuf.len++] = ch;
		}
	}
	else if(uartaData.recvBuf.len + 1 < sizeof(uartaData.recvBuf.buf))
	{
		uartaData.recvBuf.buf[uartaData.recvBuf.len++] = ch;
	}
	uartaData.recvBuf.tick = getTick();
}

// 查找指定扩展名的文件总数
static int getFileCount(char * extName)
{
	const FILE_INFO * p;
	int count;
	int i;
	int extLen;
	int n = 0;
	
	p = getAllFileInfo(&count);
	extLen = strlen(extName);
	for(i = 0; i < count; i++)
	{
		if(p[i].len != INVALID_FILE_LEN)
		{
			int nameLen = strlen(p[i].name);
			const char * extName1;
			if(nameLen < extLen)
			{
				continue;
			}
			extName1 = &p[i].name[nameLen - extLen];
			if(!strcmp(extName1, extName))
			{
				n++;
			}
		}
	}
	return n;
}

static void getFileList(FILE_INFO_5600 * info, int count, char * extName)
{
	const FILE_INFO * p;
	int totalFileCount;
	int i;
	int extLen;
	int n = 0;
	
	p = getAllFileInfo(&totalFileCount);
	extLen = strlen(extName);
	for(i = 0; i < totalFileCount; i++)
	{
		if(p[i].len != INVALID_FILE_LEN)
		{
			int nameLen = strlen(p[i].name);
			const char * extName1;
			if(nameLen < extLen)
			{
				continue;
			}
			extName1 = &p[i].name[nameLen - extLen];
			if(!strcmp(extName1, extName))
			{
				if(nameLen < sizeof(info[n].d_name))
				{
					strcpy(info[n].d_name, p[i].name);
				}
				else
				{
					strcpy(info[n].d_name, "nameTooLong");
				}
				info[n].size = p[i].len;
				info[n].FAT_DirAttr = 0x20;
				n++;
				if(n >= count)
				{
					break;
				}
			}
		}
	}
}

static int getFileBlockData(u8 * buf, char * fileName, int blockId, int blockSize)
{
	int start = blockId * blockSize;
	int size = 0;
	
	if(!strcmp(fileName, "globalParam"))
	{
		u8 * p = (u8 *)&globalParam;
		size = sizeof(globalParam) - start;
		if(size > blockSize)
		{
			size = blockSize;
		}
		memcpy(buf, &p[start], size);
	}
	else if(!strcmp(fileName, "fileList"))
	{
		int count = getFileCount(".DAT");
		FILE_INFO_5600 * info = Lmalloc(count * sizeof(FILE_INFO_5600));
		u8 * p = (u8 *)info;
		DBG(info != NULL);
		size = count * sizeof(FILE_INFO_5600) - start;
		if(size > blockSize)
		{
			size = blockSize;
		}
		getFileList(info, count, ".DAT");
		memcpy(buf, &p[start], size);
		Lfree(info);
	}
	else if(!strcmp(&fileName[strlen(fileName) - 4], ".DAT"))
	{
		int fileLen = readFile(fileName, 0, NULL, 0);
		if(fileLen > 0)
		{
			size = fileLen - start;
			if(size > blockSize)
			{
				size = blockSize;
			}
			if(size > 0)
			{
				readFile(fileName, start, buf, size);
			}
		}
	}
	else
	{
		lprintf("getFileBlockData() %s\n", fileName);
	}
	return size;
}

static int getFileSize(char * fileName)
{
	return readFile(fileName, 0, NULL, 0);
}

// 手持盒读取文件数据
static void adtReadFile(u8 * buf, u32 len)
{
	static char fileName[16];
	int frameId;
	int maxLen = 246;
	int pos = 0;
	int sendLen = 0;
	
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = buf[0];
	frameId = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6];
	if(frameId == 0)
	{
		int totalSize = 0;
		int totalFrame = 0;
		buf[len] = 0;
		if(!strcmp((char *)&buf[7], "\\PARA\\GPARA.INI"))
		{
			totalSize = sizeof(globalParam);
			strcpy(fileName, "globalParam");
		}
		else if(!strcmp((char *)&buf[7], "%DirList%\\PROG\\%DAT"))
		{
			totalSize = getFileCount(".DAT") * sizeof(FILE_INFO_5600);
			strcpy(fileName, "fileList");
		}
		else if(!memcmp(&buf[7], "%DeleteFile%", strlen("%DeleteFile%")))
		{
			delFile((char *)&buf[25]);
		}
		else if(!strcmp((char *)&buf[7 + strlen((char *)&buf[7]) - 4], ".DAT"))
		{
			totalSize = getFileSize((char *)&buf[0xd]);
			strcpy(fileName, (char *)&buf[0xd]);
		}
		else
		{
			DBG(0);
			printData(buf, len);
		}
		totalFrame = (totalSize + maxLen - 1) / maxLen;
		sendLen = 12;
		uartaData.sendBuf.buf[pos++] = sendLen >> 8;
		uartaData.sendBuf.buf[pos++] = sendLen;
		uartaData.sendBuf.buf[pos++] = frameId >> 24;
		uartaData.sendBuf.buf[pos++] = frameId >> 16;
		uartaData.sendBuf.buf[pos++] = frameId >> 8;
		uartaData.sendBuf.buf[pos++] = frameId;
		uartaData.sendBuf.buf[pos++] = totalFrame >> 24;
		uartaData.sendBuf.buf[pos++] = totalFrame >> 16;
		uartaData.sendBuf.buf[pos++] = totalFrame >> 8;
		uartaData.sendBuf.buf[pos++] = totalFrame;
		uartaData.sendBuf.buf[pos++] = totalSize >> 24;
		uartaData.sendBuf.buf[pos++] = totalSize >> 16;
		uartaData.sendBuf.buf[pos++] = totalSize >> 8;
		uartaData.sendBuf.buf[pos++] = totalSize;
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(frameId > 0)
	{
		uartaData.sendBuf.buf[pos++] = (sendLen + 4) >> 8;
		uartaData.sendBuf.buf[pos++] = (sendLen + 4);
		uartaData.sendBuf.buf[pos++] = frameId >> 24;
		uartaData.sendBuf.buf[pos++] = frameId >> 16;
		uartaData.sendBuf.buf[pos++] = frameId >> 8;
		uartaData.sendBuf.buf[pos++] = frameId;
		sendLen = getFileBlockData(&uartaData.sendBuf.buf[pos], fileName, frameId - 1, maxLen);
		pos += sendLen;
		uartaData.sendBuf.buf[2] = (sendLen + 4) >> 8;
		uartaData.sendBuf.buf[3] = (sendLen + 4);
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else
	{
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.buf[pos++] = buf[2];
		uartaData.sendBuf.buf[pos++] = buf[3];
		uartaData.sendBuf.buf[pos++] = buf[4];
		uartaData.sendBuf.buf[pos++] = buf[5];
		uartaData.sendBuf.buf[pos++] = buf[6];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
}

static void saveGlobalParam1(int i, int j)
{
	sysParam.motorParam[j].ppr = globalParam.FactoryPara.MotorPara[i].ppr;
	sysParam.motorParam[j].dpr = globalParam.FactoryPara.MotorPara[i].dpr;
	sysParam.motorParam[j].dirPol = globalParam.FactoryPara.MotorPara[i].dirPol;
	sysParam.motorParam[j].resetDir = globalParam.FactoryPara.MotorPara[i].resetDir;
	sysParam.motorParam[j].mdis = globalParam.FactoryPara.MotorPara[i].mdis;
	sysParam.motorParam[j].pdis = globalParam.FactoryPara.MotorPara[i].pdis;
	sysParam.motorParam[j].speed = globalParam.SpeedPara[i].speed;
	sysParam.motorParam[j].acc = globalParam.SpeedPara[i].acc;
}

// 手持盒下发文件数据
static void adtWriteFile(u8 * buf, u32 len)
{
	static char fileName[16];
	static u8 * p;
	static int totalSize;
	static int fileId = -1;
	int frameId;
	int maxLen = 246;
	int pos = 0;
	
	
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = buf[0];
	frameId = (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6];
	if(frameId == 0)
	{
		buf[len] = 0;
		strcpy(fileName, (char *)&buf[0x15]);
		totalSize = (buf[0xb] << 24) | (buf[0xc] << 16) | (buf[0xd] << 8) | buf[0xe];
		fileId = createFile(fileName, totalSize);
		if(p != NULL)
		{
			Lfree(p);
			p = NULL;
		}
		p = (u8 *)Lmalloc(totalSize);
		uartaData.sendBuf.buf[pos++] = 0;
		uartaData.sendBuf.buf[pos++] = 4;
		uartaData.sendBuf.buf[pos++] = frameId >> 24;
		uartaData.sendBuf.buf[pos++] = frameId >> 16;
		uartaData.sendBuf.buf[pos++] = frameId >> 8;
		uartaData.sendBuf.buf[pos++] = frameId;
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(frameId > 0)
	{
		int sendLen = 4;
		int datLen = (buf[1] << 8) | buf[2];
		datLen -= 4;
		writeFile(fileId, maxLen * (frameId - 1), &buf[7], datLen);
		if(p != NULL)
		{
			memcpy(&p[maxLen * (frameId - 1)], &buf[7], datLen);
		}
		uartaData.sendBuf.buf[pos++] = sendLen >> 8;
		uartaData.sendBuf.buf[pos++] = sendLen;
		uartaData.sendBuf.buf[pos++] = frameId >> 24;
		uartaData.sendBuf.buf[pos++] = frameId >> 16;
		uartaData.sendBuf.buf[pos++] = frameId >> 8;
		uartaData.sendBuf.buf[pos++] = frameId;
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
		if(maxLen * (frameId - 1) + datLen == totalSize)
		{
			flushFile(fileId);
			fileId = -1;
			if(!strcmp(fileName, "GPARA.INI"))
			{
				if((p != NULL) && (totalSize == sizeof(globalParam)))
				{
					memcpy(&globalParam, p, 8);
					if((globalParam.ver == 1) && (globalParam.rand == stampRand))
					{
						memcpy(&globalParam, p, sizeof(globalParam));
						globalParam.ver = 0;
						saveGlobalParam1(0, 0);
						saveGlobalParam1(1, 1);
						saveGlobalParam1(2, 2);
						saveGlobalParam1(1, 4);
						saveGlobalParam1(4, 3);
						saveGlobalParam1(5, 5);
						setUserParam(34, 1.0 * globalParam.SpeedPara[5].reserve[0] / globalParam.SpeedPara[5].speed * 100);
						setUserParam(35, 1.0 * globalParam.SpeedPara[5].reserve[1] / globalParam.SpeedPara[5].speed * 100);
						saveSysParam(&sysParam);
					}
					else
					{
						globalParam.ver = 0;
						globalParam.rand = stampRand;
						alarmStatus[3 / 32] |= (1 << (3 % 32)); // 脚本中报警代码一定要有
					}
				}
			}
			Lfree(p);
			p = NULL;
			totalSize = 0;
		}
	}
	else
	{
		fileId = -1;
		DBG(0);
		printData(buf, len);
	}
}

static void saveCurSelFile(u8 * buf, u32 len)
{
	int pos = 0;
	int fileId;
	
	memcpy(&selectFile, &buf[2], sizeof(selectFile));
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = buf[0];
	uartaData.sendBuf.buf[pos++] = buf[1];
	uartaData.sendBuf.len = pos;
	uartaData.sendBuf.valid = true;
	fileId = createFile("selFile", sizeof(selectFile));
	writeFile(fileId, 0, (u8 *)selectFile, sizeof(selectFile));
	flushFile(fileId);
}

static void reloadWorkFile(u8 * buf, u32 len)
{
	int pos = 0;
	int sel1;
	int sel2;
	
	sel1 = buf[2] | (buf[3] << 8) | (buf[4] << 16) | (buf[5] << 24);
	sel2 = buf[6] | (buf[7] << 8) | (buf[8] << 16) | (buf[9] << 24);
	if(sel1 != 0)
	{
		adtReloadWorkFile(0);
	}
	if(sel2 != 0)
	{
		adtReloadWorkFile(1);
	}
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = buf[0];
	uartaData.sendBuf.buf[pos++] = buf[1];
	uartaData.sendBuf.len = pos;
	uartaData.sendBuf.valid = true;
}

static void adtCopyFile(u8 * buf, u32 len)
{
	int i;
	char src[20] = "";
	char dst[20] = "";
	u8 * p;
	int pos;
	int fileLen;

	buf[len] = 0;
	for(i = 7; buf[i]; i++)
	{
		if(buf[i] == '%')
		{
			buf[i] = 0;
			strcat(src, (char *)&buf[8]);
			buf[i] = '%';
			strcat(dst, (char *)&buf[i + 7]);
			break;
		}
	}
	fileLen = readFile(src, 0, NULL, 0);
	p = (u8 *)Lmalloc(1024);
	if(p != NULL)
	{
		int fileId = createFile(dst, fileLen);
		pos = 0;
		while(1)
		{
			if(fileLen <= 1024)
			{
				readFile(src, pos, p, fileLen);
				writeFile(fileId, pos, p, fileLen);
				Lfree(p);
				break;
			}
			readFile(src, pos, p, 1024);
			writeFile(fileId, pos, p, 1024);
			pos += 1024;
			fileLen -= 1024;
		}
		flushFile(fileId);
	}
	else
	{
		DBG(0);
	}
	pos = 0;
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = buf[0];
	uartaData.sendBuf.buf[pos++] = buf[1];
	uartaData.sendBuf.len = pos;
	uartaData.sendBuf.valid = true;
}

static void modifyFilePara(u8 * buf, u32 len)
{
	int pos = 0;
	/*
	更改文件参数需要把整个文件读出-修改-写入,故取消文件参数
	u8 cmd = buf[0];
	work_file * p = NULL;
	char * fileName;
	int fileSize;
	
	if(cmd == 0x31)
	{
		p = pWorkFile[0];
		fileName = &selectFile[0][6];
	}
	else if(cmd == 0x36)
	{
		p = pWorkFile[1];
		fileName = &selectFile[1][6];
	}
	if(p != NULL)
	{
		memcpy(&p->FilePara, &buf[2], sizeof(FilePara_st));
		fileSize = 4 + sizeof(FilePara_st) + p->total * sizeof(work_node);
		DBG(0); // 文件太大有可能内存中放不下
		//writeFile(fileName, (u8 *)p, fileSize); // ...........................................................
	}
	*/
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = buf[0];
	uartaData.sendBuf.buf[pos++] = buf[1];
	uartaData.sendBuf.len = pos;
	uartaData.sendBuf.valid = true;
}

static void loadNodeParam(work_node * node)
{
	static float sendSnSpeed; // 送锡速度
	int type = node->cmd_type;
	
	if(type == TYPE_EMPTY_MOVE)
	{
		setUserParam(0, 3);
		setUserParam(1, node->pos[0]);
		setUserParam(2, node->pos[1]);
		setUserParam(3, node->pos[2]);
		setUserParam(4, node->pos[3]);
		setUserParam(5, node->speed);
	}
	else if(type == TYPE_NODE_WELD)
	{
		weld_para * weld = &node->public1.weld;
		float speedX = weld->speed;
		if(speedX > 100)
		{
			DBG(0);
			speedX = 100; // 这个地方待确认
		}
		setUserParam(0, 4);
		setUserParam(1, node->pos[0]);
		setUserParam(2, node->pos[1]);
		setUserParam(3, node->pos[2]);
		setUserParam(4, node->pos[3]);
		setUserParam(5, node->speed);
		
		if(globalParam.SpeedPara[5].reserve[2] < 1)
		{
			setUserParam(6, speedX);
		}
		else
		{
			setUserParam(6, 1.0 * globalParam.SpeedPara[5].reserve[2] / globalParam.SpeedPara[5].speed * 100);
		}
		setUserParam(7, weld->send_tin[0]);
		setUserParam(8, speedX);
		setUserParam(9, weld->back_tin[0]);
		setUserParam(10, weld->heat_gun);

		setUserParam(11, speedX);
		setUserParam(12, weld->send_tin[1]);
		setUserParam(13, speedX);
		setUserParam(14, weld->back_tin[1]);
		setUserParam(15, 0);

		setUserParam(16, speedX);
		setUserParam(17, weld->send_tin[2]);
		setUserParam(18, speedX);
		setUserParam(19, weld->back_tin[2]);
		setUserParam(20, weld->delay);

		setUserParam(21, weld->height);
		setUserParam(22, weld->distance);
		setUserParam(23, weld->ahead_speed);
		if(globalParam.UserPara.speed1 > 0.1f)
		{
			setUserParam(24, globalParam.UserPara.speed1);
		}
		else
		{
			setUserParam(24, node->speed);
		}
		setUserParam(25, weld->is_offset);
		setUserParam(26, weld->gun_height);
	}
	else if(type == TYPE_LINE_START)
	{
		weld_para * weld = &node->public1.weld;
		sendSnSpeed = weld->speed;
		setUserParam(0, 5);
		setUserParam(1, node->pos[0]);
		setUserParam(2, node->pos[1]);
		setUserParam(3, node->pos[2]);
		setUserParam(4, node->pos[3]);
		setUserParam(5, node->speed);

		if(globalParam.SpeedPara[5].reserve[2] < 1)
		{
			setUserParam(6, sendSnSpeed);
		}
		else
		{
			setUserParam(6, 1.0 * globalParam.SpeedPara[5].reserve[2] / globalParam.SpeedPara[5].speed * 100);
		}
		setUserParam(7, weld->send_tin[0]);
		setUserParam(8, sendSnSpeed);
		setUserParam(9, weld->back_tin[0]);
		setUserParam(10, weld->heat_gun);

		setUserParam(11, weld->height);
		setUserParam(12, weld->distance);
		setUserParam(13, weld->ahead_speed);
		if(globalParam.UserPara.speed1 > 0.1f)
		{
			setUserParam(14, globalParam.UserPara.speed1);
		}
		else
		{
			setUserParam(14, node->speed);
		}
	}
	else if(type == TYPE_LINE_WELD)
	{
		setUserParam(0, 6);
		setUserParam(1, node->pos[0]);
		setUserParam(2, node->pos[1]);
		setUserParam(3, node->pos[2]);
		setUserParam(4, node->pos[3]);
		setUserParam(5, node->speed);
		setUserParam(6, sendSnSpeed);
	}
	else if(type == TYPE_LINE_END)
	{
		weld_para * weld = &node->public1.weld;
		setUserParam(0, 7);
		setUserParam(1, node->pos[0]);
		setUserParam(2, node->pos[1]);
		setUserParam(3, node->pos[2]);
		setUserParam(4, node->pos[3]);
		setUserParam(5, node->speed);
		setUserParam(6, sendSnSpeed);
		setUserParam(7, weld->close_tin);
		setUserParam(8, 0);
		setUserParam(9, weld->delay);
		
		setUserParam(10, weld->height);
		setUserParam(11, weld->distance);
		setUserParam(12, weld->ahead_speed);
		setUserParam(13, 80);
		setUserParam(14, weld->gun_height);
		setUserParam(15, 100);
	}
	else if(type == TYPE_MOTO_RESET)
	{
		reset_para * rst = &node->public1.reset;
		setUserParam(0, 10);
		setUserParam(1, rst->reset[0]);
		setUserParam(2, rst->reset[1]);
		setUserParam(3, rst->reset[2]);
		setUserParam(4, rst->reset[3]);
	}
	else if(type == TYPE_WASH_GUN)
	{
		setUserParam(0, 11);
		setUserParam(1, globalParam.UserPara.pos[0]);
		setUserParam(2, globalParam.UserPara.pos[2]);
		setUserParam(3, globalParam.UserPara.pos[4]);
		setUserParam(4, globalParam.UserPara.add_tin);
		setUserParam(5, 100);
		setUserParam(6, globalParam.UserPara.pos[1]);
	}
	else if(type == TYPE_DELAY)
	{
		setUserParam(0, 14);
		setUserParam(1, node->public1.delay);
	}
	else if(type == TYPE_OUTPUT)
	{
		setUserParam(0, 9);
		setUserParam(1, node->public1.io.port);
		setUserParam(2, node->public1.io.value);
	}
	else if(type == TYPE_INPUT)
	{
		setUserParam(0, 8);
		setUserParam(1, node->public1.io.port);
		setUserParam(2, node->public1.io.value);
	}
	else
	{
		DBG(0);
	}
}

static void gotoWorkNode(int LorR, work_node * p)
{
	loadNodeParam(p);
	if(LorR == 0)
	{
		setUserBit(11, 1);
	}
	else
	{
		setUserBit(12, 1);
	}
}

static void sysResetDelay(int ms)
{
	resetStartTick = getTick() + ms;
}

// buf[0]: id
// buf[1]: cmd
static void uartaOnRecvFrame(u8 * buf, u32 len)
{
	u8 cmd = buf[0];
	int pos = 0;
	
	uartaData.sendBuf.buf[pos++] = 0xbb;
	uartaData.sendBuf.buf[pos++] = cmd;
	if(cmd == 0x24) // 读文件
	{
		adtReadFile(buf, len);
	}
	else if(cmd == 0x25) // 写文件
	{
		adtWriteFile(buf, len);
	}
	else if(cmd == 0x26)
	{
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x27)
	{
		int xstatus[2] = {0};
		int i;
		for(i = 0; i < 64; i++)
		{
			if(getX(i))
			{
				xstatus[i / 32] |= (1 << (i % 32));
			}
		}
		uartaData.sendBuf.buf[pos++] = sizeof(xstatus);
		memcpy(&uartaData.sendBuf.buf[pos], xstatus, sizeof(xstatus));
		pos += sizeof(xstatus);
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x28)
	{
		setY(buf[2], buf[3]);
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x2b) // 版本信息
	{
		uartaData.sendBuf.buf[pos++] = sizeof(MotionVer);
		memcpy(&uartaData.sendBuf.buf[pos], MotionVer, sizeof(MotionVer));
		strcpy((char *)&uartaData.sendBuf.buf[pos + 150], getCodeVersion());
		pos += sizeof(MotionVer);
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x2c) // 按键指令
	{
		memcpy(&manualCmd, &buf[2], sizeof(manualCmd));
		if(manualCmd.CmdType == 0x50) // dummy
		{
			
		}
		// adt: x y z y2 r c
		// my:  x y z a  b c
		#define MOTOR_MOVE_1(x, y, z) \
		else if((manualCmd.CmdType == x) && (runStatus.status == 0)) \
		{ \
			float speed; \
			float distance; \
			int axisAtAdt = y; \
			if(y == 3) \
			{ \
				axisAtAdt = 4; \
			} \
			else if(y == 4) \
			{ \
				axisAtAdt = 3; \
			} \
			if(manualCmd.SpeedRate == 1) \
			{ \
				speed = globalParam.SpeedPara[axisAtAdt].reserve[1] ; \
			} \
			else \
			{ \
				speed = globalParam.SpeedPara[axisAtAdt].reserve[0]; \
			} \
			if(manualCmd.PointMove < 0.009f) \
			{ \
				distance = z * 99999; \
			} \
			else \
			{ \
				distance = manualCmd.PointMove; \
			} \
			pmove(y, 0, speed / 60.0f * sysParam.motorParam[y].ppr, distance / sysParam.motorParam[y].dpr * sysParam.motorParam[y].ppr, true); \
		}
		MOTOR_MOVE_1(0x41, 0, 1)
		MOTOR_MOVE_1(0x42, 0, -1)
		MOTOR_MOVE_1(0x43, 1, 1)
		MOTOR_MOVE_1(0x44, 1, -1)
		MOTOR_MOVE_1(0x45, 2, 1)
		MOTOR_MOVE_1(0x46, 2, -1)
		MOTOR_MOVE_1(0x48, 3, 1)
		MOTOR_MOVE_1(0x47, 3, -1)
		MOTOR_MOVE_1(0x49, 4, 1)
		MOTOR_MOVE_1(0x4a, 4, -1)
		MOTOR_MOVE_1(0x4b, 5, 1)
		MOTOR_MOVE_1(0x4c, 5, -1)
		else if((manualCmd.CmdType == 0x4d) && (runStatus.status == 0)) // 定位到加工点,工位0
		{
			gotoWorkNode(0, &manualCmd.node);
		}
		else if((manualCmd.CmdType == 0x4e) && (runStatus.status == 0)) // 定位到加工点,工位1
		{
			gotoWorkNode(1, &manualCmd.node);
		}
		else if((manualCmd.CmdType == 0x4f) && (runStatus.status == 0))
		{
			setUserBit(41, 1);
		}
		else if(manualCmd.CmdType == 0x51) // stop,X+之类的按键,按下电机动,松开电机停
		{
			int i;
			for(i = 0; i < 6; i++)
			{
				motorStopSoft(i);
			}
		}
		else if((manualCmd.CmdType == 0x52)) // 启动工位0
		{
			runStatus.startNode = manualCmd.SpeedRate;
			setUserBit(5, 1);
		}
		else if((manualCmd.CmdType == 0x53)) // 启动工位1
		{
			runStatus.startNode = manualCmd.SpeedRate;
			setUserBit(6, 1);
		}
		else if(manualCmd.CmdType == 0x54) // 急停
		{
			setUserBit(7, 1);
		}
		else
		{
			DBG(0);
			printData(buf, len);
		}
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x2d) // 读工作状态
	{
		uartaData.sendBuf.buf[pos++] = sizeof(workStatus);
		workStatus.xpulse = motorGetPos(0);
		workStatus.ypulse = motorGetPos(1);
		workStatus.zpulse = motorGetPos(2);
		workStatus.apulse = motorGetPos(4);
		workStatus.bpulse = motorGetPos(3);
		workStatus.cpulse = motorGetPos(5);
		workStatus.cur_node = runStatus.curNode;
		workStatus.finish_piece = runStatus.workCount;
		workStatus.work_state = 0;
		if((runStatus.status > 0) && (runStatus.status < 20)) // 状态机在运行状态下
		{
			if(!workStatus.is_step) // 不在单步运行模式
			{
				if(runStatus.pause) // 暂停
				{
					workStatus.work_state = 1;
				}
				else
				{
					workStatus.work_state = 2;
				}
			}
		}
		workStatus.work_time = runStatus.workTickTotal / 1000;
		workStatus.weld_num = getUserParam(36);
		workStatus.alarm = getAlarmNumber();
		memcpy(&uartaData.sendBuf.buf[pos], &workStatus, sizeof(workStatus));
		pos += sizeof(workStatus);
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x2e) // 写工作状态
	{
		memcpy(&workStatus, &buf[2], sizeof(workStatus));
		if(workStatus.finish_piece == 0)
		{
			runStatus.workCount = 0;
		}
		setUserParam(36, workStatus.weld_num);
		setUserBit(13, workStatus.is_send_tin);
		setUserBit(14, workStatus.is_cycle);
		setUserBit(18, workStatus.is_step);
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x2f)
	{
		adtCopyFile(buf, len);
	}
	else if(cmd == 0x30)
	{
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if((cmd == 0x31) || (cmd == 0x36)) // 更改加工文件头部文件参数部分
	{
		modifyFilePara(buf, len);
	}
	else if(cmd == 0x32) // 手持盒通知控制器重新加载加工文件
	{
		reloadWorkFile(buf, len);
	}
	else if(cmd == 0x33)
	{
		saveCurSelFile(buf, len);
	}
	else if(cmd == 0x34)
	{
		uartaData.sendBuf.buf[pos++] = sizeof(selectFile);
		memcpy(&uartaData.sendBuf.buf[pos], selectFile, sizeof(selectFile));
		pos += sizeof(selectFile);
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x35)
	{
		sysResetDelay(200);
		uartaData.sendBuf.buf[pos++] = buf[1];
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else
	{
		DBG(0);
		printData(buf, len);
	}
}

static void uartaRecvScan(void)
{
	int frameLen = 0xffff;
	
	if(uartaData.recvBuf.len > 4)
	{
		if((uartaData.recvBuf.buf[1] == 0x24) || (uartaData.recvBuf.buf[1] == 0x25))
		{
			frameLen = 6 + uartaData.recvBuf.buf[3];
		}
		else if((uartaData.recvBuf.buf[1] == 0x27) || (uartaData.recvBuf.buf[1] == 0x2b)
			|| (uartaData.recvBuf.buf[1] == 0x2d) || (uartaData.recvBuf.buf[1] == 0x30)
			|| (uartaData.recvBuf.buf[1] == 0x34) || (uartaData.recvBuf.buf[1] == 0x35))
		{
			frameLen = 5;
		}
		else if((uartaData.recvBuf.buf[1] == 0x28) || (uartaData.recvBuf.buf[1] == 0x2c)
			|| (uartaData.recvBuf.buf[1] == 0x2e) || (uartaData.recvBuf.buf[1] == 0x2f)
			|| (uartaData.recvBuf.buf[1] == 0x31) || (uartaData.recvBuf.buf[1] == 0x32)
			|| (uartaData.recvBuf.buf[1] == 0x33) || (uartaData.recvBuf.buf[1] == 0x36))
		{
			frameLen = 5 + uartaData.recvBuf.buf[2];
		}
		else
		{
			lprintf("uartaRecvScan: fc=%02x\n", uartaData.recvBuf.buf[1]);
			uartaData.recvBuf.len = 0;
		}
	}
	if(uartaData.recvBuf.len >= frameLen)
	{
		if(isModbusFrameCrcValid(uartaData.recvBuf.buf, uartaData.recvBuf.len))
		{
			uartaOnRecvFrame(&uartaData.recvBuf.buf[1], uartaData.recvBuf.len - 3);
		}
		else
		{
			lprintf("uartaRecvScan: frame err, fc=%02x, len=%d\n", uartaData.recvBuf.buf[1], uartaData.recvBuf.len);
		}
		uartaData.recvBuf.len = 0;
	}

	// send scan
	if(uartaData.sendBuf.valid)
	{
		uartaSend();
		uartaData.sendBuf.valid = false;
	}
}

//------------------------------------------------------------------------------

static void clearY(void)
{
	u32 i;

	for(i = 0; i < OUTPUT_PIN_NUM; i++)
	{
		setY(i, 0);
	}
}

static bool userWaitInput(void)
{
	u32 idx = getUserParam(1);
	u32 val = getUserParam(2);

	if(val)
	{
		val = 1;
	}
	return getX(idx) == val;
}

static void userSetOutput(void)
{
	u32 idx = getUserParam(1);
	u32 val = getUserParam(2);

	setY(idx, val);
}
/*
// 主调函数要做容错处理
static void loadNode(int LorR, int idx)
{
	work_file * p = pWorkFile[LorR];
	work_node * node = &p->work_file;

	if(p->nodeId != idx)
	{
		char * fileName = &selectFile[LorR][6];
		u32 pos = (u32)node - (u32)p + idx * sizeof(work_node);
		readFile(fileName, pos, (u8 *)node, sizeof(work_node));
		p->nodeId = idx;
	}
	loadNodeParam(node);
	if(LorR == 0)
	{
		setUserBit(3, 1);
	}
	else
	{
		setUserBit(4, 1);
	}
}

// 启动一个加工点
static void runNode(void)
{
	u32 LorR = getUserBit(15);
	u32 idx = getUserParam(30);

	if((idx > pWorkFile[LorR]->total) || (pWorkFile[LorR]->total < 1))
	{
		if(workingTick > 0)
		{
			workingTick -= getTick();
		}
		setUserParam(30, -1);
		return;
	}
	if(idx == 0)
	{
		workingTick = getTick();
	}
	if(idx == pWorkFile[LorR]->total) // 回停止位
	{
		if(globalParam.UserPara.reset_piece > 0)
		{
			if(getUserParam(37) >= globalParam.UserPara.reset_piece)
			{
				setUserParam(37, 0);
				setUserParam(30, idx - 1);
				setUserParam(0, 24);
				if(LorR == 0)
				{
					setUserBit(3, 1);
				}
				else
				{
					setUserBit(4, 1);
				}
				return;
			}
		}
		if(globalParam.UserPara.wash_piece > 0)
		{
			if(getUserParam(38) >= globalParam.UserPara.wash_piece)
			{
				setUserParam(38, 0);
				setUserParam(30, idx - 1);
				setUserParam(0, 11);
				setUserParam(1, globalParam.UserPara.pos[0]);
				setUserParam(2, globalParam.UserPara.pos[2]);
				setUserParam(3, globalParam.UserPara.pos[4]);
				setUserParam(4, globalParam.UserPara.add_tin);
				setUserParam(5, 100);
				if(LorR == 0)
				{
					setUserBit(3, 1);
				}
				else
				{
					setUserBit(4, 1);
				}
				return;
			}
		}
		if(globalParam.UserPara.is_goto_stop_pos)
		{
			setUserParam(0, 23);
			setUserParam(1, globalParam.UserPara.stop_pos[0]);
			setUserParam(2, globalParam.UserPara.stop_pos[1]);
			setUserParam(3, globalParam.UserPara.stop_pos[2]);
			setUserParam(4, globalParam.UserPara.stop_pos[4]);
			if(LorR == 0)
			{
				setUserBit(3, 1);
			}
			else
			{
				setUserBit(4, 1);
			}
		}
		else
		{
			if(workingTick > 0)
			{
				workingTick -= getTick();
			}
			setUserParam(30, -1);
		}
		return;
	}
	loadNode(LorR, idx);
}
*/
static void userAlarm(void)
{
	int status = 0;
	
	if(getX(11))
	{
		status = 1;
	}
	else if(getX(12))
	{
		status = 2;
	}
	setUserParam(31, status);
}

static void getWashPos(void)
{
	setUserParam(1, globalParam.UserPara.pos[0]);
	setUserParam(2, globalParam.UserPara.pos[2]);
	setUserParam(3, globalParam.UserPara.pos[4]);
	setUserParam(4, globalParam.UserPara.add_tin);
	setUserParam(5, 100);
	setUserParam(6, globalParam.UserPara.pos[1]);
}

//------------------------------------------------------------------------------

static bool loadNodeA(int LorR, int idx)
{
	work_file * p = pWorkFile[LorR];
	work_node * node = &p->work_file;

	if(idx >= p->total)
	{
		return false;
	}
	if(p->nodeId != idx)
	{
		char * fileName = &selectFile[LorR][6];
		u32 pos = (u32)node - (u32)p + idx * sizeof(work_node);
		u32 ret = readFile(fileName, pos, (u8 *)node, sizeof(work_node));
		if(ret != 1)
		{
			return false;
		}
		p->nodeId = idx;
	}
	loadNodeParam(node);
	if(LorR == 0)
	{
		setUserBit(3, 1);
	}
	else
	{
		setUserBit(4, 1);
	}
	return true;
}

static void userReset(void)
{
	int i;
	int tmp;
	
	clearMotorResetStatus();
	inpReset();
	for(i = 0; i < MOTOR_NUM; i++)
	{
		motorStopSoft(i);
	}
	for(i = 0; i < OUTPUT_PIN_NUM; i++)
	{
		setY(i, 0);
	}
	clearManualStatus();
	clearAutoStatus();
	clearAlwaysRunStatus();
	tmp = getUserBit(42);
	clearUserBits();
	clearUserParam();
	setUserParam(34, 1.0 * globalParam.SpeedPara[5].reserve[0] / globalParam.SpeedPara[5].speed * 100);
	setUserParam(35, 1.0 * globalParam.SpeedPara[5].reserve[1] / globalParam.SpeedPara[5].speed * 100);
	if(globalParam.SpeedPara[0].reserve[2] >= 1) // X Y Z Y2 R C
	{
		setUserParam(40, globalParam.SpeedPara[0].reserve[2]);
	}
	if(globalParam.SpeedPara[1].reserve[2] >= 1)
	{
		setUserParam(41, globalParam.SpeedPara[1].reserve[2]);
	}
	if(globalParam.SpeedPara[2].reserve[2] >= 1)
	{
		setUserParam(42, globalParam.SpeedPara[2].reserve[2]);
	}
	if(globalParam.SpeedPara[4].reserve[2] >= 1)
	{
		setUserParam(43, globalParam.SpeedPara[4].reserve[2]);
	}
	setUserBit(13, workStatus.is_send_tin);
	setUserBit(42, tmp);
	{
		int workCount = runStatus.workCount;
		memset(&runStatus, 0, sizeof(RunStatus));
		runStatus.workCount = workCount;
	}
}

static void scanUserKey(void)
{
	if(getUserBit(5) == 1) // 启动工位0
	{
		runStatus.startFlag |= 1;
		setUserBit(5, 0);
		if(globalParam.FactoryPara.StartAndPause != 0) // 启动/暂停复用
		{
			runStatus.pause ^= 1;
		}
		else if(runStatus.curWorkSel == 0) // 工位0,暂停后,再按一下对应工位的启动键,可退出暂停模式,继续加工
		{
			runStatus.pause = 0;
		}
	}
	if(getUserBit(6) == 1) // 启动工位1
	{
		runStatus.startFlag |= 2;
		setUserBit(6, 0);
		if(globalParam.FactoryPara.StartAndPause != 0) // 启动/暂停复用
		{
			runStatus.pause ^= 1;
		}
		else if(runStatus.curWorkSel == 0) // 工位1
		{
			runStatus.pause = 0;
		}
	}
	if(getUserBit(7)) // 停止
	{
		setUserBit(7, 0);
		userReset();
	}
	if(getUserBit(8)) // 暂停
	{
		setUserBit(8, 0);
		runStatus.pause = 1;
	}
	if(getUserBit(9)) // 复位
	{
		setUserBit(9, 0);
		if(runStatus.status == 0) // 不在加工状态才可以复位
		{
			setUserBit(41, 1);
			runStatus.status = 20;
		}
	}
	if(getUserBit(10)) // 清洗
	{
		setUserBit(10, 0);
		if(runStatus.status == 0) // 不在加工状态才可以清洗
		{
			getWashPos();
			setUserParam(0, 11);
			setUserBit(3, 1);
			runStatus.status = 21;
		}
	}
}

static void hanXi(void)
{
	bool flag;
	
	if(getTick() > resetStartTick)
	{
		reset();
	}
	if(getAlarmNumber())
	{
		return;
	}
	init5600();
	scanUserKey();
	if(runStatus.status == 0)
	{
		if(runStatus.startFlag & 1) // 工位0启动
		{
			runStatus.curNode = runStatus.startNode;
			runStatus.curWorkSel = 0;
			runStatus.pause = false;
			if(getUserBit(42) == 0) // 未复位
			{
				setUserBit(41, 1);
			}
			setUserBit(16, 0);
			setUserBit(17, 0);
			runStatus.workTickStart = getTick();
			runStatus.status++;
		}
		else if(runStatus.startFlag & 2) // 工位1启动
		{
			runStatus.curNode = runStatus.startNode;
			runStatus.curWorkSel = 1;
			runStatus.pause = false;
			if(getUserBit(42) == 0) // 未复位
			{
				setUserBit(41, 1);
			}
			runStatus.workTickStart = getTick();
			runStatus.status++;
		}
	}
	else if(runStatus.status == 1)
	{
		if(getUserBit(41) == 0) // 复位完毕
		{
			if(getUserBit(16) == 0) // 尚未开始测温
			{
				setUserBit(16, 1);
				setUserParam(1, globalParam.UserPara.stop_pos[0]);
				setUserParam(2, globalParam.UserPara.stop_pos[1]);
				setUserParam(3, globalParam.UserPara.stop_pos[2]);
				return;
			}
			if(getUserBit(17) == 0) // 测温尚未完毕
			{
				return;
			}
			// 测温完毕,可以开始进行后续的工作
			flag = loadNodeA(runStatus.curWorkSel, runStatus.curNode++); // 加载加工点
			if(flag)
			{
				runStatus.status++;
			}
			else if(runStatus.curNode == 1) // 第0个加工点加载失败,不算
			{
				if(workStatus.is_step) // 单步运行模式
				{
					runStatus.status = 0;
					runStatus.startNode = 0;
					if(runStatus.curWorkSel == 0)
					{
						runStatus.startFlag &= ~1;
					}
					else
					{
						runStatus.startFlag &= ~2;
					}
				}
				else if(workStatus.is_cycle) // 循环运行模式
				{
					runStatus.status = 13; // 跳到最后一步,切换加工工位
				}
				else // 正常运行模式
				{
					runStatus.status = 0;
					runStatus.startNode = 0;
					if(runStatus.curWorkSel == 0)
					{
						runStatus.startFlag &= ~1;
					}
					else
					{
						runStatus.startFlag &= ~2;
					}
				}
			}
			else // 整板加工完毕
			{
				if(workStatus.is_step) // 单步运行模式
				{
					runStatus.status = 0;
					if(runStatus.curWorkSel == 0)
					{
						runStatus.startFlag &= ~1;
					}
					else
					{
						runStatus.startFlag &= ~2;
					}
				}
				else // 正常运行模式
				{
					runStatus.workCount++;
					runStatus.workTickTotal = getTick() - runStatus.workTickStart;
					runStatus.status = 10;
				}
			}
		}
	}
	else if(runStatus.status == 2)
	{
		if(((runStatus.curWorkSel == 0) && (getUserBit(3) == 0)) ||
			((runStatus.curWorkSel == 1) && (getUserBit(4) == 0))) // 单点加工完毕
		{
			if(workStatus.is_step) // 单步运行模式
			{
				runStatus.startFlag &= ~3;
			}
			else // 正常运行模式
			{
				if(runStatus.pause) // 暂停
				{
					runStatus.pauseZ = motorGetPos(2);
					pmove(2, 0, sysParam.motorParam[2].speed * sysParam.motorParam[2].ppr / 60,
						-runStatus.pauseZ, true);
				}
			}
			runStatus.status++;
		}
	}
	else if(runStatus.status == 3)
	{
		if(workStatus.is_step) // 单步运行模式
		{
			if(runStatus.curWorkSel == 0) // 工位0
			{
				if(runStatus.startFlag & 1) // 检测启动按钮
				{
					runStatus.status = 1;
				}
			}
			else // 工位1
			{
				if(runStatus.startFlag & 2) // 检测启动按钮
				{
					runStatus.status = 1;
				}
			}
		}
		else if(!runStatus.pause) // 不在暂停状态
		{
			if(runStatus.pauseZ < 0) // z先回到原来的位置
			{
				if(!motorIsRunning(2)) // z停止状态
				{
					pmove(2, 0, sysParam.motorParam[2].speed * sysParam.motorParam[2].ppr / 60,
						runStatus.pauseZ, true);
					runStatus.pauseZ = 0;
					runStatus.status++;
				}
			}
			else
			{
				runStatus.status = 1;
			}
		}
	}
	else if(runStatus.status == 4)
	{
		if(!motorIsRunning(2)) // z停止状态
		{
			runStatus.status = 1;
		}
	}
	else if(runStatus.status == 10) // 判断是否要复位
	{
		if(globalParam.UserPara.reset_piece > 0)
		{
			if((runStatus.workCount % globalParam.UserPara.reset_piece) == 0)
			{
				setUserBit(41, 1);
			}
		}
		runStatus.status++;
	}
	else if(runStatus.status == 11) // 判断是否要清洗
	{
		if(getUserBit(41) == 0) // 复位完毕
		{
			if(globalParam.UserPara.wash_piece > 0)
			{
				if((runStatus.workCount % globalParam.UserPara.wash_piece) == 0)
				{
					setUserParam(0, 11);
					setUserParam(1, globalParam.UserPara.pos[0]); // X Y Z Y2 R C
					setUserParam(2, globalParam.UserPara.pos[2]);
					setUserParam(3, globalParam.UserPara.pos[4]);
					setUserParam(4, globalParam.UserPara.add_tin);
					setUserParam(5, 100);
					setUserBit(3, 1);
				}
			}
			runStatus.status++;
		}
	}
	else if(runStatus.status == 12) // 判断是否要回停止位
	{
		if(((runStatus.curWorkSel == 0) && (getUserBit(3) == 0)) ||
			((runStatus.curWorkSel == 1) && (getUserBit(4) == 0))) // 清洗完毕
		{
			if(globalParam.UserPara.is_goto_stop_pos)
			{
				setUserParam(0, 23);
				setUserParam(1, globalParam.UserPara.stop_pos[0]);
				setUserParam(2, globalParam.UserPara.stop_pos[1]);
				setUserParam(3, globalParam.UserPara.stop_pos[2]);
				setUserParam(4, globalParam.UserPara.stop_pos[4]);
				if(runStatus.curWorkSel == 0)
				{
					setUserBit(3, 1);
				}
				else
				{
					setUserBit(4, 1);
				}
			}
			runStatus.status++;
		}
	}
	else if(runStatus.status == 13)
	{
		if(((runStatus.curWorkSel == 0) && (getUserBit(3) == 0)) ||
			((runStatus.curWorkSel == 1) && (getUserBit(4) == 0))) // 回停止位完毕
		{
			if(runStatus.curWorkSel == 0)
			{
				runStatus.startFlag &= ~1;
			}
			else
			{
				runStatus.startFlag &= ~2;
			}
			runStatus.startNode = 0;
			if(workStatus.is_cycle) // 循环加工
			{
				if(runStatus.curWorkSel == 0)
				{
					runStatus.curWorkSel = 1;
				}
				else
				{
					runStatus.curWorkSel = 0;
				}
				runStatus.curNode = 0;
				runStatus.workTickStart = getTick();
				runStatus.status = 1;
			}
			else // 正常加工结束
			{
				runStatus.status = 0;
			}
		}
	}
	else if(runStatus.status == 20)
	{
		if(getUserBit(41) == 0) // 按键复位完毕
		{
			runStatus.status = 0;
		}
	}
	else if(runStatus.status == 21)
	{
		if(getUserBit(3) == 0) // 按键清洗完毕
		{
			runStatus.status = 0;
		}
	}
}

