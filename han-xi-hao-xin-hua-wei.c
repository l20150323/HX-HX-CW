/*
���ڽ����Ƽ����޹�˾
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
	TYPE_EMPTY_MOVE = 0, // ����
	TYPE_NODE_WELD, // 1,�㺸
	TYPE_LINE_START, // 2,�Ϻ����
	TYPE_LINE_WELD, // 3,�Ϻ�
	TYPE_LINE_END, // 4,�Ϻ��յ�
	TYPE_MOTO_RESET, // 5,�����λ
	TYPE_WASH_GUN, // 6,��ϴ����		
	TYPE_DELAY, // 7,��ʱ��ͣ
	TYPE_WORK_OVER, // 8,�ӹ�����
	TYPE_OUTPUT, // 9,�˿����
	TYPE_INPUT, // 10,�ȴ�����
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
	s16 pin_number;//���ź�
	s16 pin_type;  //��ƽ��Чֵ  ���� ����1 ����0
	s16 pin_fun;   //���ܺ�(���ڲ��б�) 
	s16 pin_state; //��ǰ״̬	
} IONode_st;

typedef struct //���̲���
{
	int 	Password;		//����
	int 	Set_password;	//�Ƿ���������
	char 	PowerDown;		//���籣��
	char	StartAndPause;	//������ť����ͣ����
	char	ReadyToPos; // 1-ʹ��Y2, 0-����Y2

	short	RZero_offset;	//R��ԭ��ƫ�ƣ������������˳�
	
	int		reserve[10];	//Ԥ������
	
	MotorPara_st MotorPara[6];	//�������
	
	IONode_st input_io[34];	//�±�Ϊ�˿ں�
	IONode_st output_io[18];	//�±�Ϊ�˿ں�
	
} FactoryPara_st;

typedef struct //�û�����
{
	s16 cycle_piece;		//ѭ���ӹ��ܴ���
	s16 reset_piece;		//��λ���
	s16 wash_piece;		//��ϴ���---�ӹ�ʱ���ӹ����ٱ���ϴ
	s16 cycle_file_num;	//ѭ���ӹ��ļ�����
	s16 is_cycle;			//�Ƿ�ѭ���ӹ�
	s16 is_reset_y2;		//�Ƿ�λY2��
	
	
	float cycle_time;		//ѭ���ӹ����--���ҹ�λ�л��ļ��ʱ��
	float reset_time;		//������ʱ��λʱ��
	float safe_height;		//Z�ᰲȫ�߶�
	float add_tin;			// ��ϴʱ��
	int   max_weld_time;	//���㺸����;
	float plt_scale;		//PLT�ļ�ת������
	
	
	float pos[6];	//��ϴλ��

	int   Ip_Address;		//IP��ַ�����λ

	int   wash_time;		//����ʱ���������ϴһ��(s)
	float stop_pos[6];//ֹͣλ��
	int	  is_wash_y;		//��ϴ�Ƿ�ʹ��Y��  
	int   is_goto_stop_pos;	//�ӹ����Ƿ��ֹͣλ��
	char  filelist[2][1][15];
	float speed1; // �����ٶ�
}UserPara_st;

typedef struct SpeedPara_st //�ٶȲ���
{
	int speed; // �����ٶ�
	int acc; // ���ٶ�
	int reserve[3]; /* ����XYZA,reserve��ʾ�ֶ�����/�ֶ�����/��λ�ٶ�.����C,reserve��ʾ�����ٶ�/�����ٶ�/��ǰ�����ٶ� */
} SpeedPara_st;

typedef struct
{
	int ver; // 1
	u32 rand; // ���յ��������ݲ���д��
	u32 CheckSum;	
	FactoryPara_st 	FactoryPara;
	UserPara_st   	UserPara;
	SpeedPara_st	SpeedPara[6];
} GlobalPara_st;

//����״̬��Ϣ
typedef struct
{
	long  xpulse;	//��ǰλ��
	long  ypulse;
	long  zpulse;
	long  apulse;
	long  bpulse;
	long  cpulse;

	u16 total_node;	//�ܼӹ�����
	u16 cur_node;	//��ǰ�ӹ����
	u16 finish_piece;//��ɼӹ�����		

	u8 work_state;	//�ӹ�״̬,
	u8 cur_work_pos;	//��ǰ��λ
	u8 is_cycle;		//ѭ��
	u8 is_step;		//����
	u8 is_send_tin;	//�Ƿ�����

	int work_time;		//�ӹ�ʱ��
	int weld_num;		//�������
	int alarm; 			// bit0:����, bit1:����, bit2:��ȫ��

	int file_change[2];	//�ļ��޸ı�־,ɨ�赽BCD��֮�����ˢ��
	int file_BCD;		//�ļ���BCD��

	int	reserver[28];	//��������	
}work_st;

typedef struct //�������в���
{
	float gun_height;		//ǹ��(mm)
	float send_tin[3];		//����(s)
	float back_tin[3];		//����(s)
	float close_tin;		//��ǰ����(mm)
	float speed;			//�����ٶ�(mm/s)
	float ahead_speed;		//��ǰ�����ٶ�(mm/s),��������y

	float heat_gun;			//��ǹԤ��ʱ��(s)
	float delay;			//ͣ��ʱ��(s)

	float distance;			//ƫ�ƾ���x(mm)
	float height;			//ƫ�Ƹ߶�(mm)
	int is_offset;			//�Ƿ��˳�	0���˳� 1�˳�
} weld_para;

typedef struct //IO�������в���
{
	s8 port;
	s8 value;
	float time;
} io_para;

typedef struct //�����λ���в���
{
	char reset[4];	//�������ѭ��
} reset_para;


typedef union
{
	weld_para	weld;
	io_para	  	io;
	reset_para	reset;
	float		delay;
} PUBLIC_1;

typedef struct //�̵��ӹ���
{
	char cmd_type;	//ָ��
	char group;		//���	0��ʾ�����飬ֻ�к�����ص����Ͳ��������ͬ��ĺ��Ӳ�����ͬ
	float speed;	// �ٶ�,%
	
	//��������
	float	pos[4];	//�������ѭ��
	PUBLIC_1 public1;
} work_node;

typedef	struct
{
	u8		CmdType;
	u8		SpeedRate;
	float		PointMove;
	work_node	node;		//�ӹ��㶨λ��Ϣ	
} ManualCmd;

typedef struct
{
	char d_name[13];
	char ext_name[4];
	char FAT_DirAttr;
	int size;
} FILE_INFO_5600;

typedef struct // �ļ�����
{
	float close_tin;	// ��ǰ��������
	float offset_speed;	// �����ٶ�
	int wash_piece;	// ��ϴ����
	int wash_time;	// ��ϴ��ʱʱ��

	// �Ӿ�У����׼��,ֻ��֪��XY����	
	float stdpoint[2];
	int para[8]; // ��������Ԥ��
} FilePara_st;

typedef struct
{
	int total; //��ǰ�̵��ӹ�����0~N
	FilePara_st FilePara;	//�ļ������������ļ���,����������ͬ
	work_node work_file;
	int nodeId; // ��ǰ���صļӹ�������
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
	bool valid; // true-���ڷ���
	u8 buf[256];
} UARTA_SEND_FRAME;

typedef struct
{
	UARTA_RECV_BUF recvBuf;
	UARTA_RECV_FRAME recvFrame;
	UARTA_SEND_FRAME sendFrame; // buf���ʱ,�պ�'sendBuf.buf'ǰ��Ŀռ��Ѿ��ڳ�����
	UARTA_SEND_BUF sendBuf;
} UARTA_DATA;

typedef struct
{
	float manualLo; // �ֶ�����
	float manualHi; // �ֶ�����
	float washTime; // ��ϴʱ��(s)
} OTHER_PARAM; // �������

typedef struct
{
	int ver;
	int len; // ��Ч����
	u32 crc;
	float dat[1][20];
} WORK_FILE_DATA;

typedef struct
{
	int startFlag; // b0-��λ0����������,b1-��λ1����������
	int status; // ״̬��״̬, 0-idle, 10-�ӹ���, 11-�ж��Ƿ�Ҫ��λ, 12-�ж��Ƿ�Ҫ��ϴ,
		// 13-�ж��Ƿ�Ҫ��ֹͣλ, 14-�ж��Ƿ�ѭ���ӹ�
	int curNode; // ��ǰ�ӹ������
	int curWorkSel; // ��ǰ�ӹ���λ,0 or 1
	int workCount; // �ӹ�����,2����λ�ܹ��ӹ��İ�������
	int startNode; // ��ʼ�ӹ���
	bool pause; // ����/��ͣ����ʱ,��1�ΰ�����,ִ����������,��2�ΰ�����,ִ����ͣ����,
		// ��Ҫ�ȵ�һ���ӹ������,��ͣ����,��˵�2�ΰ�����ʱ,�ò���Ϊtrue,һ���ӹ������ʱ,
		// ͣ����,pauseZ��¼��z����,Ȼ��z̧��0��
	float pauseZ; // ��ͣʱz����
	int workTickStart; // �ӹ�����ʱ��
	int workTickTotal; // �ӹ���ʱ
} RunStatus;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
static UARTA_DATA uartaData;
static UART_HandleTypeDef * sendUart = &huart2;
static char selectFile[2][50];
static int curManualSpeed = 0; // 0:���� 1:����
static OTHER_PARAM otherParam;
static WORK_FILE_DATA * workFileData[2];
static GlobalPara_st globalParam;
static work_st workStatus;
static ManualCmd manualCmd;
static work_file * pWorkFile[2];
static const char MotionVer[3][2][30] = 
{
	{{""},	{""}},
	{{"�˶�������:"},	{"���ش�ʦ-407ZG/JC603X2"}},	
	{{"��������:"},	{__DATE__" "__TIME__}},	
};
static int workingTick; // �ӹ���ʱ,��ֵ��ʾ�ӹ���ʼʱ��(ms),��ֵ��ʾ�ӹ���ʱ(ms)
static RunStatus runStatus;
static u32 stampRand; // ���ֳֺ�ͨ�ŵ������
static u32 resetStartTick = 0xffffffff; // �������ʱ��,ϵͳ����

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

// i-��Ϊ�����, j-ʵ�����
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
			alarmStatus[4 / 32] |= (1 << (4 % 32)); // �ű��б�������һ��Ҫ��
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

// ����ָ����չ�����ļ�����
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

// �ֳֺж�ȡ�ļ�����
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

// �ֳֺ��·��ļ�����
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
						alarmStatus[3 / 32] |= (1 << (3 % 32)); // �ű��б�������һ��Ҫ��
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
	�����ļ�������Ҫ�������ļ�����-�޸�-д��,��ȡ���ļ�����
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
		DBG(0); // �ļ�̫���п����ڴ��зŲ���
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
	static float sendSnSpeed; // �����ٶ�
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
			speedX = 100; // ����ط���ȷ��
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
	if(cmd == 0x24) // ���ļ�
	{
		adtReadFile(buf, len);
	}
	else if(cmd == 0x25) // д�ļ�
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
	else if(cmd == 0x2b) // �汾��Ϣ
	{
		uartaData.sendBuf.buf[pos++] = sizeof(MotionVer);
		memcpy(&uartaData.sendBuf.buf[pos], MotionVer, sizeof(MotionVer));
		strcpy((char *)&uartaData.sendBuf.buf[pos + 150], getCodeVersion());
		pos += sizeof(MotionVer);
		uartaData.sendBuf.len = pos;
		uartaData.sendBuf.valid = true;
	}
	else if(cmd == 0x2c) // ����ָ��
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
		else if((manualCmd.CmdType == 0x4d) && (runStatus.status == 0)) // ��λ���ӹ���,��λ0
		{
			gotoWorkNode(0, &manualCmd.node);
		}
		else if((manualCmd.CmdType == 0x4e) && (runStatus.status == 0)) // ��λ���ӹ���,��λ1
		{
			gotoWorkNode(1, &manualCmd.node);
		}
		else if((manualCmd.CmdType == 0x4f) && (runStatus.status == 0))
		{
			setUserBit(41, 1);
		}
		else if(manualCmd.CmdType == 0x51) // stop,X+֮��İ���,���µ����,�ɿ����ͣ
		{
			int i;
			for(i = 0; i < 6; i++)
			{
				motorStopSoft(i);
			}
		}
		else if((manualCmd.CmdType == 0x52)) // ������λ0
		{
			runStatus.startNode = manualCmd.SpeedRate;
			setUserBit(5, 1);
		}
		else if((manualCmd.CmdType == 0x53)) // ������λ1
		{
			runStatus.startNode = manualCmd.SpeedRate;
			setUserBit(6, 1);
		}
		else if(manualCmd.CmdType == 0x54) // ��ͣ
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
	else if(cmd == 0x2d) // ������״̬
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
		if((runStatus.status > 0) && (runStatus.status < 20)) // ״̬��������״̬��
		{
			if(!workStatus.is_step) // ���ڵ�������ģʽ
			{
				if(runStatus.pause) // ��ͣ
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
	else if(cmd == 0x2e) // д����״̬
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
	else if((cmd == 0x31) || (cmd == 0x36)) // ���ļӹ��ļ�ͷ���ļ���������
	{
		modifyFilePara(buf, len);
	}
	else if(cmd == 0x32) // �ֳֺ�֪ͨ���������¼��ؼӹ��ļ�
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
// ��������Ҫ���ݴ���
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

// ����һ���ӹ���
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
	if(idx == pWorkFile[LorR]->total) // ��ֹͣλ
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
	if(getUserBit(5) == 1) // ������λ0
	{
		runStatus.startFlag |= 1;
		setUserBit(5, 0);
		if(globalParam.FactoryPara.StartAndPause != 0) // ����/��ͣ����
		{
			runStatus.pause ^= 1;
		}
		else if(runStatus.curWorkSel == 0) // ��λ0,��ͣ��,�ٰ�һ�¶�Ӧ��λ��������,���˳���ͣģʽ,�����ӹ�
		{
			runStatus.pause = 0;
		}
	}
	if(getUserBit(6) == 1) // ������λ1
	{
		runStatus.startFlag |= 2;
		setUserBit(6, 0);
		if(globalParam.FactoryPara.StartAndPause != 0) // ����/��ͣ����
		{
			runStatus.pause ^= 1;
		}
		else if(runStatus.curWorkSel == 0) // ��λ1
		{
			runStatus.pause = 0;
		}
	}
	if(getUserBit(7)) // ֹͣ
	{
		setUserBit(7, 0);
		userReset();
	}
	if(getUserBit(8)) // ��ͣ
	{
		setUserBit(8, 0);
		runStatus.pause = 1;
	}
	if(getUserBit(9)) // ��λ
	{
		setUserBit(9, 0);
		if(runStatus.status == 0) // ���ڼӹ�״̬�ſ��Ը�λ
		{
			setUserBit(41, 1);
			runStatus.status = 20;
		}
	}
	if(getUserBit(10)) // ��ϴ
	{
		setUserBit(10, 0);
		if(runStatus.status == 0) // ���ڼӹ�״̬�ſ�����ϴ
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
		if(runStatus.startFlag & 1) // ��λ0����
		{
			runStatus.curNode = runStatus.startNode;
			runStatus.curWorkSel = 0;
			runStatus.pause = false;
			if(getUserBit(42) == 0) // δ��λ
			{
				setUserBit(41, 1);
			}
			setUserBit(16, 0);
			setUserBit(17, 0);
			runStatus.workTickStart = getTick();
			runStatus.status++;
		}
		else if(runStatus.startFlag & 2) // ��λ1����
		{
			runStatus.curNode = runStatus.startNode;
			runStatus.curWorkSel = 1;
			runStatus.pause = false;
			if(getUserBit(42) == 0) // δ��λ
			{
				setUserBit(41, 1);
			}
			runStatus.workTickStart = getTick();
			runStatus.status++;
		}
	}
	else if(runStatus.status == 1)
	{
		if(getUserBit(41) == 0) // ��λ���
		{
			if(getUserBit(16) == 0) // ��δ��ʼ����
			{
				setUserBit(16, 1);
				setUserParam(1, globalParam.UserPara.stop_pos[0]);
				setUserParam(2, globalParam.UserPara.stop_pos[1]);
				setUserParam(3, globalParam.UserPara.stop_pos[2]);
				return;
			}
			if(getUserBit(17) == 0) // ������δ���
			{
				return;
			}
			// �������,���Կ�ʼ���к����Ĺ���
			flag = loadNodeA(runStatus.curWorkSel, runStatus.curNode++); // ���ؼӹ���
			if(flag)
			{
				runStatus.status++;
			}
			else if(runStatus.curNode == 1) // ��0���ӹ������ʧ��,����
			{
				if(workStatus.is_step) // ��������ģʽ
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
				else if(workStatus.is_cycle) // ѭ������ģʽ
				{
					runStatus.status = 13; // �������һ��,�л��ӹ���λ
				}
				else // ��������ģʽ
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
			else // ����ӹ����
			{
				if(workStatus.is_step) // ��������ģʽ
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
				else // ��������ģʽ
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
			((runStatus.curWorkSel == 1) && (getUserBit(4) == 0))) // ����ӹ����
		{
			if(workStatus.is_step) // ��������ģʽ
			{
				runStatus.startFlag &= ~3;
			}
			else // ��������ģʽ
			{
				if(runStatus.pause) // ��ͣ
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
		if(workStatus.is_step) // ��������ģʽ
		{
			if(runStatus.curWorkSel == 0) // ��λ0
			{
				if(runStatus.startFlag & 1) // ���������ť
				{
					runStatus.status = 1;
				}
			}
			else // ��λ1
			{
				if(runStatus.startFlag & 2) // ���������ť
				{
					runStatus.status = 1;
				}
			}
		}
		else if(!runStatus.pause) // ������ͣ״̬
		{
			if(runStatus.pauseZ < 0) // z�Ȼص�ԭ����λ��
			{
				if(!motorIsRunning(2)) // zֹͣ״̬
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
		if(!motorIsRunning(2)) // zֹͣ״̬
		{
			runStatus.status = 1;
		}
	}
	else if(runStatus.status == 10) // �ж��Ƿ�Ҫ��λ
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
	else if(runStatus.status == 11) // �ж��Ƿ�Ҫ��ϴ
	{
		if(getUserBit(41) == 0) // ��λ���
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
	else if(runStatus.status == 12) // �ж��Ƿ�Ҫ��ֹͣλ
	{
		if(((runStatus.curWorkSel == 0) && (getUserBit(3) == 0)) ||
			((runStatus.curWorkSel == 1) && (getUserBit(4) == 0))) // ��ϴ���
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
			((runStatus.curWorkSel == 1) && (getUserBit(4) == 0))) // ��ֹͣλ���
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
			if(workStatus.is_cycle) // ѭ���ӹ�
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
			else // �����ӹ�����
			{
				runStatus.status = 0;
			}
		}
	}
	else if(runStatus.status == 20)
	{
		if(getUserBit(41) == 0) // ������λ���
		{
			runStatus.status = 0;
		}
	}
	else if(runStatus.status == 21)
	{
		if(getUserBit(3) == 0) // ������ϴ���
		{
			runStatus.status = 0;
		}
	}
}

