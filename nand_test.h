
#ifndef  _NAND_TEST_H_
#define  _NAND_TEST_H_

#if defined(RL6447_VA)
#define NAND_TEST_BUF_SIZE              0xA8000    //0x80000 + 0xc0000
#define NAND_TEST_BUF_BASE_PHY          0x08002040
#define NAND_TEST_BUF_BASE_CACHE        0x88002040
#define NAND_TEST_BUF_BASE_UNCACHE      0xa8002040
#define NAND_TEST_ERROR_BIT      		0xa802F040

#elif defined(RL6577_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
#define NAND_TEST_BUF_SIZE              0xA0000
#define NAND_TEST_BUF_BASE_PHY          0x0d000000
#define NAND_TEST_BUF_BASE_CACHE        0x8d000000
#define NAND_TEST_BUF_BASE_UNCACHE      0xad000000
#define NAND_TEST_ERROR_BIT      		0xad028000

#elif defined(RL6531_VA)||defined(RL6643_VA)||defined(RL6643_FPGA)
#define NAND_TEST_BUF_SIZE              0xA0000
#define NAND_TEST_BUF_BASE_PHY          0xd0000000
#define NAND_TEST_BUF_BASE_CACHE        0xf0000000
#define NAND_TEST_BUF_BASE_UNCACHE      0xd0000000
#define NAND_TEST_ERROR_BIT      		0xd0028000
#elif defined(RL6531_VB)
#define NAND_TEST_BUF_SIZE              0xA0000
#define NAND_TEST_BUF_BASE_PHY          0xc0000000
#define NAND_TEST_BUF_BASE_CACHE        0xe0000000
#define NAND_TEST_BUF_BASE_UNCACHE      0xc0000000
#define NAND_TEST_ERROR_BIT      		0xc0028000
#endif

#define NAND_TEST_CMD_BUF_SIZE          1024
#define NAND_TEST_CONFIG_BUF_SIZE       8*1024


#define NAND_TEST_RESP_BUF_SIZE         1024
#define NAND_TEST_READ_HEAD_SIZE        1024
#define NAND_TEST_WRITE_HEAD_SIZE       1024

#define NAND_TEST_WRITE_BUF_SIZE        (64 * 1024)
#define NAND_TEST_READ_BUF_SIZE         (68 * 1024)

#define NAND_TEST_TEMP_BUF_SIZE         (64 * 1024)
#define NAND_TEST_DATA_BUF_SIZE         (32 * 1024)

#define NAND_TEST_OUT1_BUF_SIZE         (2 * 1024)
#define NAND_TEST_OUT2_BUF_SIZE         (1024 * 1024)//1M




#define NAND_TEST_CMD_BUF_VA_ADDR       (NAND_TEST_BUF_BASE_CACHE) //0x88002040  //88000000~88180000 1.5M SRAM
#define NAND_TEST_CMD_BUF_PHY_ADDR      (NAND_TEST_BUF_BASE_PHY)   //0x08002040
#define NAND_TEST_CONFIG_BUF_VA_ADDR       (NAND_TEST_BUF_BASE_UNCACHE + NAND_TEST_CMD_BUF_SIZE) //0xa8002440  
#define NAND_TEST_CONFIG_BUF_PHY_ADDR      (NAND_TEST_BUF_BASE_PHY + NAND_TEST_CMD_BUF_SIZE)   //0x08002440


#define NAND_TEST_RESP_BUF_VA_ADDR      (NAND_TEST_CONFIG_BUF_VA_ADDR + NAND_TEST_CONFIG_BUF_SIZE)       //88004440
#define NAND_TEST_RESP_BUF_PHY_ADDR     (NAND_TEST_CONFIG_BUF_PHY_ADDR + NAND_TEST_CONFIG_BUF_SIZE)      //08004440
#define NAND_TEST_READ_HEAD_VA_ADDR     (NAND_TEST_RESP_BUF_VA_ADDR + NAND_TEST_RESP_BUF_SIZE)     //88004840
#define NAND_TEST_READ_HEAD_PHY_ADDR    (NAND_TEST_RESP_BUF_PHY_ADDR + NAND_TEST_RESP_BUF_SIZE)    //08004840

#define NAND_TEST_WRITE_HEAD_VA_ADDR    (NAND_TEST_READ_HEAD_VA_ADDR + NAND_TEST_READ_HEAD_SIZE)    //88004c40
#define NAND_TEST_WRITE_HEAD_PHY_ADDR   (NAND_TEST_READ_HEAD_PHY_ADDR + NAND_TEST_READ_HEAD_SIZE)   //08004c40
//------------------------------------------------------------------------------------------------------

#define NAND_TEST_WRITE_BUF_VA_ADDR     (NAND_TEST_WRITE_HEAD_VA_ADDR + NAND_TEST_WRITE_HEAD_SIZE)   //88005040
#define NAND_TEST_WRITE_BUF_PHY_ADDR    (NAND_TEST_WRITE_HEAD_PHY_ADDR + NAND_TEST_WRITE_HEAD_SIZE)


#define NAND_TEST_READ_BUF_VA_ADDR      (NAND_TEST_WRITE_BUF_VA_ADDR + NAND_TEST_WRITE_BUF_SIZE)  //88015040
#define NAND_TEST_READ_BUF_PHY_ADDR     (NAND_TEST_WRITE_BUF_PHY_ADDR + NAND_TEST_WRITE_BUF_SIZE)


#define NAND_TEST_TEMP_BUF_VA_ADDR      (NAND_TEST_READ_BUF_VA_ADDR + NAND_TEST_READ_BUF_SIZE) //88026040
#define NAND_TEST_TEMP_BUF_PHY_ADDR     (NAND_TEST_READ_BUF_VA_ADDR + NAND_TEST_READ_BUF_SIZE)

#define WL_DATA_BUF_BASE_CACHE          (NAND_TEST_TEMP_BUF_VA_ADDR + NAND_TEST_TEMP_BUF_SIZE)  //88036040
#define WL_DATA_BUF_BASE_PHY_CACHE      (NAND_TEST_TEMP_BUF_PHY_ADDR + NAND_TEST_TEMP_BUF_SIZE) //08036040

#define WL_DATA_TEMP_RECORD_BASE_CACHE          (WL_DATA_BUF_BASE_CACHE + NAND_TEST_DATA_BUF_SIZE)      //8803e040
#define WL_DATA_TEMP_RECORD_BASE_PHY_CACHE      (WL_DATA_BUF_BASE_PHY_CACHE + NAND_TEST_DATA_BUF_SIZE)  //0803e040
//-----------------------------------0x8803e040
// RESERVE
//-----------------------------------0x8806A840
#define WL_DATA_OUT1_BASE_CACHE          0x8806A840      //0x8806A840
#define WL_DATA_OUT1_BASE_PHY_CACHE      0x0806A840      //0x0806A840

#define WL_DATA_OUT2_BASE_CACHE          WL_DATA_OUT1_BASE_CACHE + NAND_TEST_OUT1_BUF_SIZE//0x8806b040      
#define WL_DATA_OUT2_BASE_PHY_CACHE      WL_DATA_OUT1_BASE_PHY_CACHE + NAND_TEST_OUT1_BUF_SIZE//0x0806b040     


#define WL_DATA_OUT_BASE_END_CACHE          (WL_DATA_OUT2_BASE_CACHE + NAND_TEST_OUT2_BUF_SIZE)      //8816b040
#define WL_DATA_OUT_BASE_END_PHY_CACHE      (WL_DATA_OUT2_BASE_PHY_CACHE + NAND_TEST_OUT2_BUF_SIZE)  //0816b040


//#define NAND_TEST_READ_HEAD_PHY_ADDR     (NAND_TEST_RESP_BUF_VA_ADDR + NAND_TEST_RESP_BUF_SIZE)
//#define NAND_TEST_READ_HEAD_PHY_ADDR    0x0810b9a0

//#define NAND_TEST_WRITE_HEAD_PHY_ADDR   NAND_TEST_WRITE_HEAD_VA_ADDR
//#define NAND_TEST_WRITE_HEAD_PHY_ADDR   0x0810b960

//#define NAND_TEST_READ_BUF_PHY_ADDR     NAND_TEST_READ_BUF_VA_ADDR
//#define NAND_TEST_READ_BUF_PHY_ADDR     0x0810d900
//#define NAND_TEST_WRITE_BUF_PHY_ADDR    NAND_TEST_WRITE_BUF_VA_ADDR
//#define NAND_TEST_WRITE_BUF_PHY_ADDR    0x08178000


#define FUNCTION_RW_CACHE        1
#define FUNCTION_READID             2
#define FUNCTION_SCAN_FDB         3
#define FUNCTION_READ_UID         4
#define FUNCTION_SET_FEATURE   5
#define FUNCTION_GET_FEATURE   6
#define FUNCTION_READCACHE     7
#define FUNCTION_WRITECACHE    8
#define FUNCTION_LUNDETECT     9

//#define NAND_TEST_EN

#define NAND_TEST_ERASE
#define NAND_TEST_READID
#define NAND_TEST_RESET
#define NAND_TEST_READPARA

#define NAND_TEST_READ
#define NAND_TEST_NANDTEST

#define NAND_TEST_WRITE
#define NAND_TEST_GETFEATURE
#define NAND_TEST_SETFEATURE

#ifndef RL6447_VA
#define NAND_VTH_DRAW1
#define NAND_VTH_DRAW2
#endif

#define NT_NAX_BANK_NUM 32

// Samsung v4 page type, also page-programed count in one shot
#define SAMSUNG_TLC       3
#define SAMSUNG_EDGE_MLC  2
#define SAMSUNG_EDGE_SLC  1

#define ASCII_ONFI  0x49464E4F
#define ASCII_JESD  0x4453454A



typedef enum
{
    SINGLE_SLC = 0,
    SINGLE_TLC = 1,
    MULTI_SLC = 2,
    MULTI_TLC = 3,
} TEST_NAND_MODE;

typedef enum
{
    TEST_NAND_READ = 0x80,
    TEST_NAND_READPARA = 0x81,
    TEST_NAND_READRETRY = 0x82,
    TEST_NAND_READID = 0x83,
    TEST_NAND_GETFEATURE = 0x84,
    TEST_NAND_GETSTATUS = 0x85,
    TEST_NAND_READUID = 0x86,
} TEST_VENDOR_READ_SUBCMD_OP;

typedef enum
{
    TEST_NAND_WRITE = 0x80,
    TEST_PWRCUT_WRITE = 0x81,
    TEST_NAND_SETFEATURE = 0x84,
} TEST_VENDOR_WRITE_SUBCMD_OP;

typedef enum
{
    TEST_NAND_CALIBRATE = 0x45,
    TEST_NAND_ERASE = 0x80,
    TEST_NAND_RESET = 0x81,
    TEST_NAND_TEST = 0x82,
    TEST_NAND_CONFIG = 0x83,
    TEST_NAND_VTH = 0xe0,
    TEST_FUNCTION = 0xc1,
} TEST_VENDOR_NONDAT_SUBCMD_OP;
#ifdef VID_CHECK
typedef enum
{
    VID_OK = 0,			//return ok
    VID_NO_SNAPREAD = 0x1	//return NO_SNAPREAD
} VID_STATUS;
#endif
#ifdef UID_CHECK
typedef enum
{
    UID_OK = 0,			//return ok
    UID_FAIL = 0x1	//return fail
} UID_STATUS;
#ifdef YX2T_STATUS_CHECK
typedef enum
{
    YX2T_CHECKSTATUS_INIT = 0,
    YX2T_CHECKSTATUS_HAVE_TO_RETRIM = 0x1,
    YX2T_CHECKSTATUS_NOT_TO_RETRIM = 0x2,
    YX2T_CHECKSTATUS_FAIL = 0x3
} ReadCheck_STATUS;
#endif
#endif
typedef struct _NandTestCmd
{
    U8 command;         //0  Command
    U8 feature;         //1  Feature
    U8 lbaHighExp;      //2  LBA_H(exp)
    U8 lbaMidExp;       //3  LBA_M(exp)
    U8 lbaLowExp;       //4  LBA_L(exp)
    U8 lbaHigh;         //5  LBA_H
    U8 lbaMid;          //6  LBA_M
    U8 lbaLow;          //7  LBA_L
    U8 sector;          //8  Sector Cnt
    U8 sectorExp;       //9  SectorCnt(exp)
    U8 featureExp;      //10 feature(exp)
    U8 aux;             //11 Aux
    U8 rsvd[4];
} NandTestCmd, *PNandTestCmd;

void NandTestGenData(U32 addr, U32 len, U32 pattern);
void NandTestGenSeqData(U32 addr, U32 len);
void NandTestGenRandomData(U32 addr, U32 len);
U32 NandTestCalEccCount(U32 sourceAddr, U32 destAddr, U32 len);

U32 NandTestNoDataCmd();
U32 NandTestReadCmd();
U32 NandTestWriteCmd();

U32 NandErase();
U32 NandReset();
U32 NandTest();
U32 NandCalibrate();
U32 NandRead();
U32 NandWrite();
U32 NandReadId();
U32 NandReadUniqueId();
U32 NandReadPara();
U32 NandReadAllPara();
#ifdef VID_CHECK
U32 NandReadVID();
#endif
#ifdef UID_CHECK
U32 NandReadUID();
#ifdef YX2T_STATUS_CHECK
U32 FCYX2TBugStateCheck(U32 ulMode, U8 ubBankNo, U8 ubLunNo);
void FCCmd1(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8 ubCmd0);
void FCCmd1Addr1(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8 ubCmd0, U8 ubAddr0);
void FCCmd1Addr4(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8 ubCmd0, U8 ubAddr0, U8 ubAddr1, U8 ubAddr2,
                 U8 ubAddr3);
U32 FCCmdReadCheck(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8* pPara, U8 step);
void FCCmdPollStatus(U32 ulMode, U8 ubBankNo);
#endif

#endif
U32 NandGetFeature();
U32 NandSetFeature();
U32 NandTestDispWrite(U32 writeMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo, U16 pageCount,
                      U8 eccState,
                      U8 aesState, U8 scrambleState, U32* tmpWbusy, U32* tmpPFmark, U8 isAllBank);
U32 NandTestDispRead(U32 readMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo, U16 pageCount,
                     U8 eccState,
                     U8 aesState, U8 scrambleState, U8 tmpRmarkFlag, U8 isAllBank);
U32 NandTestDispErase(U32 eraseMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 blockCount,
                      U32* tmpEbusy, U32* tmpEFmark, U8 isAllBank);

void EraseWriteWholeBlock(U32 ulMode, U8 bankNo, U8 ubLunNo, U16 uwBlockNo, U8 flashMode,
                          U8 isInterLeave);
void ReadWholeBlock(U32 ulMode, U8 bankNo, U8 ubLunNo, U16 uwBlockNo, U8 flashMode,
                    U8 isInterLeave);

U32 NandTestCompletionPolling(U32 *pCMP, U16 expectTag, U32 *pInfo0, U32 *pInfo1);
U32 NandTestCompletion(U32 *pInfo0, U32 *pInfo1);
U32 NanddoDistribution(void);
U32 NANDTestFunction(void);
void NandTestWriteRedundant(U8 ulMode, U8 ubBankNo, U8 ubLunNo, U16 uwBlock, U16 uwPage,
                            U32 dataAddr, U32 dataLen, U32 headAddr, U32 headLen, U8 ubSLCMode);

U16 Get_N18A_Page(U16 Page, U32 mapping_addr, U8 op);
U8  Get_SS_WordLineType(U16 uwPage);
void PageNumFill_IM(U32 mapping_addr);

void FCCmdSetFeatureLun(U32 ulMode, U8 ubBank, U8 ubLunNo, U8 ubFAddr, U32 uldata);
void llfFcCmdWriteConfirm(U8 ubMode, U8 ubBank, U16 uwBlock, U16 uwPage, U8 ubPlaneCnt);
void llfFCCmdWriteNoData(U8 ubMode, U8 ubBank, U16 uwBlock, U8 ubPlaneCnt);

U32 bl_BELlfReadID(U8 arg0, U8 arg1); //for llf init
U32 NandTestPollCmp(U8 isAllBank);

void NandTest_ReadCache(U8 ulMode, U8 bank);
void NandTest_WriteCache(U8 ulMode, U8 bank);
void NandTest_LunDetect(void);
void llfResetAndRecoverFC(U8 ubFcMode, U8 ubClockMode, U16 ubCycleMode);

void NandTestHandle();
#ifdef VID_CHECK
void FCReadVersionID(U32 ulMode, U8 ubBankNo, U8 ubFAddr, U16 uwCycle, U32 ulDataAddr,
                     U32 ulDataLen);
#endif
void FCReadUniqueID(U32 ulMode, U8 ubBankNo, U8 ubFAddr, U16 uwCycle, U32 ulDataAddr,
                    U32 ulDataLen);

U32 ParseParaPage();

#endif

