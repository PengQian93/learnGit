
#include "lib_basetype.h"
#include "timer.h"
#include "system_reg.h"
#include "platform.h"
#include "memory.h"
#include "spic.h"
#include "codebank.h"
#include "lib_phy_public.h"
#include "lib_fc_public.h"
#include "fc_reg.h"
#include "fe_be_public.h"
#include "scheduler0.h"
#include "scheduler1.h"
#include "serial.h"
#include "smp.h"
#include "llf_public.h"
#include "llfmp.h"
#include "llf_lib_public.h"
#include "llf_global.h"
#include "nand_test_public.h"
#include "nand_test.h"
#include "platform_global.h"


#include "lib_retcode.h"
#include "lib_fc_public.h"
#include "lib_retcode.h"
#include "lib_debug.h"
#include "lib_cpu.h"
#include "lib_fio_public.h"
#include "lib_sblk_config.h"
#include "lib_hlc_com.h"

#ifdef LLF_ALONE

#ifdef AUTO_K_ENABLE

#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RL6643_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA) ||defined(RL6531_VB) ||  defined(RL6643_FPGA)
U32 llfSetFeatureWithMask(U8 ubBankNo, U8 ubFAddr, U8 ubBitIndex, U8 ubBitStart, U8 ubBitLen)
{
    U32 ulmode;
    U32 cmp;
    U32 para, ulBitMask, i;
    U32 ret = ERR_OK;

    ulBitMask = 0;
    for(i = 0; i < ubBitLen; i++)
    {
        ulBitMask |= 1 << (ubBitStart + i);
    }

    ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    _REG32(GETFEATURE_BY_BANK_UC) = 0;

    gul_FW_TAG = llfBETagSetting(TAG_GETFEATURE, ubBankNo);
    FCGetfeature(ulmode, ubBankNo, ubFAddr, GETFEATURE_BY_BANK_PHY, 0x10);
    ret = FCCompletionPolling(&cmp, gul_FW_TAG);
    if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
    {
        llfDbgPrintk(ALWAYS_MSG, "Get Feature Fail bank %d,%x\r\n", ubBankNo, cmp);
        return ERR_SET_FEATURE;
    }

    if(ulmode == 0)
    {
        para = _REG32(GETFEATURE_BY_BANK_UC);
    }
    else
    {
        para = (_REG08(GETFEATURE_BY_BANK_UC + 6) << 24) | (_REG08(GETFEATURE_BY_BANK_UC + 4) << 16) |
               (_REG08(GETFEATURE_BY_BANK_UC + 2) << 8) | _REG08(GETFEATURE_BY_BANK_UC);
    }
    //llfDbgPrintk(ALWAYS_MSG, "bank%d, addr: 0x%x, get para %x\r\n", ubBankNo, ubFAddr, para);
    para = para & (~(ulBitMask));
    para = para | (ubBitIndex << ubBitStart);
    //llfDbgPrintk(ALWAYS_MSG, "bank%d, addr: 0x%x, set para %x\r\n", ubBankNo, ubFAddr, para);

    ret = FCSetFeatureAndCheckByBank(ulmode, ubBankNo, ubFAddr, para);
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "SetFeature Fail,bank %d,FAddr=0x%x\r\n", ubBankNo, ubFAddr);
        return ERR_SET_FEATURE;
    }
    return ret;
}

U32 llfSetNandTxOCD(U8 ubOcdIndex, U8 ubFcMode, U8 ubClockMode, U16 ubCycleMode)
{
    U8 ubBankNo = 0;
    U32 ret = 0;
    U32 ulTempDqsOdtEn = 0;
    U32 ulTempDqReOdtEn = 0;
    U32 ulRegValue = 0;

    ulTempDqsOdtEn = gulFc_dqs_odt_en;
    ulTempDqReOdtEn = gulFc_dq_re_odt_en;
    gulFc_dqs_odt_en = 0x0f;
    gulFc_dq_re_odt_en = 0x0f;
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    U8 ubTempFix90, coarse_rx_delay = 0, thin_rx_delay = 0, fifo_delay = 0;

    ubTempFix90 = gubDqsFix90;
    gubDqsFix90 = 1;
    default_delay_chain_setting(ubFcMode, 0, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
    onfi4_zq_dqs_default_setting(ubFcMode, 0, coarse_rx_delay, thin_rx_delay, fifo_delay,
                                 FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG, 1);
#elif defined(RL6643_VA)
    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
#endif

    llfDbgPrintk(ALWAYS_MSG, "[Nand Tx] OCD = %d, (FcMode = %d, Clock mode = %d, %dMHz)\r\n",
                 ubOcdIndex, ubFcMode, ubClockMode, ubCycleMode);
    ChangeFCClk(ubFcMode, FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_FC_MODE, gubStartCH) & 0xFFFC00FF) | 0x500);
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_FC_MODE, gubStartCH) & 0xFFFC00FF) | 0xa00);
#endif
    if(IS_6855_VERSION_TAG)
    {
        ulRegValue = FR_G_CTRL_REG32(FR_PAD_CFG1);
        ulRegValue &= (~(0xf << 24));//clear bit24
        ulRegValue |= (0xa << 24);//set bit25,27,only enable DQS SMT
        FR_G_CTRL_REG32_W(FR_PAD_CFG1, ulRegValue);
    }

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)
        {
            ubOcdIndex = 3 + ubOcdIndex;
        }
        else
        {
            ubOcdIndex = 3 - ubOcdIndex;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
    {
        ubOcdIndex = 3 - ubOcdIndex;
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK
            || FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        ubOcdIndex = (ubOcdIndex << 1) + 2;
    }

    for (ubBankNo = 0; ubBankNo < NandPara.ubBankNum; ubBankNo++)
    {
        ret = llfSetFeatureWithMask(ubBankNo, 0x10, ubOcdIndex, 0, 8);
        if(ret != ERR_OK)
        {
            break;
        }
        if((FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL || FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B16 || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A))
        {
            ret = llfSetFeatureWithMask(ubBankNo, 0x80, ubOcdIndex, 0, 8);
            if(ret != ERR_OK)
            {
                break;
            }
        }
    }

    gulFc_dqs_odt_en = ulTempDqsOdtEn;
    gulFc_dq_re_odt_en = ulTempDqReOdtEn;
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    gubDqsFix90 = ubTempFix90;
    default_delay_chain_setting(ubFcMode, 0, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
    onfi4_zq_dqs_default_setting(ubFcMode, ubClockMode, coarse_rx_delay, thin_rx_delay, fifo_delay,
                                 (3 - gubFcOcdIndex), (5 - gubFcDqsOdtIndex), (5 - gubFcDqOdtIndex), 1);
#elif defined(RL6643_VA)
    llfSetFcPadRxODT(gubFcDqsOdtIndex, gubFcDqOdtIndex);
#endif
    ChangeFCClk(ubFcMode, ubClockMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  ((ubCycleMode >> 1) << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  (ubCycleMode << 8)));
#endif
    if((ubClockMode > FC_PLL_CLK_10M) && (IS_6855_VERSION_TAG))
    {
        ulRegValue = FR_G_CTRL_REG32(FR_PAD_CFG1);
        ulRegValue &= (~(0xf << 24));//clear bit24/25/26/27
        FR_G_CTRL_REG32_W(FR_PAD_CFG1, ulRegValue);
    }
    return ret;
}

U32 llfSetNandRxODT(U8 ubOdtIndex, U8 ubFcMode, U8 ubClockMode, U16 ubCycleMode)
{
    U8 ubBankNo = 0;
    U32 ret = 0;
    U32 ulTempDqsOdtEn = 0;
    U32 ulTempDqReOdtEn = 0;
    U32 ulRegValue = 0;
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_9T23_SDR_TOGGLE
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB)
        {
            return ERR_OK;
        }
    }
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_9T23_SDR_TOGGLE
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC)
        {
            return ERR_OK;
        }
    }
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4_64G)
        {
            return ERR_OK;
        }
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT)))
    {
        if(!gubNANDODTEnable)
            return ERR_OK;
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_64GB)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_128GB)))
    {
        if(!gubNANDODTEnable)
            return ERR_OK;
    }
    llfDbgPrintk(ALWAYS_MSG, "[Nand Rx] ODT = %d, (FcMode = %d, Clock mode = %d, %dMHz)\r\n",
                 ubOdtIndex, ubFcMode, ubClockMode, ubCycleMode);

    ulTempDqsOdtEn = gulFc_dqs_odt_en;
    ulTempDqReOdtEn = gulFc_dq_re_odt_en;
    gulFc_dqs_odt_en = 0x0f;
    gulFc_dq_re_odt_en = 0x0f;
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    U8 ubTempFix90, coarse_rx_delay = 0, thin_rx_delay = 0, fifo_delay = 0;

    ubTempFix90 = gubDqsFix90;
    gubDqsFix90 = 1;
    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
    default_delay_chain_setting(ubFcMode, 0, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
    onfi4_dqs_calibrate_default_setting(ubFcMode, 0, coarse_rx_delay, thin_rx_delay,
                                        fifo_delay);
#elif defined(RL6643_VA)
    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
#endif

    ChangeFCClk(ubFcMode, FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_FC_MODE, gubStartCH) & 0xFFFC00FF) | 0x500);
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_FC_MODE, gubStartCH) & 0xFFFC00FF) | 0xa00);
#endif
    if(IS_6855_VERSION_TAG)
    {
        ulRegValue = FR_G_CTRL_REG32(FR_PAD_CFG1);
        ulRegValue &= (~(0xf << 24));//clear bit24
        ulRegValue |= (0xa << 24);//set bit25,27,only enable DQS SMT
        FR_G_CTRL_REG32_W(FR_PAD_CFG1, ulRegValue);
    }

    for(ubBankNo = 0; ubBankNo < NandPara.ubBankNum; ubBankNo++)
    {
        if(FLASH_VENDOR(gulFlashVendorNum) != IS_SAMSUNG)
        {
            ret = llfSetFeatureWithMask(ubBankNo, 0x02, ubOdtIndex, 4, 4);
        }
        else
        {
            ret = llfSetFeatureWithMask(ubBankNo, 0x02, (ubOdtIndex | (ubOdtIndex << 2)), 4, 4);
        }
        if(ret != ERR_OK)
        {
            break;
        }
        if(FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
        {
            if(ubOdtIndex > 0)
            {
                ret = llfSetFeatureWithMask(ubBankNo, 0x02, ((ubOdtIndex << 4) + 0x03), 16, 8);
            }
            else
            {
                ret = llfSetFeatureWithMask(ubBankNo, 0x02, 0, 16, 8);
            }
            if(ret != ERR_OK)
            {
                break;
            }
        }
    }

    gulFc_dqs_odt_en = ulTempDqsOdtEn;
    gulFc_dq_re_odt_en = ulTempDqReOdtEn;
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    gubDqsFix90 = ubTempFix90;
    onfi4_ocd_odt_setting((3 - gubFcOcdIndex), (5 - gubFcDqsOdtIndex), (5 - gubFcDqOdtIndex));
    default_delay_chain_setting(ubFcMode, ubClockMode, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
    onfi4_dqs_calibrate_default_setting(ubFcMode, ubClockMode, coarse_rx_delay, thin_rx_delay,
                                        fifo_delay);
#elif defined(RL6643_VA)
    onfi4_ocd_odt_setting((3 - gubFcOcdIndex), (3 - gubFcOcdIndex), (5 - gubFcDqsOdtIndex),
                          (5 - gubFcDqOdtIndex));
#endif
    ChangeFCClk(ubFcMode, ubClockMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  ((ubCycleMode >> 1) << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  (ubCycleMode << 8)));
#endif
    if((ubClockMode > FC_PLL_CLK_10M) && (IS_6855_VERSION_TAG))
    {
        ulRegValue = FR_G_CTRL_REG32(FR_PAD_CFG1);
        ulRegValue &= (~(0xf << 24));//clear bit24/25/26/27
        FR_G_CTRL_REG32_W(FR_PAD_CFG1, ulRegValue);
    }
    return ret;
}

void llfSetFcPadTxOCD(U8 ubOcd)
{
#if defined(RL6531_VB)
    U32 ocd = ubOcd ;
    U32 dqsOdt = gubFcDqsOdtIndex;
    U32 dqOdt = gubFcDqOdtIndex;
#else
    U32 ocd = 3 - ubOcd;
    U32 dqsOdt = 5 - gubFcDqsOdtIndex;
    U32 dqOdt = 5 - gubFcDqOdtIndex;
#endif

    llfDbgPrintk(ALWAYS_MSG, "[FC TX] OCD = %d, DqsODT = %d, DqODT = %d\r\n", ocd, dqsOdt, dqOdt);
#if defined(RL6643_VA) || defined(RL6643_FPGA)
    onfi4_ocd_odt_setting(ocd, ocd, dqsOdt, dqOdt);
#elif defined(RL6531_VB)
    Fc_ocd_odt_setting(ocd, dqsOdt);
#else
    onfi4_ocd_odt_setting(ocd, dqsOdt, dqOdt);
#endif
    gubFcOcdIndex = ubOcd;
    gulFc_ocd = ocd;
}

void llfSetFcPadRxODT(U8 ubDqsOdt, U8 ubDqOdt)
{
#if defined(RL6531_VB)
    U32 ocd = gubFcOcdIndex;
    U32 dqsOdt = ubDqsOdt;
    U32 dqOdt = ubDqOdt;
#else
    U32 ocd = 3 - gubFcOcdIndex;
    U32 dqsOdt = 5 - ubDqsOdt;
    U32 dqOdt = 5 - ubDqOdt;
#endif
    U32 uwDqsOdtenTemp, uwDqOdtenTemp;

    uwDqsOdtenTemp = gulFc_dqs_odt_en;
    uwDqOdtenTemp = gulFc_dq_re_odt_en;

    if(ubDqsOdt == 0)
    {
        gulFc_dqs_odt_en = 0;
    }
    else
    {
        gulFc_dqs_odt_en = 0x0f;
    }
    if(ubDqOdt == 0)
    {
        gulFc_dq_re_odt_en = 0;
    }
    else
    {
        gulFc_dq_re_odt_en = 0x0f;
    }

    llfDbgPrintk(ALWAYS_MSG, "[FC RX] OCD = %d, DqsODT = %d, DqODT = %d\r\n", ocd, dqsOdt, dqOdt);

#if defined(RL6643_VA) || defined(RL6643_FPGA)
    onfi4_ocd_odt_setting(ocd, ocd, dqsOdt, dqOdt);
#elif defined(RL6531_VB)
    Fc_ocd_odt_setting(ocd, dqsOdt);
#else
    onfi4_ocd_odt_setting(ocd, dqsOdt, dqOdt);
#endif
    gubFcDqsOdtIndex = ubDqsOdt;
    gubFcDqOdtIndex = ubDqOdt;
    gulFc_dqs_odt = dqsOdt;
    gulFc_dq_re_odt = dqOdt;

    gulFc_dqs_odt_en = uwDqsOdtenTemp;//odt_en keep config setting
    gulFc_dq_re_odt_en = uwDqOdtenTemp;
}

U32 llfPollingAllBankCMP(U32 *pCMP, U16 uwExpectTag)
{
    U8 i = 0;
    U32 cmp = 0, time_cnt = 0, err_info0 = 0, err_info1 = 0;
    U32 cmp_mask = 0;

#if defined(RL6577_VA) || defined(RL6643_VA) || defined(RTS5771_VA)
    if(gubCmpSel)
        cmp_mask = 0x3ff800;
    else
        cmp_mask = 0x7ff;
#elif defined(RL6447_VA) || defined(RL6531_VB)
    cmp_mask = 0xffff;
#else
    cmp_mask = 0xffff;
#endif

    while(i != (NandPara.ubChNum * NandPara.ubCENumPerCh))
    {
        while((FC_TOP_REG(FR_CMPQ_BC) & cmp_mask) != 0)
        {
            cmp = FcPollCompletion(&err_info0, &err_info1);
            if((cmp & PROGRAM_DATA_DONE) != 0)
                continue;
            if((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR)
            {
                if((cmp & 0xffff0000) >> 16 == uwExpectTag)
                {
                    //llfDbgPrintk(ALWAYS_MSG, "bank %d err %x\r\n", (cmp >> FC_CMP_FW_TAG_SHITF), cmp);
                    *pCMP = cmp;
                }
            }
            i++;
        }
        BeFcBusyWait10us(10);
        time_cnt++;
        if(time_cnt > 1000)
        {
            //llfDbgPrintk(ALWAYS_MSG, "llfPollingAllBankCMP Timeout\r\n");
            break;
        }
    }

    if(time_cnt > 100)
    {
        return ERR_FIO_TIMEOUT;
    }

    return ERR_OK;
}

U32 llfWriteCache(U8 ubBankNo)
{
    U32 ulAddr, ulHeadAddr, cmp, ulMode;
    U32 ret = ERR_OK;
    U8 lun_no;

    lun_no = ubBankNo / (NandPara.ubChNum * NandPara.ubCENumPerCh);
    ulHeadAddr = TEMP_HBUF_PHY_ADDR;
    ulAddr = TEMP_BUF_PHY_ADDR;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    gul_FW_TAG = llfBETagSetting(TAG_WRITE_CACHE, ubBankNo);
    FCWriteCacheDRAM(ulMode, ubBankNo, lun_no, 0, ulAddr, NandPara.ubSectorNumPerPage * 512,
                     ulHeadAddr, DRAM_HEAD_SIZE);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[WriteCacheTimeout]bank %d\r\n", ubBankNo);
    }
    else
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "[WriteCachePF]bank %d, cmp %x\r\n", ubBankNo, cmp);
            ret = ERR_FLASH_CALIBRATE;
        }
    }
    return ret;
}

U32 llfWriteCachePipeline(U8 ubBankNo)
{
    U32 ulAddr, ulHeadAddr, cmp, ulMode;
    U32 ret = ERR_OK;
    U8 lun_no;
    U8 i;

    lun_no = ubBankNo / (NandPara.ubChNum * NandPara.ubCENumPerCh);
    ulHeadAddr = TEMP_HBUF_PHY_ADDR;
    ulAddr = TEMP_BUF_PHY_ADDR;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    for(i = 0; i < NandPara.ubChNum * NandPara.ubCENumPerCh; i++)
    {
        //llfDbgPrintk(ALWAYS_MSG, "llfWriteCachePipeline bank: %d\r\n", i);
        gul_FW_TAG = i;
        FCWriteCacheDRAM(ulMode, i, lun_no, 0, ulAddr, NandPara.ubSectorNumPerPage * 512,
                         ulHeadAddr, DRAM_HEAD_SIZE);
    }

    ret = llfPollingAllBankCMP(&cmp, ubBankNo);
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[WriteCacheTimeout]bank %d\r\n", ubBankNo);
    }
    else
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            //llfDbgPrintk(ALWAYS_MSG, "[WriteCachePF]bank %d, cmp %x\r\n", ubBankNo, cmp);
            ret = ERR_FLASH_CALIBRATE;
        }
    }
    return ret;
}

void llfResetAndRecoverFC(U8 ubFcMode, U8 ubClockMode, U16 ubCycleMode)
{
    //U8 bank = 0;
    //U8 mode = ONFI_SDR;
    //U8 i = 0;
    //U32 cmp = 0;
    //U32 ret = ERR_OK;
    //U32 nand_odt_diff_vref_value = 0;
    U32 ulRegValue;
    PVENDOR_CMD_RESPONSE pLLFResponseInfo;
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    U8 ubTempFix90;

    ubTempFix90 = gubDqsFix90;
#endif
    pLLFResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFResponseInfo->err_msg_num = 0;

    llfDbgPrintk(ALWAYS_MSG, "FCSoftReset %d %d %d\r\n", ubFcMode, ubClockMode, ubCycleMode);
    FC_TOP_REG(FC_BIU_OCP_STOP) = 1; //stop ocp bus
    while(1)
    {
        if(FC_TOP_REG(FC_BIU_OCP_DONE) == 1)
            break;
    }
    ulRegValue = _REG32(U_SOFT_RST_CTRL);
    _REG32(U_SOFT_RST_CTRL) = ulRegValue & (~U_SOFT_RST_FC_BIT);
    _REG32(U_SOFT_RST_CTRL) = ulRegValue | (U_SOFT_RST_FC_BIT);

    llfDbgPrintk(ALWAYS_MSG, "init onfi4 start, default mode: %d\r\n", gubNandDefaultMode);
#if defined(RL6643_VA) || defined(RL6531_VB)
    if(gubNandDefaultMode == 0)
    {
        ChangeFCClk(ONFI_SDR, FC_PLL_CLK_10M);
#ifdef RL6643_VA
        onfi4_ocd_odt_setting(3, 3, 4, 4);//10M,ocd:50ohm,odt:150ohm
#else
        Fc_ocd_odt_setting(3, 4);
#endif
    }
    else
    {
        ChangeFCClk(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M);
#ifdef RL6643_VA
        onfi4_ocd_odt_setting(3, 3, 4, 4);//10M,ocd:50ohm,odt:150ohm
#else
        Fc_ocd_odt_setting(3, 4);
#endif
        fc_diff_setting(ONFI_DDR2_TOGGLE, 0, gubFCDiffEnable);
        FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_DDR2_TOGGLE);
    }
#elif defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    gubDqsFix90 = 1;
    if(gubNandDefaultMode == 0)
    {
        ChangeFCClk(ONFI_SDR, FC_PLL_CLK_10M);
        onfi4_change_setting(ONFI_SDR, FC_PLL_CLK_10M, 1, 2, 2, gubFCDiffEnable);
        FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_SDR);
    }
    else
    {
        ChangeFCClk(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M);
        onfi4_change_setting(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M, 3, 1, 1, gubFCDiffEnable);
        FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_DDR2_TOGGLE);
    }
#endif
    SetParserSequencerTable(PARSER_TABLE_VA_ADDR, SEQUENCER_TABLE_VA_ADDR);
    SetParserSequencerIndex(PARSER_INDEX_ADDR, SEQUENCER_INDEX_ADDR, AUTO_INSERT_INDEX_ADDR);
    FcCopySeedFromROM();
    llfFcInitReg();
    HlcParserIndexInit();
    Change_ldpc(gubECC_CFG);

#if defined(RL6643_VA) || defined(RTS5771_VA) || defined(RTS5771_FPGA) || defined(RL6643_FPGA)
//6643/5771 not support AES;
#else
    if(!AES_BYPASS)
    {
        InitAESEngine();
    }
#endif
//    for(i = 0; i < NandPara.ubBankNum; i++)
//    {
//        llfprintk("Bank %d reset\r\n", i);
//        gul_FW_TAG = llfBETagSetting(TAG_RESET, i);
//        llfFCCmdReset(0, i);
//        FcBusyWait1ms(5);
//        ret = llfFCCmdCompletionPolling(&cmp, (gul_FW_TAG));
//        if(ret != ERR_OK)
//        {
//            llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: bank = %d  ret = %x\r\n", i, ret);
//            //llfAddErrorMessage(i, 0, ERR_FIO_TIMEOUT);
//        }
//        else
//        {
//            gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, i);
//            llfFCCmdStatusPolling(0, i);
//            ret = llfFCCmdCompletionPolling(&cmp, (gul_FW_TAG));
//            if(ret != ERR_OK)
//            {
//                llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: bank = %d  ret = %x\r\n", i, ret);
//                //llfAddErrorMessage(i, 0, ERR_FIO_TIMEOUT);
//            }
//            else
//            {
//                if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
//                {
//                    llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: bank = %d cmp = %x\r\n", i, cmp);
//                    //llfAddErrorMessage(i, 0, ERR_FC_CMP);
//                }
//            }
//        }
//    }
//
//    FcBusyWait1ms(10);
//    ret = FCInterfaceChange(ubIFType, FC_PLL_CLK_10M);
//    if(ret != ERR_OK)
//    {
//        return;
//    }
//
//    if((ubClockMode) > FC_DIFF_OPEN_FREQ)
//    {
//        ret = SetWarmUpOdtDrvDiff(ubClockMode, &nand_odt_diff_vref_value);
//        if(ret != ERR_OK)
//        {
//            return;
//        }
//    }
//    llfFCTimingModeChange(ubIFType, ubClockMode);

    llfLoadTimingFromConfig();
    ChangeFCClk(ubFcMode, ubClockMode);
    FR_G_CFG_REG32_W(FR_FC_MODE, ubFcMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  ((ubCycleMode >> 1) << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  (ubCycleMode << 8)));
#endif

    if(IS_6855_VERSION_TAG)
    {
        if((ubClockMode > FC_PLL_CLK_10M))
        {
            ulRegValue = FR_G_CTRL_REG32(FR_PAD_CFG1);
            ulRegValue &= (~(0xf << 24));//clear bit24/25/26/27
            FR_G_CTRL_REG32_W(FR_PAD_CFG1, ulRegValue);
        }
        else
        {
            ulRegValue = FR_G_CTRL_REG32(FR_PAD_CFG1);
            ulRegValue &= (~(0xf << 24));//clear bit24
            ulRegValue |= (0xa << 24);//set bit25,27,only enable DQS SMT
            FR_G_CTRL_REG32_W(FR_PAD_CFG1, ulRegValue);
        }
    }

#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    U32 ocd = 3 - gubFcOcdIndex;
    U32 dqsOdt = 5 - gubFcDqsOdtIndex;
    U32 dqOdt = 5 - gubFcDqOdtIndex;
    onfi4_change_setting(ubFcMode, ubClockMode, ocd, dqsOdt, dqOdt, gubFCDiffEnable);
#elif defined(RL6643_VA) || defined(RL6531_VB)
    llfSetFcPadTxOCD(gubFcOcdIndex);
    llfSetFcPadRxODT(gubFcDqsOdtIndex, gubFcDqOdtIndex);
    fc_diff_setting(ubFcMode, ubClockMode, gubFCDiffEnable);
    llfSetTimingDrivingPerCh(SBLK_ADDR);
#endif

#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
    gubDqsFix90 = ubTempFix90;
#endif

//    if(ubClockMode > FC_DIFF_OPEN_FREQ)
//    {
//        ret = FcGetDiffFeature(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), nand_odt_diff_vref_value);
//    }
}

void llfWriteReadCache10MTest(U8 ubIFType, U8 ubClkMode)
{
    U8 ubBankNo = 0;
    PVENDOR_CMD_RESPONSE pLLFResponseInfo = NULL;
    U32 ret = ERR_OK;

    pLLFResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

    if (FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
    {
        llfDbgPrintk(ALWAYS_MSG, "TOSHIBA: IF %d Clk %d\r\n", ubIFType, ubClkMode);
    }
    else if (FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON
             || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
    {
        llfDbgPrintk(ALWAYS_MSG, "MICRON : IF %d Clk %d\r\n", ubIFType, ubClkMode);
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
    {
        llfDbgPrintk(ALWAYS_MSG, "SANDISK : IF %d Clk %d\r\n", ubIFType, ubClkMode);
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
    {
        llfDbgPrintk(ALWAYS_MSG, "HYNIX : IF %d Clk %d\r\n", ubIFType, ubClkMode);
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        llfDbgPrintk(ALWAYS_MSG, "SAMSUNG : IF %d Clk %d\r\n", ubIFType, ubClkMode);
    }

    for (ubBankNo = 0; ubBankNo < NandPara.ubBankNum; ubBankNo++)
    {
        llfprintk("WriteReadCache10M bank%d\r\n", ubBankNo);
        ret = WriteReadFlashCache(ubBankNo, FC_PLL_CLK_10M);
        if (ret != ERR_OK)
        {
            AddErrorMessage(ubBankNo, 0, ret);
        }
    }
    if (pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "Failed to r/w with 10Mhz\r\n");
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
    }
}

U32 llfWriteCacheRedundant(U8 ubBankNo)
{
    U32 ulAddr, ulHeadAddr, cmp, ulMode;
    U32 ret = ERR_OK;
    U16 temp_2k_lemgth;

    ulHeadAddr = TEMP_HBUF_PHY_ADDR;
    ulAddr = TEMP_BUF_PHY_ADDR;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    gul_FW_TAG = llfBETagSetting(TAG_WRITE_CACHE, ubBankNo);
    temp_2k_lemgth = gub_Total_len_per_2K;
    gub_Total_len_per_2K = 2048;
    FCWriteCacheRedundant(ulMode, ubBankNo, 0, ulAddr, NandPara.ubSectorNumPerPage * 512,
                          ulHeadAddr, DRAM_HEAD_SIZE);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[WriteCacheTimeout]bank %d\r\n", ubBankNo);
    }
    else
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "[WriteCachePF]bank %d, cmp %x\r\n", ubBankNo, cmp);
            ret = ERR_FLASH_CALIBRATE;
        }
    }
    gub_Total_len_per_2K = temp_2k_lemgth;
    return ret;
}

U32 llfWriteCacheRedundantPipeline(U8 ubBankNo)
{
    U32 ulAddr, ulHeadAddr, cmp, ulMode;
    U32 ret = ERR_OK;
    U16 temp_2k_lemgth;
    U8 i;

    ulHeadAddr = TEMP_HBUF_PHY_ADDR;
    ulAddr = TEMP_BUF_PHY_ADDR;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    temp_2k_lemgth = gub_Total_len_per_2K;
    gub_Total_len_per_2K = 2048;

    for(i = 0; i < NandPara.ubChNum * NandPara.ubCENumPerCh; i++)
    {
        llfDbgPrintk(ALWAYS_MSG, "llfWriteCacheRedundantPipeline bank: %d\r\n", i);
        gul_FW_TAG = i;
        FCWriteCacheRedundant(ulMode, i, 0, ulAddr, NandPara.ubSectorNumPerPage * 512,
                              ulHeadAddr, DRAM_HEAD_SIZE);
    }

    ret = llfPollingAllBankCMP(&cmp, ubBankNo);
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[WriteCacheTimeout]bank %d\r\n", ubBankNo);
    }
    else
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            //llfDbgPrintk(ALWAYS_MSG, "[WriteCachePF]bank %d, cmp %x\r\n", ubBankNo, cmp);
            ret = ERR_FLASH_CALIBRATE;
        }
    }

    gub_Total_len_per_2K = temp_2k_lemgth;
    return ret;
}

#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)
void llfChangeInterFace(U8 ubIFType)
{
    llfDbgPrintk(ALWAYS_MSG, "[NAND]Clock mode:%x, %d MHz\r\n", FC_PLL_CLK_10M, 10);
    FCInterfaceChange(ubIFType, FC_PLL_CLK_10M);
    llfLoadTimingFromConfig();

    ChangeFCClk(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | (0x05 << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | (0x0a << 8)));
#endif
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX))
    {
        onfi4_change_setting(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M, 3, 1, 1, 1);
    }
    else
    {
        onfi4_change_setting(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M, 3, 1, 1, 0);
    }
}

void llfSaveTimingDrivingPerCh (U32 addr, U32 mode, U32 Speed)
{
    int i, j;
    U32 Offset = (SBLK_FC_HW_CONFIG2_PER_SETTING_LEN * mode);

    for(i = 0; i < CH_NUM_MAX; i++)
    {
        FR_REG_DLL_BASE_X = FR_DLL_REG_BASE + i * FR_ONFI_REG_SIZE;
        for(j = 0; j < 4; j ++)
        {
            _REG32(addr + SBLK_OFFSET_NORMAL_FR_PHY_DELAY_CFG0_PER_CH + Offset + (i * 0x10) +
                   (j * WORD_BYTE_SIZE) + 0) = FR_REG32_DLL_X(ONFI_DQS_IN_DLY_0 + j * WORD_BYTE_SIZE);
        }
        _REG32(addr + FR_PHY_FIFO_DELAY_PER_CH + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_DLL_X(
                    ONFI_READ_CTRL_1);
#ifdef RTS5771_VA
        _REG32(addr + FR_PHY_RX_DELAY_PER_CH + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_DLL_X(
                    ONFI_CE_SEL_DLY_1);
#else
        _REG32(addr + FR_PHY_RX_DELAY_PER_CH + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_DLL_X(
                    ONFI_READ_CTRL_0);
#endif

        FR_REG_BASE_X = FR_REG_BASE + i * FR_REG_SIZE;
//        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_PULL_PER_CH + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(
//                    FR_PAD_PULL);

        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_CFG0_PER_CH + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(
                    FR_PAD_CFG0);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_CFG1_PER_CH + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(
                    FR_PAD_CFG1);
    }

//    _REG32(addr + FR_PHY_OCD_X + Offset + (i * WORD_BYTE_SIZE)) = gubFcOcd;
//    _REG32(addr + FR_PHY_OCD_Y + Offset + (i * WORD_BYTE_SIZE)) = gubFcOcd;
//    _REG32(addr + FR_PHY_ODT_X + Offset + (i * WORD_BYTE_SIZE)) = gubFcOdt;
//    _REG32(addr + FR_PHY_ODT_Y + Offset + (i * WORD_BYTE_SIZE)) = gubFcOdt;

    cache_area_dwbinval(addr, SBLK_OFFSET_FC_HW_CONFIG2_LENGTH);
    cache_dummy_update_read(); // dummy read back
}

void llfLoadSblkPerCh(U32 addr, U16 mode)
{
    U8 i;
    U32 Offset = (SBLK_FC_HW_CONFIG2_PER_SETTING_LEN * mode);
    U32 coarse_rx_delay, thin_rx_delay, fifo_delay;

    for(i = 0; i < CH_NUM_MAX; i++)
    {
        FR_REG_DLL_BASE_X = FR_DLL_REG_BASE + i * FR_ONFI_REG_SIZE;

        thin_rx_delay = (_REG32(addr + SBLK_OFFSET_NORMAL_FR_PHY_DELAY_CFG0_PER_CH
                                + (i * 0x20))) & 0xff;
        fifo_delay = (_REG32(addr + FR_PHY_FIFO_DELAY_PER_CH + Offset
                             + (i * WORD_BYTE_SIZE))) & 0xff;
        coarse_rx_delay = (_REG32(addr + FR_PHY_RX_DELAY_PER_CH + Offset
                                  + (i * WORD_BYTE_SIZE)) >> 20) & 0xff;
        printk("bank:%d,coarse:%d,thin:%d,fifo:%d\r\n", i,
               coarse_rx_delay, thin_rx_delay, fifo_delay);

        // reset fifo pointer
        FR_REG32_DLL_X(ONFI_DPI_CTRL_0) = FR_REG32_DLL_X(ONFI_DPI_CTRL_0) | 0xc;
        FR_REG32_DLL_X(ONFI_DPI_CTRL_0) = FR_REG32_DLL_X(ONFI_DPI_CTRL_0) & 0xfffffff3;

        onfi4_dqs_calibrate_ch(i, FR_CONFIG_CH(FR_FC_MODE, gubStartCH), 0, coarse_rx_delay, thin_rx_delay,
                               fifo_delay);
    }
}


void llfSetTxDqDelay(U8 ubChNo, U8 ubIndex)
{
    U8 tx_pd = 0;
#ifdef RTS5771_VA
    tx_pd = 1;
#endif
    //llfprintk("TX DQ PI: CH%d, %d\r\n", ubChNo, ubIndex);
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = FC_PHY_CRT_CH( ONFI_DPI_CTRL_0,
            ubChNo) | 0xc;//reset fifo pointer
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = FC_PHY_CRT_CH( ONFI_DPI_CTRL_0,
            ubChNo) & 0xfffffff3;//disable reset fifo pointer
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = FC_PHY_CRT_CH( ONFI_DPI_CTRL_0,
            ubChNo) | 0x30;//disable auto calibration

    //DQ PI and wrlvl_dly/wrlvl_fb
    if(ubIndex < 21)
    {
        //fb_dq:1,dly_dq:0
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) | 0x40;
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) & 0xfffffbff;
    }
    else if(ubIndex < 53)
    {
        //fb_dq:0,dly_dq:1
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) | 0x400;
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) & 0xffffffbf;
    }
    else if(ubIndex < 64)
    {
        //fb_dq:1,dly_dq:1
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) | 0x400;
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) | 0x40;
    }

    //turn off clk, ofi_mck_clk_en=0 -> ofi_clk_en=0
    FC_PHY_CRT_CH(PLL_CTL0, ubChNo) = 0x0000000F;
    FC_PHY_CRT_CH(PLL_CTL1, ubChNo) = 0x0;
    fc_phy_delay(100);

    FC_PHY_CRT_CH(PLL_PI0, ubChNo) &= (~(0x3f << 16));
    FC_PHY_CRT_CH(PLL_PI0, ubChNo) |= ((ubIndex & 0x3f) << 16);

    //turn on clk
    FC_PHY_CRT_CH(PLL_CTL0, ubChNo) = 0x0007000F;
    fc_phy_delay(500);

    //DQ PI 0 - 63
#ifndef RTS5771_VA
    if(ubIndex == 15 || ubIndex == 31 || ubIndex == 47 || ubIndex == 63)
        return;
#endif
    if(((ubIndex > 16) && (ubIndex <= 31)) || ((ubIndex > 48) && (ubIndex <= 63)))
        FC_PHY_CRT_CH(PLL_CTL1, ubChNo) = 0x00070004;
    else
        FC_PHY_CRT_CH(PLL_CTL1, ubChNo) = 0x00070000;

    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = 0x020f0032 | (tx_pd << 31);
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_1, ubChNo) = 0x00000003;
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = 0x020f0030 | (tx_pd << 31);

//    llfprintk("Index: %d, %x %x %x %x\r\n", ubIndex, FC_PHY_CRT_CH(PLL_PI0, ubChNo),
//           FC_PHY_CRT_CH(PLL_CTL1, ubChNo), FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo), FC_PHY_CRT_CH(CRT_CTL,
//                   ubChNo));
    fc_phy_delay(100);
}

void llfSetTxDqsDelay(U8 ubChNo, U8 ubIndex)
{
    U8 tx_pd = 0;
#ifdef RTS5771_VA
    tx_pd = 1;
#endif
    //llfprintk("TX DQS PI: CH%d, -%d\r\n", ubChNo, ubIndex);
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) | 0xc;
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) & 0xfffffff3;
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) | 0x30;

    //DQS PI and wrlvl_dly/wrlvl_fb
    if(ubIndex < 53)//equivalent to DQ PI -52~-1
    {
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) & 0xfffffddf;
    }
    else if(ubIndex < 64)//equivalent to DQ PI -63~-53
    {
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) | 0x20;
        FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) = FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo) & 0xfffffdff;
    }

    //turn off clk, ofi_mck_clk_en=0 -> ofi_clk_en=0
    FC_PHY_CRT_CH(PLL_CTL0, ubChNo) = 0x0000000F;
    FC_PHY_CRT_CH(PLL_CTL1, ubChNo) = 0x0;
    fc_phy_delay(100);

    FC_PHY_CRT_CH(PLL_PI0, ubChNo) &= (~(0x3f << 8));
    FC_PHY_CRT_CH(PLL_PI0, ubChNo) |= ((ubIndex & 0x3f) << 8);

    //turn on clk
    FC_PHY_CRT_CH(PLL_CTL0, ubChNo) = 0x0007000F;
    fc_phy_delay(500);

    //DQS PI -63 - -1
#ifndef RTS5771_VA
    if(ubIndex == 15 || ubIndex == 31 || ubIndex == 47 || ubIndex == 63)
        return;
#endif
    if(((ubIndex > 16) && (ubIndex <= 31)) || ((ubIndex > 48) && (ubIndex <= 63)))
        FC_PHY_CRT_CH(PLL_CTL1, ubChNo) = 0x00070002;
    else
        FC_PHY_CRT_CH(PLL_CTL1, ubChNo) = 0x00070000;

    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = 0x020f0032 | (tx_pd << 31);
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_1, ubChNo) = 0x00000003;
    FC_PHY_CRT_CH( ONFI_DPI_CTRL_0, ubChNo) = 0x020f0030 | (tx_pd << 31);

//    llfprintk("Index: %d, %x %x %x %x\r\n", ubIndex, FC_PHY_CRT_CH(PLL_PI0, ubChNo),
//           FC_PHY_CRT_CH(PLL_CTL1, ubChNo), FC_PHY_DLL_CH(ONFI_WRLVL_CTRL, ubChNo), FC_PHY_CRT_CH(CRT_CTL,
//                   ubChNo));
    fc_phy_delay(100);
}

U32 llfDQSDQFix90WriteReadTest(U8 ubBankNo, U8 ubFcMode, U8 ubClockMode, U16 ubCycleMode,
                               U32 *ulPhyCfgGap)
{
    U32 ret = ERR_OK;
    U8 be_no, ce_no;
    U16 round = 0;
    U32 retryCount = 10;
    U32 retCompare = ERR_OK;
    U8 coarse_rx_delay = 0, thin_rx_delay = 0, fifo_delay = 0;
    U8 ubPipelineK = 0;
    *ulPhyCfgGap = 0;
    ubPipelineK = (gubCalibrateConfig >> 6) & 0x01;
    be_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);

    for(round = 0; round < retryCount; round++)
    {
        //llfprintk("90Readwritetest[%d]: bank%d\r\n", round, ubBankNo);
        ChangeFCClk(ubFcMode, ubClockMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
        FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                         ((ubCycleMode >> 1) << 8));
#else
        FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                         (ubCycleMode << 8));
#endif
        default_delay_chain_setting(ONFI_DDR2_TOGGLE, ubClockMode, &coarse_rx_delay, &thin_rx_delay,
                                    &fifo_delay);
        onfi4_dqs_calibrate_default_setting(ONFI_DDR2_TOGGLE, ubClockMode, coarse_rx_delay, thin_rx_delay,
                                            fifo_delay);

        if(ubPipelineK)
            ret = llfWriteCachePipeline(ubBankNo);
        else
            ret = llfWriteCache(ubBankNo);
        if(ret == ERR_FIO_TIMEOUT)
        {
            return ret;
        }

        ChangeFCClk(ubFcMode, FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
        FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0x500);
#else
        FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
#endif
        default_delay_chain_setting(ONFI_DDR2_TOGGLE, 0, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
        onfi4_dqs_calibrate_default_setting(ONFI_DDR2_TOGGLE, 0, coarse_rx_delay, thin_rx_delay,
                                            fifo_delay);

        retCompare = llfReadCacheAndCompare(ubBankNo);
        if(retCompare == ERR_FIO_TIMEOUT)
        {
            return ret;
        }
        ret = retCompare;
    }

    if(ret == ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]bank%d CH%dCE%d DQSDQFix90WriteReadTest OK\r\n", ubBankNo,
                     be_no, ce_no);
        *ulPhyCfgGap = AUTO_CALIBRATE_TX_GAP;
    }
    return ret;
}

U32 llfCalibrateTxDqDelayChain(U8 ubBankNo, U8 ubFcMode, U8 ubClockMode, U16 ubCycleMode,
                               U32 *ulPhyCfg, U32 *ulPhyCfgGap )
{
    U32 ret = ERR_OK;
    U16 index;
    U32 ulPhyCfgMin = 0xffffffff;
    U32 ulPhyCfgMax = 0;
    U32 ulPhyCfgMinTmp = 0xffffffff;
    U32 ulPhyCfgMaxTmp = 0;
    U32 ret_data_compare = ERR_OK;
    U32 be_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    U32 ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
    U8 coarse_rx_delay = 0, thin_rx_delay = 0, fifo_delay = 0;
    U8 ubRawDataK = 0;
    U8 ubPipelineK = 0;

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;
    ubPipelineK = (gubCalibrateConfig >> 6) & 0x01;
    *ulPhyCfg = 0;
    *ulPhyCfgGap = 0;
    llfprintk("bank %d CH%dCE%d FcMode = %d ClockMode = %d, %dMHz\r\n", ubBankNo, be_no, ce_no,
              ubFcMode, ubClockMode, ubCycleMode);

    for(index = 0; index < 64; index++)
    {
        //Set TX
        //Set Timing&interface to desired Speed
        ChangeFCClk(ubFcMode, ubClockMode);//change gubDqsFix90
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
        FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                      ((ubCycleMode >> 1) << 8)));
#else
        FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                      (ubCycleMode << 8)));
#endif
        default_delay_chain_setting(ONFI_DDR2_TOGGLE, ubClockMode, &coarse_rx_delay, &thin_rx_delay,
                                    &fifo_delay);
        onfi4_dqs_calibrate_default_setting(ONFI_DDR2_TOGGLE, ubClockMode, coarse_rx_delay, thin_rx_delay,
                                            fifo_delay);

#ifndef RTS5771_VA
        if(index == 15 || index == 31 || index == 47 || index == 63)
            continue;
        else
#endif
            llfSetTxDqDelay(be_no, index);

        //WriteCache
        if(ubRawDataK)
        {
            if(ubPipelineK)
                ret = llfWriteCacheRedundantPipeline(ubBankNo);
            else
                ret = llfWriteCacheRedundant(ubBankNo);
            if(ret == ERR_FIO_TIMEOUT)
            {
                return ret;
            }
            else if(ret == ERR_OK)
            {
                //Set Timing&interface to 10MHz
                ChangeFCClk(ubFcMode, FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
                FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0x500);
#else
                FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
#endif
                default_delay_chain_setting(ONFI_DDR2_TOGGLE, 0, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
                onfi4_dqs_calibrate_default_setting(ONFI_DDR2_TOGGLE, 0, coarse_rx_delay, thin_rx_delay,
                                                    fifo_delay);
                //ReadCache
                ret_data_compare = llfReadCacheRawDataCompare(ubBankNo);
            }
            else
            {
                ret_data_compare = ERR_FC_CMP;
            }
        }
        else
        {
            if(ubPipelineK)
                ret = llfWriteCachePipeline(ubBankNo);
            else
                ret = llfWriteCache(ubBankNo);
            if(ret == ERR_FIO_TIMEOUT)
            {
                return ret;
            }
            else if(ret == ERR_OK)
            {
                //Set Timing&interface to 10MHz
                ChangeFCClk(ubFcMode, FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
                FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0x500);
#else
                FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
#endif
                default_delay_chain_setting(ONFI_DDR2_TOGGLE, 0, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
                onfi4_dqs_calibrate_default_setting(ONFI_DDR2_TOGGLE, 0, coarse_rx_delay, thin_rx_delay,
                                                    fifo_delay);
                //ReadCache
                ret_data_compare = llfReadCacheAndCompare(ubBankNo);
            }
            else
            {
                ret_data_compare = ERR_FC_CMP;
            }
        }

        if(ret_data_compare == ERR_FIO_TIMEOUT)
        {
            return ret;
        }
        else if (ret_data_compare == ERR_OK)
        {
            //llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]index: %d -> pass\r\n", index);
            if(index < ulPhyCfgMinTmp)
                ulPhyCfgMinTmp = index;
            if(index > ulPhyCfgMaxTmp)
                ulPhyCfgMaxTmp = index;
        }
        else
        {
            //llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]index: %d -> fail\r\n", index);
            if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
            {
                llfDbgPrintk(ALWAYS_MSG, "bank %d Tx K temp: %d-%d\r\n", ubBankNo, ulPhyCfgMinTmp, ulPhyCfgMaxTmp);
                ulPhyCfgMin = ulPhyCfgMinTmp;
                ulPhyCfgMax = ulPhyCfgMaxTmp;
                *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
            }
            ulPhyCfgMinTmp = 0xffffffff;
            ulPhyCfgMaxTmp = 0;
        }
    }
    if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
    {
        ulPhyCfgMin = ulPhyCfgMinTmp;
        ulPhyCfgMax = ulPhyCfgMaxTmp;
        *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
    }
    if((ulPhyCfgMin == 0xffffffff) || ((ulPhyCfgMax - ulPhyCfgMin + 1) < 1))
    {
        ret = ERR_FLASH_CALIBRATE;
        FC_PHY_CRT_CH(PLL_PI0, be_no) &= (~(0x3f << 16));
        llfDbgPrintk(ALWAYS_MSG, "bank %d Tx K fail %x %x-%x\r\n", ubBankNo, index, ulPhyCfgMin,
                     ulPhyCfgMax);
    }
    else
    {
        ret = ERR_OK;
        *ulPhyCfg = ((ulPhyCfgMax + ulPhyCfgMin) >> 1);
        if(*ulPhyCfg == 15 || *ulPhyCfg == 31 || *ulPhyCfg == 47 || *ulPhyCfg == 63)
            *ulPhyCfg += 1;
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_TX_K]bank %d CH%dCE%d TX_PI min=%d max=%d gap=%d set=%d\r\n",
                     ubBankNo, be_no, ce_no, ulPhyCfgMin, ulPhyCfgMax, *ulPhyCfgGap, *ulPhyCfg);
        llfSetTxDqDelay(be_no, *ulPhyCfg);

        if((gubNvmeTxDelayMin[be_no] == 0xff)
                || (gubNvmeTxDelayMin[be_no] < ulPhyCfgMin))
        {
            gubNvmeTxDelayMin[be_no] = ulPhyCfgMin;
        }

        if((gubNvmeTxDelayMax[be_no] == 0)
                || (gubNvmeTxDelayMax[be_no] > ulPhyCfgMax))
        {
            gubNvmeTxDelayMax[be_no] = ulPhyCfgMax;
        }
    }
    return ret;
}

U32 llfCalibrateRxDelayChain(U8 ubBankNo, U8 ubFcMode, U8 ubMode, U32 *ulPhyCfgGap)
{
    U32 ret = ERR_OK;
    U8 be_no, ce_no;
    U8 fifo_delay = 0;
    U8 coarse_rx_delay = 0;
    U8 thin_rx_delay = 0;
    U8 coarse_range_max = 0x20;
    U8 ubCalibrateAll = 0;
    U32 coarse_rx_delay_min = 0;
    U8 tx_pd = 0;
#ifdef RTS5771_VA
    tx_pd = 1;
#endif

    default_delay_chain_setting(ubFcMode, ubMode, &coarse_rx_delay, &thin_rx_delay, &fifo_delay);
    if(ubMode < 9)
    {
        gubNvmeThinGap = 30;
    }
    else if (ubMode < 12)
    {
        gubNvmeThinGap = 30;
    }
    else if (ubMode < 13)
    {
        gubNvmeThinGap = 25;
    }
    else if (ubMode < 14)
    {
        gubNvmeThinGap = 20;
    }
    else if (ubMode < 15)
    {
        gubNvmeThinGap = 16;
    }
    else if (ubMode < 16)
    {
        gubNvmeThinGap = 14;
    }
    else if(ubMode < 17)
    {
        gubNvmeThinGap = 13;
    }
    else if (ubMode < 18)
    {
        gubNvmeThinGap = 12;
    }
    else if (ubMode < 19)
    {
        gubNvmeThinGap = 12;
    }
    else if (ubMode < 20)
    {
        gubNvmeThinGap = 12;
    }
    else
    {
        gubNvmeThinGap = 12;
    }

    ubCalibrateAll = (gubCalibrateConfig >> 2) & 0x1;
    be_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
    FR_REG_DLL_BASE_X = FR_DLL_REG_BASE + be_no * FR_ONFI_REG_SIZE;
    *ulPhyCfgGap = 0;

    if(ubFcMode == ONFI_SDR)
    {
        // SDR mode need not to calibrate dqs input
        coarse_rx_delay = 0x0;
#ifdef RTS5771_VA
        FR_REG32_DLL_X(ONFI_CE_SEL_DLY_1) =  0x00000000 | coarse_rx_delay << 16;
#else
        FR_REG32_DLL_X(ONFI_READ_CTRL_0) =  0x00000008 | coarse_rx_delay << 20;
#endif

        FR_REG32_DLL_X(ONFI_READ_CTRL_1) = 0x0000000 | fifo_delay;
        ret = ERR_OK;
        return ret;
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "======bank %d======\r\n", ubBankNo);
        if(!ubCalibrateAll)
        {
            ret = coarse_rx_delay_calibration(ubFcMode, ubMode, ubBankNo, thin_rx_delay, fifo_delay);
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "bank %d K course delay timeout\r\n", ubBankNo);
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
                return ERR_FIO_TIMEOUT;
            }

#ifdef RTS5771_VA
            coarse_rx_delay = FR_REG32_DLL_X(ONFI_CE_SEL_DLY_1) >> 16;
#else
            coarse_rx_delay = FR_REG32_DLL_X(ONFI_READ_CTRL_0) >> 20;
#endif
            ret = thin_rx_delay_calibration(ubFcMode, ubMode, ubBankNo, coarse_rx_delay, coarse_range_max,
                                            fifo_delay);
            if(ret == ERR_FIO_TIMEOUT)
            {
                llfDbgPrintk(ALWAYS_MSG, "bank %d K thin delay timeout\r\n", ubBankNo);
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
                return ERR_FIO_TIMEOUT;
            }

            while(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "bank %d rx_delay coarse: %d\r\n", ubBankNo, coarse_rx_delay);
                coarse_rx_delay++;
                if(coarse_rx_delay >= coarse_range_max)
                {
                    llfDbgPrintk(ALWAYS_MSG, "bank %d K thin delay fail\r\n", ubBankNo);
                    AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
                    return ERR_FLASH_CALIBRATE;
                }
                ret = thin_rx_delay_calibration(ubFcMode, ubMode, ubBankNo, coarse_rx_delay, coarse_range_max,
                                                fifo_delay);
            }
            thin_rx_delay = FR_REG32_DLL_X(ONFI_DQS_IN_DLY_1) & 0x1f;
            llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]bank %d CH%dCE%d Thin_Delay min=%d max=%d gap=%d\r\n",
                         ubBankNo, be_no, ce_no, gubNvmeRxDelayMinTemp, gubNvmeRxDelayMaxTemp,
                         (gubNvmeRxDelayMaxTemp - gubNvmeRxDelayMinTemp + 1));
        }
        else
        {
            gubNvmeRxDelayMinTemp = 0xff;
            gubNvmeRxDelayMaxTemp = 0;
            for(coarse_rx_delay_min = 0; coarse_rx_delay_min < coarse_range_max; coarse_rx_delay_min++)
            {
                gubNvmeThinMax = 0xff;
                ret = thin_rx_delay_calibration(ubFcMode, ubMode, ubBankNo, coarse_rx_delay_min, coarse_range_max,
                                                fifo_delay);
                if(gubNvmeThinMax < 31)
                {
                    thin_rx_delay = ((gubNvmeRxDelayMinTemp + gubNvmeRxDelayMaxTemp) >> 1);
                    coarse_rx_delay = 0;
                    while(thin_rx_delay >= 31)
                    {
                        coarse_rx_delay++;
                        thin_rx_delay -= 2;
                    }

#ifdef RTS5771_VA
                    FR_REG32_DLL_X(ONFI_CE_SEL_DLY_1) =  0x00000000 | coarse_rx_delay << 16;
#else
                    FR_REG32_DLL_X(ONFI_READ_CTRL_0) =  0x00000008 | coarse_rx_delay << 20;
#endif
                    FR_REG32_DLL_X(ONFI_READ_CTRL_1) = 0x0000000 | fifo_delay;

                    FR_REG32_DLL_X(ONFI_DQS_IN_DLY_0) = 0x0 | (thin_rx_delay << 0) | (thin_rx_delay << 8) |
                                                        (thin_rx_delay << 16) | (thin_rx_delay << 24);
                    FR_REG32_DLL_X(ONFI_DQS_IN_DLY_1) = 0x0 | (thin_rx_delay << 0) | (thin_rx_delay << 8) |
                                                        (thin_rx_delay << 16) | (thin_rx_delay << 24);
                    FR_REG32_DLL_X(ONFI_DQS_IN_DLY_2) = 0x0 | (thin_rx_delay << 0) | (thin_rx_delay << 8) |
                                                        (thin_rx_delay << 16) | (thin_rx_delay << 24);
                    FR_REG32_DLL_X(ONFI_DQS_IN_DLY_3) = 0x0 | (thin_rx_delay << 0) | (thin_rx_delay << 8) |
                                                        (thin_rx_delay << 16) | (thin_rx_delay << 24);
                    //update the value of DQS_IN_DLY_*
                    FR_REG32_DLL_X( ONFI_DPI_CTRL_0) = 0x020f0032 | (tx_pd << 31);
                    FR_REG32_DLL_X( ONFI_DPI_CTRL_1) = 0x0000000f;
                    FR_REG32_DLL_X( ONFI_DPI_CTRL_0) = 0x020f0030 | (tx_pd << 31);

                    break;
                }
            }
            if(gubNvmeThinMax == 0xff)
            {
                return ret;
            }
            else
            {
                llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]bank %d CH%dCE%d Thin_Delay min=%d max=%d gap=%d\r\n",
                             ubBankNo, be_no, ce_no, gubNvmeRxDelayMinTemp, gubNvmeRxDelayMaxTemp,
                             (gubNvmeRxDelayMaxTemp - gubNvmeRxDelayMinTemp + 1));
            }
        }


        ret = fifo_rx_calibration(ubFcMode, ubMode, ubBankNo, coarse_rx_delay, thin_rx_delay);
        fifo_delay = (FR_REG32_DLL_X(ONFI_READ_CTRL_1)) & 0x1f;
        //llfDbgPrintk(ALWAYS_MSG, "bank %d CH%dCE%d coarse=0x%x thin=0x%x fifo=0x%x\r\n", ubBankNo, be_no, ce_no,
        //          coarse_rx_delay, thin_rx_delay, fifo_delay);
        if(ret == ERR_OK)
        {
            *ulPhyCfgGap = gubNvmeThinGap;
        }
        FR_REG32_DLL_X(ONFI_DPI_CTRL_0) = FR_REG32_DLL_X(ONFI_DPI_CTRL_0) & 0xffffffcf;
    }

    FcBusyWait1ms(1);

    return ret;
}

void llfAPBECalibrateTxAuto(U8 ubIFType, U8 ubClkMode)
{
    U8 ubBankNo = 0, ubBankNum = 0;
    U8 ubBeNo = 0;
    U8 ubFcMode = ONFI_DDR2_TOGGLE;
    U8 ubCalibrateAll = 0;
    U8 ubTxCalibrate = 0;
    U8 ubFcDQSOdtIndex = 1, ubFcDQOdtIndex = 1;
    U8 ubNandDefaultOcd = 1, ubNandOcdIndex = 1;
    PLLF_UNI_INFO pLLFInfo = NULL;
    PVENDOR_CMD_RESPONSE pLLFResponseInfo = NULL;
    U8 calibrateResult[CH_NUM_MAX * CE_NUM_MAX];
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};
    U32 ret = ERR_OK, ulPhyCfg = 0, ulPhyCfgGap = 0;

    U8 ubFcOcdIndex = 0, ubFcMaxOcdOption = 4, ubFcBestOcd = 0;
    U8 ubNandOdtIndex = 0, ubNandMaxOdtOption = 1, ubNandBestOdt = 0;
    U8 ubPassBankNum = 0, passDrivingCount = 0;
    U8 ubTempTxPi[CH_NUM_MAX] = { 0 };

    U8 ubFcOcdList[4] = { 50, 35, 25, 18 };
    U8 ubNandOdtList[5] = { 0xff, 150, 100, 75, 50 };

    ubCalibrateAll = (gubCalibrateConfig >> 1) & 0x01;
    ubTxCalibrate = (gubCalibrateConfig >> 4) & 0x01;
    pLLFResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
    if (pLLFResponseInfo->res_state == VENDOR_CMD_START)
    {
        pLLFResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pLLFResponseInfo->res_progress = 0;
        pLLFResponseInfo->res_err_code = ERR_OK;
        pLLFResponseInfo->err_msg_num = 0;

        pLLFInfo->ubBankNo = 0;
        pLLFInfo->block = 0;
        pLLFInfo->page = 0;
        pLLFInfo->ulWorkFunc = CALIBRATE_FUNC;
    }

    ubBankNum = UnbalancedGetBankNum();
    llfWriteReadCache10MTest(ubIFType, ubClkMode);
    if(pLLFResponseInfo->res_err_code != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]10M read write cache fail\r\n");
        return;
    }

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_8T22_SDR
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_9T23_SDR_TOGGLE
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT23_SDR_TOGGLE_64GB)
        {
            ubNandMaxOdtOption = 5;
        }
        else
        {
            ubNandMaxOdtOption = 1;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_8T22_SDR
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_9T23_SDR_TOGGLE
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT23_SDR_TOGGLE_64GB
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24_64G
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS4P5_256Gb
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS4P5_512Gb
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS4_QLC
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS5_QLC)
        {
            ubNandMaxOdtOption = 5;
        }
        else
        {
            ubNandMaxOdtOption = 1;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4_64G)
        {
            ubNandMaxOdtOption = 1;
        }
        else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV7_512Gb
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_1v8
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_512Gb
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_512Gb_1v8
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5_64G)
        {
            ubNandMaxOdtOption = 4;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON
            || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL
            || FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX
            || FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
    {
        ubNandMaxOdtOption = 5;
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Not ready!!!\r\n");
        ASSERT(ALWAYS_MSG, 0);
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT)))
    {
        if(!gubNANDODTEnable)
            ubNandMaxOdtOption = 1;
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_64GB)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_128GB)))
    {
        if(!gubNANDODTEnable)
            ubNandMaxOdtOption = 1;
    }
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]FC OCD has %d options\r\n", ubFcMaxOcdOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]NAND ODT has %d options\r\n", ubNandMaxOdtOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]Calibrate all: %d\r\n", ubCalibrateAll);

    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]Clock mode:%d, %d MHz\r\n", ubClkMode, FcCycleNum[ubClkMode]);
    llfChangeInterFace(ubIFType);
    ret = FCTimingModeChange(ubIFType, ubClkMode);
    if(ret != ERR_OK)
    {
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ret;
        return;
    }

    llfprintk("[AUTO_TX_K]Odt drive diff\r\n");
    FCGetNandODTDiffVrefValue();
    ret = SetWarmUpOdtDrvDiff(gulNandODTDiffVrefValue, gubNandDriv,
                              GETFEATURE_BY_BANK_UC, GETFEATURE_BY_BANK_PHY);
    if(ret != ERR_OK)
    {
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ret;
        return;
    }


    llfLoadTimingFromConfig();
    ubFcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    ChangeFCClk(ubFcMode, ubClkMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | ((FcCycleNum[ubClkMode] >> 1) << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | (FcCycleNum[ubClkMode] << 8)));
#endif
    onfi4_change_setting(ubFcMode, ubClkMode, 3, 1, 1, gubFCDiffEnable);

#if 0
#ifdef RL6577_VA
#if defined(FTL_B27B) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6)
    if(ubClkMode > FC_PLL_CLK_400M)
    {
        U32 Reg_v;
        //Nand_ZQ_Calibration(FR_CONFIG_CH(FR_FC_MODE, 0));//nand zq k,temp mask
        DCC_Training(FR_CONFIG_CH(FR_FC_MODE, 0), ubClkMode);
        //reset onfi fifo pointer
        Reg_v = FC_PHY_DLL_CH(ONFI_DPI_CTRL_0, 0) | 0xc;
        FR_G_PHY_REG32_W(ONFI_DPI_CTRL_0, Reg_v );
        Reg_v = FC_PHY_DLL_CH(ONFI_DPI_CTRL_0, 0) & 0xfffffff3;
        FR_G_PHY_REG32_W(ONFI_DPI_CTRL_0, Reg_v );
    }
#endif
#endif
#endif

    FR_G_CFG_REG32_W(FR_ECC_THV, gubCacheErrbitLimit);
    gubFcOcdIndex = 3;
    if((gubCalibrateConfig >> 3) & 0x01)
    {
        ubNandDefaultOcd = gubNandDriv;
        ubFcDQSOdtIndex = 5 - gulFc_dqs_odt;
        ubFcDQOdtIndex = 5 - gulFc_dq_re_odt;

        if(FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
        {
            if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)
            {
                ubNandOcdIndex = ubNandDefaultOcd - 3;
            }
            else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
            {
                ubNandOcdIndex = ubNandDefaultOcd - 2;
            }
            else
            {
                ubNandOcdIndex = 3 - ubNandDefaultOcd;
            }
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
        {
            ubNandOcdIndex = 3 - ubNandDefaultOcd;
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
                || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK
                || FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX
                || FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
        {
            ubNandOcdIndex = (ubNandDefaultOcd - 2) >> 1;
        }
    }
    else
    {
        ubFcDQSOdtIndex = 1;
        ubFcDQOdtIndex = 1;
        ubNandOcdIndex = 1;
    }
    llfSetFcPadRxODT(ubFcDQSOdtIndex, ubFcDQOdtIndex);
    llfSetNandTxOCD(ubNandOcdIndex, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
    for(ubNandOdtIndex = 0; ubNandOdtIndex < ubNandMaxOdtOption; ubNandOdtIndex++)
    {
        if((ubNandOdtIndex == 2) && (((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)) &&
                                     (FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON ||
                                      FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)))
            continue;
        llfSetNandRxODT(ubNandOdtIndex, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);

        for(ubFcOcdIndex = 0; ubFcOcdIndex < ubFcMaxOcdOption; ubFcOcdIndex++)
        {
            if(ubNandOdtIndex == 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "==> [AUTO_TX_K] NAND ODT=%d(dis), FC OCD=%d(%dohm) <==\r\n",
                             ubNandOdtIndex, ubFcOcdIndex, ubFcOcdList[ubFcOcdIndex]);
            }
            else
            {
                llfDbgPrintk(ALWAYS_MSG, "==> [AUTO_TX_K] NAND ODT=%d(%dohm), FC OCD=%d(%dohm) <==\r\n",
                             ubNandOdtIndex, ubNandOdtList[ubNandOdtIndex], ubFcOcdIndex, ubFcOcdList[ubFcOcdIndex]);
            }
            llfSetFcPadTxOCD(ubFcOcdIndex);

            ubPassBankNum = 0;
            for(ubBeNo = 0; ubBeNo < CH_NUM_MAX; ubBeNo++)
            {
                gubNvmeTxDelayMin[ubBeNo] = 0xff;
                gubNvmeTxDelayMax[ubBeNo] = 0;
            }
            for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
            {
                calibrateResult[ubBankNo] = 0;
                if(!ubTxCalibrate)
                {
                    gubDqsFix90 = 1;
                    ret = llfDQSDQFix90WriteReadTest(ubBankNo, ubFcMode, ubClkMode,
                                                     FcCycleNum[ubClkMode], &ulPhyCfgGap);
                }
                else
                {
                    gubDqsFix90 = 0;
                    ret = llfCalibrateTxDqDelayChain(ubBankNo, ubFcMode, ubClkMode,
                                                     FcCycleNum[ubClkMode], &ulPhyCfg, &ulPhyCfgGap);
                }

                if(ret == ERR_FIO_TIMEOUT)
                {
                    llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                    llfSetFcPadRxODT(ubFcDQSOdtIndex, ubFcDQOdtIndex);
                    break;
                }
                else
                {
                    if(ulPhyCfgGap >= AUTO_CALIBRATE_TX_GAP)
                    {
                        ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                        ubPassBankNum++;
                        calibrateResult[ubBankNo] = 1;
                        ubTempTxPi[ubBeNo] = ulPhyCfg;
                    }
                    else
                    {
                        llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                        llfSetFcPadRxODT(ubFcDQSOdtIndex, ubFcDQOdtIndex);
                        break;
                    }
                }
            }

            if(ubPassBankNum == ubBankNum)
            {
                if(passDrivingCount == 0)
                {
                    passDrivingCount++;
                    ubNandBestOdt = ubNandOdtIndex;
                    ubFcBestOcd = ubFcOcdIndex;
                    if(ubTxCalibrate)
                    {
                        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
                        {
                            ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                            ubTempTxPi[ubBeNo] = ((gubNvmeTxDelayMin[ubBeNo] + gubNvmeTxDelayMax[ubBeNo]) >> 1);
                            if(ubTempTxPi[ubBeNo] == 15 || ubTempTxPi[ubBeNo] == 31 || ubTempTxPi[ubBeNo] == 47
                                    || ubTempTxPi[ubBeNo] == 63)
                                ubTempTxPi[ubBeNo] += 1;
                            llfDbgPrintk(ALWAYS_MSG, "TX DQ PI: CH%d %d\r\n", ubBeNo, ubTempTxPi[ubBeNo]);
                            gubOnfiTxPi[ubBeNo] = ubTempTxPi[ubBeNo];
                        }
                    }
                }
                if(!ubCalibrateAll)
                {
                    break;
                }
            }
        }

        if(ubPassBankNum == ubBankNum)
        {
            if(!ubCalibrateAll)
            {
                break;
            }
        }
    }

    //Auto-K error handle
    if(passDrivingCount < 1)
    {
        if(ubNandOdtIndex == ubNandMaxOdtOption && ubFcOcdIndex == ubFcMaxOcdOption
                && ulPhyCfgGap == 0)
        {
            for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
            {
                if(calibrateResult[ubBankNo] == 0)
                {
                    AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
                }
            }
        }
    }
    if(pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]Failed for TX with %dMhz\r\n", FcCycleNum[ubClkMode]);
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    if(passDrivingCount > 0)
    {
        gubFcOcdIndex = ubFcBestOcd;
        llfSetFcPadTxOCD(ubFcBestOcd);
        llfSetNandRxODT(ubNandBestOdt, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
        if(ubTxCalibrate)
        {
            for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
            {
                ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                llfSetTxDqDelay(ubBeNo, gubOnfiTxPi[ubBeNo]);
            }
        }
    }
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K] FC_OCD = %d  (0:50ohm 1:35ohm 2:25ohm 3:18ohm)\r\n",
                 ubFcBestOcd);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K] NAND_ODT = %d  (0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm)\r\n",
                 ubNandBestOdt);
    llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] FC_OCD: Config_0x840 = 0xc%d, FC_OCD_Set_for_Reg = %d\r\n",
              3 - ubFcBestOcd, 3 - ubFcBestOcd);
    if(ubNandBestOdt)
    {
#if defined(RL6447_VA)
        llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] NAND_ODT_EN: Config_0x7dd = 0xc1, NAND_ODT: Config_0x838 = 0xc%d, NAND_ODT_Set_base_SPEC = %d, \r\n",
                  ubNandBestOdt, ubNandBestOdt);
#else
        llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] NAND_ODT_EN: Config_0x7dd = 0x1, NAND_ODT: Config_0x838 = 0x%d, NAND_ODT_Set_base_SPEC = %d, \r\n",
                  ubNandBestOdt, ubNandBestOdt);
#endif
    }
    else
    {
        llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] NAND_ODT_EN: Config_0x7dd = 0x0, NAND_ODT: Config_0x838 = 0x0\r\n");
    }

    if(!((gubCalibrateConfig >> 3) & 0x01))
    {
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH) = 3 - ubFcBestOcd;
        gulFc_ocd = 3 - ubFcBestOcd;
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_CFG) = ubNandBestOdt;
        gubNANDODTCfg = ubNandBestOdt;
        if(ubNandBestOdt > 0)
        {
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN) = 1;
            gubNANDODTEnable = 1;
        }
    }

    pLLFInfo->ubData0 = AUTO_CALIBRATE_RX_START;
}

void llfAPBECalibrateRxAuto(U8 ubIFType, U8 ubClkMode)
{
    U8 ubBankNo = 0, ubBankNum = 0;
    U8 ubFcMode = ONFI_DDR2_TOGGLE;
    U8 ubCalibrateAll = 0;
    U8 ubBeNo = 0;
    PLLF_UNI_INFO pLLFInfo = NULL;
    PVENDOR_CMD_RESPONSE pLLFResponseInfo = NULL;
#ifndef NEW_MUL_WR_CACHE
    PVENDOR_CMD pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
    entry_t entry = 0;
    U32 lock_flag;
#endif
    U8 calibrateResult[CH_NUM_MAX * CE_NUM_MAX];
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};
    U32 ret = ERR_OK, ulPhyCfgGap = 0;

    U8 ubFcDqsOdtIndex = 0, ubFcDqOdtIndex = 0, ubFcMaxOdtOption = 6, ubFcDqsBestOdt = 0,
       ubFcDqBestOdt = 0;
    U8 ubNandOcdIndex = 0, ubNandMaxOcdOption = 3, ubNandBestOcd = 0;
    U8 ubPassBankNum = 0, passDrivingCount = 0;
    U8 ubNandOdtCfg = 0, ubFcOcdCfg = 0, ubNandOdtEnable = 0;
    U8 ubRawDataK = 0;
    U8 ubPipelineK = 0;

    U8 ubNandOcdList[3] = { 50, 35, 25 };
    U8 ubFcOdtList[6] = { 0xff, 150, 100, 75, 50, 35 };

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;
    ubPipelineK = (gubCalibrateConfig >> 6) & 0x01;
    ubCalibrateAll = (gubCalibrateConfig >> 1) & 0x01;

    pLLFResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

    ubBankNum = UnbalancedGetBankNum();

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
            || FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON
            || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK
            || FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
    {
        ubNandMaxOcdOption = 3;
        if((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON
                || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
                && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38B
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_Q5171A))
        {
            ubNandMaxOcdOption = 2;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4_64G)
        {
            ubNandMaxOcdOption = 2;
        }
        else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV2_128G
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5_64G
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_1v8
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_512Gb
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_512Gb_1v8
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV7_512Gb)
        {
            ubNandMaxOcdOption = 3;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
    {
        ubNandMaxOcdOption = 3;
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS
                || !vdd_1v8_en)
        {
            ubNandMaxOcdOption = 2;
        }
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Not ready!!!\r\n");
        ASSERT(ALWAYS_MSG, 0);
    }
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]NAND OCD has %d options\r\n", ubNandMaxOcdOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]FC ODT has %d options\r\n", ubFcMaxOdtOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Calibrate all: %d\r\n", ubCalibrateAll);

    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Clock mode:%d, %d MHz\r\n", ubClkMode, FcCycleNum[ubClkMode]);
    ubFcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    ChangeFCClk(ubFcMode, ubClkMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | ((FcCycleNum[ubClkMode] >> 1) << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | (FcCycleNum[ubClkMode] << 8)));
#endif
    if(!gubDqsFix90)
    {
        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
            llfSetTxDqDelay(ubBeNo, gubOnfiTxPi[ubBeNo]);
            //llfprintk("bank:%d CH%d,txdelay:%d\r\n", ubBankNo, ubBeNo, gubOnfiTxPi[ubBeNo]);
        }
    }

    for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
    {
        if(ubRawDataK)
        {
            if(llfWriteCacheRedundant(ubBankNo) != ERR_OK)
            {
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
            }
        }
        else
        {
            if(llfWriteCache(ubBankNo) != ERR_OK)
            {
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
            }
        }
    }
    if (pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Failed to write with %dMhz\r\n", FcCycleNum[ubClkMode]);
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    if((gubCalibrateConfig >> 3) & 0x01)
    {
        ubFcOcdCfg      = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH);
        ubNandOdtEnable = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN);
        ubNandOdtCfg    = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_CFG);

        if((ubFcOcdCfg & 0xF0) == 0xc0)
        {
            ubFcOcdCfg = ubFcOcdCfg & 0xf;
        }
        if((ubNandOdtEnable & 0xF0) == 0xc0)
        {
            ubNandOdtEnable = ubNandOdtEnable & 0xf;
        }
        if(!ubNandOdtEnable)
        {
            ubNandOdtCfg = 0;
        }
        gubFcOcdIndex = 3 - ubFcOcdCfg;
        gulFc_ocd = ubFcOcdCfg;
        gubNANDODTEnable = ubNandOdtEnable;
        gubNANDODTCfg = ubNandOdtCfg;
        llfprintk("[Set TX driving in RX] Nand: ODTEN %x ODTCfg %x\r\n", ubNandOdtEnable, ubNandOdtCfg);
        llfprintk("[Set TX driving in RX] FC :  OCD %x\r\n", ubFcOcdCfg);

        llfSetFcPadTxOCD(gubFcOcdIndex);
        llfSetNandRxODT(ubNandOdtCfg, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
    }

    for(ubFcDqOdtIndex = 0; ubFcDqOdtIndex < ubFcMaxOdtOption; ubFcDqOdtIndex++)
    {
        for(ubFcDqsOdtIndex = 0; ubFcDqsOdtIndex < ubFcMaxOdtOption; ubFcDqsOdtIndex++)
        {
            if(ubFcDqsOdtIndex < ubFcDqOdtIndex)
                continue;
            llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);

            for(ubNandOcdIndex = 0; ubNandOcdIndex < ubNandMaxOcdOption; ubNandOcdIndex++)
            {
                FcBusyWait1ms(500);
                llfSetNandTxOCD(ubNandOcdIndex, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);

                if(ubFcDqOdtIndex == 0 && ubFcDqsOdtIndex == 0)
                {
                    llfDbgPrintk(ALWAYS_MSG,
                                 "==> [AUTO_RX_K] FC DQS ODT=%d(dis), FC DQ ODT=%d(dis), NAND OCD=%d(%dohm) <==\r\n",
                                 ubFcDqsOdtIndex, ubFcDqOdtIndex, ubNandOcdIndex, ubNandOcdList[ubNandOcdIndex]);
                }
                else if(ubFcDqOdtIndex == 0 && ubFcDqsOdtIndex > 0)
                {
                    llfDbgPrintk(ALWAYS_MSG,
                                 "==> [AUTO_RX_K] FC DQS ODT=%d(%dohm), FC DQ ODT=%d(dis), NAND OCD=%d(%dohm) <==\r\n",
                                 ubFcDqsOdtIndex, ubFcOdtList[ubFcDqsOdtIndex], ubFcDqOdtIndex,
                                 ubNandOcdIndex, ubNandOcdList[ubNandOcdIndex]);
                }
                else
                {
                    llfDbgPrintk(ALWAYS_MSG,
                                 "==> [AUTO_RX_K] FC DQS ODT=%d(%dohm), FC DQ ODT=%d(%dohm), NAND OCD=%d(%dohm) <==\r\n",
                                 ubFcDqsOdtIndex, ubFcOdtList[ubFcDqsOdtIndex], ubFcDqOdtIndex, ubFcOdtList[ubFcDqOdtIndex],
                                 ubNandOcdIndex, ubNandOcdList[ubNandOcdIndex]);
                }
                ubPassBankNum = 0;
                for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
                {
                    calibrateResult[ubBankNo] = 0;
                    if(!gubDqsFix90)
                    {
                        ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                        llfSetTxDqDelay(ubBeNo, gubOnfiTxPi[ubBeNo]);
                    }
                    if(ubPipelineK)
                    {
                        if(ubRawDataK)
                            ret = llfWriteCacheRedundant(ubBankNo);
                        else
                            ret = llfWriteCache(ubBankNo);
                        if(ret != ERR_OK)
                        {
                            llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                            llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);
                            break;
                        }
                    }
                    ret = llfCalibrateRxDelayChain(ubBankNo, ubFcMode, ubClkMode, &ulPhyCfgGap);
                    if(ret == ERR_FIO_TIMEOUT)
                    {
                        llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                        llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);
                        break;
                    }
                    else
                    {
                        if(ulPhyCfgGap >= AUTO_CALIBRATE_RX_GAP)
                        {
                            ubPassBankNum++;
                            calibrateResult[ubBankNo] = 1;
                        }
                        else
                        {
                            llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                            llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);
                            break;
                        }
                    }
                }

                if(ubPassBankNum == ubBankNum)
                {
                    if(passDrivingCount == 0)
                    {
                        passDrivingCount++;
                        ubNandBestOcd = ubNandOcdIndex;
                        ubFcDqsBestOdt = ubFcDqsOdtIndex;
                        ubFcDqBestOdt = ubFcDqOdtIndex;
                        llfSettingPerCh(SBLK_ADDR, 0, ubClkMode);
                    }
                    if(!ubCalibrateAll)
                    {
                        break;
                    }
                }
            }

            if(ubPassBankNum == ubBankNum)
            {
                if(!ubCalibrateAll)
                {
                    break;
                }
            }
        }

        if(ubPassBankNum == ubBankNum)
        {
            if(!ubCalibrateAll)
            {
                break;
            }
        }
    }

    //Auto-K error handle
    if(passDrivingCount < 1)
    {
        if(ubFcDqsOdtIndex == ubFcMaxOdtOption && ubFcDqOdtIndex == ubFcMaxOdtOption
                && ubNandOcdIndex == ubNandMaxOcdOption && ulPhyCfgGap == 0)
        {
            for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
            {
                if(calibrateResult[ubBankNo] == 0)
                {
                    AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
                }
            }
        }
    }
    if(pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Failed for RX with %dMhz.\r\n", FcCycleNum[ubClkMode]);
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    if(passDrivingCount > 0)
    {
        llfLoadSblkPerCh(SBLK_ADDR, 0);
        llfSetFcPadRxODT(ubFcDqsBestOdt, ubFcDqBestOdt);
        llfSetNandTxOCD(ubNandBestOcd, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
        if(!gubDqsFix90)
        {
            for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
            {
                ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                llfSetTxDqDelay(ubBeNo, gubOnfiTxPi[ubBeNo]);
                llfDbgPrintk(ALWAYS_MSG, "bank:%d CH%d, txdelay:%d\r\n", ubBeNo, ubBeNo, gubOnfiTxPi[ubBeNo]);
            }
        }
    }

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K] ------> NAND_OCD = %d <--------(0:50ohm 1:37.5ohm)\r\n",
                     ubNandBestOcd);
    else
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K] ------>NAND_OCD = %d <-------(0:50ohm 1:35ohm 2:25ohm)\r\n", ubNandBestOcd);
    if(gulFc_dqs_odt_en)
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K] ------>FC_DQS_ODT = %d <------(0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n",
                     ubFcDqsBestOdt);
    else
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K] ------>FC_DQS_ODT = 0 <------(0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n");
    if(gulFc_dq_re_odt_en)
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K] ------>FC_DQRE_ODT = %d <------(0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n",
                     ubFcDqBestOdt);
    else
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K] ------>FC DQRE ODT = 0 <------(0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n");
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)
        {
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = 3 + ubNandBestOcd;
            gubNandDriv = 3 + ubNandBestOcd;
        }
        else
        {
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = 3 - ubNandBestOcd;
            gubNandDriv = 3 - ubNandBestOcd;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK
            || FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = (ubNandBestOcd << 1) + 2;
        gubNandDriv = (ubNandBestOcd << 1) + 2;
    }
    //llfSettingPerCh(SBLK_ADDR, 0, ubClkMode);
#if defined(RL6447_VA)
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG))
    {
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] NAND_OCD: Config_0x7ee = 0xc%d, NAND_OCD_Set_base_SPEC = %d\r\n",
                  gubNandDriv, gubNandDriv);
    }
    else
    {
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] NAND_OCD: Config_0x7ed = 0xc%d, NAND_OCD_Set_base_SPEC = %d\r\n",
                  gubNandDriv, gubNandDriv);
    }
#else
    llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] NAND_OCD: Config_0x7ed = %d, NAND_OCD_Set_base_SPEC = %d\r\n",
              gubNandDriv, gubNandDriv);
#endif
    if(gulFc_dqs_odt_en)
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC_DQS_ODT_EN: Config_0x830 = 0xc0f, Config_0x834 = 0xc%d, FC_DQS_ODT_For_Reg = 0xc%d\r\n",
                  5 - ubFcDqsBestOdt, 5 - ubFcDqsBestOdt);
    else
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC_DQS_ODT Disable: Config_0x830 = 0xc00, Config_0x834 = 0xc4\r\n");
    if(gulFc_dq_re_odt_en)
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC_DQ_ODT_EN: Config_0x844 = 0xc0f, Config_0x848 = 0xc%d, FC_DQ_ODT_For_Reg = 0xc%d\r\n",
                  5 - ubFcDqBestOdt, 5 - ubFcDqBestOdt);
    else
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC DQ ODT Disable: Config_0x844 = 0xc00, Config_0x848 = 0xc4\r\n");

    FR_G_CFG_REG32_W(FR_ECC_THV, 0x50);

    pLLFInfo->ubData0 = AUTO_CALIBRATE_TX_START;
#ifndef NEW_MUL_WR_CACHE
    entry = pLLFResponseInfo->entry;
    pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
    pLLFResponseInfo->res_err_code = ret;
    pLLFResponseInfo->res_progress = 100;
    if(pVendorCmd->subcmd == BE_LLF_ALL)
    {
        if(ret != ERR_OK)
        {
            ASSERT(ALWAYS_MSG, 0);
        }
        else
        {
            pLLFResponseInfo->err_msg_num = 0;
            pLLFResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
            pLLFResponseInfo->res_err_code = ret;
            pLLFResponseInfo->res_progress = 30;
            if(gubLLFMode <= LLF_FORCE_INHERIT || (gubLLFMode == LLF_DEFECT_BAD_BLOCK))
            {
                gubLLFALLStep = STEP_LOAD_RDT_START;
                pLLFResponseInfo->res_state = VENDOR_CMD_ERASE;
            }
            else if((gubLLFMode == LLF_FORCE_FORMAT))
            {
                gubLLFALLStep = STEP_FORMAT_INIT;
                pLLFResponseInfo->res_state = VENDOR_CMD_ERASE;
            }
            else
            {
                gubLLFALLStep = STEP_FORMAT_INIT;
                pLLFResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
            }
            _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = 0;
        }
    }
    else if(pVendorCmd->subcmd == BE_LLF_CALIBRATE)
    {
        gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
        spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
        SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
        spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
    }
    else
    {
        llfprintk("[AUTO_RX_K]Cmd: 0x%x\r\n", pVendorCmd->subcmd);
    }
#endif
}

U32 llfCalibrateTxDelayChain(U8 ubBankNo, U8 ubFcMode, U8 ubClkMode, U16 uwSpeed)
{
    U8 ubBeNo = 0;
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};
    U32 ret = ERR_OK, ulPhyCfg = 0, ulPhyCfgGap = 0;
    U8 ubTempTxPi = 0;
    ubBeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    gubDqsFix90 = 0;
    ret = llfCalibrateTxDqDelayChain(ubBankNo, ubFcMode, ubClkMode,
                                     FcCycleNum[ubClkMode], &ulPhyCfg, &ulPhyCfgGap);
    if(ret == ERR_FIO_TIMEOUT)
    {
        llfDbgPrintk(ALWAYS_MSG, "bank %d tx calibrate timeout\r\n", ubBankNo);
        return ret;
    }
    else
    {
        if(ulPhyCfgGap >= AUTO_CALIBRATE_TX_GAP)
        {
            //ubTempTxPi[ubBeNo] = ulPhyCfg;
        }
        else
        {
            ret = ERR_FLASH_CALIBRATE;
            return ret;
        }

        if(gubNvmeTxDelayMin[ubBeNo] >= 10 || gubNvmeTxDelayMax[ubBeNo] <= 22)
        {
            ret = ERR_FLASH_CALIBRATE;
            return ret;
        }
    }

    ubTempTxPi = ((gubNvmeTxDelayMin[ubBeNo] + gubNvmeTxDelayMax[ubBeNo]) >> 1);
#ifndef RTS5771_VA
    if(ubTempTxPi == 15 || ubTempTxPi == 31 || ubTempTxPi == 47 || ubTempTxPi == 63)
        ubTempTxPi += 1;
#endif
    llfDbgPrintk(ALWAYS_MSG, "bank %d TX DQ PI: CH%d %d(%d %d)\r\n",
                 ubBankNo, ubBeNo, ubTempTxPi, gubNvmeTxDelayMin[ubBeNo], gubNvmeTxDelayMax[ubBeNo]);
    gubOnfiTxPi[ubBeNo] = ubTempTxPi;

    ChangeFCClk(ubFcMode, ubClkMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | ((FcCycleNum[ubClkMode] >> 1) << 8)));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | (FcCycleNum[ubClkMode] << 8)));
#endif

    return ret;
}

#elif defined(RL6643_VA) || defined(RL6531_VB) ||  defined(RL6643_FPGA)
void llfChangeInterFace(U8 ubIFType)
{
    llfDbgPrintk(ALWAYS_MSG, "[NAND]Clock mode:%x, %d MHz\r\n", FC_PLL_CLK_10M, 10);
    FCInterfaceChange(ubIFType, FC_PLL_CLK_10M);
    llfLoadTimingFromConfig();

    ChangeFCClk(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M);
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | (0x0a << 8)));
#if defined(RL6643_VA) || defined(RL6643_FPGA)
    onfi4_ocd_odt_setting(3, 3, 4, 4);
#else
    Fc_ocd_odt_setting(3, 4);
#endif
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX))
    {
        fc_diff_setting(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M, 1);
    }
    else
    {
        fc_diff_setting(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M, 0);
    }
}

void llfSaveTimingDrivingPerCh(U32 addr)
{
    U8 i, j;
    U32 Offset = SBLK_FC_HW_CONFIG2_PER_SETTING_LEN;
    U32 ulDelayPhase = 0;

    for(i = 0; i < CH_NUM_MAX; i++)
    {
        FR_REG_BASE_X = FR_REG_BASE + i * FR_REG_SIZE;

        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PHY_SDR_CFG_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PHY_SDR_CFG);
        for(j = 0; j < CE_NUM_MAX; j++)
        {
            if((FR_REG32_X(FR_PHY_DELAY_CFG0 + (j * WORD_BYTE_SIZE)) & 0x7f00) == 0)//DQS TX=0
            {
                ulDelayPhase = FR_REG32_X(FR_PHY_DELAY_CFG0 + (j * WORD_BYTE_SIZE)) & 0x7f;//save DQS RX
                ulDelayPhase += (((FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + (j * WORD_BYTE_SIZE)) & 0x7f) + 0x80) << 8);
                _REG32(addr + SBLK_OFFSET_NORMAL_FR_PHY_DELAY_CFG0_PER_CH
                       + Offset + (i * 0x20) + (j * WORD_BYTE_SIZE)) = ulDelayPhase;
                printk("CH%d CE%d Delay Phase: 0x%x\r\n", i, j, ulDelayPhase);
            }
            else
            {
                _REG32(addr + SBLK_OFFSET_NORMAL_FR_PHY_DELAY_CFG0_PER_CH
                       + Offset + (i * 0x20) + (j * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PHY_DELAY_CFG0 +
                               (j * WORD_BYTE_SIZE));
            }
        }

#ifdef RL6531_VB
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_PULL_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_PULL);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG0_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_DRV_CFG0);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG1_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_DRV_CFG1);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG2_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_DRV_CFG2);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG3_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_DRV_CFG3);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_ODT_CFG_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_ODT_CFG);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_ODT_CTRL_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_ODT_CTRL);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_CFG0_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_CFG0);
        _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_CFG1_PER_CH
               + Offset + (i * WORD_BYTE_SIZE)) = FR_REG32_X(FR_PAD_CFG1);
#endif
    }

    cache_area_dwbinval(addr, SBLK_OFFSET_FC_HW_CONFIG2_LENGTH);
    cache_dummy_update_read(); // dummy read back
}

void llfSetTimingDrivingPerCh(U32 addr)
{
    U8 i, j;
    U32 Offset = SBLK_FC_HW_CONFIG2_PER_SETTING_LEN;

    for(i = 0; i < CH_NUM_MAX; i++)
    {
        FR_REG_BASE_X = FR_REG_BASE + i * FR_REG_SIZE;

        FR_REG32_X(FR_PHY_SDR_CFG) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PHY_SDR_CFG_PER_CH
                                            + Offset + (i * WORD_BYTE_SIZE));
        for(j = 0; j < CE_NUM_MAX; j++)
        {
            if(IS_6855_VERSION_TAG)
            {
                //before cfg delay chain,need to disable output
                FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
            }
            FR_REG32_X(FR_PHY_DELAY_CFG0 + (j * WORD_BYTE_SIZE)) = _REG32(addr +
                    SBLK_OFFSET_NORMAL_FR_PHY_DELAY_CFG0_PER_CH + Offset + (i * 0x20) + (j * WORD_BYTE_SIZE));
            if(IS_6855_VERSION_TAG)
                FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
        }


#ifdef RL6531_VB
        FR_REG32_X(FR_PAD_PULL) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_PULL_PER_CH
                                         + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_DRV_CFG0) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG0_PER_CH
                                             + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_DRV_CFG1) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG1_PER_CH
                                             + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_DRV_CFG2) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG2_PER_CH
                                             + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_DRV_CFG3) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG3_PER_CH
                                             + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_ODT_CFG) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_ODT_CFG_PER_CH
                                            + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_ODT_CTRL) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_ODT_CTRL_PER_CH
                                             + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_CFG0) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_CFG0_PER_CH
                                         + Offset + (i * WORD_BYTE_SIZE));
        FR_REG32_X(FR_PAD_CFG1) = _REG32(addr + SBLK_OFFSET_NORMAL_FR_PAD_CFG1_PER_CH
                                         + Offset + (i * WORD_BYTE_SIZE));
#endif
    }
}

// Calibrate DQS delay output
U32 llfCalibrateTxDqsDelayChain(U8 ubBankNo, U8 ubIFType, U8 ubClockMode, U16 ubCycleMode,
                                U32 *ulPhyCfgGap, U32 *minCfg, U32 *maxCfg)
{
    U32 ret = ERR_OK;
    U8 ubChNo, ubCeNo;
    U16 index;
    U32 ulPhyCfgMin = 0xffffffff;
    U32 ulPhyCfgMax = 0;
    U32 ulPhyCfgMinTmp = 0xffffffff;
    U32 ulPhyCfgMaxTmp = 0;
    U32 ulPhyCfg = 0;
    U32 ulCE0PhyCfgMax = 0xffffffff;
    U32 ulCE0PhyCfgMin = 0;
    U32 ret_data_compare = ERR_OK;
    U8 ubRawDataK = 0;
    U8 ubPipelineK = 0;

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;
    ubPipelineK = (gubCalibrateConfig >> 6) & 0x01;

    *ulPhyCfgGap = 0;
    *minCfg = 0;
    *maxCfg = 0;

    ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);

    // only for BE0, CE0, Lun0
    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;

    for(index = 0; index < 40; index++)
    {
        //Set TX
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8);
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (index & 0x7f) << 8;
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output

        //Set Timing&interface to desired Speed
        ChangeFCClk(ubIFType, ubClockMode);
        FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                      (ubCycleMode << 8)));
        //WriteCache
        if(ubRawDataK)
        {
            if(ubPipelineK)
                ret = llfWriteCacheRedundantPipeline(ubBankNo);
            else
                ret = llfWriteCacheRedundant(ubBankNo);
            if(ret == ERR_FIO_TIMEOUT)
            {
                return ret;
            }

            //Set Timing&interface to 10MHz
            //enable DQS SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if ( IS_6855_VERSION_TAG )
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    reg_v |= (0xa << 24);//set bit25/27,only enable DQS SMT
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
                }
            }
            ChangeFCClk(ubIFType, FC_PLL_CLK_10M);
            FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
            //ReadCache
            ret_data_compare = llfReadCacheRawDataCompare(ubBankNo);
            //Disable SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if(IS_6855_VERSION_TAG)
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);

                }
            }
        }
        else
        {
            if(ubPipelineK)
                ret = llfWriteCachePipeline(ubBankNo);
            else
                ret = llfWriteCache(ubBankNo);
            if(ret == ERR_FIO_TIMEOUT)
            {
                return ret;
            }

            //Set Timing&interface to 10MHz
            //enable DQS SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if ( IS_6855_VERSION_TAG )
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    reg_v |= (0xa << 24);//set bit25/27,only enable DQS SMT
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
                }
            }
            ChangeFCClk(ubIFType, FC_PLL_CLK_10M);
            FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
            //ReadCache
            ret_data_compare = llfReadCacheAndCompare(ubBankNo);
            //Disable SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if(IS_6855_VERSION_TAG)
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);

                }
            }
        }

        if(ret_data_compare == ERR_FIO_TIMEOUT)
        {
            return ret;
        }
        //Judgement of pass/fail
        if (ret_data_compare == ERR_OK)
        {
            //llfDbgPrintk(ALWAYS_MSG, "DQS TX K index:%d-->pass\r\n", index);
            if(index < ulPhyCfgMinTmp)
                ulPhyCfgMinTmp = index;
            if(index > ulPhyCfgMaxTmp)
                ulPhyCfgMaxTmp = index;
        }
        else
        {
            //llfDbgPrintk(ALWAYS_MSG, "DQS TX index:%d-->fail\r\n", index);
            if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
            {
                ulPhyCfgMin = ulPhyCfgMinTmp;
                ulPhyCfgMax = ulPhyCfgMaxTmp;
                *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
                *minCfg = ulPhyCfgMin;
                *maxCfg = ulPhyCfgMax;
            }
            ulPhyCfgMinTmp = 0xffffffff;
            ulPhyCfgMaxTmp = 0;
        }
    }
    if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
    {
        ulPhyCfgMin = ulPhyCfgMinTmp;
        ulPhyCfgMax = ulPhyCfgMaxTmp;
        *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
        *minCfg = ulPhyCfgMin;
        *maxCfg = ulPhyCfgMax;
    }
    if((ulPhyCfgMin == 0xffffffff) || (*ulPhyCfgGap < 1))
    {
        ret = ERR_FLASH_CALIBRATE;
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0xFF << 8); // Clean first
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output

        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]bank %d CH%dCE%d DQS fail %x %x-%x\r\n",
                     ubBankNo, ubChNo, ubCeNo, index, ulPhyCfgMin, ulPhyCfgMax);
    }
    else
    {
        ret = ERR_OK;

        if(ulPhyCfgMin > ulCE0PhyCfgMax)
            ret = ERR_FLASH_CALIBRATE;
        if(ulCE0PhyCfgMin < ulPhyCfgMin)
            ulCE0PhyCfgMin = ulPhyCfgMin;
        if(ulCE0PhyCfgMax > ulPhyCfgMax)
            ulCE0PhyCfgMax = ulPhyCfgMax;

        ulPhyCfg = ((ulCE0PhyCfgMin + ulCE0PhyCfgMax) >> 1);
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]bank %d CH%dCE%d DQS %d-%d=%d set%d\r\n",
                     ubBankNo, ubChNo, ubCeNo, ulCE0PhyCfgMin, ulCE0PhyCfgMax, *ulPhyCfgGap, ulPhyCfg);
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8); // Clean first
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (ulPhyCfg & 0x7f) << 8;
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
    }
    return ret;
}

// Calibrate DQ delay output
U32 llfCalibrateTxDqDelayChain(U8 ubBankNo, U8 ubIFType, U8 ubClockMode, U16 ubCycleMode,
                               U32 *ulPhyCfgGap, U32 *minCfg, U32 *maxCfg)
{
    U32 ret = ERR_OK;
    U8 ubChNo, ubCeNo;
    U16 index;
    U32 ulPhyCfgMin = 0xffffffff;
    U32 ulPhyCfgMax = 0;
    U32 ulPhyCfgMinTmp = 0xffffffff;
    U32 ulPhyCfgMaxTmp = 0;
    U32 ulPhyCfg = 0;
    U32 dqDelayOut0 = 0;
    U32 dqDelayOut1 = 0;
    U32 ret_data_compare = ERR_OK;
    U8 ubRawDataK = 0;
    U8 ubPipelineK = 0;

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;
    ubPipelineK = (gubCalibrateConfig >> 6) & 0x01;

    *ulPhyCfgGap = 0;
    *minCfg = 0;
    *maxCfg = 0;

    ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;

    //DQ Delay K
    dqDelayOut0 = FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4);
    dqDelayOut1 = FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4);
    if(IS_6855_VERSION_TAG)
    {
        //before cfg delay chain,need to disable output
        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
    }
    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8);
    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (0 & 0x7f) << 8;
    if(IS_6855_VERSION_TAG)
        FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output

    for(index = 0; index < 128; index++)
    {
        //llfprintk("DQ TX K %d Delay: 0x%x, 0x%x\r\n", index, dqDelayOut0, dqDelayOut1);

        //Set Timing&interface to desired Speed
        ChangeFCClk(ubIFType, ubClockMode);
        FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                      (ubCycleMode << 8)));

        //WriteCache
        if(ubRawDataK)
        {
            if(ubPipelineK)
                ret = llfWriteCacheRedundantPipeline(ubBankNo);
            else
                ret = llfWriteCacheRedundant(ubBankNo);
            if(ret == ERR_FIO_TIMEOUT)
            {
                return ret;
            }

            //Set Timing&interface to 10MHz
            //enable DQS SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if ( IS_6855_VERSION_TAG )
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    reg_v |= (0xa << 24);//set bit25/27,only enable DQS SMT
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
                }
            }
            ChangeFCClk(ubIFType, FC_PLL_CLK_10M);
            FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
            //ReadCache
            ret_data_compare = llfReadCacheRawDataCompare(ubBankNo);
            //Disable SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if(IS_6855_VERSION_TAG)
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);

                }
            }
        }
        else
        {
            if(ubPipelineK)
                ret = llfWriteCachePipeline(ubBankNo);
            else
                ret = llfWriteCache(ubBankNo);
            if(ret == ERR_FIO_TIMEOUT)
            {
                return ret;
            }

            //Set Timing&interface to 10MHz
            //enable DQS SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if ( IS_6855_VERSION_TAG )
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    reg_v |= (0xa << 24);//set bit25/27,only enable DQS SMT
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
                }
            }
            ChangeFCClk(ubIFType, FC_PLL_CLK_10M);
            FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
            //ReadCache
            ret_data_compare = llfReadCacheAndCompare(ubBankNo);
            //Disable SMT
            if(ubClockMode != FC_PLL_CLK_10M)
            {
                if(IS_6855_VERSION_TAG)
                {
                    U32 reg_v;
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);

                }
            }
        }

        if(ret_data_compare == ERR_FIO_TIMEOUT)
        {
            return ret;
        }

        //Judgement of pass/fail
        if(ret_data_compare == ERR_OK)
        {
            //llfDbgPrintk(ALWAYS_MSG, "DQ TX K index:%d-->pass\r\n", index);
            if(index < ulPhyCfgMinTmp)
                ulPhyCfgMinTmp = index;
            if(index > ulPhyCfgMaxTmp)
                ulPhyCfgMaxTmp = index;
        }
        else
        {
            //llfDbgPrintk(ALWAYS_MSG, "DQ TX K index:%d-->fail\r\n", index);
            if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
            {
                ulPhyCfgMin = ulPhyCfgMinTmp;
                ulPhyCfgMax = ulPhyCfgMaxTmp;
                *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
                *minCfg = ulPhyCfgMin;
                *maxCfg = ulPhyCfgMax;
            }
            ulPhyCfgMinTmp = 0xffffffff;
            ulPhyCfgMaxTmp = 0;
        }

        dqDelayOut0 += 0x01010101;
        dqDelayOut1 += 0x01010101;
        if((dqDelayOut0 & 0x7f) >= 128 || ((dqDelayOut0 >> 8) & 0x7f) >= 128 ||
                ((dqDelayOut0 >> 16) & 0x7f) >= 128 || ((dqDelayOut0 >> 24) & 0x7f) >= 128)
        {
            //llfprintk("DQ TX K get max setting\r\n");
            break;
        }
        if((dqDelayOut1 & 0x7f) >= 128 || ((dqDelayOut1 >> 8) & 0x7f) >= 128 ||
                ((dqDelayOut1 >> 16) & 0x7f) >= 128 || ((dqDelayOut1 >> 24) & 0x7f) >= 128)
        {
            //llfprintk("DQ TX K get max setting\r\n");
            break;
        }
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = dqDelayOut0;
        FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = dqDelayOut1;
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
    }

    if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
    {
        ulPhyCfgMin = ulPhyCfgMinTmp;
        ulPhyCfgMax = ulPhyCfgMaxTmp;
        *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
        *minCfg = ulPhyCfgMin;
        *maxCfg = ulPhyCfgMax;
    }
    if((ulPhyCfgMin == 0xffffffff) || (*ulPhyCfgGap < 1))
    {
        ret = ERR_FLASH_CALIBRATE;
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]bank %d CH%dCE%d DQ fail %x %x-%x\r\n",
                     ubBankNo, ubChNo, ubCeNo, index, ulPhyCfgMin, ulPhyCfgMax);
    }
    else
    {
        ret = ERR_OK;
        ulPhyCfg = ((ulPhyCfgMin + ulPhyCfgMax) >> 1);
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]bank %d CH%dCE%d DQ %d-%d=%d set%d\r\n",
                     ubBankNo, ubChNo, ubCeNo, ulPhyCfgMin, ulPhyCfgMax, *ulPhyCfgGap, ulPhyCfg);
    }

    return ret;
}

// Calibrate read (DQS input delay)
U32 llfCalibrateRxDqsDelayChain(U8 ubBankNo, U8 ubIFType, U8 ubMode, U32 *ulPhyCfgGap, U32 *minCfg,
                                U32 *maxCfg)
{
    U32 ret = ERR_OK;
    U8 ubChNo, ubCeNo;
    U16 index;
    U32 ulPhyCfgMin = 0xffffffff;
    U32 ulPhyCfgMax = 0;
    U32 ulPhyCfgMinTmp = 0xffffffff;
    U32 ulPhyCfgMaxTmp = 0;
    U32 ulPhyCfg = 0;
    U32 ret_data_compare = ERR_OK;
    U32 ulDelayChainMode = 0;
    U8 ubRawDataK = 0;

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;

    *ulPhyCfgGap = 0;
    *minCfg = 0;
    *maxCfg = 0;

    ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);

    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;
    ulDelayChainMode = FR_REG32_X(FR_PHY_DELAY_CTRL) & 0x1;
    //llfprintk("[DB] ulDelayChainMode:%d\r\n", ulDelayChainMode);

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_7TDK_SDR)) //Toshiba TLC flash
    {
        FR_REG32_X(FR_PHY_SDR_CFG) = 0;
    }
    else if((ubIFType == IF_ONFI_SDR) || (ubIFType == IF_TOGGLE_SDR))
    {
        // SDR mode need not to calibrate dqs input
        if(ulDelayChainMode)
        {
            if(IS_6855_VERSION_TAG)
            {
                //before cfg delay chain,need to disable output
                FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
            }
            FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) = 0;
            if(IS_6855_VERSION_TAG)
                FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
        }
        else
        {
            if(IS_6855_VERSION_TAG)
            {
                //before cfg delay chain,need to disable output
                FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
            }
            FR_REG32_X(FR_PHY_DELAY_CFG0) = 0;
            if(IS_6855_VERSION_TAG)
                FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
        }
        ret = ERR_OK;
    }
    else
    {
        for(index = 0; index < 64; index++)
        {
            if((ubIFType == IF_ONFI_SDR) || (ubIFType == IF_TOGGLE_SDR))
            {
                ASSERT(ALWAYS_MSG, 0);
            }
            else// ONFI DDR, ONFI DDR 2, Toggle DDR
            {
                if(IS_6855_VERSION_TAG)
                {
                    //before cfg delay chain,need to disable output
                    FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                }
                FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f);
                FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (index & 0x7f);
                if(IS_6855_VERSION_TAG)
                    FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
            }
            FcBusyWait1ms(1);

            if(ubRawDataK)
            {
                ret_data_compare = llfReadCacheRawDataCompare(ubBankNo);
            }
            else
            {
                ret_data_compare = llfReadCacheAndCompare(ubBankNo);
            }
            if(ret_data_compare == ERR_FIO_TIMEOUT)
            {
                return ERR_FIO_TIMEOUT;
            }
            //llfprintk("index:%d-->%d\r\n", index, (ret_data_compare == ERR_OK)?0:1);

            if(ret_data_compare == ERR_OK)
            {
                llfDbgPrintk(LLF_MSG, "index:%d-->pass\r\n", index);
                if(index < ulPhyCfgMinTmp)
                    ulPhyCfgMinTmp = index;
                if(index > ulPhyCfgMaxTmp)
                    ulPhyCfgMaxTmp = index;
            }
            else
            {

                llfDbgPrintk(LLF_MSG, "index:%d-->fail\r\n", index);
                if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
                {
                    ulPhyCfgMin = ulPhyCfgMinTmp;
                    ulPhyCfgMax = ulPhyCfgMaxTmp;
                    *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
                    *minCfg = ulPhyCfgMin;
                    *maxCfg = ulPhyCfgMax;
                }
                ulPhyCfgMinTmp = 0xffffffff;
                ulPhyCfgMaxTmp = 0;
            }
        }
        if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
        {
            ulPhyCfgMin = ulPhyCfgMinTmp;
            ulPhyCfgMax = ulPhyCfgMaxTmp;
            *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
            *minCfg = ulPhyCfgMin;
            *maxCfg = ulPhyCfgMax;
        }

        if((ulPhyCfgMin == 0xffffffff) || ((ulPhyCfgMax - ulPhyCfgMin + 1) < 1))
        {
            ret = ERR_FLASH_CALIBRATE;
            if(IS_6855_VERSION_TAG)
            {
                //before cfg delay chain,need to disable output
                FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
            }
            FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f);
            if(IS_6855_VERSION_TAG)
                FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output

            llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]bank %d CH%dCE%d DQS fail %d %d-%d\r\n",
                         ubBankNo, ubChNo, ubCeNo, index, ulPhyCfgMin, ulPhyCfgMax);
        }
        else
        {
            ret = ERR_OK;

            if(ubIFType == IF_ONFI_SDR)
            {
                FR_REG32_X(FR_PHY_SDR_CFG) = (index & 0xF);
            }
            else
            {
                ulPhyCfg = ((ulPhyCfgMin + ulPhyCfgMax) >> 1);
                llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]bank %d CH%dCE%d DQS %d-%d=%d set%d\r\n",
                             ubBankNo, ubChNo, ubCeNo, ulPhyCfgMin, ulPhyCfgMax, *ulPhyCfgGap, ulPhyCfg);
                if(IS_6855_VERSION_TAG)
                {
                    //before cfg delay chain,need to disable output
                    FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                }
                FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f);
                FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (ulPhyCfg & 0x7f);
                if(IS_6855_VERSION_TAG)
                    FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
            }
        }
    }

    FcBusyWait1ms(1);

    return ret;
}

// Calibrate read (DQ input delay)
U32 llfCalibrateRxDqDelayChain(U8 ubBankNo, U8 ubIFType, U8 ubMode, U32 *ulPhyCfgGap, U32 *minCfg,
                               U32 *maxCfg)
{
    U32 ret = ERR_OK;
    U8 ubChNo, ubCeNo;
    U16 index;
    U32 ulPhyCfgMin = 0xffffffff;
    U32 ulPhyCfgMax = 0;
    U32 ulPhyCfgMinTmp = 0xffffffff;
    U32 ulPhyCfgMaxTmp = 0;
    U32 ulPhyCfg = 0;
    U32 dqDelayIn0 = 0;
    U32 dqDelayIn1 = 0;
    U32 ret_data_compare = ERR_OK;
    U8 ubRawDataK = 0;

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;

    *ulPhyCfgGap = 0;
    *minCfg = 0;
    *maxCfg = 0;

    ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;

    //DQ Delay K
    dqDelayIn0 = FR_REG32_X(FR_PHY_DELAY_DQI0_CE0 + ubCeNo * 4);
    dqDelayIn1 = FR_REG32_X(FR_PHY_DELAY_DQI1_CE0 + ubCeNo * 4);

    if(IS_6855_VERSION_TAG)
    {
        //before cfg delay chain,need to disable output
        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
    }
    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f);
    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (0 & 0x7f);
    if(IS_6855_VERSION_TAG)
        FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output

    for(index = 0; index < 128; index++)
    {
        //llfDbgPrintk(ALWAYS_MSG, "DQ RX K %d Delay: 0x%x, 0x%x\r\n", index, dqDelayIn0, dqDelayIn1);
        if((ubIFType == IF_ONFI_SDR) || (ubIFType == IF_TOGGLE_SDR))
        {
            ASSERT(ALWAYS_MSG, 0);
        }

        if(ubRawDataK)
        {
            ret_data_compare = llfReadCacheRawDataCompare(ubBankNo);
        }
        else
        {
            ret_data_compare = llfReadCacheAndCompare(ubBankNo);
        }

        if(ret_data_compare == ERR_FIO_TIMEOUT)
        {
            return ERR_FIO_TIMEOUT;
        }

        if(ret_data_compare == ERR_OK)
        {
            llfDbgPrintk(LLF_MSG, "index:%d-->pass\r\n", index);
            if(index < ulPhyCfgMinTmp)
                ulPhyCfgMinTmp = index;
            if(index > ulPhyCfgMaxTmp)
                ulPhyCfgMaxTmp = index;
        }
        else
        {
            llfDbgPrintk(LLF_MSG, "index:%d-->fail\r\n", index);
            if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
            {
                ulPhyCfgMin = ulPhyCfgMinTmp;
                ulPhyCfgMax = ulPhyCfgMaxTmp;
                *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
                *minCfg = ulPhyCfgMin;
                *maxCfg = ulPhyCfgMax;
            }
            ulPhyCfgMinTmp = 0xffffffff;
            ulPhyCfgMaxTmp = 0;
        }

        dqDelayIn0 += 0x01010101;
        dqDelayIn1 += 0x01010101;
        if((dqDelayIn0 & 0x7f) >= 128 || ((dqDelayIn0 >> 8) & 0x7f) >= 128 ||
                ((dqDelayIn0 >> 16) & 0x7f) >= 128 || ((dqDelayIn0 >> 24) & 0x7f) >= 128)
        {
            //llfDbgPrintk(ALWAYS_MSG, "DQ RX K get max setting\r\n");
            break;
        }
        if((dqDelayIn1 & 0x7f) >= 128 || ((dqDelayIn1 >> 8) & 0x7f) >= 128 ||
                ((dqDelayIn1 >> 16) & 0x7f) >= 128 || ((dqDelayIn1 >> 24) & 0x7f) >= 128)
        {
            //llfDbgPrintk(ALWAYS_MSG, "DQ RX K get max setting\r\n");
            break;
        }
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        FR_REG32_X(FR_PHY_DELAY_DQI0_CE0 + ubCeNo * 4) = dqDelayIn0;
        FR_REG32_X(FR_PHY_DELAY_DQI1_CE0 + ubCeNo * 4) = dqDelayIn1;
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
    }
    if((ulPhyCfgMinTmp != 0xffffffff) && ((ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1) > *ulPhyCfgGap))
    {
        ulPhyCfgMin = ulPhyCfgMinTmp;
        ulPhyCfgMax = ulPhyCfgMaxTmp;
        *ulPhyCfgGap = ulPhyCfgMaxTmp - ulPhyCfgMinTmp + 1;
        *minCfg = ulPhyCfgMin;
        *maxCfg = ulPhyCfgMax;
    }

    if((ulPhyCfgMin == 0xffffffff) || ((ulPhyCfgMax - ulPhyCfgMin + 1) < 1))
    {
        ret = ERR_FLASH_CALIBRATE;
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]bank %d CH%dCE%d DQ fail %d %d-%d\r\n",
                     ubBankNo, ubChNo, ubCeNo, index, ulPhyCfgMin, ulPhyCfgMax);
    }
    else
    {
        ret = ERR_OK;

        if(ubIFType == IF_ONFI_SDR)
        {
            FR_REG32_X(FR_PHY_SDR_CFG) = (index & 0xF);
        }
        else
        {
            ulPhyCfg = ((ulPhyCfgMin + ulPhyCfgMax) >> 1);
            llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]bank %d CH%dCE%d DQ %d-%d=%d set%d\r\n",
                         ubBankNo, ubChNo, ubCeNo, ulPhyCfgMin, ulPhyCfgMax, *ulPhyCfgGap, ulPhyCfg);
        }
    }

    return ret;
}


void llfAPBECalibrateTxAuto(U8 ubIFType, U8 ubClkMode)
{
    U8 ubChNo = 0, ubCeNo = 0, ubBankNo = 0, ubBankNum = 0;
    PLLF_UNI_INFO pLLFInfo;
    PVENDOR_CMD_RESPONSE pLLFResponseInfo;
    U8 ubFcMode = ONFI_DDR2_TOGGLE;
    U8 calibrateResult[CH_NUM_MAX * CE_NUM_MAX];
    U16 FcCycleNum[14] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266};
    U32 ret, ulDqsCfgGap = 0, ulDqCfgGap = 0;
    U8 ubCalibrateAll = 0;
    U8 ubFcDQSOdtIndex = 1, ubFcDQOdtIndex = 1;
    U8 ubNandDefaultOcd = 1, ubNandOcdIndex = 1;
    U8 ubFcOcdIndex = 0, ubFcBestOcd = 0, ubFcMaxOcdOption = 4;
    U8 ubNandOdtIndex = 0, ubNandBestOdt = 0, ubNandMaxOdtOption = 1;
    U8 ubPassBankNum = 0;
    U8 passDrivingCount = 0;
    U32 dqDelayOut0 = 0;
    U32 dqDelayOut1 = 0;
    U32 dqsCfgMin = 0, dqsCfgMax = 0;
    U32 dqCfgMin = 0, dqCfgMax = 0;
    U32 phyDelayIndex = 0;
    U32 cfgDqsDelay = 0;
    U32 ulVendor = FLASH_VENDOR(gulFlashVendorNum);
    U32 ulSerialNum = FLASH_SERIAL_NUM(gulFlashVendorNum);
    U8 ubFcOcdList[4] = { 50, 35, 25, 18 };
    U8 ubNandOdtList[5] = { 0xff, 150, 100, 75, 50 };
    U32 reg_v;
    U8 ubTxDQSet = 0;

    if(IS_6855_VERSION_TAG)
        ubTxDQSet = 1;

    pLLFResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
    if (pLLFResponseInfo->res_state == VENDOR_CMD_START)
    {
        pLLFResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pLLFResponseInfo->res_progress = 0;
        pLLFResponseInfo->res_err_code = ERR_OK;
        pLLFResponseInfo->err_msg_num = 0;

        pLLFInfo->ubBankNo = 0;
        pLLFInfo->block = 0;
        pLLFInfo->page = 0;
        pLLFInfo->ulWorkFunc = CALIBRATE_FUNC;
    }

#ifdef RL6643_VA
    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
#else
    Fc_ocd_odt_setting(FC_OCD_DRIVE, FC_ODT_CFG);
#endif

    ubBankNum = UnbalancedGetBankNum();
    llfWriteReadCache10MTest(ubIFType, ubClkMode);
    if(pLLFResponseInfo->res_err_code != ERR_OK)
    {
        return;
    }

    ubCalibrateAll = (gubCalibrateConfig >> 1) & 0x01;
    if(ulVendor == IS_TOSHIBA)
    {
        if(ulSerialNum != IS_8T22_SDR
                && ulSerialNum != IS_9T23_SDR_TOGGLE
                && ulSerialNum != IS_XT23_SDR_TOGGLE_64GB)
        {
            ubNandMaxOdtOption = 5;
        }
        else
        {
            ubNandMaxOdtOption = 1;
        }
    }
    else if(ulVendor == IS_SANDISK)
    {
        if(ulSerialNum != IS_8T22_SDR
                && ulSerialNum != IS_9T23_SDR_TOGGLE
                && ulSerialNum != IS_XT23_SDR_TOGGLE_64GB
                && ulSerialNum != IS_XT24
                && ulSerialNum != IS_XT24_64G
                && ulSerialNum != IS_BICS4P5_256Gb
                && ulSerialNum != IS_BICS4P5_512Gb
                && ulSerialNum != IS_BICS4_QLC
                && ulSerialNum != IS_BICS5_QLC)
        {
            ubNandMaxOdtOption = 5;
        }
        else
        {
            ubNandMaxOdtOption = 1;
        }
    }
    else if(ulVendor == IS_SAMSUNG)
    {
        if(ulSerialNum == IS_SSV4
                || ulSerialNum == IS_SSV4_64G)
        {
            ubNandMaxOdtOption = 1;
        }
        else if(ulSerialNum == IS_SSV2_128G
                || ulSerialNum == IS_SSV6
                || ulSerialNum == IS_SSV6_512Gb
                || ulSerialNum == IS_SSV5
                || ulSerialNum == IS_SSV5_64G
                || ulSerialNum == IS_SSV7_512Gb)
        {
            ubNandMaxOdtOption = 4;
        }
    }
    else if(ulVendor == IS_MICRON
            || ulVendor == IS_INTEL
            || ulVendor == IS_HYNIX
            || ulVendor == IS_YMTC)
    {
        ubNandMaxOdtOption = 5;
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Not ready!!!\r\n");
        ASSERT(ALWAYS_MSG, 0);
    }

    if((ulVendor == IS_SANDISK) && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb)
                                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb)
                                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT)))
    {
        if(!gubNANDODTEnable)
            ubNandMaxOdtOption = 1;
    }

    if((ulVendor == IS_TOSHIBA) && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_64GB)
                                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_128GB)))
    {
        if(!gubNANDODTEnable)
            ubNandMaxOdtOption = 1;
    }
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]FC OCD has %d options\r\n", ubFcMaxOcdOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]NAND ODT has %d options\r\n", ubNandMaxOdtOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]Calibrate all: %d\r\n", ubCalibrateAll);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]Clock mode:%x, %d MHz--------\r\n", ubClkMode,
                 FcCycleNum[ubClkMode]);

    llfChangeInterFace(ubIFType);
    ret = FCTimingModeChange(ubIFType, ubClkMode);
    if(ret != ERR_OK)
    {
        pLLFResponseInfo->err_msg_num++;
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    llfprintk("[AUTO_TX_K]Odt drive diff\r\n");
    FCGetNandODTDiffVrefValue();
    ret = SetWarmUpOdtDrvDiff(gulNandODTDiffVrefValue, gubNandDriv,
                              GETFEATURE_BY_BANK_UC, GETFEATURE_BY_BANK_PHY);
    if(ret != ERR_OK)
    {
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    llfSaveTimingDrivingPerCh(SBLK_ADDR);
    llfLoadTimingFromConfig();
    ubFcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    if(IS_6855_VERSION_TAG)
    {
        reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
        reg_v &= (~(0xf << 24));//clear bit24/25/26/27
        FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
    }
    ChangeFCClk(ubFcMode, ubClkMode);
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | (FcCycleNum[ubClkMode] << 8)));
    fc_diff_setting(ubFcMode, ubClkMode, gubFCDiffEnable);
    FR_G_CFG_REG32_W(FR_ECC_THV, gubCacheErrbitLimit);

    gubFcOcdIndex = 1;
    if((gubCalibrateConfig >> 3) & 0x01)
    {
        ubNandDefaultOcd = gubNandDriv;
        ubFcDQSOdtIndex = 5 - gulFc_dqs_odt;
        ubFcDQOdtIndex = 5 - gulFc_dq_re_odt;

        if(FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
        {
            if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R
                    || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)
            {
                ubNandOcdIndex = ubNandDefaultOcd - 3;
            }
            else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
            {
                ubNandOcdIndex = ubNandDefaultOcd - 2;
            }
            else
            {
                ubNandOcdIndex = 3 - ubNandDefaultOcd;
            }
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
        {
            ubNandOcdIndex = 3 - ubNandDefaultOcd;
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
                || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK
                || FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX
                || FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
        {
            ubNandOcdIndex = (ubNandDefaultOcd - 2) >> 1;
        }
    }
    else
    {
        ubFcDQSOdtIndex = 1;
        ubFcDQOdtIndex = 1;
        ubNandOcdIndex = 1;
    }
    llfSetFcPadRxODT(ubFcDQSOdtIndex, ubFcDQOdtIndex);
    llfSetNandTxOCD(ubNandOcdIndex, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
    for(ubNandOdtIndex = 0; ubNandOdtIndex < ubNandMaxOdtOption; ubNandOdtIndex++)
    {
        if((ubNandOdtIndex == 2) && (((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R) ||
                                      (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)) &&
                                     (FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON ||
                                      FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)))
            continue;
        llfSetNandRxODT(ubNandOdtIndex, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);

        for(ubFcOcdIndex = 0; ubFcOcdIndex < ubFcMaxOcdOption; ubFcOcdIndex++)
        {
            if(ubNandOdtIndex == 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "==> [AUTO_TX_K] NAND ODT=%d(dis), FC OCD=%d(%dohm) <==\r\n",
                             ubNandOdtIndex, ubFcOcdIndex, ubFcOcdList[ubFcOcdIndex]);
            }
            else
            {
                llfDbgPrintk(ALWAYS_MSG, "==> [AUTO_TX_K] NAND ODT=%d(%dohm), FC OCD=%d(%dohm) <==\r\n",
                             ubNandOdtIndex, ubNandOdtList[ubNandOdtIndex], ubFcOcdIndex, ubFcOcdList[ubFcOcdIndex]);
            }
            llfSetFcPadTxOCD(ubFcOcdIndex);

            ubPassBankNum = 0;
            for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
            {
                ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
                FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;
                FR_REG32_X(FR_PHY_DELAY_CTRL) = 0x1;//calibrate by ce
                if(IS_6855_VERSION_TAG)
                {
                    //before cfg delay chain,need to disable output
                    FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                }
                FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = 0;//temp need to clear to 0
                FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = 0;
                dqDelayOut0 = FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4);//reserved for DQ equal
                dqDelayOut1 = FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4);
                if(IS_6855_VERSION_TAG)
                    FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
                calibrateResult[ubBankNo] = 0;
                //FcDQEqualSetting();
                ret = llfCalibrateTxDqsDelayChain(ubBankNo, ubFcMode, ubClkMode, FcCycleNum[ubClkMode],
                                                  &ulDqsCfgGap, &dqsCfgMin, &dqsCfgMax);
                if(ret == ERR_FIO_TIMEOUT || ulDqsCfgGap == 0)
                {
                    llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                    llfSetFcPadTxOCD(ubFcOcdIndex);
                    llfSetFcPadRxODT(ubFcDQSOdtIndex, ubFcDQOdtIndex);
                    break;
                }

                ret = llfCalibrateTxDqDelayChain(ubBankNo, ubFcMode, ubClkMode, FcCycleNum[ubClkMode], &ulDqCfgGap,
                                                 &dqCfgMin, &dqCfgMax);
                if(ret == ERR_FIO_TIMEOUT || ulDqCfgGap == 0)
                {
                    llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                    llfSetFcPadTxOCD(ubFcOcdIndex);
                    llfSetFcPadRxODT(ubFcDQSOdtIndex, ubFcDQOdtIndex);
                    //break;
                }

                if(dqsCfgMin != 0 && dqCfgMin != 0)
                {
                    llfprintk("Invalid delay phase\r\n");
                    break;
                }
                if(dqsCfgMin == 0 && dqCfgMin == 0)
                {
                    if(ulDqsCfgGap + ulDqCfgGap >= AUTO_CALIBRATE_TX_GAP)
                    {
                        ubPassBankNum++;
                        calibrateResult[ubBankNo] = 1;
                        if(ulDqsCfgGap >= ulDqCfgGap)
                        {
                            cfgDqsDelay = 1;
                            phyDelayIndex = (ulDqsCfgGap - ulDqCfgGap) / 2;
                        }
                        else
                        {
                            cfgDqsDelay = 0;
                            phyDelayIndex = (ulDqCfgGap -  ulDqsCfgGap) / 2;
                            phyDelayIndex = (phyDelayIndex << 24) + (phyDelayIndex << 16) +
                                            (phyDelayIndex << 8) + phyDelayIndex;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else if(dqsCfgMin > 0 && dqCfgMin == 0)
                {
                    if(ulDqsCfgGap >= AUTO_CALIBRATE_TX_GAP)
                    {
                        ubPassBankNum++;
                        calibrateResult[ubBankNo] = 1;
                        cfgDqsDelay = 1;
                        phyDelayIndex = (ulDqsCfgGap - ulDqCfgGap) / 2 + dqsCfgMin;
                    }
                    else
                    {
                        if(ulDqCfgGap >= AUTO_CALIBRATE_TX_GAP * 4 / 5)
                        {
                            continue;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else if(dqsCfgMin == 0 && dqCfgMin > 0)
                {
                    if(ulDqCfgGap >= AUTO_CALIBRATE_TX_GAP)
                    {
                        ubPassBankNum++;
                        calibrateResult[ubBankNo] = 1;
                        cfgDqsDelay = 1;
                        phyDelayIndex = (ulDqCfgGap - ulDqsCfgGap) / 2 + dqCfgMin;
                    }
                    else
                    {
                        if(ulDqCfgGap >= AUTO_CALIBRATE_TX_GAP * 4 / 5)
                        {
                            continue;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    ASSERT(ALWAYS, 0);
                }
                llfprintk("bank %d CH%dCE%d CfgDqs: %d, PhyDelayIndex: 0x%x\r\n",
                          ubBankNo, ubChNo, ubCeNo, cfgDqsDelay, phyDelayIndex);
                FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;
                if(cfgDqsDelay)
                {
                    if(IS_6855_VERSION_TAG)
                    {
                        //before cfg delay chain,need to disable output
                        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                    }
                    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8);
                    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (phyDelayIndex & 0x7f) << 8;
                    FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = dqDelayOut0;
                    FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = dqDelayOut1;
                    if(IS_6855_VERSION_TAG)
                        FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
                }
                else
                {
                    if(IS_6855_VERSION_TAG)
                    {
                        //before cfg delay chain,need to disable output
                        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                    }
                    //temp set DQ & DQS delay to 0
                    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8);
                    FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (0 & 0x7f) << 8;
                    FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = dqDelayOut0 + (phyDelayIndex * ubTxDQSet);
                    FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = dqDelayOut1 + (phyDelayIndex * ubTxDQSet);
                    if(IS_6855_VERSION_TAG)
                        FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
                }
            }

            if(ubPassBankNum == ubBankNum)
            {
                if(passDrivingCount == AUTO_ENHANCE_DRIVING)
                {
                    ubNandBestOdt = ubNandOdtIndex;
                    ubFcBestOcd = ubFcOcdIndex;
                    llfSaveTimingDrivingPerCh(SBLK_ADDR);
                }
                passDrivingCount++;
                if(!ubCalibrateAll && passDrivingCount > AUTO_ENHANCE_DRIVING)
                {
                    break;
                }
            }
        }
        if(!ubCalibrateAll && passDrivingCount > AUTO_ENHANCE_DRIVING)
        {
            break;
        }
    }

    //Auto-K error handle
    if(passDrivingCount < (AUTO_ENHANCE_DRIVING + 1))
    {
        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            if(calibrateResult[ubBankNo] == 0)
            {
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
            }
        }
    }
    if(pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]Failed for TX with %dMhz\r\n", FcCycleNum[ubClkMode]);
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    if(passDrivingCount >= (AUTO_ENHANCE_DRIVING + 1))
    {
        llfSetFcPadTxOCD(ubFcBestOcd);
        llfSetNandRxODT(ubNandBestOdt, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
        llfSetTimingDrivingPerCh(SBLK_ADDR);
    }

    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]FC OCD: %d (0:50ohm 1:35ohm 2:25ohm 3:18ohm)\r\n",
                 ubFcBestOcd);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_TX_K]NAND ODT: %d (0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm)\r\n",
                 ubNandBestOdt);
    llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] FC_OCD: Config_0x840 = 0xc%d, FC_OCD_Set_for_Reg = %d\r\n",
              3 - ubFcBestOcd, 3 - ubFcBestOcd);
    if(ubNandBestOdt)
    {
        llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] NAND_ODT_EN: Config_0x7dd = 0x1, NAND_ODT: Config_0x838 = 0x%d, NAND_ODT_Set_base_SPEC = %d, \r\n",
                  ubNandBestOdt, ubNandBestOdt);
    }
    else
    {
        llfprintk("[-->>> AUTO_TX_K_FINAL_SET <<<--] NAND_ODT_EN: Config_0x7dd = 0x0, NAND_ODT: Config_0x838 = 0x0\r\n");
    }

    if(!((gubCalibrateConfig >> 3) & 0x01))
    {
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH) = 3 - ubFcBestOcd;
        gulFc_ocd = 3 - ubFcBestOcd;
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_CFG) = ubNandBestOdt;
        gubNANDODTCfg = ubNandBestOdt;
        if(ubNandBestOdt > 0)
        {
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN) = 1;
            gubNANDODTEnable = 1;
        }
    }

    pLLFInfo->ubData0 = AUTO_CALIBRATE_RX_START;
}

void llfAPBECalibrateRxAuto(U8 ubIFType, U8 ubClkMode)
{
    U8 ubChNo = 0, ubCeNo = 0, ubBankNo = 0, ubBankNum = 0;
    PLLF_UNI_INFO pLLFInfo;
    PVENDOR_CMD_RESPONSE pLLFResponseInfo;
    PVENDOR_CMD pVendorCmd;
    entry_t entry = 0;
    U8 calibrateResult[CH_NUM_MAX * CE_NUM_MAX];
    U16 FcCycleNum[14] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266};
    U32 ret = ERR_OK, ulDqsCfgGap = 0;
    U8 ubCalibrateAll = 0;
    U8 ubFcDqsOdtIndex = 0, ubFcDqOdtIndex = 0, ubFcDqsBestOdt = 0, ubFcDqBestOdt = 0,
#if defined(RL6531_VB)
       ubFcMaxOdtOption = 5;
#elif defined(RL6643_VA) || defined(RL6643_FPGA)
       ubFcMaxOdtOption = 6;
#endif
    U8 ubNandOcdIndex = 0, ubNandBestOcd = 0, ubNandMaxOcdOption = 1;
    U8 ubPassBankNum = 0;
    U8 passDrivingCount = 0;
    U32 dqsCfgMin = 0, dqsCfgMax = 0;
    U32 phyDelayIndex = 0;
    U32 cfgDqsDelay = 0;
    U8 ubFcMode;
    U32 ulVendor = FLASH_VENDOR(gulFlashVendorNum);
    U32 ulSerialNum = FLASH_SERIAL_NUM(gulFlashVendorNum);
    U8 ubNandOdtCfg = 0, ubFcOcdCfg = 0, ubNandOdtEnable = 0;
    U8 ubRawDataK = 0;
    U8 ubPipelineK = 0;

    U8 ubNandOcdList[3] = { 50, 35, 25 };
    U8 ubFcOdtList[6] = { 0xff, 150, 100, 75, 50, 35 };

    ubRawDataK = (gubCalibrateConfig >> 5) & 0x01;
    ubPipelineK = (gubCalibrateConfig >> 6) & 0x01;

    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
    pLLFResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

    ubCalibrateAll = (gubCalibrateConfig >> 1) & 0x01;
    if(ulVendor == IS_TOSHIBA
            || ulVendor == IS_MICRON
            || ulVendor == IS_INTEL
            || ulVendor == IS_SANDISK
            || ulVendor == IS_HYNIX)
    {
        ubNandMaxOcdOption = 3;
        if((ulVendor == IS_MICRON || ulVendor == IS_INTEL)
                && (ulSerialNum == IS_B27B || ulSerialNum == IS_B37R || ulSerialNum == IS_B36R ||
                    ulSerialNum == IS_B47R || ulSerialNum == IS_N28 || ulSerialNum == IS_N38A ||
                    ulSerialNum == IS_B58R || ulSerialNum == IS_N38B || ulSerialNum == IS_Q5171A))
        {
            ubNandMaxOcdOption = 2;
        }
    }
    else if(ulVendor == IS_SAMSUNG)
    {
        if(ulSerialNum == IS_SSV4
                || ulSerialNum == IS_SSV4_64G)
        {
            ubNandMaxOcdOption = 2;
        }
        else if(ulSerialNum == IS_SSV2_128G
                || ulSerialNum == IS_SSV5
                || ulSerialNum == IS_SSV5_64G
                || ulSerialNum == IS_SSV6
                || ulSerialNum == IS_SSV6_512Gb
                || ulSerialNum == IS_SSV7_512Gb)
        {
            ubNandMaxOcdOption = 3;
        }
    }
    else if(ulVendor == IS_YMTC)
    {
        ubNandMaxOcdOption = 3;
        if(ulSerialNum == IS_YX2T || ulSerialNum == IS_YX2Q || ulSerialNum == IS_YX3T_WYS
                || ulSerialNum == IS_YX3T_WDS || !vdd_1v8_en)
        {
            ubNandMaxOcdOption = 2;
        }
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Not ready!!!\r\n");
        ASSERT(ALWAYS_MSG, 0);
    }
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]NAND OCD has %d options\r\n", ubNandMaxOcdOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]FC ODT has %d options\r\n", ubFcMaxOdtOption);
    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Calibrate all: %d\r\n", ubCalibrateAll);


    llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Clock mode:%x, %d MHz--------\r\n", ubClkMode,
                 FcCycleNum[ubClkMode]);
    ubFcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    ChangeFCClk(ubFcMode, ubClkMode);
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | (FcCycleNum[ubClkMode] << 8)));
    //Set default rx delay phase
    if(ubClkMode > FC_PLL_CLK_100M)
    {
        for(ubChNo = 0; ubChNo < CE_NUM_MAX; ubChNo++)
        {
            if(IS_6855_VERSION_TAG)
            {
                //before cfg delay chain,need to disable output
                FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                FR_G_CTRL_REG32(FR_PHY_DELAY_CFG0 + ubChNo * 4) &= ~(0x7F); // Clean first
                FR_G_CTRL_REG32(FR_PHY_DELAY_CFG0 + ubChNo * 4) |= 20;
                FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
            }
            else
            {
                FR_G_CTRL_REG32(FR_PHY_DELAY_CFG0 + ubChNo * 4) &= ~(0x7F); // Clean first
                FR_G_CTRL_REG32(FR_PHY_DELAY_CFG0 + ubChNo * 4) |= 30;
            }
        }
    }

    ubBankNum = UnbalancedGetBankNum();
    for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
    {
        if(ubRawDataK)
        {
            if(llfWriteCacheRedundant(ubBankNo) != ERR_OK)
            {
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
            }
        }
        else
        {
            if(llfWriteCache(ubBankNo) != ERR_OK)
            {
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
            }
        }
    }
    if (pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Failed to write with %dMhz\r\n", FcCycleNum[ubClkMode]);
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    if((gubCalibrateConfig >> 3) & 0x01)
    {
        ubFcOcdCfg      = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH);
        ubNandOdtEnable = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN);
        ubNandOdtCfg    = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_CFG);

        if((ubFcOcdCfg & 0xF0) == 0xc0)
        {
            ubFcOcdCfg = ubFcOcdCfg & 0xf;
        }
        if((ubNandOdtEnable & 0xF0) == 0xc0)
        {
            ubNandOdtEnable = ubNandOdtEnable & 0xf;
        }
        if(!ubNandOdtEnable)
        {
            ubNandOdtCfg = 0;
        }
        gubFcOcdIndex = 3 - ubFcOcdCfg;
        gulFc_ocd = ubFcOcdCfg;
        gubNANDODTEnable = ubNandOdtEnable;
        gubNANDODTCfg = ubNandOdtCfg;
        llfprintk("[Set TX driving in RX] Nand: ODTEN %x ODTCfg %x\r\n", ubNandOdtEnable, ubNandOdtCfg);
        llfprintk("[Set TX driving in RX] FC :  OCD %x\r\n", ubFcOcdCfg);

        llfSetFcPadTxOCD(gubFcOcdIndex);
        llfSetNandRxODT(ubNandOdtCfg, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
    }

    for(ubFcDqOdtIndex = 0; ubFcDqOdtIndex < ubFcMaxOdtOption; ubFcDqOdtIndex++)
    {
        for(ubFcDqsOdtIndex = 0; ubFcDqsOdtIndex < ubFcMaxOdtOption; ubFcDqsOdtIndex++)
        {
            if(ubFcDqsOdtIndex < ubFcDqOdtIndex)
                continue;
            llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);

            for(ubNandOcdIndex = 0; ubNandOcdIndex < ubNandMaxOcdOption; ubNandOcdIndex++)
            {
                if(ubFcDqOdtIndex == 0 && ubFcDqsOdtIndex == 0)
                {
                    llfDbgPrintk(ALWAYS_MSG,
                                 "==> [AUTO_RX_K] FC DQS ODT=%d(dis), FC DQ ODT=%d(dis), NAND OCD=%d(%dohm) <==\r\n",
                                 ubFcDqsOdtIndex, ubFcDqOdtIndex, ubNandOcdIndex, ubNandOcdList[ubNandOcdIndex]);
                }
                else if(ubFcDqOdtIndex == 0 && ubFcDqsOdtIndex > 0)
                {
                    llfDbgPrintk(ALWAYS_MSG,
                                 "==> [AUTO_RX_K] FC DQS ODT=%d(%dohm), FC DQ ODT=%d(dis), NAND OCD=%d(%dohm) <==\r\n",
                                 ubFcDqsOdtIndex, ubFcOdtList[ubFcDqsOdtIndex], ubFcDqOdtIndex,
                                 ubNandOcdIndex, ubNandOcdList[ubNandOcdIndex]);
                }
                else
                {
                    llfDbgPrintk(ALWAYS_MSG,
                                 "==> [AUTO_RX_K] FC DQS ODT=%d(%dohm), FC DQ ODT=%d(%dohm), NAND OCD=%d(%dohm) <==\r\n",
                                 ubFcDqsOdtIndex, ubFcOdtList[ubFcDqsOdtIndex], ubFcDqOdtIndex, ubFcOdtList[ubFcDqOdtIndex],
                                 ubNandOcdIndex, ubNandOcdList[ubNandOcdIndex]);
                }

                llfSetNandTxOCD(ubNandOcdIndex, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);

                ubPassBankNum = 0;
                for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
                {
                    ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
                    ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
                    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;
                    FR_REG32_X(FR_PHY_DELAY_CTRL) = 0x1;//calibrate by ce
                    if(IS_6855_VERSION_TAG)
                    {
                        //before cfg delay chain,need to disable output
                        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
                    }
                    FR_REG32_X(FR_PHY_DELAY_DQI0_CE0 + ubCeNo * 4) = 0;
                    FR_REG32_X(FR_PHY_DELAY_DQI1_CE0 + ubCeNo * 4) = 0;
                    if(IS_6855_VERSION_TAG)
                        FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output

                    if(ubPipelineK)
                    {
                        if(ubRawDataK)
                            ret = llfWriteCacheRedundant(ubBankNo);
                        else
                            ret = llfWriteCache(ubBankNo);
                        if(ret != ERR_OK)
                        {
                            llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                            llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);
                            break;
                        }
                    }

                    calibrateResult[ubBankNo] = 0;
                    //FcDQEqualSetting();
                    ret = llfCalibrateRxDqsDelayChain(ubBankNo, ubFcMode, ubClkMode, &ulDqsCfgGap,
                                                      &dqsCfgMin, &dqsCfgMax);
                    if(ret == ERR_FIO_TIMEOUT || ulDqsCfgGap == 0)
                    {
                        llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
                        llfSetFcPadRxODT(ubFcDqsOdtIndex, ubFcDqOdtIndex);
                        break;
                    }
                    else
                    {
                        if(ulDqsCfgGap >= AUTO_CALIBRATE_RX_GAP)
                        {
                            ubPassBankNum++;
                            calibrateResult[ubBankNo] = 1;
                        }
                    }

                    phyDelayIndex = ((dqsCfgMin + dqsCfgMax) >> 1);
                    llfDbgPrintk(ALWAYS_MSG, "bank %d CH%dCE%d CfgDqs: %d, PhyDelayIndex: 0x%x\r\n",
                                 ubBankNo, ubChNo, ubCeNo, cfgDqsDelay, phyDelayIndex);
                }

                if(ubPassBankNum == ubBankNum)
                {
                    if(passDrivingCount == AUTO_ENHANCE_DRIVING)
                    {
                        ret = ERR_OK;
                        ubNandBestOcd = ubNandOcdIndex;
                        ubFcDqsBestOdt = ubFcDqsOdtIndex;
                        ubFcDqBestOdt = ubFcDqOdtIndex;
                        llfSaveTimingDrivingPerCh(SBLK_ADDR);
                    }
                    passDrivingCount++;
                    if(!ubCalibrateAll && passDrivingCount > AUTO_ENHANCE_DRIVING)
                    {
                        break;
                    }
                }
            }

            if(!ubCalibrateAll && passDrivingCount > AUTO_ENHANCE_DRIVING)
            {
                break;
            }
        }
        if(!ubCalibrateAll && passDrivingCount > AUTO_ENHANCE_DRIVING)
        {
            break;
        }
    }
    //Auto-K error handle
    if(passDrivingCount < (AUTO_ENHANCE_DRIVING + 1))
    {
        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            if(calibrateResult[ubBankNo] == 0)
            {
                AddErrorMessage(ubBankNo, 0, ERR_FLASH_CALIBRATE);
            }
        }
    }
    if(pLLFResponseInfo->err_msg_num != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]Failed for RX with %dMhz.\r\n", FcCycleNum[ubClkMode]);
        pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
        pLLFResponseInfo->res_progress = 100;
        pLLFResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
        return;
    }

    if(passDrivingCount >= (AUTO_ENHANCE_DRIVING + 1))
    {
        llfSetFcPadRxODT(ubFcDqsBestOdt, ubFcDqBestOdt);
        llfSetNandTxOCD(ubNandBestOcd, ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
        llfSetTimingDrivingPerCh(SBLK_ADDR);
    }
    if(ulVendor == IS_SAMSUNG)
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]NAND OCD = %d (0:50ohm 1:37.5ohm)\r\n", ubNandBestOcd);
    else
        llfDbgPrintk(ALWAYS_MSG, "[AUTO_RX_K]NAND OCD = %d (0:50ohm 1:35ohm 2:25ohm)\r\n",
                     ubNandBestOcd);
    if(gulFc_dqs_odt_en)
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K]FC DQS ODT = %d (0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n",
                     ubFcDqsBestOdt);
    else
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K]FC DQS ODT = 0 (0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n");
    if(gulFc_dq_re_odt_en)
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K]FC DQRE ODT = %d (0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n",
                     ubFcDqBestOdt);
    else
        llfDbgPrintk(ALWAYS_MSG,
                     "[AUTO_RX_K]FC DQRE ODT = 0 (0:dis 1:150ohm 2:100ohm 3:75ohm 4:50ohm 5:35ohm)\r\n");
    if(ulVendor == IS_MICRON || ulVendor == IS_INTEL)
    {
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R)
        {
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = 3 + ubNandBestOcd;
            gubNandDriv = 3 + ubNandBestOcd;
        }
        else
        {
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = 3 - ubNandBestOcd;
            gubNandDriv = 3 - ubNandBestOcd;
        }
    }
    else if(ulVendor == IS_TOSHIBA
            || ulVendor == IS_SANDISK
            || ulVendor == IS_HYNIX
            || ulVendor == IS_SAMSUNG)
    {
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = (ubNandBestOcd << 1) + 2;
        gubNandDriv = (ubNandBestOcd << 1) + 2;
    }
    else if(ulVendor == IS_YMTC)
    {
        _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR) = 3 - ubNandBestOcd;
        gubNandDriv = 3 - ubNandBestOcd;
    }
    llfSettingPerCh(SBLK_ADDR, 0, ubClkMode);
    llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] NAND_OCD: Config_0x7ed = %d, NAND_OCD_Set_base_SPEC = %d\r\n",
              gubNandDriv, gubNandDriv);
    if(gulFc_dqs_odt_en)
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC_DQS_ODT_EN: Config_0x830 = 0xc0f, Config_0x834 = 0xc%d, FC_DQS_ODT_For_Reg = 0xc%d\r\n",
                  5 - ubFcDqsBestOdt, 5 - ubFcDqsBestOdt);
    else
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC_DQS_ODT Disable: Config_0x830 = 0xc00, Config_0x834 = 0xc4\r\n");
    if(gulFc_dq_re_odt_en)
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC_DQ_ODT_EN:  Config_0x844 = 0xc0f, Config_0x848 = 0xc%d, FC_DQ_ODT_For_Reg = 0xc%d\r\n",
                  5 - ubFcDqBestOdt, 5 - ubFcDqBestOdt);
    else
        llfprintk("[-->>> AUTO_RX_K_FINAL_SET <<<--] FC DQ ODT Disable: Config_0x844 = 0xc00, Config_0x848 = 0xc4\r\n");

    if(ubRawDataK)
    {
        llfResetAndRecoverFC(ubFcMode, ubClkMode, FcCycleNum[ubClkMode]);
        llfSetFcPadTxOCD(gubFcOcdIndex);
        llfSetFcPadRxODT(ubFcDqsBestOdt, ubFcDqBestOdt);
    }

    FR_G_CFG_REG32_W(FR_ECC_THV, 0x50);

    pLLFInfo->ubData0 = AUTO_CALIBRATE_VREF_START;
#ifndef NEW_MUL_WR_CACHE
    entry = pLLFResponseInfo->entry;
    pLLFResponseInfo->res_state = VENDOR_CMD_IDLE;
    pLLFResponseInfo->res_err_code = ret;
    pLLFResponseInfo->res_progress = 100;
    if(pVendorCmd->subcmd == BE_LLF_ALL)
    {
        if(ret != ERR_OK)
        {
            ASSERT(ALWAYS_MSG, 0);
        }
        else
        {
            pLLFResponseInfo->err_msg_num = 0;
            pLLFResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
            pLLFResponseInfo->res_err_code = ret;
            pLLFResponseInfo->res_progress = 30;
            if(gubLLFMode <= LLF_FORCE_INHERIT || (gubLLFMode == LLF_DEFECT_BAD_BLOCK))
            {
                gubLLFALLStep = STEP_LOAD_RDT_START;
                pLLFResponseInfo->res_state = VENDOR_CMD_ERASE;
            }
            else if((gubLLFMode == LLF_FORCE_FORMAT))
            {
                gubLLFALLStep = STEP_FORMAT_INIT;
                pLLFResponseInfo->res_state = VENDOR_CMD_ERASE;
            }
            else
            {
                gubLLFALLStep = STEP_FORMAT_INIT;
                pLLFResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
            }
            _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = 0;
        }
    }
    else if(pVendorCmd->subcmd == BE_LLF_CALIBRATE)
    {
        gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
        spin_lock_irqsave(&g_be2fe_admin_lock, NULL);
        SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
        spin_unlock_irqrestore(&g_be2fe_admin_lock, NULL);
    }
    else
    {
        llfprintk("[AUTO_RX_K]Cmd: 0x%x\r\n", pVendorCmd->subcmd);
    }
#endif
}

U32 llfCalibrateTxDelayChain(U8 ubBankNo, U8 ubFcMode, U8 ubClockMode, U16 uwSpeed)
{
    U8 ubChNo = 0, ubCeNo = 0;
    U32 dqDelayOut0 = 0, dqDelayOut1 = 0;
    U32 dqsCfgMin = 0, dqsCfgMax = 0;
    U32 dqCfgMin = 0, dqCfgMax = 0;
    U32 phyDelayIndex = 0, cfgDqsDelay = 0;
    U32 ret = ERR_OK, ulDqsCfgGap = 0, ulDqCfgGap = 0;

    ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xFF) >> 4;
    ubCeNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + ubBankNo) & 0xF);
    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;
    FR_REG32_X(FR_PHY_DELAY_CTRL) = 0x1;//calibrate by ce
    if(IS_6855_VERSION_TAG)
    {
        //before cfg delay chain,need to disable output
        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
    }
    FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = 0;//temp need to clear to 0
    FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = 0;
    if(IS_6855_VERSION_TAG)
        FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
    dqDelayOut0 = FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4);//reserved for DQ equal
    dqDelayOut1 = FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4);

    //FcDQEqualSetting();
    ret = llfCalibrateTxDqsDelayChain(ubBankNo, ubFcMode, ubClockMode, uwSpeed,
                                      &ulDqsCfgGap, &dqsCfgMin, &dqsCfgMax);

    ret = llfCalibrateTxDqDelayChain(ubBankNo, ubFcMode, ubClockMode, uwSpeed, &ulDqCfgGap,
                                     &dqCfgMin, &dqCfgMax);

    if(dqsCfgMin != 0 && dqCfgMin != 0)
    {
        ret = ERR_FLASH_CALIBRATE;
        return ret;
    }
    if(dqsCfgMin == 0 && dqCfgMin == 0)
    {
        if(ulDqsCfgGap + ulDqCfgGap >= AUTO_CALIBRATE_TX_GAP)
        {
            if(ulDqsCfgGap >= ulDqCfgGap)
            {
                cfgDqsDelay = 1;
                phyDelayIndex = (ulDqsCfgGap - ulDqCfgGap) / 2;
            }
            else
            {
                cfgDqsDelay = 0;
                phyDelayIndex = (ulDqCfgGap -  ulDqsCfgGap) / 2;
                phyDelayIndex = (phyDelayIndex << 24) + (phyDelayIndex << 16) +
                                (phyDelayIndex << 8) + phyDelayIndex;
            }
        }
        else
        {
            ret = ERR_FLASH_CALIBRATE;
            return ret;
        }
    }
    else if(dqsCfgMin > 0 && dqCfgMin == 0)
    {
        if(ulDqsCfgGap >= AUTO_CALIBRATE_TX_GAP)
        {
            cfgDqsDelay = 1;
            phyDelayIndex = (ulDqsCfgGap - ulDqCfgGap) / 2;
        }
        else
        {
            ret = ERR_FLASH_CALIBRATE;
            return ret;
        }
    }
    else if(dqsCfgMin == 0 && dqCfgMin > 0)
    {
        if(ulDqCfgGap >= AUTO_CALIBRATE_TX_GAP)
        {
            cfgDqsDelay = 0;
            phyDelayIndex = (ulDqCfgGap - ulDqsCfgGap) / 2;
            phyDelayIndex = (phyDelayIndex << 24) + (phyDelayIndex << 16) +
                            (phyDelayIndex << 8) + phyDelayIndex;
        }
        else
        {
            ret = ERR_FLASH_CALIBRATE;
            return ret;
        }
    }
    else
    {
        ret = ERR_FLASH_CALIBRATE;
        return ret;
    }
    llfprintk("CH%d CE%d CfgDqs: %d, PhyDelayIndex: 0x%x\r\n", ubChNo, ubCeNo, cfgDqsDelay,
              phyDelayIndex);
    FR_REG_BASE_X = FR_REG_BASE + ubChNo * FR_REG_SIZE;
    if(cfgDqsDelay)
    {
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        //temp set DQ & DQS delay to 0
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8);
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (phyDelayIndex & 0x7f) << 8;
        FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = dqDelayOut0;
        FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = dqDelayOut1;
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
    }
    else
    {
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
        }
        //temp set DQ & DQS delay to 0
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) &= ~(0x7f << 8);
        FR_REG32_X(FR_PHY_DELAY_CFG0 + ubCeNo * 4) |= (0 & 0x7f) << 8;
        FR_REG32_X(FR_PHY_DELAY_DQO0_CE0 + ubCeNo * 4) = dqDelayOut0 + phyDelayIndex;
        FR_REG32_X(FR_PHY_DELAY_DQO1_CE0 + ubCeNo * 4) = dqDelayOut1 + phyDelayIndex;
        if(IS_6855_VERSION_TAG)
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
    }

    //Set Timing&interface to desired Speed
    ChangeFCClk(ubFcMode, ubClockMode);
    FR_G_CFG_REG32_W(FR_PAR_CFG, ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) |
                                  (uwSpeed << 8)));
    return ret;
}

void Vref_10M_calibrate()
{
    U8 bank, ubChNo, TempCalibrateCfg;
    U32 i, ret = ERR_OK;
    U8 ubBankNum;
    ubBankNum = UnbalancedGetBankNum();

    PLLF_UNI_INFO pLLFInfo;

    pLLFInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
    TempCalibrateCfg = gubCalibrateConfig;
    gubCalibrateConfig &= 0xbf;//temp clear pipelinek bit6

    for (i = 0; i < (HEADER_DMA_MAX_LEN); i++)
    {
        _REG32(TEMP_HBUF_ADDR + (i * 4)) = ((i << 24) | (i << 16) | (i << 8) | i);
    }
    // DWB that cache line
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();

    // Program DRAM for Data DMA
    llfProgramAllMixData(TEMP_BUF_ADDR, (NandPara.ubSectorNumPerPage * 512));

    // DWB that cache line
    cache_area_dwbinval(TEMP_BUF_ADDR, 0x4000);
    cache_dummy_update_read();

    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);

    for(bank = 0; bank < ubBankNum; bank++)
    {
        ubChNo = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xFF) >> 4;

        //ret = llfWriteCacheRedundant(bank);
        ret = llfWriteCache(bank);
        if(ret == ERR_FIO_TIMEOUT)
        {
            llfDbgPrintk(ALWAYS_MSG, "[WriteCacheTimeout]bank %d\r\n", bank);
            break;
        }
        for(i = 0; i < 0x71; i++)
        {
            if(i < 0x31)
                FR_REG32_CH(FR_RX_VREF_CFG, ubChNo) = (0x100 | i);//32%~56%
            else
                FR_REG32_CH(FR_RX_VREF_CFG, ubChNo) = (i - 0x31);//56.5%~88%

            //ret = WriteReadFlashCache(bank, 0);

            //ret = llfReadCacheRawDataCompare(bank);
            ret = llfReadCacheAndCompare(bank);

            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "i=%d fail,Vref=%x\r\n", i, FR_REG32_CH(FR_RX_VREF_CFG, ubChNo));
                llfResetAndRecoverFC10M(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M);
            }
            else
            {
                llfDbgPrintk(ALWAYS_MSG, "i=%d pass,Vref%x\r\n", i, FR_REG32_CH(FR_RX_VREF_CFG, ubChNo));
            }
        }
    }
    gubCalibrateConfig = TempCalibrateCfg;
    //recover vref to 50%
    if(IS_6855_VERSION_TAG)
    {
        FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, 0x11e);
    }
    else
    {
        FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, 0x124);
    }
    pLLFInfo->ubData0 = AUTO_CALIBRATE_TX_START;
}

#endif
#endif
#endif

#ifndef NAND_TEST_EN
#if defined(RL6577_VA) || defined(RTS5771_VA) || defined(RTS5771_FPGA) || defined(RL6643_VA) || defined(RL6643_FPGA)
void NandInitBuff(unsigned int addr, unsigned int length, U8 isSetZero)
{
    if (isSetZero)
    {
        memset((void*)addr, 0x0, length);
    }
    cache_area_dwbinval(addr, length);
    cache_dummy_update_read();

}
#endif
#endif

#ifdef NAND_TEST_EN
U32 doAbs(U32 ulA, U32 ulB)
{
    U32 value = (ulA > ulB) ? (ulA - ulB) : (ulB - ulA);
    //printk("%d, %d\r\n", ulA ,ulB);
    return value;
}

#if defined(FTL_B17A) || defined(FTL_B16A)
WL_INFO Micron3DT_B16_WordLine_trans(U32 page)
{
    WL_INFO info;
    U32 tmp_page;

    U32 locat_i;
    U32 locat_j; // 0,1,2 means Low, Up, Extra
    U32 locat_k;
    //U32 locat_h; locat_h = locat_i*3 + locat_j;  // total 12

    if( page < 12 )
    {
        locat_i = page % 4;
        locat_j = 0;
        locat_k = page / 4;
    }
    else if( page < 36 )
    {
        tmp_page = page - 12;

        locat_i = (tmp_page / 2) % 4;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 8) + 3;
    }
    else if( page < 60 )
    {
        tmp_page = page - 36;

        locat_i = (tmp_page  % 4);
        locat_j = 0;
        locat_k = (tmp_page / 4) + 6;
    }
    else if( page < 2220 )
    {
        tmp_page = page - 60;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 12) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 12) + 12;
        }
        else
        {
            locat_i = (tmp_page % 12) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 12) + 6;
        }
    }
    else if( page < 2268 )
    {
        tmp_page = page - 2220;

        locat_i = (tmp_page % 16) / 4;

        locat_j = (tmp_page % 16) % 4;
        if( locat_j == 0 )
        {
            locat_j = 2;
            locat_k = (tmp_page / 16) + 186;
        }
        else if( locat_j == 1 )
        {
            locat_j = 1;
            locat_k = (tmp_page / 16) + 186;
        }
        else if( locat_j == 2 )
        {
            locat_j = 0;
            locat_k = (tmp_page / 16) + 192;
        }
        else // if( locat_j == 3 )
        {
            locat_j = 1;
            locat_k = (tmp_page / 16) + 192;
        }

    }
    else if( page < 2292 )
    {
        tmp_page = page - 2268;

        locat_i = (tmp_page % 8) / 2;
        locat_j = (tmp_page % 2) ? (1) : (2);
        locat_k = (tmp_page / 8) + 189;
    }
    else if( page < 2304 )
    {
        tmp_page = page - 2292;

        locat_i = (tmp_page % 4);
        locat_j = 0;
        locat_k = (tmp_page / 4) + 195;

    }
    else
    {
        ASSERT(FC_MSG, 0);
        locat_i = 0;
        locat_j = 0;
        locat_k = 0;

    }

    info.loc_i = locat_i;
    info.loc_j = locat_j;
    info.loc_k = locat_k;

    return info;

}
U16 Get_B16A_ExtraPage(U16 Page, U32 mapping_addr)
{
    WL_INFO info;
    U16 next_page;
    Micron_3DT_PAGE *table;

    table = (Micron_3DT_PAGE *)mapping_addr;

    if(Page < 36 || Page > 2290)
    {
        printk("[ERR] Page: %d is not TLC Cell.\r\n", Page);
        return 0xffff;
    }

    info = Micron3DT_B16_WordLine_trans(Page);

    if( info.loc_j == Micron3DT_PAGE_IS_LOW || info.loc_j == Micron3DT_PAGE_IS_UP)
    {
        printk("[WARN] Page: %d is not Extra Page\r\n", Page);
        next_page = table->PageMapping[info.loc_k][info.loc_i * 3 + Micron3DT_PAGE_IS_EXTRA];
        return next_page;
    }
    else if( info.loc_j == Micron3DT_PAGE_IS_EXTRA )
    {
        return Page;
    }

    return 0xffff;

}

#endif
#if defined(FTL_N28A) || defined(FTL_N38A)
WL_INFO Micron_3DQ_N28_WordLine_trans(U32 page)
{
    WL_INFO info;
    U32 tmp_page;

    U32 locat_i;
    U32 locat_j;
    U32 locat_k;
    U32 locat_l;

    if (page < 24)
    {
        locat_i = page % 4;
        locat_j = Micron3DQ_PAGE_IS_LOW;
        locat_k = page / 12;
        locat_l = (page % 12) / 4;
    }
    else if(page < 132)
    {
        tmp_page = page - 24;
        locat_i = (tmp_page / 3) % 4;
        if ((tmp_page % 3) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 3) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 36) + 1;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        locat_l = (tmp_page % 36) / 12;
    }
    else if(page < 2292)
    {
        tmp_page = page - 132;
        locat_i = (tmp_page / 4) % 4;
        if ((tmp_page % 4) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 4) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
        }
        else if ((tmp_page % 4) == 2)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }

        locat_k = (tmp_page / 48) + 4;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        else if (locat_j == Micron3DQ_PAGE_IS_TOP)
        {
            locat_k -= 2;
        }
        locat_l = (tmp_page % 48) / 16;
    }
    else if (page < 2340)
    {
        tmp_page = page - 2292;
        locat_i = (tmp_page / 4) % 4;
        if ((tmp_page % 4) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 4) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
        }
        else if ((tmp_page % 4) == 2)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 48) + 49;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        else if (locat_j == Micron3DQ_PAGE_IS_TOP)
        {
            locat_k--;
        }
        locat_l = (tmp_page % 48) / 16;
    }
    else if (page < 2412)
    {
        tmp_page = page - 2340;
        locat_i = (tmp_page / 3) % 4;
        if ((tmp_page % 3) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 3) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 36) + 50;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        locat_l = (tmp_page % 36) / 12;
    }
    else if (page < 2460)
    {
        tmp_page = page - 2412;
        locat_i = (tmp_page / 4) % 4;
        if ((tmp_page % 4) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 4) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
        }
        else if ((tmp_page % 4) == 2)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 48) + 52;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        else if (locat_j == Micron3DQ_PAGE_IS_TOP)
        {
            locat_k -= 2;
        }
        locat_l = (tmp_page % 48) / 16;
    }
    else if (page < 2496)
    {
        tmp_page = page - 2460;
        locat_i = (tmp_page / 3) % 4;
        if ((tmp_page % 3) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 3) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 36) + 53;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        locat_l = (tmp_page % 36) / 12;
    }
    else if (page < 2544)
    {
        tmp_page = page - 2496;
        locat_i = (tmp_page / 4) % 4;
        if ((tmp_page % 4) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 4) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
        }
        else if ((tmp_page % 4) == 2)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 48) + 54;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        else if (locat_j == Micron3DQ_PAGE_IS_TOP)
        {
            locat_k -= 2;
        }
        locat_l = (tmp_page % 48) / 16;
    }
    else if (page < 2580)
    {
        tmp_page = page - 2544;
        locat_i = (tmp_page / 3) % 4;
        if ((tmp_page % 3) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 3) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 36) + 55;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        locat_l = (tmp_page % 36) / 12;
    }
    else if (page < 4548)
    {
        tmp_page = page - 2580;
        locat_i = (tmp_page / 4) % 4;
        if ((tmp_page % 4) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
        }
        else if ((tmp_page % 4) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
        }
        else if ((tmp_page % 4) == 2)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
        }
        locat_k = (tmp_page / 48) + 56;
        if (locat_j == Micron3DQ_PAGE_IS_LOW)
        {
            locat_k++;
        }
        else if (locat_j == Micron3DQ_PAGE_IS_TOP)
        {
            locat_k -= 2;
        }
        locat_l = (tmp_page % 48) / 16;
    }
    else if (page < 4608)
    {
        tmp_page = page - 4548;
        locat_i = (tmp_page / 5) % 4;
        if ((tmp_page % 5) == 0)
        {
            locat_j = Micron3DQ_PAGE_IS_LOW;
            locat_k = 97 + 1;
        }
        else if ((tmp_page % 5) == 1)
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
            locat_k = 97 - 2;
        }
        else if ((tmp_page % 5) == 2)
        {
            locat_j = Micron3DQ_PAGE_IS_EXTRA;
            locat_k = 97;
        }
        else if ((tmp_page % 5) == 3)
        {
            locat_j = Micron3DQ_PAGE_IS_UP;
            locat_k = 97;
        }
        else
        {
            locat_j = Micron3DQ_PAGE_IS_TOP;
            locat_k = 97 - 1;
        }
        locat_l = (tmp_page % 60) / 20;
    }
    else
    {
        locat_i = 0;
        locat_j = 0;
        locat_k = 0;
        locat_l = 0;
    }

    info.loc_i = locat_i;
    info.loc_j = locat_j;
    info.loc_k = locat_k;
    info.loc_l = locat_l;

    return info;
}
U16 GetWordLine_N28APage(U8 to_get_pagetype, U16 Page, U32 mapping_addr)
{
    WL_INFO info;
    Micron_3DQ_N28A_PAGE *table;

    table = (Micron_3DQ_N28A_PAGE *)mapping_addr;

    if(Page < 24 || (Page < 60 && Page > 24 && ((Page % 3) != 0))
            || (Page < 2244 && Page > 2197 && ((Page % 4) > 1))
            || (Page < 2340 && Page > 2293 && ((Page % 4) > 1))
            || (Page < 2412 && Page > 2376 && ((Page % 3) > 0))
            || (Page < 2496 && Page > 2460 && ((Page % 3) > 0))
            || (Page < 2193 && Page > 2147 && ((Page % 4) == 0))
            || (Page < 2289 && Page > 2243 && ((Page % 4) == 0))
            || (Page < 2374 && Page > 2339 && ((Page % 3) == 0))
            || (Page < 2457 && Page > 2411 && ((Page % 4) == 0))
            || (Page < 4545 && Page > 4499 && ((Page % 4) == 0))
            || (Page < 4607 && Page > 4549 && ((Page % 5) < 2)))
    {
        printk("[ERR] Page: %d is not QLC Cell.\r\n", Page);
        return 0xffff;
    }


    info = Micron_3DQ_N28_WordLine_trans(Page);
    return table->PageMapping[info.loc_l][info.loc_k][info.loc_i * 4 + to_get_pagetype];
}

#endif
#ifdef FTL_N18A
WL_INFO Micron_3DQ_N18_WordLine_trans(U32 page)
{
    WL_INFO info;
    U32 tmp_page;

    U32 locat_i;
    U32 locat_j;    //0,1,2,3 means low, up, extra, top
    U32 locat_k;

    if (page < 12)
    {
        locat_i = page % 4;
        locat_j = 0;                            //low
        locat_k = page / 4;
    }
    else if(page < 120)
    {
        tmp_page = page - 12;

        locat_i = (tmp_page / 3) % 4;
        locat_j = (tmp_page % 3);               //low/up/extra
        locat_k = (tmp_page / 12) + 3;
    }
    else if (page < 3048)
    {
        tmp_page = page - 120;

        if ((tmp_page % 4) == 0)
        {
            locat_i = (tmp_page % 16) / 4;
            locat_j = 3;                        //top
            locat_k = (tmp_page / 16) + 6;
        }
        else
        {
            locat_i = (tmp_page % 16) / 4;
            locat_j = (tmp_page % 4) - 1;       //0,1,2
            locat_k = (tmp_page / 16) + 12;
        }
    }
    else if (page < 3072)
    {
        tmp_page = page - 3048;

        if((tmp_page % 2) == 0)
        {
            locat_i = (tmp_page % 8) / 2;
            locat_j = 3;                        //top
            locat_k = (tmp_page / 8) + 189;
        }
        else
        {
            locat_i = (tmp_page % 8) / 2;
            locat_j = 0;                        //low
            locat_k = (tmp_page / 8) + 195;
        }
    }
    else
    {
        ASSERT(FC_MSG, 0);
        locat_i = 0;
        locat_j = 0;
        locat_k = 0;
    }

    info.loc_i = locat_i;
    info.loc_j = locat_j;
    info.loc_k = locat_k;

    return info;
}
U16 Get_N18A_Page(U16 Page, U32 mapping_addr, U8 op)
{
    WL_INFO info;
    Micron_3DQ_N18A_PAGE *table;

    table = (Micron_3DQ_N18A_PAGE *)mapping_addr;

    if(Page < 48 || (Page < 3049 && Page > 3000 && (((Page - 3001) & 3) != 3))
            || (Page < 3071 && Page >= 3049 && Page & 1))
    {
        printk("[ERR] Page: %d is not TLC Cell.\r\n", Page);
        return 0xffff;
    }

    info = Micron_3DQ_N18_WordLine_trans(Page);
    if (op == Micron3DQ_PAGE_IS_EXTRA)
    {
        if(info.loc_j == Micron3DQ_PAGE_IS_LOW || info.loc_j == Micron3DQ_PAGE_IS_UP
                || info.loc_j == Micron3DQ_PAGE_IS_TOP)
        {
            return table->PageMapping[info.loc_k][info.loc_i * 4 + Micron3DQ_PAGE_IS_EXTRA];
        }
        else if(info.loc_j == Micron3DQ_PAGE_IS_EXTRA)
        {
            return Page;
        }
    }
#if 0
    else
    {

        if(info.loc_j == Micron3DQ_PAGE_IS_LOW || info.loc_j == Micron3DQ_PAGE_IS_UP
                || info.loc_j == Micron3DQ_PAGE_IS_EXTRA)
        {
            return table->PageMapping[info.loc_k][info.loc_i * 4 + Micron3DQ_PAGE_IS_TOP];
        }
        else if(info.loc_j == Micron3DQ_PAGE_IS_TOP)
        {
            return Page;
        }
    }
#endif
    return 0xffff;
}

#endif

#ifdef FTL_B27A
WL_INFO Micron3DT_B27A_WordLine_trans(U32 page)
{
    WL_INFO info;
    U32 tmp_page;

    U32 locat_i;
    U32 locat_j; // 0,1,2 means Low, Up, Extra
    U32 locat_k;

    if( page < 18 ) //L
    {
        locat_i = page % 6;
        locat_j = 0;
        locat_k = page / 6;
    }
    else if( page < 54 ) //LU
    {
        tmp_page = page - 18;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 3;
    }
    else if( page < 90 ) //L
    {
        tmp_page = page - 54;

        locat_i = (tmp_page  % 6);
        locat_j = 0;
        locat_k = (tmp_page / 6) + 6;
    }
    else if( page < 2412 ) //EU-L
    {
        tmp_page = page - 90;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 12;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 6;
        }
    }
    else if( page < 2466 ) //EU-L
    {
        tmp_page = page - 2412;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 144;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 135;
        }
    }
    else if( page < 2502 ) //LU
    {
        tmp_page = page - 2466;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 141;
    }
    else if( page < 2538 ) //EU
    {
        tmp_page = page - 2502;

        locat_i = (tmp_page / 2) % 6;
        locat_j = 2 - (tmp_page % 2);
        locat_k = (tmp_page / 12) + 138;
    }
    else if( page < 2574 ) //EU
    {
        tmp_page = page - 2538;

        locat_i = (tmp_page / 2) % 6;
        locat_j = 2 - (tmp_page % 2);
        locat_k = (tmp_page / 12) + 144;
    }
    else if( page < 2592 ) //L
    {
        tmp_page = page - 2574;

        locat_i = (tmp_page % 6);
        locat_j = 0;
        locat_k = (tmp_page / 6) + 150;
    }
    else if( page < 2628 ) //LU
    {
        tmp_page = page - 2592;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 147;
    }
    else if( page < 2646 ) //L
    {
        tmp_page = page - 2628;

        locat_i = (tmp_page % 6);
        locat_j = 0;
        locat_k = (tmp_page / 6) + 156;
    }
    else if( page < 2682 ) //LU
    {
        tmp_page = page - 2646;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 153;
    }
    else if( page < 2736 ) //EU-L
    {
        tmp_page = page - 2682;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 159;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 150;
        }
    }
    else if( page < 5058 ) //EU-L
    {
        tmp_page = page - 2736;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 162;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 156;
        }
    }
    else if( page < 5112 ) //EU-L
    {
        tmp_page = page - 5058;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 294;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 285;
        }
    }
    else if( page < 5148 ) //LU
    {
        tmp_page = page - 5112;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 291;
    }
    else if( page < 5184 ) //EU
    {
        tmp_page = page - 5148;

        locat_i = (tmp_page / 2) % 6;
        locat_j = 2 - (tmp_page % 2);
        locat_k = (tmp_page / 12) + 288;
    }
    else
    {
        ASSERT(FC_MSG, 0);
        locat_i = 0;
        locat_j = 0;
        locat_k = 0;

    }

    info.loc_i = locat_i;
    info.loc_j = locat_j;
    info.loc_k = locat_k;

    return info;
}
U16 Get_B27A_ExtraPage(U16 Page, U32 mapping_addr)
{
    WL_INFO info;
    U16 next_page;
    Micron_3DT_B27A_PAGE *table;

    table = (Micron_3DT_B27A_PAGE *)mapping_addr;

    if ((Page < 54) || ((Page > 2465) && (Page < 2502)) || ((Page > 2591) && (Page < 2628)) ||
            ((Page > 2645) && (Page < 2682)) || ((Page > 5111) && (Page < 5148)) ||
            ((Page > 5059) && (Page < 5112) && (((Page - 5060) % 3) == 0)))
    {
        printk("[ERR] Page: %d is not TLC Cell.\r\n", Page);
        return 0xffff;
    }

    info = Micron3DT_B27A_WordLine_trans(Page);

    if( info.loc_j == Micron3DT_PAGE_IS_LOW || info.loc_j == Micron3DT_PAGE_IS_UP)
    {
        printk("[WARN] Page: %d is not Extra Page\r\n", Page);
        next_page = table->PageMapping[info.loc_k][info.loc_i * 3 + Micron3DT_PAGE_IS_EXTRA];
        return next_page;
    }
    else if( info.loc_j == Micron3DT_PAGE_IS_EXTRA )
    {
        return Page;
    }

    return 0xffff;
}

#endif
#if 0//def FTL_B27B
MICRON_3DT_B27B_WL_INFO Micron3DT_B27B_WordLine_trans(U32 page)
{
    MICRON_3DT_B27B_WL_INFO info;
    U32 tmp_page;

    U32 locat_i;
    U32 locat_j; // 0,1,2 means Low, Up, Extra
    U32 locat_k;

    if( page < 12 ) //L
    {
        locat_i = page % 6;
        locat_j = 0;
        locat_k = page / 6;
    }
    else if( page < 36 ) //LU
    {
        tmp_page = page - 12;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 2;
    }
    else if( page < 60 ) //L
    {
        tmp_page = page - 36;

        locat_i = (tmp_page  % 6);
        locat_j = 0;
        locat_k = (tmp_page / 6) + 4;
    }
    else if( page < 1608 ) //EU-L
    {
        tmp_page = page - 60;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 8;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 4;
        }
    }
    else if( page < 1656 ) //EU-LU
    {
        tmp_page = page - 1608;

        if( (tmp_page % 4) == 2 || (tmp_page % 4) == 3)
        {
            locat_i = (tmp_page % 24) / 4;
            locat_j = (tmp_page % 4) - 2;
            locat_k = (tmp_page / 24) + 94;
        }
        else
        {
            locat_i = (tmp_page % 24) / 4;
            locat_j = 2 - (tmp_page % 4); // 1, 2
            locat_k = (tmp_page / 24) + 90;
        }
    }
    else if( page < 1692 ) //EU-L
    {
        tmp_page = page - 1656;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 96;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 92;
        }
    }
    else if( page < 1740 ) //EU-LU
    {
        tmp_page = page - 1692;

        if( (tmp_page % 4) == 2 || (tmp_page % 4) == 3)
        {
            locat_i = (tmp_page % 24) / 4;
            locat_j = (tmp_page % 4) - 2;
            locat_k = (tmp_page / 24) + 98;
        }
        else
        {
            locat_i = (tmp_page % 24) / 4;
            locat_j = 2 - (tmp_page % 4); // 1, 2
            locat_k = (tmp_page / 24) + 96;
        }
    }
    else if( page < 1752 ) //L
    {
        tmp_page = page - 1740;

        locat_i = (tmp_page % 6);
        locat_j = 0;
        locat_k = (tmp_page / 6) + 100;
    }
    else if( page < 1776 ) //LU
    {
        tmp_page = page - 1752;

        locat_i = (tmp_page / 2) % 6;
        locat_j = (tmp_page % 2);
        locat_k = (tmp_page / 12) + 102;
    }
    else if( page < 1788 ) //L
    {
        tmp_page = page - 1776;

        locat_i = (tmp_page % 6);
        locat_j = 0;
        locat_k = (tmp_page / 6) + 104;
    }
    else if( page < 1824 ) //EU-L
    {
        tmp_page = page - 1788;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 106;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 100;
        }
    }
    else if( page < 3372 ) //EU-L
    {
        tmp_page = page - 1824;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 108;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 104;
        }
    }
    else if( page < 3420 ) //EU-LU
    {
        tmp_page = page - 3372;

        if( (tmp_page % 4) == 2 || (tmp_page % 4) == 3)
        {
            locat_i = (tmp_page % 24) / 4;
            locat_j = (tmp_page % 4) - 2;
            locat_k = (tmp_page / 24) + 194;
        }
        else
        {
            locat_i = (tmp_page % 24) / 4;
            locat_j = 2 - (tmp_page % 4); // 1, 2
            locat_k = (tmp_page / 24) + 190;
        }
    }
    else if( page < 3456 ) //EU-L
    {
        tmp_page = page - 3420;

        if( (tmp_page % 3) == 2)
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 0;
            locat_k = (tmp_page / 18) + 196;
        }
        else
        {
            locat_i = (tmp_page % 18) / 3;
            locat_j = 2 - (tmp_page % 3); // 1, 2
            locat_k = (tmp_page / 18) + 192;
        }
    }
    else
    {
        ASSERT(FC_MSG, 0);
        locat_i = 0;
        locat_j = 0;
        locat_k = 0;

    }

    info.loc_i = locat_i;
    info.loc_j = locat_j;
    info.loc_k = locat_k;

    return info;
}

U16 Get_B27B_ExtraPage(U16 Page, U32 mapping_addr)
{
    MICRON_3DT_B27B_WL_INFO info;
    U16 next_page;
    Micron_3DT_B27B_PAGE *table;

    table = (Micron_3DT_B27B_PAGE *)mapping_addr;


    if ((Page < 36) || ((Page > 1609) && (Page < 1656) && (((Page - 1610) % 4) < 2)) || ((Page > 1693)
            && (Page < 1740) && (((Page - 1694) % 4) < 2)) ||
            ((Page > 1751) && (Page < 1776) && (((Page - 1738) % 4) < 2)) || ((Page > 3373) && (Page < 3420)
                    && (((Page - 3374) % 4) < 2)) ||
            ((Page > 3421) && (Page < 3456) && (((Page - 3437) % 3) == 0)))
    {
        printk("[ERR] Page: %d is not TLC Cell.\r\n", Page);
        return 0xffff;
    }


    info = Micron3DT_B27B_WordLine_trans(Page);

    if( info.loc_j == Micron3DT_PAGE_IS_LOW || info.loc_j == Micron3DT_PAGE_IS_UP)
    {
        printk("[WARN] Page: %d is not Extra Page\r\n", Page);
        next_page = table->PageMapping[info.loc_k][info.loc_i * 3 + Micron3DT_PAGE_IS_EXTRA];
        return next_page;
    }
    else if( info.loc_j == Micron3DT_PAGE_IS_EXTRA )
    {
        return Page;
    }

    return 0xffff;
}

#endif

#ifdef FTL_SSV4
U8 Get_SS_WordLineType(U16 uwPage)
{
    // Page 0x000 - 0x007 edge MLC
    // Page 0x008 - 0x2EF TLC
    // Page 0x2F0 - 0x2F7 edge MLC
    // Page 0x2F8 - 0x2FF edge SLC

    ASSERT(ALWAYS_MSG, uwPage <= 0x2FF)

    if(uwPage >= 0x2F8)
    {
        return SAMSUNG_EDGE_SLC;
    }
    else if((uwPage >= 0x8) && (uwPage <= 0x2EF))
    {
        return SAMSUNG_TLC;
    }
    else
    {
        return SAMSUNG_EDGE_MLC;
    }
}
U16 Get_SSV4_LowerPage(U16 orignalPage)
{
    U16 lowerPage = 0;
    U16 firstTLCPageNum = 0x8, lastTLCPageNum = 0x2EF, offsetPage = 0;
    U8 remainder = 0;

    if(orignalPage < firstTLCPageNum || orignalPage > lastTLCPageNum)
    {
        printk("[ERR] [SSV4] This Page: %d is not TLC Cell.\r\n", orignalPage);
        return 0xffff;
    }

    offsetPage = (orignalPage - firstTLCPageNum);
    remainder = offsetPage % 3; //three pages as a group
    lowerPage = orignalPage - remainder;

    return lowerPage;
}
#endif

#if defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B17A) || defined(FTL_B16A) || defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N38A)
void PageNumFill_IM(U32 mapping_addr)
{


#if defined(FTL_B17A) || defined(FTL_B16A)
    WL_INFO info1;
    Micron_3DT_PAGE *table1;
    U32 h, k, page;
#elif defined(FTL_B27A)
    Micron_3DT_B27A_PAGE *table2;
    WL_INFO info2;
    U32 h, k, page;
#elif defined(FTL_B27B)
    //Micron_3DT_B27B_PAGE *table2;
    //MICRON_3DT_B27B_WL_INFO info2;
    //U32 h, k, page;
#elif defined(FTL_N18A)
    Micron_3DQ_N18A_PAGE *table3;
    WL_INFO info3;
    U32 h, k, page;
#elif defined(FTL_N28A) || defined(FTL_N38A)
    Micron_3DQ_N28A_PAGE *table4;
    WL_INFO info4;
    U32 col, row, frame, page;
#endif
#if defined(FTL_B17A) || defined(FTL_B16A)
    if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B16) ||
            (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B17))
    {
        table1 = (Micron_3DT_PAGE *)mapping_addr;
        for(h = 0; h < 12; h++)
        {
            for(k = 0; k < 6; k++)
                table1->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;

            for(k = 192; k < 198; k++)
                table1->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;
        }

        for(page = 0; page < 2304; page++)
        {
            info1 = Micron3DT_B16_WordLine_trans(page);
            table1->PageMapping[info1.loc_k][info1.loc_i * 3 + info1.loc_j] = page;
        }
    }
#endif


#if 0//def FTL_B27B
    if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B)
    {

        table2 = (Micron_3DT_B27B_PAGE *)mapping_addr;
        for (h = 0; h < 18; h++)
        {
            for(k = 0; k < 4; k++)
                table2->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;

            for(k = 94; k < 104; k++)
                table2->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;

            for(k = 194; k < 198; k++)
                table2->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;
        }

        for(page = 0; page < 3456; page++)
        {
            info2 = Micron3DT_B27B_WordLine_trans(page);
            table2->PageMapping[info2.loc_k][info2.loc_i * 3 + info2.loc_j] = page;
        }
    }

#endif

#ifdef FTL_B27A
    if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27A)
    {

        table2 = (Micron_3DT_B27A_PAGE *)mapping_addr;
        for (h = 0; h < 18; h++)
        {
            for(k = 0; k < 6; k++)
                table2->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;

            for(k = 141; k < 156; k++)
                table2->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;

            for(k = 290; k < 296; k++)
                table2->PageMapping[k][h] = Micron3DT_PAGE_IS_DUMMY;
        }

        for(page = 0; page < 5184; page++)
        {
            info2 = Micron3DT_B27A_WordLine_trans(page);
            table2->PageMapping[info2.loc_k][info2.loc_i * 3 + info2.loc_j] = page;
        }
    }
#endif


#ifdef FTL_N18A
    if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
    {
        table3 = (Micron_3DQ_N18A_PAGE *)mapping_addr;
        for(h = 0; h < 16; h++)
        {
            for(k = 0; k < 6; k++)
            {
                table3->PageMapping[k][h] = Micron3DQ_PAGE_IS_DUMMY;
            }
            for (k = 192; k < 198; k++)
            {
                table3->PageMapping[k][h] = Micron3DQ_PAGE_IS_DUMMY;
            }
        }
        for (page = 0; page < 3072; page++)
        {
            info3 = Micron_3DQ_N18_WordLine_trans(page);
            table3->PageMapping[info3.loc_k][info3.loc_i * 4 + info3.loc_j] = page;
        }

    }
#endif

#if defined(FTL_N28A) || defined(FTL_N38A)
    if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28 ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
    {
        table4 = (Micron_3DQ_N28A_PAGE *)mapping_addr;

        for (frame = 0; frame < 3; frame++)
        {
            for (row = 0; row < 99; row++)
            {
                if (row <= 1 || row >= 97 || row == 47 || row == 49 || row == 51 || row == 53)
                {
                    for (col = 0; col < 16; col++)
                    {
                        table4->PageMapping[frame][row][col] = Micron3DQ_PAGE_IS_DUMMY;
                    }
                }

            }
        }

        for (page = 0; page < 4608; page++)
        {
            info4 = Micron_3DQ_N28_WordLine_trans(page);
            table4->PageMapping[info4.loc_l][info4.loc_k][info4.loc_i * 4 + info4.loc_j] = page;
        }
    }
#endif

}
#endif

#if 0
void memcompare(void * dest, const void *src, size_t count)
{
    char *tmp = (char *) dest, *s = (char *) src;

    while (count--)
    {
        if ((*tmp) != (*s))
        {
            printk("[err] %x %x count%d %x %x\r\n", tmp, s, count, (*tmp), (*s));
            return;
        }
        printk("[test] %x %x count%d %x %x\r\n", tmp, s, count, (*tmp), (*s));
        tmp++;
        s++;
    }
    printk("[test] memcompare ok\r\n");
    return;
}
#endif
void NandTestGenData(U32 addr, U32 len, U32 pattern)
{
    U32 i = 0;
    for(i = 0; i < len; i += 4)
    {
        _REG32(addr + i) = pattern;
    }
    printk("[pattern] %x\r\n", pattern);
    cache_area_dwb(addr, len);
    cache_dummy_update_read();
}

void NandTestGenSeqData(U32 addr, U32 len)
{
    U32 i = 0;
    for(i = 0; i < len; i += 4)
    {
        _REG32(addr + i) = i;
    }
    cache_area_dwb(addr, len);
    cache_dummy_update_read();
}

void NandTestGenRandomData(U32 addr, U32 len)
{
    U32 i = 0;
    for(i = 0; i < len; i += 4)
    {
        U32 tick = gulTicks;
        _REG32(addr + i) = (tick + (i << 16));
    }
    cache_area_dwb(addr, len);
    cache_dummy_update_read();
}

U32 NandTestCalEccCount(U32 sourceAddr, U32 destAddr, U32 len)
{
    U32 dataLen = 2 * 1024;
//    U32 headLen = FC_HEADER_LEN;
//    U32 crcLen = 4;
//    U32 redundantLen = gub_ECC_parity_len_per_2K;
    U32 loopCount = 0;
    U32 i = 0;
    U32 j = 0;
    U32 addr1 = 0;
    U32 addr2 = 0;
    U32 eccCount = 0;
    U8 xor = 0;
    U8 *pSource = NULL;
    U8 *pDest = NULL;
    U8 table[256] =
    {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
    };

    loopCount = (len + gub_Total_len_per_2K - 1) / gub_Total_len_per_2K;
    pSource = (U8*)sourceAddr;
    pDest = (U8*)destAddr;
    printk("[NT] Cal ecc, loop: %d, s: 0x%x, d: 0x%x\r\n", loopCount, pSource, pDest);
    for(i = 0; i < loopCount; i++)
    {
        for(j = 0; j < dataLen; j++)
        {
            addr1 = i * dataLen + j;
            addr2 = i * gub_Total_len_per_2K + j;
            xor = pSource[addr1] ^ pDest[addr2];
            eccCount += table[xor];
        }
    }
    printk("[NT] Ecc count: %d\r\n", eccCount);
    return eccCount;
}

void NandTestHandle(void)
{
    PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    PNandTestCmd pNandTestCmd = NULL;
    entry_t entry = INVALID_ENTRY;
    U32 ret = ERR_OK;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U32 lock_flag;

    pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pRespInfo->res_state = VENDOR_CMD_EXECUTE;
    pRespInfo->over_ecc_thrsh_error_bits = 0;
    pRespInfo->res_err_code = ERR_OK;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    CONFIG_FAST_TIMER(FAST_TIMER_1us_CNT);
    printk("[NT] Command: 0x%x 0x%x\r\n", command, subCommand);
    switch(command)
    {
    case BE_VEN_READ:
        ret = NandTestReadCmd();
        break;
    case BE_VEN_WRITE:
        ret = NandTestWriteCmd();
        break;
    case BE_VEN_NONDAT:
        ret = NandTestNoDataCmd();
        break;
    default:
        printk("[NT] Invalid command\r\n");
        ASSERT(ALWAYS_MSG, 0);
        break;
    }

    pRespInfo->res_state = VENDOR_CMD_IDLE;
    pRespInfo->res_progress = 100; // final progress
    pRespInfo->res_err_code = ret;
    printk("[NT] Return error code: 0x%x\r\n\r\n", pRespInfo->res_err_code);

    entry = pRespInfo->entry;
    gpHostAdminCmd[entry].message_type = MSG_BE_RESP;

    spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
    SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
    spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);

    memset((void*)NAND_TEST_CMD_BUF_VA_ADDR, 0x0, NAND_TEST_CMD_BUF_SIZE);
    cache_area_dwbinval(NAND_TEST_CMD_BUF_VA_ADDR, NAND_TEST_CMD_BUF_SIZE);
    cache_dummy_update_read();
}

U32 NandTestNoDataCmd()
{
    PNandTestCmd pNandTestCmd = NULL;
    U32 ret = ERR_OK;
    U8 command = 0xff;
    U8 subCommand = 0xff;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, command == BE_VEN_NONDAT)
    switch(subCommand)
    {
    case TEST_NAND_ERASE:
        ret = NandErase();
        break;
    case TEST_NAND_RESET:
        ret = NandReset();
        break;
    case TEST_NAND_TEST:
        ret = NandTest();
        break;
    case TEST_NAND_CALIBRATE:
        ret = NandCalibrate();
        break;
    case TEST_NAND_VTH:
        ret = NanddoDistribution();
        break;
    case TEST_FUNCTION:
        ret = NANDTestFunction();
        break;
    default:
        printk("[NT] Invalid command\r\n");
        ASSERT(ALWAYS_MSG, 0);
        break;
    }
    return ret;
}

U32 NandTestReadCmd()
{
    PNandTestCmd pNandTestCmd = NULL;
    U32 ret = ERR_OK;
    U8 command = 0xff;
    U8 subCommand = 0xff;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, command == BE_VEN_READ)

    switch(subCommand)
    {
    case TEST_NAND_READ:
        ret = NandRead();
        break;
    case TEST_NAND_GETFEATURE:
        ret = NandGetFeature();
        break;
    case TEST_NAND_READID:
        ret = NandReadId();
        break;
    case TEST_NAND_READPARA:
        ret = NandReadPara();
        break;
    case TEST_NAND_READUID:
        ret = NandReadUniqueId();
        break;
    default:
        printk("[NT] Invalid command\r\n");
        ASSERT(ALWAYS_MSG, 0);
        break;
    }

    return ret;
}

U32 NandTestWriteCmd()
{
    PNandTestCmd pNandTestCmd = NULL;
    U32 ret = ERR_OK;
    U8 command = 0xff;
    U8 subCommand = 0xff;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, command == BE_VEN_WRITE)
    switch(subCommand)
    {
    case TEST_NAND_WRITE:
        ret = NandWrite();
        break;
    case TEST_NAND_SETFEATURE:
        ret = NandSetFeature();
        break;
    default:
        printk("[NT] Invalid command\r\n");
        ASSERT(ALWAYS_MSG, 0);
        break;
    }
    return ret;
}

#if defined(NAND_TEST_ERASE) || defined(NAND_TEST_NANDTEST)
void EraseMarkByBank(U32 * tmpEbusy, U32 * tmpEFmark, U8 bankNo, U16 blockNo, U8 isAllBank)
{

    U8 endbank, testbank, startbank;
    if (tmpEbusy == ((U32*)0xffffffff))
    {
        return;
    }

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }


    for (testbank = startbank; testbank < endbank; testbank ++)
    {
        tmpEbusy[testbank] =  TIMER_GAP(gulCmpDone[testbank],
                                        gulCmdStart[testbank]) * 10; //* report Erase busy time
        if (gubDisablePrintTag != 0xcd)
        {
            printk("[dbg_erase] b %d %x %x %x %d\r\n", testbank, gulCmdStart[testbank], gulTxDone[testbank],
                   gulCmpDone[testbank], tmpEbusy[testbank]);
        }
        if( gulCmp[testbank] & FLASH_ERR)
        {
            tmpEFmark[testbank] = 1; //* report erase fail
            printk("[EF] bank %d block %d\r\n", testbank, blockNo);
        }
    }

    return;
}

U32 NandTestDispErase(U32 eraseMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 blockCount,
                      U32* tmpEbusy, U32* tmpEFmark, U8 isAllBank)
{
    U32 ret = ERR_OK;
    U8 endbank, testbank, startbank;
    U32 i = 0, failcnt = 0;
    U32 testblock = blockNo;
    PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    U8  ubSLCMode = 0;
    //init
    if(eraseMode == SINGLE_SLC || eraseMode == MULTI_SLC)
    {
        ubSLCMode = BS_SLC_MODE; //slc
    }
    else
    {
        ubSLCMode = 0;
    }

    if(eraseMode == MULTI_SLC || eraseMode == MULTI_TLC)
    {
        blockNo &= ~((U16)NandPara.ubPlaneNumPerLun - 1);
    }

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }

    for(i = 0; i < blockCount; i++)
    {
        //Disp

        for (testbank = startbank; testbank < endbank; testbank ++)
        {
            gul_FW_TAG = llfBETagSetting(TAG_ERASE, testbank);

            if(eraseMode == SINGLE_SLC || eraseMode == SINGLE_TLC)
            {
                testblock = blockNo + i;
                FCSingleErase(fcMode, testbank, 0, testblock, 0, ubSLCMode);

            }
            else if(eraseMode == MULTI_SLC || eraseMode == MULTI_TLC)
            {
                if(NandPara.ubPlaneNumPerLun == 1)
                {
                    testblock = blockNo + i;
                    FCSingleErase(fcMode, testbank, 0, testblock, 0, ubSLCMode);
                }
                else if(NandPara.ubPlaneNumPerLun == 2)
                {
                    testblock = blockNo + i * NandPara.ubPlaneNumPerLun;
                    FCMultiErase(fcMode, testbank, 0, testblock, 0, ubSLCMode);
                }
                else if(NandPara.ubPlaneNumPerLun == 4)
                {
                    testblock = blockNo + i * NandPara.ubPlaneNumPerLun;
                    FCQuadErase(fcMode, testbank, 0, testblock, 0, ubSLCMode);
                }

            }
            ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);
        }
        //cmp
        ret = NandTestPollCmp(isAllBank);

        //Write result

        EraseMarkByBank(tmpEbusy, tmpEFmark, bankNo, testblock, isAllBank);

        if (ret != ERR_OK)
        {
            pRespInfo->over_ecc_thrsh_error_bits = 0xffffffff;
            failcnt ++;
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[Err] block: %d\r\n", i);
            }

            //break;
        }
        else
        {
            pRespInfo->over_ecc_thrsh_error_bits = TIMER_GAP(gulCmpDone[bankNo],
                                                   gulCmdStart[bankNo]); //erase time
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[NT] bank %d, cmd start-Done %d us\r\n", startbank, TIMER_GAP(gulCmpDone[startbank],
                        gulCmdStart[startbank]));
            }
        }

    }
    if (failcnt > 0)
    {
        return ERR_ECC;
    }

    return ERR_OK;
}


#endif
U32 NandErase()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_ERASE
    U8 ch;
    U8 ce;
    U8 lun;
    U16 blockNo, blockCnt;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U32 mappingAddr = 0;
    U8 bankNo = 0;
    U8 eraseMode = 0;
    U8 DisableTimingRecording = 0;
    U8 DisablePringLog = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_NONDAT) && (subCommand == TEST_NAND_ERASE));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    blockNo = pNandTestCmd->lbaLowExp;
    blockNo = (blockNo << 8) + pNandTestCmd->lbaHigh;
    blockCnt = pNandTestCmd->sectorExp;
    blockCnt = (blockCnt << 8) + pNandTestCmd->sector;
    eraseMode = (pNandTestCmd->lbaMidExp >> 3) &
                0x03; //0:SINGE_SLC 1:SINGLE_TLC 2:MULTI_SLC 3:MULTI_TLC
    DisableTimingRecording = (pNandTestCmd->lbaMidExp >> 5) & 0x01; //bit5
    DisablePringLog = (pNandTestCmd->lbaMidExp >> 6) & 0x01; //bit6
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }


    printk("[NT] Erase: (ch %d ce %d lun %d bank %d) (blk %d blockcnt %d era_mode %d fc %d) (Dis time %d print %d)\r\n",
           ch,
           ce, lun, bankNo,
           blockNo, blockCnt, eraseMode, fcMode, DisableTimingRecording, DisablePringLog);


    NandTestDispErase(eraseMode, fcMode, bankNo, blockNo, blockCnt, (U32*)0xffffffff,
                      (U32*)0xffffffff, 0);

    gubDisablePrintTag = 0;

#endif
    return ret;
}
#ifdef NAND_TEST_NANDTEST
void FillOutResult1(U32 bankBase1, U8 bankNo, U32* tmpEbusy, U32* tmpEFmark, U32* tmpPFmark,
                    U8 isAllBank)
{
    U8 testCh, testCe, maxTestCePerCh = 8, out1bankSize = 12, endbank, testbank, startbank;
    U32 testOffset, outAddrBase;

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }

    for (testbank = startbank; testbank < endbank; testbank ++)
    {
        testCh = (_MEM08(BANK_IMAPPING_TABLE_ADDR + testbank) & 0xFF) >> 4;
        testCe = (_MEM08(BANK_IMAPPING_TABLE_ADDR + testbank) & 0xF);
        testOffset = testCh * maxTestCePerCh + testCe;
        outAddrBase = bankBase1 + testOffset * out1bankSize;

        _REG32(outAddrBase) = tmpEbusy[testbank];
        _REG32(outAddrBase + 4) = tmpEFmark[testbank];
        _REG32(outAddrBase + 8) = tmpPFmark[testbank];

        if (gubDisablePrintTag != 0xcd)
        {
            printk("[R1] b %d %x %d %d %d %d %d %d \r\n", testbank, outAddrBase, tmpEbusy[testbank],
                   tmpEFmark[testbank], tmpPFmark[testbank], _REG32(outAddrBase), _REG32(outAddrBase + 4),
                   _REG32(outAddrBase + 8));
        }
    }
}
void FillOutResult2(U32 bankBase2, U8 bankNo, U16 uwPageNo, U32* valAddrByPage, U8 isAllBank)
{
    U8 testCh, testCe, maxTestCePerCh = 8;
    U32 testOffset, outAddrBase, out2bankSize = 32768; //8192 page * 4 byte
    U8 endbank, testbank, startbank;

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }

    for (testbank = startbank; testbank < endbank; testbank ++)
    {
        testCh = (_MEM08(BANK_IMAPPING_TABLE_ADDR + testbank) & 0xFF) >> 4;
        testCe = (_MEM08(BANK_IMAPPING_TABLE_ADDR + testbank) & 0xF);
        testOffset = testCh * maxTestCePerCh + testCe;
        outAddrBase = bankBase2 + testOffset * out2bankSize;

        _REG32(outAddrBase + (uwPageNo * 4)) = valAddrByPage[testbank];
        if (gubDisablePrintTag != 0xcd)
        {
            if ((uwPageNo % 6) == 0)
            {
                printk("[R2] b %d %x %d %d %d \r\n", testbank, outAddrBase, uwPageNo, valAddrByPage[testbank],
                       _REG32(outAddrBase + (uwPageNo * 4)));
            }
        }
    }

}

void NandInitBuff(unsigned int addr, unsigned int length, U8 isSetZero)
{
    if (isSetZero)
    {
        memset((void*)addr, 0x0, length);
    }
    cache_area_dwbinval(addr, length);
    cache_dummy_update_read();

}
void NandInitEraseWriteWholeBlock(U32 flashMode, U8 isInterLeave, U32* tmpEbusy, U32* tmpWbusy,
                                  U32* tmpEFmark,
                                  U32* tmpPFmark, U8* eccState, U8* aesState, U8* scrambleState, U16* uwPageNum, U8* isAllBank)
{
    U32 out1AddrBase, i = 0;
    U16 uwSlcPageNum = 0;
    //initial data base
    NandInitBuff(WL_DATA_OUT1_BASE_CACHE, NAND_TEST_OUT1_BUF_SIZE, 1);
    NandInitBuff(WL_DATA_OUT2_BASE_CACHE, NAND_TEST_OUT2_BUF_SIZE, 1);
    if (gubDisablePrintTag != 0xcd)
    {
        printk("[OUT1] %x ~ %x\r\n", WL_DATA_OUT1_BASE_CACHE,
               WL_DATA_OUT1_BASE_CACHE + NAND_TEST_OUT1_BUF_SIZE);
        printk("[OUT2] %x ~ %x\r\n", WL_DATA_OUT2_BASE_CACHE,
               WL_DATA_OUT2_BASE_CACHE + NAND_TEST_OUT2_BUF_SIZE);
    }
    out1AddrBase = WL_DATA_OUT1_BASE_CACHE;
    _REG32(out1AddrBase) = LOCAL_TIMER_1ms_CNT / 1000; //* report cpu clk //MHz as unit

    if (isInterLeave)
    {
        (*isAllBank) = 1;
    }
    for (i = 0; i < NandPara.ubBankNum; i++)
    {
        //init
        tmpEbusy[i] = 0;
        tmpWbusy[i] = 0;
        tmpEFmark[i] = 0;
        tmpPFmark[i] = 0;
    }

    (*eccState) = 1;
    (*aesState) = 1;
    (*scrambleState) = 1;


    if ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
            || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28)
            || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
    {
        uwSlcPageNum = (NandPara.uwPageNumPerBlock / 4);
    }
    else
    {
        uwSlcPageNum = (NandPara.uwPageNumPerBlock / 3);
    }

    (*uwPageNum) =  ((flashMode & 0x1) == 0) ? uwSlcPageNum : (NandPara.uwPageNumPerBlock);

}

void EraseWriteWholeBlock(U32 ulMode, U8 bankNo, U8 ubLunNo,
                          U16 uwBlockNo, U8 flashMode, U8 isInterLeave)
{
    U8 eccState, aesState, scrambleState;
    U16 uwPageNo = 0;
    U16 uwPageNum = (NandPara.uwPageNumPerBlock);
    U32 tmpEbusy[NT_NAX_BANK_NUM], tmpWbusy[NT_NAX_BANK_NUM], tmpEFmark[NT_NAX_BANK_NUM],
        tmpPFmark[NT_NAX_BANK_NUM];
    U8 isAllBank = 0;

    //init
    NandInitEraseWriteWholeBlock(flashMode, isInterLeave, tmpEbusy, tmpWbusy, tmpEFmark, tmpPFmark,
                                 &eccState,
                                 &aesState, &scrambleState, &uwPageNum, &isAllBank);

    //Erase whole block
    NandTestDispErase(flashMode, ulMode, bankNo, uwBlockNo, 1, tmpEbusy, tmpEFmark, isAllBank);


    //Write whole block & Write result 2
    NandTestDispWrite(flashMode, ulMode, bankNo, uwBlockNo, uwPageNo, uwPageNum, eccState, aesState,
                      scrambleState, tmpWbusy, tmpPFmark, isAllBank);
    //Write result 1

    FillOutResult1(WL_DATA_OUT1_BASE_CACHE + 4, bankNo, tmpEbusy, tmpEFmark, tmpPFmark, isAllBank);

    //update cache
    NandInitBuff(WL_DATA_OUT1_BASE_CACHE, NAND_TEST_OUT1_BUF_SIZE, 0);
    NandInitBuff(WL_DATA_OUT2_BASE_CACHE, NAND_TEST_OUT2_BUF_SIZE, 0);


    return;
}
void NandInitReadWholeBlock(U32 flashMode, U8 isInterLeave, U8* eccState, U8* aesState,
                            U8* scrambleState, U16* uwPageNum, U8* isAllBank)
{
    U16 uwSlcPageNum = 0;
    if (gubDisablePrintTag != 0xcd)
    {
        printk("[OUT2] %x ~ %x\r\n", WL_DATA_OUT2_BASE_CACHE,
               WL_DATA_OUT2_BASE_CACHE + NAND_TEST_OUT2_BUF_SIZE);
    }
    //initial data base
    NandInitBuff(WL_DATA_OUT2_BASE_CACHE, NAND_TEST_OUT2_BUF_SIZE, 1);

    if (isInterLeave)
    {
        (*isAllBank) = 1;
    }

    (*eccState) = 1;
    (*aesState) = 1;
    (*scrambleState) = 1;

    if ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
            || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28)
            || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
    {
        uwSlcPageNum = (NandPara.uwPageNumPerBlock / 4);
    }
    else
    {
        uwSlcPageNum = (NandPara.uwPageNumPerBlock / 3);
    }

    (*uwPageNum) =  ((flashMode & 0x1) == 0) ? uwSlcPageNum : (NandPara.uwPageNumPerBlock);

}

void ReadWholeBlock(U32 ulMode, U8 bankNo, U8 ubLunNo,
                    U16 uwBlockNo, U8 flashMode, U8 isInterLeave)
{
    U8 eccState, aesState, scrambleState;
    U8 isAllBank = 0;
    U16 uwPageNo = 0;
    U16 uwPageNum = (NandPara.uwPageNumPerBlock);
    U8 tmpRmarkFlag = 1;

    //init
    NandInitReadWholeBlock(flashMode, isInterLeave, &eccState,
                           &aesState, &scrambleState, &uwPageNum, &isAllBank);

    //Read whole block & Write result
    NandTestDispRead(flashMode, ulMode, bankNo, uwBlockNo, uwPageNo, uwPageNum, eccState, aesState,
                     scrambleState, tmpRmarkFlag, isAllBank);

    //update cache
    NandInitBuff(WL_DATA_OUT2_BASE_CACHE, NAND_TEST_OUT2_BUF_SIZE, 0);

    return;
}

#endif
U32 NandTest()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_NANDTEST
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 bankNo = 0;
    U8 ch = 0xff;
    U8 ce = 0xff;
    U8 lun = 0xff;
    U8 testModeNo = 0;
    U8 flashMode = 0;
    U8 isInterLeave = 0;
    U16 uwBlockNo;
    U32 mappingAddr = 0;
    //U8 DisableTimingRecording = 0;
    U8 DisablePringLog = 0;
    //pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    FcCmdBypass.bits.ecc_bypass = 0;
    FcCmdBypass.bits.aes_bypass = 0;
    FcCmdBypass.bits.scramble_bypass = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    testModeNo = pNandTestCmd->lbaHighExp; //byte0
    flashMode = pNandTestCmd->lbaMidExp & 0x3; //byte1: bit0-1
    isInterLeave = (pNandTestCmd->lbaMidExp >> 2) & 0x01; //byte1:bit2
    DisablePringLog = (pNandTestCmd->lbaMidExp >> 3) & 0x01; //byte1:bit3
    ch = pNandTestCmd->lbaLowExp; //byte2
    ce = pNandTestCmd->lbaHigh; //byte3
    lun = pNandTestCmd->lbaMid; //byte4
    uwBlockNo = pNandTestCmd->sector; //byte6
    uwBlockNo = (uwBlockNo << 8) + pNandTestCmd->lbaLow;
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);

    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }

    ASSERT(ALWAYS_MSG, (command == BE_VEN_NONDAT) && (subCommand == TEST_NAND_TEST));
    printk("[NT] NandTest: (Tm %d Fm %x interleave %d ch %d ce %d lun %d block %d fc %d bank %d\r\n",
           testModeNo, flashMode, isInterLeave, ch, ce, lun, uwBlockNo, fcMode, bankNo);


    switch(testModeNo) //byte0:test mode
    {
    case 0x0: //Mode0: erase write whole block
        EraseWriteWholeBlock(fcMode, bankNo, lun, uwBlockNo, flashMode, isInterLeave);
        break;
#if 0
    case 0x1: //Mode1: erase write read whole block
        break;
    case 0x2: //Mode2: command set test
        break;
    case 0x3: //Mode3: retention test
        break;
    case 0x4: //Mode4: fc reset
        break;
    case 0x5: //Mode5: gen golden data for tool
        break;
#endif
    case 0x6: //Mode6: read write whole block
        ReadWholeBlock(fcMode, bankNo, lun, uwBlockNo, flashMode, isInterLeave);
        break;
    default:
        printk("[NT] Mode%d is not support\r\n", testModeNo);
        break;
    }

    gubDisablePrintTag = 0;
#endif

    return ret;
}
U32 NReset(U8 fcMode, U8 bankNo)
{
    U32 ret = ERR_OK;
    U32 cmp = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0, recdTime, recdTimeEnd;
    U32 val = 500 * 1000; //300 * 1000 * 1000; // 300 ms => ns
    U32 cycletime = (1000 / guwFcClk); //ns as unit

    val = (val / cycletime) * 2; // 1ms / FC CLK        //Freq_clk_bk = Freq_clk_fc / 2
    //printk("val %x guwFcClk %d\r\n", val, guwFcClk);
    FR_G_CFG_REG32_W(FR_PAR_WDT_CFG, val);
    FR_G_CFG_REG32_W(FR_PAR_WDT_CTRL, 0x1);
    FR_G_CFG_REG32_W(FR_SEQ_WDT_CFG, val);
    FR_G_CFG_REG32_W(FR_SEQ_WDT_CTRL, 0x1);


    gul_FW_TAG = llfBETagSetting(TAG_RESET, bankNo);
    FCReset(fcMode, bankNo);
    FcBusyWait1ms(2);

    ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
    if(ret != ERR_OK)
    {
        if (gubDisablePrintTag != 0xcd)
        {
            DbgPrintk(ALWAYS_MSG, "[NT][RESET][ERR] bank %d ret %x errinfo: %x %x\r\n", bankNo, ret, errInfo0,
                      errInfo1);
        }
    }
    else
    {
        gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, bankNo);
        FCStatusPolling(fcMode, bankNo);
        ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTime);
        ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
        if(ret != ERR_OK || ((cmp & BE_COMPLETION_ERROR_MASK) != 0))
        {
            if (gubDisablePrintTag != 0xcd)
            {
                ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTimeEnd);
                DbgPrintk(ALWAYS_MSG, "[NT][RESET][ERR] bank %d ret %x cmp %x testtime %d us\r\n", bankNo, ret, cmp,
                          recdTimeEnd - recdTime);
            }
            ret = ERR_RESET_FLASH;
        }
    }


    FR_G_CFG_REG32_W(FR_PAR_WDT_CTRL, 0);
    FR_G_CFG_REG32_W(FR_SEQ_WDT_CTRL, 0);

    return ret;
}

U32 NandReset()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_RESET
    U8 ch = 0xff;
    U8 ce = 0xff, DisablePringLog;
    U8 lun = 0xff;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 bankNo = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    DisablePringLog = pNandTestCmd->aux & 0x01;
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 mappingAddr = 0;
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    ASSERT(ALWAYS_MSG, (command == BE_VEN_NONDAT) && (subCommand == TEST_NAND_RESET));
    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }
    if (gubDisablePrintTag != 0xcd)
    {
        printk("[NT] NandReset: (ch %d ce %d lun %d fc %d bank %d\r\n",
               ch, ce, lun, fcMode, bankNo);
    }


    ret = NReset(fcMode, bankNo);

    gubDisablePrintTag = 0;
    if (ret != ERR_OK)
    {
        return  ERR_RESET_FLASH;
    }
#endif
    return ret;
}

U32 NandCalibrate()
{
    PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    PNandTestCmd pNandTestCmd = NULL;
    U32 ret = ERR_OK;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 clkMode = FC_PLL_CLK_10M;
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};

    pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_NONDAT) && (subCommand == TEST_NAND_CALIBRATE));

    fcMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
    clkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);
    gubCalibrateConfig = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_AUTO_CALIBRATE_EN);
    printk("[NT] (%d %d) Auto calibrate: %d\r\n", fcMode, clkMode, gubCalibrateConfig);
    guwFcClk = FcCycleNum[clkMode];


    // === New Config for Driving ===
#if 0
    gubFCDiffEnable     = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FC_DIFF_EN);
    gubNANDODTEnable    = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_CFG);
    gubNANDVrefEn       = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_NAND_VREF_EN);
    gubMicronNANDDriv   = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_MICRON_DRISTR);
    gubTSBNANDDriv      = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_TSB_DRISTR);
    fc_ocd              = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH);
    fc_dqs_odt          = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQS_ODT_CFG);
    fc_dq_re_odt        = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQ_RE_ODT_CFG);
    fc_dqs_odt_en       = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQS_ODT_CTRL);
    fc_dq_re_odt_en     = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQ_RE_ODT_CTRL);

    if((gubFCDiffEnable & 0xF0) == 0xc0)
    {
        gubFCDiffEnable = gubFCDiffEnable & 0xf;
    }
    else
    {
        gubFCDiffEnable = 1;
    }

    if((gubNANDODTEnable & 0xF0) == 0xc0)
    {
        gubNANDODTEnable = gubNANDODTEnable & 0xf;
    }
    else
    {
        gubNANDODTEnable = 0;
    }

    if((gubNANDVrefEn & 0xF0) == 0xc0)
    {
        gubNANDVrefEn = gubNANDVrefEn & 0xf;
    }
    else
    {
        gubNANDVrefEn = 1;
    }

    if((gubMicronNANDDriv & 0xF0) == 0xc0)
    {
        gubMicronNANDDriv = gubMicronNANDDriv & 0xf;
    }
    else
    {
        gubMicronNANDDriv = 1;
    }

    if((gubTSBNANDDriv & 0xF0) == 0xc0)
    {
        gubTSBNANDDriv = gubTSBNANDDriv & 0xf;
    }
    else
    {
        gubTSBNANDDriv = 1;
    }

    if((fc_ocd & 0xF0) == 0xc0)
    {
        fc_ocd = fc_ocd & 0xf;
    }
    else
    {
        fc_ocd = 3;
    }
    if((fc_dqs_odt & 0xF0) == 0xc0)
    {
        fc_dqs_odt = fc_dqs_odt & 0xf;
    }
    else
    {
        fc_dqs_odt = 4;
    }
    if((fc_dq_re_odt & 0xF0) == 0xc0)
    {
        fc_dq_re_odt = fc_dq_re_odt & 0xf;
    }
    else
    {
        fc_dq_re_odt = 4;
    }
    if((fc_dqs_odt_en & 0xF00) == 0xc00)
    {
        fc_dqs_odt_en = fc_dqs_odt_en & 0xff;
    }
    else
    {
        fc_dqs_odt_en = 0;
    }

    if((fc_dq_re_odt_en & 0xF00) == 0xc00)
    {
        fc_dq_re_odt_en = fc_dq_re_odt_en & 0xff;
    }
    else
    {
        fc_dq_re_odt_en = 0;
    }



    printk("D %x O %x V %x M %x T %x\r\n",
           gubFCDiffEnable, gubNANDODTEnable, gubNANDVrefEn, gubMicronNANDDriv, gubTSBNANDDriv);

    printk("FC ocd %x dqs_odt_en %x dq_re_odt_en %x dqs_odt %x dq_re_odt %x\r\n",
           fc_ocd, fc_dqs_odt_en, fc_dq_re_odt_en, fc_dqs_odt, fc_dq_re_odt);
#endif


    if(gubCalibrateConfig)
    {
        //llfAPBECalibrateTxAuto(fcMode, clkMode);
        if(pRespInfo->err_msg_num != 0)
        {
            ret = pRespInfo->res_err_code;
            return ret;
        }
        //llfAPBECalibrateRxAuto(fcMode, clkMode);
        if(pRespInfo->err_msg_num != 0)
        {
            ret = pRespInfo->res_err_code;
            return ret;
        }
    }
    else
    {
        printk("[before]guwFcClk %d\r\n", guwFcClk);
        llfAPBECalibrate(fcMode, clkMode);
        printk("[after]guwFcClk %d\r\n", guwFcClk);
        //ASSERT(ALWAYS, 0);
    }
    return ret;
}
#if defined(NAND_TEST_READ) || defined(NAND_TEST_NANDTEST)
void ReadMarkByBank(U8 tmpRmarkFlag, U8 bankNo, U16 pageNo, U8 isAllBank)
{
    if (!tmpRmarkFlag)
    {
        return;
    }

    FillOutResult2(WL_DATA_OUT2_BASE_CACHE, bankNo, pageNo, gulTxDone, isAllBank);
    return;
}

U32 NandTestDispRead_DRAM(U32 readMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo,
                          U32 dataLen, U32 headLen, U8 isAllBank, U8 ubSLCMode)
{
    U32 ret = ERR_OK, tmpRCC_THV = 0xa4;
    U8 endbank, testbank, startbank = bankNo;
    U32 TLC_ECC_Threshold;
    U32 SLC_ECC_Threshold;
    PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    tmpRCC_THV = FR_CONFIG_CH(FR_ECC_THV, gubStartCH);

    if(ubSLCMode)
    {
        SLC_ECC_Threshold  = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_RDT_SLCECC_THRESHOLD);
        if(SLC_ECC_Threshold == 0)
        {
            SLC_ECC_Threshold = 1;
        }
        FR_G_CFG_REG32_W(FR_ECC_THV, SLC_ECC_Threshold);
    }
    else
    {
        TLC_ECC_Threshold  = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_RDT_ECC_THRESHOLD);
        if(SLC_ECC_Threshold == 0)
        {
            SLC_ECC_Threshold = 1;
        }
        FR_G_CFG_REG32_W(FR_ECC_THV, TLC_ECC_Threshold);
    }

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }


    for (testbank = startbank; testbank < endbank; testbank ++)
    {
        gul_FW_TAG = llfBETagSetting(TAG_READ, testbank);

        if(readMode == SINGLE_SLC || readMode == SINGLE_TLC)
        {
            llfFCCmdRead_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                              NAND_TEST_READ_BUF_PHY_ADDR, dataLen,
                              NAND_TEST_READ_HEAD_PHY_ADDR, headLen);
        }

        else if(readMode == MULTI_SLC || readMode == MULTI_TLC)
        {
            if(NandPara.ubPlaneNumPerLun == 1)
            {
                llfFCCmdRead_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                  NAND_TEST_READ_BUF_PHY_ADDR, dataLen,
                                  NAND_TEST_READ_HEAD_PHY_ADDR, headLen);
            }
            else if(NandPara.ubPlaneNumPerLun == 2)
            {
                llfFCCmdMultiRead_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                       NAND_TEST_READ_BUF_PHY_ADDR, dataLen,
                                       NAND_TEST_READ_HEAD_PHY_ADDR, headLen);
            }
            else if(NandPara.ubPlaneNumPerLun == 4)
            {
                llfFCCmdQuadRead_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                      NAND_TEST_READ_BUF_PHY_ADDR, dataLen,
                                      NAND_TEST_READ_HEAD_PHY_ADDR, headLen);
            }
        }
#if 0
        for (k = 1; k < (dataLen / raw_2k_size); k++)
        {
            //memcpy((void*)(curReadBuffVaAddr + raw_2k_size * k), (void*)(curReadBuffVaAddr + gub_Total_len_per_2K * k),  (raw_2k_size));
            printk("[r%x]k %d %x\r\n", curReadBuffVaAddr, k, _REG32(curReadBuffVaAddr + raw_2k_size * k));
        }
#endif
        ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);
    }

    ret = NandTestPollCmp(isAllBank);

    if (ret == ERR_OK)
    {
        pRespInfo->over_ecc_thrsh_error_bits = gulTxDone[bankNo];//record ecc for read
        if (gubDisablePrintTag != 0xcd)
        {
            printk("[NT] bank %d, ECC_ErrBit %d\r\n", startbank, pRespInfo->over_ecc_thrsh_error_bits);
        }
    }
    else
    {
        pRespInfo->over_ecc_thrsh_error_bits = 0xffffffff;
    }

    FR_G_CFG_REG32_W(FR_ECC_THV, tmpRCC_THV);

    return ret;

}
void NandTestReadRedundant(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U16 uwBlock, U16 uwPage,
                           U32 uwReadAddress, U32 uwWriteAddress, U32 uwReadDataLength, U8 ubSLCMode )
{
    Read_Para readPara;
    U32 ulMulNum, datalength;
    U32 ulDcnt = 0;
    FC_Cmd_Bypass bypass;
    ulMulNum = ((uwReadDataLength % 16) != 0) ? ((uwReadDataLength >> 4) + 1) : (uwReadDataLength >> 4);
    uwReadDataLength = ulMulNum * 16;
    datalength = (ulMode != 0) ? (uwReadDataLength >> 1) : (uwReadDataLength);
    bypass.bits.aes_bypass = 1;
    bypass.bits.ecc_bypass = 1;
    bypass.bits.scramble_bypass = 1;
    readPara.bank = ubBankNo;
    readPara.lun = ubLunNo;
    readPara.block = uwBlock;
    readPara.page = uwPage;
    readPara.fc_data_len = datalength - 1;
    if (ubSLCMode)
    {
        g_parser.index.ReadRedundant = g_parser.index.SlcRead;
    }
    else
    {
        g_parser.index.ReadRedundant = g_parser.index.Read;
    }
    ulDcnt += FCCmdSetCtrlDram(&gsHlcReadRedundant.control_bit, &bypass, g_parser.index.ReadRedundant,
                               1, 0);
    ulDcnt += FCCmdSetHead2(&gsHlcReadRedundant.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcReadRedundant.head3, 0);
    ulDcnt += FCCmdSetDramAddr(&gsHlcReadRedundant.data_info, NULL, uwWriteAddress, uwReadDataLength, 0,
                               0);
    ulDcnt += FCCmdSetParaReadRedundant(gsHlcReadRedundant.parameter, ubSLCMode, &readPara);
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcReadRedundant.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcReadRedundant.head3.bits.head_chk_lbn_en = 0;
#endif

    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcReadRedundant.control_bit.AsU32, ulDcnt);
}

U32 NandTestCompareData(U32 ulSrcAddr, U32 ulDstAddr, U32 ulLength)
{
    U32 i;
    U32 ulSrcData, ulDstData;
    U32 ret = ERR_OK;
    U32 ulCount = 0;
    for (i = 0; i < ulLength; i += 4)
    {
        ulSrcData = _REG32(ulSrcAddr + i);
        ulDstData = _REG32(ulDstAddr + i);
        if(ulSrcData != ulDstData)
        {
            llfprintk("i=%d writeData=%x  readData=%x\r\n", i,  ulSrcData, ulDstData);
            ret = ERR_COMPARE_DATA;
            ulCount++;
            break;
        }
    }
    return ret;
}

U32 NandTestDispReadRedundant(U32 readMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo,
                              U32 dataLen, U32 headLen, U8 isAllBank, U8 ubSLCMode)
{
    U8 testbank, startbank, endbank, testPlaneNo = 1, j, k;
    U32 ret = ERR_OK;
    U32 curReadBuffPHYAddr, curReadBuffVaAddr;
    const U16 raw_2k_size = 2048;
    if(readMode == SINGLE_SLC || readMode == SINGLE_TLC)
    {
        testPlaneNo = 1;
    }
    else if(readMode == MULTI_SLC || readMode == MULTI_TLC)
    {
        testPlaneNo = NandPara.ubPlaneNumPerLun;
    }
    else
    {
        //printk("wrong read mode!\r\n");
    }

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }

    for (j = 0; j < testPlaneNo; j++)
    {
        curReadBuffPHYAddr = NAND_TEST_READ_BUF_PHY_ADDR + j * DRAM_DATA_SIZE;
        curReadBuffVaAddr = NAND_TEST_READ_BUF_VA_ADDR + j * DRAM_DATA_SIZE;
        NandInitBuff(NAND_TEST_READ_BUF_VA_ADDR, NAND_TEST_READ_BUF_SIZE, 0);

        for (testbank = startbank; testbank < endbank; testbank ++)
        {
            gul_FW_TAG = llfBETagSetting(TAG_READ, testbank);
            NandTestReadRedundant(fcMode, testbank, 0, blockNo + j, pageNo, 0,
                                  curReadBuffPHYAddr, gub_Total_len_per_2K * 8, ubSLCMode);
            ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);
        }
        ret = NandTestPollCmp(isAllBank);
        if (ret != ERR_OK)
        {
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[Err] bank: %d, blk: %d, page: %d\r\n", testbank, blockNo + j, pageNo);
            }
        }
        ret |= NandTestCompareData( NAND_TEST_WRITE_BUF_VA_ADDR, NAND_TEST_READ_BUF_VA_ADDR,
                                    DRAM_DATA_SIZE);
        if (ret != ERR_OK)
        {
            printk("[Compare Data Err] bank: %d, blk: %d, page: %d, ret: %d\r\n", testbank, blockNo + j, pageNo,
                   ret);
        }

        for (testbank = startbank; testbank < endbank; testbank ++)
        {
            for (k = 1; k < (DRAM_DATA_SIZE / raw_2k_size); k++)
            {
                NandInitBuff(NAND_TEST_READ_BUF_VA_ADDR, NAND_TEST_READ_BUF_SIZE, 0);
                memcpy((void*)(curReadBuffVaAddr + raw_2k_size * k), \
                       (void*)(curReadBuffVaAddr + gub_Total_len_per_2K * k),  (raw_2k_size));
                //printk("p: %d k %d[p%d][r%x] %x\r\n", pageNo, k, j, curReadBuffVaAddr,
                //_REG32(curReadBuffVaAddr + raw_2k_size * k));
            }
        }
    }
    if (testPlaneNo == 1)
    {
        NandInitBuff(NAND_TEST_READ_BUF_VA_ADDR, NAND_TEST_READ_BUF_SIZE, 0);
        NandInitBuff(NAND_TEST_READ_BUF_VA_ADDR + DRAM_DATA_SIZE, NAND_TEST_READ_BUF_SIZE - DRAM_DATA_SIZE,
                     1);
    }

    return ret;
}


U32 NandTestDispRead(U32 readMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo, U16 pageCount,
                     U8 eccState,
                     U8 aesState, U8 scrambleState, U8 tmpRmarkFlag, U8 isAllBank)
{
    U32 ret = ERR_OK, failcnt = 0;
    U32 dataLen = 0;
    U32 headLen = 0;
    U32 i = 0;
    U8  ubSLCMode = 0;
    FcCmdBypass.bits.ecc_bypass = (eccState == 0);
    FcCmdBypass.bits.aes_bypass = (aesState == 0);
    FcCmdBypass.bits.scramble_bypass = (scrambleState == 0);

    //FC_TOP_REG(FR_SEED_AGITATION_EN) = 1;

    if(readMode == SINGLE_SLC || readMode == SINGLE_TLC)
    {
        dataLen = DRAM_DATA_SIZE;
        headLen = DRAM_HEAD_SIZE;
    }
    else if(readMode == MULTI_SLC || readMode == MULTI_TLC)
    {
        dataLen = DRAM_DATA_SIZE * NandPara.ubPlaneNumPerLun;
        headLen = DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun;
        blockNo &= ~((U16)NandPara.ubPlaneNumPerLun - 1);
    }

    if(readMode == SINGLE_SLC || readMode == MULTI_SLC)
    {
        ubSLCMode = BS_SLC_MODE;
        gubNandFlashType = 1;
    }
    else
    {
        ubSLCMode = 0;
        gubNandFlashType = 0;
    }

    for(i = 0; i < pageCount; i++)
    {
        NandInitBuff(NAND_TEST_READ_BUF_VA_ADDR, NAND_TEST_READ_BUF_SIZE, 1);
        if (gubDisablePrintTag != 0xcd)
        {
            printk("[NT] bank: %d, Blk: %d, Page: %d\r\n", bankNo, blockNo, pageNo + i);
        }

        if(eccState)
        {
            ret = NandTestDispRead_DRAM(readMode, fcMode, bankNo, blockNo, pageNo + i, dataLen, headLen,
                                        isAllBank, ubSLCMode);
        }
        else
        {
            ret = NandTestDispReadRedundant(readMode, fcMode, bankNo, blockNo, pageNo + i,
                                            dataLen, headLen, isAllBank, ubSLCMode);
        }
        if (ret != ERR_OK)
        {
            failcnt ++;
        }
        ReadMarkByBank(tmpRmarkFlag, bankNo, pageNo + i, isAllBank);
    }
    NandInitBuff(NAND_TEST_READ_BUF_VA_ADDR, NAND_TEST_READ_BUF_SIZE, 0);

    if (failcnt > 0)
    {
        return ERR_ECC;
    }

    return ERR_OK;
}
#endif

void NandTestChangeLdpc(U16 blockNo, U16 pageNo)
{
#ifndef SBLK_EXPAND
    if((blockNo < SYS_BLK) && (pageNo == 0))
#else
    if((blockNo < gubSblkStart + SYS_BLK) && (pageNo == 0))
#endif
    {
#if defined(RL6643_VA)
        printk("[---> min LDPC <---]blockNo = %d, pageNo=%d\r\n", blockNo, pageNo);
        Change_ldpc(3);
#elif defined(RL6577_VA) || defined(RTS5771_VA)
        Change_ldpc(5);
#endif
    }
    else
    {
        printk("[---> normal LDPC <---]blockNo = %d, pageNo=%d, gubECC_CFG=%d\r\n", blockNo, pageNo,
               gubECC_CFG);
        Change_ldpc(gubECC_CFG);
    }
}

U32 NandRead()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_READ
    U8 eccState;
    U8 aesState;
    U8 scrambleState;
    U8 ch;
    U8 ce;
    U8 lun;
    U16 blockNo;
    U16 pageNo;
    U16 pageCount;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U32 mappingAddr = 0;
    U8 bankNo = 0;
    U8 readMode = 0;
    U8 DisableTimingRecording = 0;
    U8 DisablePringLog = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_READ) && (subCommand == TEST_NAND_READ));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    blockNo = pNandTestCmd->lbaLowExp;
    blockNo = (blockNo << 8) + pNandTestCmd->lbaHigh;
    pageNo = pNandTestCmd->lbaMid;
    pageNo = (pageNo << 8) + pNandTestCmd->lbaLow;
    pageCount = pNandTestCmd->sectorExp;
    pageCount = (pageCount << 8) + pNandTestCmd->sector;
    eccState = (pNandTestCmd->lbaMidExp) & 0x01; //bit0
    aesState = (pNandTestCmd->lbaMidExp >> 1) & 0x01; //bit1
    scrambleState = (pNandTestCmd->lbaMidExp >> 2) & 0x01; //bit2
    readMode = (pNandTestCmd->lbaMidExp >> 3) & 0x03; //bit3-4
    DisableTimingRecording = (pNandTestCmd->lbaMidExp >> 5) & 0x01; //bit5
    DisablePringLog = (pNandTestCmd->lbaMidExp >> 6) & 0x01; //bit6
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }

    printk("[NT] Read: (ch %d ce %d lun %d bank %d blk %d pg %d) pgcnt %d rm %d (ecc %d aes %d scr %d) fc %d (Dis time %d print %d)\r\n",
           ch, ce, lun, bankNo, blockNo, pageNo,
           pageCount, readMode, eccState, aesState, scrambleState, fcMode, DisableTimingRecording,
           DisablePringLog);

    NandTestChangeLdpc(blockNo, pageNo);
    ret = NandTestDispRead(readMode, fcMode, bankNo, blockNo, pageNo, pageCount, eccState, aesState,
                           scrambleState, 0, 0);

    gubDisablePrintTag = 0;
#endif
    return ret;
}

void FcStatePrint()
{
    U8 i;
    for(i = 0 ; i < gubCHNum; i++)
    {
        printk("Parser State ch%d:%x_%x_%x_%x_%x_%x_%x_%x\r\n", i,
               *(U32 *)(0xff100430 + i * 0x4000), *(U32 *)(0xff100434 + i * 0x4000),
               *(U32 *)(0xff100438 + i * 0x4000), *(U32 *)(0xff10043C + i * 0x4000),
               *(U32 *)(0xff100440 + i * 0x4000), *(U32 *)(0xff100444 + i * 0x4000),
               *(U32 *)(0xff100448 + i * 0x4000), *(U32 *)(0xff10044C + i * 0x4000));
    }

    if (gubCHNum == 4)
    {
        printk("Sequencer State ch0 %x ch1 %x ch2 %x ch3 %x\r\n",
               *(U32 *)0xff100688, *(U32 *)0xff104688, *(U32 *)0xff108688, *(U32 *)0xff10C688);
    }
    else
    {
        printk("Sequencer State ch0 %x ch1 %x ch2 %x ch3 %x ch4 %x ch5 %x ch6 %x ch7 %x\r\n",
               *(U32 *)0xff100688, *(U32 *)0xff104688, *(U32 *)0xff108688, *(U32 *)0xff10C688,
               *(U32 *)0xff110688, *(U32 *)0xff114688, *(U32 *)0xff118688, *(U32 *)0xff11C688);
    }

    for(i = 0 ; i < NandPara.ubBankNum; i += 2)
    {
        printk("Bank %d, FC cmd fifo room %d  Bank %d, FC cmd fifo room %d\r\n",
               i, FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + i * FR_CMDQ_CE_SIZE),
               (i + 1), FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + (i + 1) * FR_CMDQ_CE_SIZE));
    }

}
U32 NandReadId()
{
    U32 ret = ERR_OK;

#ifdef NAND_TEST_READID
    U8 ch = 0xff;
    U8 ce = 0xff;
    U8 lun = 0xff;
    //PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 bankNo = 0, DisablePringLog;
    U32 cmp = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0, recdTime, recdTimeEnd;
    U8 *Manufacturer = (U8 *)NAND_TEST_RESP_BUF_VA_ADDR;

    //pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_READ) && (subCommand == TEST_NAND_READID));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    DisablePringLog = pNandTestCmd->aux & 0x01;
    U32 mappingAddr = 0;
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);

    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }
    if (gubDisablePrintTag != 0xcd)
    {
        printk("[NT] ReadID: (ch %d ce %d lun %d fcmode %d bank %d fcclk %d\r\n",
               ch, ce, lun, fcMode, bankNo, guwFcClk);
    }




    NandInitBuff(NAND_TEST_RESP_BUF_VA_ADDR, NAND_TEST_RESP_BUF_SIZE, 1);

    //reset cmd
    ret = NReset(fcMode, bankNo);
    if (ret != ERR_OK)
    {
        return  ERR_RESET_FLASH;
    }


    //read id cmd
    gul_FW_TAG = llfBETagSetting(TAG_READ_ID, bankNo);
    FCReadID(fcMode, bankNo, 0x00, 7, NAND_TEST_RESP_BUF_PHY_ADDR, 0x10);
    FcBusyWait1ms(1);
    ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTime);
    ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
    if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        if (gubDisablePrintTag != 0xcd)
        {
            ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTimeEnd);
            printk("[NT][READID][ERR] Info: 0x%x 0x%x, ret: 0x%x cmp: 0x%x testtime %d us\r\n", ret, errInfo0,
                   errInfo1, ret,
                   cmp, recdTimeEnd - recdTime);
        }
    }

    cache_dwbinval(NAND_TEST_RESP_BUF_VA_ADDR);
    cache_dummy_update_read();
    if((fcMode == ONFI_DDR2_TOGGLE) || (fcMode == ONFI_DDR))
    {
        Manufacturer[0] = Manufacturer[0];
        Manufacturer[1] = Manufacturer[2];
        Manufacturer[2] = Manufacturer[4];
        Manufacturer[3] = Manufacturer[6];
        Manufacturer[4] = Manufacturer[8];
        Manufacturer[5] = Manufacturer[10];
        Manufacturer[6] = Manufacturer[12];
        Manufacturer[7] = Manufacturer[14];
    }
    memset((void*)NAND_TEST_RESP_BUF_VA_ADDR + 8, 0x0, 8);
    gubDisablePrintTag = 0;
    if (ret == ERR_OK)
    {
        printk("[NT] ReadID done %x %x %x %x %x %x %x %x\r\n", Manufacturer[0], Manufacturer[1],
               Manufacturer[2], Manufacturer[3], Manufacturer[4], Manufacturer[5], Manufacturer[6],
               Manufacturer[7]);
    }
    else
    {
        return ERR_READ_ID;
    }

    //pRespInfo->res_err_code = ret;
#endif
    return ret;
}

U32 NandReadPara()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_READPARA
    U8 ch = 0xff;
    U8 ce = 0xff;
    U8 lun = 0xff;
    //PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 bankNo = 0;
    U32 cmp = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0, recdTime, recdTimeEnd;
    U8 Parameter_Address = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_READ) && (subCommand == TEST_NAND_READPARA));
    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    U32 mappingAddr = 0;
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    Parameter_Address = pNandTestCmd->lbaLow ;
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    printk("[NT] ReadPARA: (ch %d ce %d lun %d fcmode %d bank %d fcclk %d\r\n",
           ch, ce, lun, fcMode, bankNo, guwFcClk);

    NandInitBuff(NAND_TEST_RESP_BUF_VA_ADDR, NAND_TEST_RESP_BUF_SIZE, 1);


    gul_FW_TAG = llfBETagSetting(TAG_READ_PARA, bankNo);
    FCReadParams(fcMode, bankNo, Parameter_Address,
                 (fcMode == 0) ? (0xff) : (0x7f),
                 NAND_TEST_RESP_BUF_PHY_ADDR, 0x100);

    FcBusyWait1ms(1);
    ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTime);
    ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
    if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTimeEnd);
        printk("[NT][READID][ERR] Info: 0x%x 0x%x, ret: 0x%x cmp: 0x%x testtime %d us\r\n", ret, errInfo0,
               errInfo1, ret,
               cmp, recdTimeEnd - recdTime);
    }

    cache_dwbinval(NAND_TEST_RESP_BUF_VA_ADDR);
    cache_dummy_update_read();
    memset((void*)NAND_TEST_RESP_BUF_VA_ADDR + 8, 0x0, 8);

    if (ret == ERR_OK)
    {
        printk("[NT] number of data of per page %d\r\n", _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 0x50));
        printk("[NT] number of spare byte of per page %d\r\n", _REG16(NAND_TEST_RESP_BUF_VA_ADDR + 0x54));
        printk("[NT] number of pages of per block %d\r\n", _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 0x5C));
        printk("[NT] number of blocks of per lun %d\r\n", _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 0x60));
        printk("[NT] number of luns of per chips %d\r\n", _REG08(NAND_TEST_RESP_BUF_VA_ADDR + 0x64));
        printk("[NT] number of column address cycle is %d,number of row address cycle is %d\r\n",
               ((_REG08(NAND_TEST_RESP_BUF_VA_ADDR + 0x65)) & 0xf0) >> 4,
               (_REG08(NAND_TEST_RESP_BUF_VA_ADDR + 0x65)) & 0x0f);
        printk("[NT] number of bits per cell %d\r\n", _REG08(NAND_TEST_RESP_BUF_VA_ADDR + 0x66));
    }
    else
    {
        return ERR_READ_ID;
    }

#endif

    return ret;
}
#endif

U32 NandReadAllPara()
{
    U32 ret = ERR_OK;
    U8 fcMode = ONFI_SDR;
    U8 bankNo = 0;
    U32 cmp = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0, recdTime, recdTimeEnd;
    U8 Parameter_Address = 0;

    if (FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA ||
            FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG ||
            FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
    {
        Parameter_Address = 0x40;
    }
    else
    {
        Parameter_Address = 0x0;
    }
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    NandInitBuff(NAND_TEST_RESP_BUF_VA_ADDR, NAND_TEST_RESP_BUF_SIZE, 1);


    gul_FW_TAG = llfBETagSetting(TAG_READ_PARA, bankNo);
    FCReadParams(fcMode, bankNo, Parameter_Address,
                 (fcMode == 0) ? (0xff) : (0x7f),
                 NAND_TEST_RESP_BUF_PHY_ADDR, 0x100);

    FcBusyWait1ms(1);
    ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTime);
    ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
    if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTimeEnd);
        printk("[NT][READID][ERR] Info: 0x%x 0x%x, ret: 0x%x cmp: 0x%x testtime %d us\r\n", ret, errInfo0,
               errInfo1, ret,
               cmp, recdTimeEnd - recdTime);
    }

    cache_dwbinval(NAND_TEST_RESP_BUF_VA_ADDR);
    cache_dummy_update_read();
    memset((void*)NAND_TEST_RESP_BUF_VA_ADDR + 8, 0x0, 8);

    if (ret == ERR_OK)
    {
#if defined(RL6577_VA)
        ParseParaPage();
#else
        U32 i;
        printk("Print Parameter byte 0~511.\r\n");
        for(i = 0; i < 512; i += 32)
        {
            printk("%x_%x_%x_%x_%x_%x_%x_%x\r\n",
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 0),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 4),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 8),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 12),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 16),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 20),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 24),
                   _REG32(NAND_TEST_RESP_BUF_VA_ADDR + i + 28));
        }
#endif
    }
    else
    {
        return ERR_READ_ID;
    }

    return ret;
}
#ifdef VID_CHECK
U32 NandReadVID()
{
    U8 bank = 0;
    U32 ret = 0;
    U32 cmp = 0;
    U32 VIDret = VID_OK;
    U8 fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        _REG32(TEMP_BUF_ADDR) = 0xFFFFFFFF;
        gul_FW_TAG = llfBETagSetting(TAG_READ_ID, bank);
        FCReadVersionID(fcMode, bank, 0x00, 0x17, TEMP_BUF_PHY_ADDR, 0x30);
        FcBusyWait1ms(1);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[READ-VID][ERR] bank %d ret: 0x%x cmp: 0x%x\r\n", bank, ret, cmp);
        }
        else
        {
            cache_area_dinval(TEMP_BUF_ADDR, 128);
            cache_dummy_update_read();
            while(_REG32(TEMP_BUF_ADDR) == 0xFFFFFFFF);
            U32 i;
            for(i = 0; i < 5; i++)
            {
                printk("Bank %d VID%d output = %x %x\r\n", bank, i + 1, _REG32(TEMP_BUF_ADDR + i * 8),
                       _REG32(TEMP_BUF_ADDR + 4 + i * 8));
            }
            printk("Bank %d VID = %x %x %x %x %x\r\n", bank, _REG08(TEMP_BUF_ADDR), _REG08(TEMP_BUF_ADDR + 8),
                   _REG08(TEMP_BUF_ADDR + 16), _REG08(TEMP_BUF_ADDR + 24), _REG08(TEMP_BUF_ADDR + 32));
#ifdef FTL_B47R
            if((_REG08(TEMP_BUF_ADDR + 8) < 0x8) || (_REG08(TEMP_BUF_ADDR + 16) < 0xB))
            {
                printk("Bank %d read VID disable snap read.\r\n", bank);
                VIDret |= VID_NO_SNAPREAD;
            }
#endif
        }
    }
    return VIDret;
}
#endif
#ifdef UID_CHECK
U32 NandReadUID()
{
    U32 UIDret = UID_OK;
    U8 bank = 0;
    U32 ret = 0;
    U32 cmp = 0;
    U8 fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        gul_FW_TAG = llfBETagSetting(TAG_READ_ID, bank);
        FCReadUniqueID(fcMode, bank, 0x00, 0xf, TEMP_BUF_PHY_ADDR + bank * 32, 0x20);
        FcBusyWait1ms(1);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[READ-UID][ERR] Bank %d ret: 0x%x cmp: 0x%x\r\n", bank, ret, cmp);
            UIDret = UID_FAIL;
        }
        else
        {
            printk("Bank %d Unique ID is  : %x %x %x %x\r\n", bank, _REG32(TEMP_BUF_ADDR + bank * 32),
                   _REG32(TEMP_BUF_ADDR + 4 + bank * 32), _REG32(TEMP_BUF_ADDR + 8 + bank * 32),
                   _REG32(TEMP_BUF_ADDR + 12 + bank * 32));
            printk("Bank %d Unique ID-c is: %x %x %x %x\r\n", bank, _REG32(TEMP_BUF_ADDR + 16),
                   _REG32(TEMP_BUF_ADDR + 20 + bank * 32), _REG32(TEMP_BUF_ADDR + 24 + bank * 32),
                   _REG32(TEMP_BUF_ADDR + 28 + bank * 32));
        }
    }
    return UIDret;
}
#ifdef YX2T_STATUS_CHECK
U32 FCYX2TBugStateCheck(U32 ulMode, U8 ubBankNo, U8 ubLunNo)
{
    U32 ret = YX2T_CHECKSTATUS_INIT;
    U8 parameter[16] = { 0 };
    U8 *pPara = (U8*)parameter;
    gul_FW_TAG = llfBETagSetting(0x0, ubBankNo);
#ifdef YX2T_TKUDEBUG
    printk("Bank %d FCYX2TBugStateCheck\r\n", ubBankNo);
#endif

    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xa7); //25699 ['0xa7']
    FCCmd1Addr1(ulMode, ubBankNo, ubLunNo, 0x08, 0x10); //25700  ['0x08', '0x10']
    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xa7); //25702 ['0xa7']
    FCCmd1Addr1(ulMode, ubBankNo, ubLunNo, 0x08, 0x10); //25703  ['0x08', '0x10']
    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xb5); //25705 ['0xb5']

    pPara[0] = 0x00;
    pPara[1] = 0xf0;
    pPara[2] = 0x20;
    pPara[3] = 0x3a;
    pPara[4] = 0x02;
    pPara[5] = 0x00;
    pPara[6] = 0x00;
    pPara[7] = 0x30;
    ret |= FCCmdReadCheck(ulMode, ubBankNo, ubLunNo, pPara, 1);

    pPara[0] = 0x00;
    pPara[1] = 0x20;
    pPara[2] = 0x20;
    pPara[3] = 0x3a;
    pPara[4] = 0x02;
    pPara[5] = 0x00;
    pPara[6] = 0x00;
    pPara[7] = 0x30;
    ret |= FCCmdReadCheck(ulMode, ubBankNo, ubLunNo, pPara, 2);

    pPara[0] = 0x00;
    pPara[1] = 0xe8;
    pPara[2] = 0x24;
    pPara[3] = 0x3a;
    pPara[4] = 0x02;
    pPara[5] = 0x00;
    pPara[6] = 0x00;
    pPara[7] = 0x30;
    ret |= FCCmdReadCheck(ulMode, ubBankNo, ubLunNo, pPara, 3);

    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xa7); //25739 ['0xa7']
    FCCmd1Addr1(ulMode, ubBankNo, ubLunNo, 0x08, 0x00); //25740  ['0x08', '0x10']
    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xa7); //25742 ['0xa7']
    FCCmd1Addr1(ulMode, ubBankNo, ubLunNo, 0x08, 0x00); //25743  ['0x08', '0x10']

    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xee); //25745 ['0xee']
    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xcc); //25746 ['0xcc']

    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xff); //Checkout_20210913
    FCCmdPollStatus(ulMode, ubBankNo); //Checkout_20210913

    FCCmd1Addr4(ulMode, ubBankNo, ubLunNo, 0x78, 0x00, 0x00, 0x00,
                0x00); //25747 ['0x78', '0x00', '0x00', '0x00', '0x00']
    FCCmd1(ulMode, ubBankNo, ubLunNo, 0xfd); //25752 ['0xfd']

    FCCmdPollStatus(ulMode, ubBankNo);

#ifdef YX2T_TKUDEBUG
    printk("Bank %d end of FCYX2TBugStateCheck\r\n", ubBankNo);
#endif
    return ret;
}

void FCCmd1(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8 ubCmd0)
{
    U32 ret = 0;
    U32 cmp = 0;
    U32 ulDcnt = 0;
    gul_FW_TAG = llfBETagSetting(((gul_FW_TAG >> 8) + 1) & 0xff, ubBankNo);
    ulDcnt += FCCmdSetCtrlNoData(&gsHlcOneCmd.control_bit, g_parser.index.OneCmd);
    ulDcnt += FCCmdSetHead2(&gsHlcOneCmd.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcOneCmd.head3, 0);
    ulDcnt += FCCmdSetPara0(&gsHlcOneCmd.parameter, ubCmd0);
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcOneCmd.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcOneCmd.head3.bits.head_chk_lbn_en = 0;
#endif

    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcOneCmd.control_bit.AsU32, ulDcnt);

    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_OK)
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[WARN] FC cmd1 error: bank %d %x cmp %x \r\n", ubBankNo, ubCmd0, cmp);
        }
#ifdef YX2T_TKUDEBUG
        else
        {
            printk("bank %d FC cmd1 %x cmp %x OK\r\n", ubBankNo, ubCmd0, cmp);
        }
#endif
    }
    else
    {
        printk("[ERR] FC cmd1 timeout: bank %d %x \r\n", ubBankNo, ubCmd0);
    }
#ifdef PRINTCMD
    printk("cmd(0x%x)\r\n", ubCmd0);
#endif
}

void FCCmd1Addr1(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8 ubCmd0, U8 ubAddr0)
{
    U32 ret = 0;
    U32 cmp = 0;
    U32 ulDcnt = 0;
    gul_FW_TAG = llfBETagSetting(((gul_FW_TAG >> 8) + 1) & 0xff, ubBankNo);
    ulDcnt += FCCmdSetCtrlNoData(&gsHlcOneCmd.control_bit, g_parser.index.CmdAddr1);
    ulDcnt += FCCmdSetHead2(&gsHlcOneCmd.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcOneCmd.head3, 0);
    ulDcnt += FCCmdSetPara0(&gsHlcOneCmd.parameter, ((ubAddr0 << 8) | ubCmd0));
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcOneCmd.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcOneCmd.head3.bits.head_chk_lbn_en = 0;
#endif

    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcOneCmd.control_bit.AsU32, ulDcnt);

    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_OK)
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[WARN] FC cmd1addr1 error: bank %d %x Addr %x cmp %x\r\n", ubBankNo, ubCmd0, ubAddr0,
                   cmp);
        }
#ifdef YX2T_TKUDEBUG
        else
        {
            printk("bank %d FC cmd1addr1 %x Addr %x cmp %x OK\r\n", ubBankNo, ubCmd0, ubAddr0, cmp);
        }
#endif

    }
    else
    {
        printk("[ERR] FC cmd1addr1 timeout: bank %d %x Addr %x \r\n", ubBankNo, ubCmd0, ubAddr0);
    }
#ifdef PRINTCMD
    printk("cmd(0x%x)\r\n", ubCmd0);
    printk("addr(0x%x)\r\n", ubAddr0);
#endif
}

void FCCmd1Addr4(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8 ubCmd0, U8 ubAddr0, U8 ubAddr1, U8 ubAddr2,
                 U8 ubAddr3)
{
    U32 ret = 0;
    U32 cmp = 0;
    U32 ulDcnt = 0;
    gul_FW_TAG = llfBETagSetting(((gul_FW_TAG >> 8) + 1) & 0xff, ubBankNo);
    ulDcnt += FCCmdSetCtrlNoData(&gsHlcOneCmd.control_bit, g_parser.index.CmdAddr4);
    ulDcnt += FCCmdSetHead2(&gsHlcOneCmd.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcOneCmd.head3, 0);
    ulDcnt += FCCmdSetPara1(&gsHlcOneCmd.parameter,
                            ((ubAddr2 << 24) | (ubAddr1 << 16) | (ubAddr0 << 8) | ubCmd0), ubAddr3);
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcOneCmd.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcOneCmd.head3.bits.head_chk_lbn_en = 0;
#endif
    /*printk("gsHlcOneCmd.parameter %x %x %x %x\r\n", gsHlcOneCmd.parameter, gsHlcOneCmd.parameter1,
           gsHlcOneCmd.parameter2, gsHlcOneCmd.parameter3);*/
    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcOneCmd.control_bit.AsU32, ulDcnt);

    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_OK)
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[WARN] FC cmd1addr4 error: bank %d %x Addr %x %x %x %x cmp %x\r\n", ubBankNo, ubCmd0,
                   ubAddr0, ubAddr1,
                   ubAddr2, ubAddr3, cmp);
        }
#ifdef YX2T_TKUDEBUG
        else
        {
            printk("bank %d FC cmd1addr4 %x Addr %x %x %x %x cmp %x OK\r\n", ubBankNo, ubCmd0, ubAddr0, ubAddr1,
                   ubAddr2, ubAddr3, cmp);
        }
#endif
    }
    else
    {
        printk("[ERR] FC cmd1addr4 timeout: bank %d %x Addr %x %x %x %x \r\n", ubBankNo, ubCmd0, ubAddr0,
               ubAddr1,
               ubAddr2, ubAddr3);
    }
#ifdef PRINTCMD
    printk("cmd(0x%x)\r\n", ubCmd0);
    printk("addr(0x%x)\r\n", ubAddr0);
    printk("addr(0x%x)\r\n", ubAddr1);
    printk("addr(0x%x)\r\n", ubAddr2);
    printk("addr(0x%x)\r\n", ubAddr3);
#endif
}

U32 FCCmdReadCheck(U32 ulMode, U8 ubBankNo, U8 ubLunNo, U8* pPara, U8 step)
{
    U32 ret = 0;
    U32 cmp = 0;
    FC_Cmd_Bypass bypass;
    U32 ulDcnt = 0;
    gul_FW_TAG = llfBETagSetting(((gul_FW_TAG >> 8) + 1) & 0xff, ubBankNo);

    bypass.bits.aes_bypass = 1;
    bypass.bits.ecc_bypass = 1;
    bypass.bits.scramble_bypass = 1;
    ulDcnt += FCCmdSetCtrlDram(&gsHlcReadRedundant.control_bit, &bypass,
                               g_parser.index.ReadCheck, 1, 0);
    ulDcnt += FCCmdSetHead2(&gsHlcReadRedundant.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetDramAddr(&gsHlcReadRedundant.data_info, NULL, TEMP_BUF_PHY_ADDR + 1024,
                               16, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcReadRedundant.head3, 0);

    U32 i = 0;
    for(i = 0; i < 8; i++)
    {
        gsHlcReadRedundant.parameter[i] = pPara[i];
    }
    ulDcnt += 2;

    gsHlcReadRedundant.parameter[8] = 0x03;
    gsHlcReadRedundant.parameter[9] = 0;
    ulDcnt += 1;
#if 0
    for(i = 0; i < 24; i++)
        printk("gsHlcReadRedundant.parameter[%d] = %x\r\n", i, gsHlcReadRedundant.parameter[i]);
#endif
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcReadRedundant.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcReadRedundant.head3.bits.head_chk_lbn_en = 0;
#endif

    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcReadRedundant.control_bit.AsU32, ulDcnt);
    FcBusyWait1ms(1);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_OK)
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[ERR] FC readcheck error: bank %d step %d cmp %x\r\n", ubBankNo, step, cmp);
#ifdef YX2T_TKUDEBUG
            for(i = 0; i < 24; i++)
                printk("gsHlcReadRedundant.parameter[%d] = %x\r\n", i, gsHlcReadRedundant.parameter[i]);
#endif
            return YX2T_CHECKSTATUS_FAIL;
        }
        else
        {
#ifdef YX2T_TKUDEBUG
            printk("bank %d FC readcheck OK cmp %x \r\n", ubBankNo, cmp);
#endif

            U32 cmp_mask;
            if(gubCmpSel)
                cmp_mask = 0x3ff800;
            else
                cmp_mask = 0x7ff;

            if((FC_TOP_REG(FR_CMPQ_BC) & cmp_mask) != 0)
            {
                U32  err_info0, err_info1;
#ifdef YX2T_TKUDEBUG
                U32 pCMP;
                pCMP = FcPollCompletion(&err_info0, &err_info1);
                printk("%x %x %x\r\n", pCMP, err_info0, err_info1);
#else
                FcPollCompletion(&err_info0, &err_info1);
#endif
            }

            printk("Bank %d data out step %d %x_%x_%x_%x\r\n", ubBankNo, step, _REG32(TEMP_BUF_ADDR + 1024 + 0),
                   _REG32(TEMP_BUF_ADDR + 1024 + 4), _REG32(TEMP_BUF_ADDR + 1024 + 8),
                   _REG32(TEMP_BUF_ADDR + 1024 + 12));
        }

        if(step == 1)
        {
            if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x06f906f9 && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x06f906f9)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_HAVE_TO_RETRIM;
            }
            else if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x19e619e6
                    && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x19e619e6)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and does not have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_NOT_TO_RETRIM;
            }
            else if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x09f609f6
                    && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x09f609f6)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and does not have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_NOT_TO_RETRIM;
            }
            else
            {
                printk("[ERR] Bank %d Step %d state is strange.\r\n", ubBankNo, step);
                return YX2T_CHECKSTATUS_FAIL;
            }
        }
        else if(step == 2)
        {
            if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x29d629d6 && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x29d629d6)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_HAVE_TO_RETRIM;
            }
            else if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x3bc43bc4
                    && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x3bc43bc4)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and does not have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_NOT_TO_RETRIM;
            }
            else
            {
                printk("[ERR] Bank %d Step %d state is strange.\r\n", ubBankNo, step);
                return YX2T_CHECKSTATUS_FAIL;
            }
        }
        else if(step == 3)
        {
            if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x06f906f9 && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x06f906f9)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_HAVE_TO_RETRIM;
            }
            else if(_REG32(TEMP_BUF_ADDR + 1024 + 0) == 0x07f807f8
                    && _REG32(TEMP_BUF_ADDR + 1024 + 4) == 0x07f807f8)
            {
#ifdef YX2T_TKUDEBUG
                printk("Bank %d Step %d checked and does not have to retrim\r\n", ubBankNo, step);
#endif
                return YX2T_CHECKSTATUS_NOT_TO_RETRIM;
            }
            else
            {
                printk("[ERR] Bank %d Step %d state is strange.\r\n", ubBankNo, step);
                return YX2T_CHECKSTATUS_FAIL;
            }
        }
        else
        {
            printk("[ERR] No step %d.\r\n", step);
            return YX2T_CHECKSTATUS_FAIL;
        }
    }
    else
    {
        printk("[ERR] FC readcheck step %d timeout: bank %d\r\n", ubBankNo, step);
#ifdef YX2T_TKUDEBUG
        for(i = 0; i < 24; i++)
            printk("gsHlcReadRedundant.parameter[%d] = %x\r\n", i, gsHlcReadRedundant.parameter[i]);
#endif
        return YX2T_CHECKSTATUS_FAIL;
    }
}

void FCCmdPollStatus(U32 ulMode, U8 ubBankNo)
{
    U32 ret = 0;
    U32 cmp = 0;
    U32 ulDcnt = 0;
    gul_FW_TAG = llfBETagSetting(((gul_FW_TAG >> 8) + 1) & 0xff, ubBankNo);
    ulDcnt += FCCmdSetCtrlNoData(&gsHlcStatusPolling.control_bit, g_parser.index.StatusPolling);
    ulDcnt += FCCmdSetHead2(&gsHlcStatusPolling.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcStatusPolling.head3, 0);
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcStatusPolling.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcStatusPolling.head3.bits.head_chk_lbn_en = 0;
#endif
    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcStatusPolling.control_bit.AsU32, ulDcnt);

    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));

    if(ret == ERR_OK)
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            printk("[WARN] FC FCCmdPollStatus error: bank %d cmp %x\r\n", ubBankNo, cmp);
        }
#ifdef YX2T_TKUDEBUG
        else
        {
            printk("bank %d FC FCCmdPollStatus OK cmp %x \r\n", ubBankNo, cmp);
        }
#endif
    }
    else
    {
        printk("[ERR] FC FCCmdPollStatus timeout: bank %d\r\n", ubBankNo);
    }
#ifdef PRINTCMD
    printk("polling_rs\r\n");
#endif
#ifdef YX2T_TKUDEBUG
    printk("======================================================\r\n");
#endif

}

#endif
#endif

#ifdef NAND_TEST_EN
U32 NandGetFeature()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_GETFEATURE
    U8 ch;
    U8 ce;
    U8 lun;
    U8 bankNo = 0;
    U16 featureAddr;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U32 cmp = 0;
    U32 mappingAddr = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_READ) && (subCommand == TEST_NAND_GETFEATURE));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    featureAddr = pNandTestCmd->lbaLow;
    printk("[NT] Get feature: (%d %d %d) addr: %d\r\n", ch, ce, lun, featureAddr);

    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    printk("[NT] FC: %d, bank: %d, addr: %d\r\n", fcMode, bankNo, featureAddr);

    NandInitBuff(NAND_TEST_RESP_BUF_VA_ADDR, NAND_TEST_RESP_BUF_SIZE, 1);

    gul_FW_TAG = llfBETagSetting(TAG_GETFEATURE, bankNo);
    FCGetfeature(fcMode, bankNo, featureAddr, NAND_TEST_RESP_BUF_PHY_ADDR, 0x10);
    ret = FCCompletionPolling(&cmp, gul_FW_TAG);
    if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
    {
        DBGPRINTK(ALWAYS_MSG, "Get feature fail bank: %d, cmp: 0x%x\r\n", bankNo, cmp);
    }
    else
    {
        cache_area_dinval(NAND_TEST_RESP_BUF_VA_ADDR, 0x20);
        cache_dummy_update_read();
        printk("[NT] Get feature: 0x%x 0x%x\r\n",
               _MEM32(NAND_TEST_RESP_BUF_VA_ADDR), _MEM32(NAND_TEST_RESP_BUF_VA_ADDR + 4));
    }
#endif
    return ret;
}

U32 NandReadUniqueId()
{
    U32 ret = ERR_OK;
    U8 ch = 0xff;
    U8 ce = 0xff;
    U8 lun = 0xff;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 bankNo = 0, DisablePringLog;
    U32 cmp = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0, recdTime, recdTimEnd;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_READ) && (subCommand == TEST_NAND_READUID));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    DisablePringLog = pNandTestCmd->aux & 0x01;
    U32 mappingAddr = 0;
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);

    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }
    if (gubDisablePrintTag != 0xcd)
    {
        printk("[NT] Read-UID: (ch %d ce %d lun %d fcmode %d bank %d fcclk %d\r\n",
               ch, ce, lun, fcMode, bankNo, guwFcClk);
    }


    //read Uniqueid cmd
    gul_FW_TAG = llfBETagSetting(TAG_READ_ID, bankNo);
    FCReadUniqueID(fcMode, bankNo, 0x00, 0x40, NAND_TEST_RESP_BUF_PHY_ADDR, 0x20);
    FcBusyWait1ms(1);
    ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTime);
    ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
    if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        if (gubDisablePrintTag != 0xcd)
        {
            ACCESS_FAST_TIMER(FAST_TIMER_VALUE, recdTimEnd);
            printk("[NT][READ-UID][ERR] Info: 0x%x 0x%x, ret: 0x%x cmp: 0x%x testtime %d us\r\n", ret, errInfo0,
                   errInfo1, ret, cmp, recdTimEnd - recdTime);
        }
    }

    cache_dwbinval(NAND_TEST_RESP_BUF_VA_ADDR);
    cache_dummy_update_read();
    memset((void*)NAND_TEST_RESP_BUF_VA_ADDR + 64, 0x0, 64);

    gubDisablePrintTag = 0;
    if (ret == ERR_OK)
    {
        printk("Unique ID is  : %x %x %x %x\r\n", _REG32(NAND_TEST_RESP_BUF_VA_ADDR),
               _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 4), _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 8),
               _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 12));
        printk("Unique ID-c is: %x %x %x %x\r\n", _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 16),
               _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 20), _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 24),
               _REG32(NAND_TEST_RESP_BUF_VA_ADDR + 28));
    }
    else
    {
        return ERR_READ_ID;
    }

    return ret;
}

#if defined(NAND_TEST_WRITE) || defined(NAND_TEST_NANDTEST)
U32 NandTestDispWriteRedundant(U32 writeMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo,
                               U8 isAllBank, U8 testPlaneIdx, U8 ubSLCMode)
{
    U32 ret = ERR_OK;
    U8 endbank, testbank, startbank;
    PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

    U16 gub_Total_len_per_2K_tmp = gub_Total_len_per_2K;
    gub_Total_len_per_2K = 2 * 1024;

    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }

    pRespInfo->over_ecc_thrsh_error_bits = 0;
    for (testbank = startbank; testbank < endbank; testbank ++)
    {
        gul_FW_TAG = llfBETagSetting(TAG_WRITE, testbank);
        NandTestWriteRedundant(fcMode, testbank, 0,  blockNo + testPlaneIdx, pageNo,
                               NAND_TEST_WRITE_BUF_PHY_ADDR + DRAM_DATA_SIZE * testPlaneIdx, DRAM_DATA_SIZE,
                               NAND_TEST_WRITE_HEAD_PHY_ADDR + DRAM_HEAD_SIZE * testPlaneIdx, DRAM_HEAD_SIZE, ubSLCMode);
        ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);
    }

    ret = NandTestPollCmp(isAllBank);
    if (ret != ERR_OK)
    {
        pRespInfo->over_ecc_thrsh_error_bits = 0xffffffff;

    }
    else
    {
        pRespInfo->over_ecc_thrsh_error_bits += TIMER_GAP(gulCmpDone[startbank], gulTxDone[startbank]);
    }

    gub_Total_len_per_2K = gub_Total_len_per_2K_tmp;

    return ret;
}

U32 NandTestDispFirstWrite(U32 writeMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo,
                           U32 dataLen, U32 headLen, U8 isAllBank, U8 ubSLCMode)
{
    U32 ret = ERR_OK;
    U8 endbank, testbank, startbank;
#ifdef FTL_H3DQV5
    U32 WordLine;
    U8 ProgramTimes;
    U32 ulPageNo;
    U8 string;
    U8 cycle;
    WL_INFO info;
    U32 cmp;
    U32 ulret = 0;
#endif
    PVENDOR_CMD_RESPONSE pRespInfo = NULL;
    pRespInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;


    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }

#if defined (FTL_SSV4) || (FTL_SSV5)
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4
                 || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4_64G)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5 )
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5_64G)))
    {
        if(ubSLCMode != BS_SLC_MODE)
        {
            //Get TLC WL Type
            if(GetWordLineType(pageNo) == SAMSUNG_EDGE_MLC)
            {
                WL_INFO wlInfo;
                wlInfo = WordLineTrans(pageNo);

                if(wlInfo.loc_j == Micron3DT_PAGE_IS_UP)
                {

                    for (testbank = startbank; testbank < endbank; testbank ++)
                    {

                        gul_FW_TAG = llfBETagSetting(TAG_WRITE, testbank);
                        // TLC edge MLC have to dispatch 1 write non-data
                        if(writeMode == SINGLE_TLC)
                        {
                            FCWriteNonData(fcMode, testbank, 0, blockNo);
                            //llfFCCmdWriteNoData(fcMode, testbank, blockNo, 1);
                        }
                        else if(writeMode == MULTI_TLC)
                        {
                            FCMultiWriteNonData(fcMode, testbank, 0, blockNo);
                            //llfFCCmdWriteNoData(fcMode, testbank, blockNo, NandPara.ubPlaneNumPerLun);

                        }
                    }
                    ret = NandTestPollCmp(isAllBank);
                    printk("[nodata]\r\n");
                }
            }
        }
    }
#endif

#ifdef FTL_H3DQV5
    if((writeMode == MULTI_TLC) || (writeMode == SINGLE_TLC))
    {
        for (testbank = startbank; testbank < endbank; testbank ++)
        {
            info = WordLineTrans(pageNo);
            WordLine = info.loc_k;
            ProgramTimes = info.loc_j;
            string = info.loc_i;
            for(cycle = 0; cycle < 4; cycle++)
            {
                ulPageNo = WordLine * 16 + string * 4 + cycle;
                gul_FW_TAG = llfBETagSetting(TAG_WRITE, (testbank | ulPageNo));
                if(writeMode == SINGLE_TLC)
                {
                    if(ProgramTimes == 0)
                    {
                        FCSingleWriteFirstDRAM(fcMode, testbank, 0, blockNo, ulPageNo,
                                               NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen, NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen, 0);
                    }
                    else
                    {
                        FCSingleWriteDRAM(fcMode, testbank, 0, blockNo, ulPageNo,
                                          NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen, NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen, 0);
                    }
                }
                else if(NandPara.ubPlaneNumPerLun == 4)
                {
                    if(ProgramTimes == 0)
                    {
                        FCQuadWriteFirstDRAM(fcMode, testbank, 0, blockNo, ulPageNo,
                                             NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen, NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen, 0);
                    }
                    else
                    {
                        FCQuadWriteDRAM(fcMode, testbank, 0, blockNo, ulPageNo,
                                        NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen, NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen, 0);
                    }
                }
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret == ERR_OK)
                {
                    ulret ++;
                }
                else
                {
                    printk("Write fail programnum(%d) WordLine(%d) cycle(%d) ulPageNo(%d) . \r\n", pageNo,
                           WordLine, cycle, ulPageNo);
                }

            }
            ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);

            if(ulret == 4)
            {
                pRespInfo->over_ecc_thrsh_error_bits = TIMER_GAP(gulCmpDone[startbank],
                                                       gulTxDone[startbank]); //write busy time
                if (gubDisablePrintTag != 0xcd)
                {
                    printk("[NT] bank %d, cmd tx-Done %d us\r\n", startbank, TIMER_GAP(gulCmpDone[startbank],
                            gulTxDone[startbank]));
                }
                ulret = 0;
            }
            else
            {

                pRespInfo->over_ecc_thrsh_error_bits = 0xffffffff;
            }

        }

    }
    else
    {
        for (testbank = startbank; testbank < endbank; testbank ++)
        {
            gul_FW_TAG = llfBETagSetting(TAG_WRITE, testbank);
            if(writeMode == SINGLE_SLC)
            {
                llfFCCmdWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                   NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                   NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
            }
            else if(writeMode == MULTI_SLC)
            {
                if(NandPara.ubPlaneNumPerLun == 1)
                {
                    llfFCCmdWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                       NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                       NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                }
                else if(NandPara.ubPlaneNumPerLun == 2)
                {
                    llfFCCmdMultiWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                            NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                            NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                }
                else if (NandPara.ubPlaneNumPerLun == 4)
                {
                    llfFCCmdQuadWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                           NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                           NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                }
            }
            ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);
        }
        ret = NandTestPollCmp(isAllBank);
        if (ret != ERR_OK)
        {
            pRespInfo->over_ecc_thrsh_error_bits = 0xffffffff;
        }
        else
        {
            pRespInfo->over_ecc_thrsh_error_bits = TIMER_GAP(gulCmpDone[startbank],
                                                   gulTxDone[startbank]); //write busy time
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[NT] bank %d, cmd tx-Done %d us\r\n", startbank, TIMER_GAP(gulCmpDone[startbank],
                        gulTxDone[startbank]));
            }
        }
    }

#else
    for (testbank = startbank; testbank < endbank; testbank ++)
    {
        gul_FW_TAG = llfBETagSetting(TAG_WRITE, testbank);
        if(writeMode == SINGLE_SLC || writeMode == SINGLE_TLC)
        {
            llfFCCmdWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                               NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                               NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
        }
        else if(writeMode == MULTI_SLC || writeMode == MULTI_TLC)
        {
            if(NandPara.ubPlaneNumPerLun == 1)
            {
                llfFCCmdWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                   NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                   NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
            }
            else if(NandPara.ubPlaneNumPerLun == 2)
            {

                llfFCCmdMultiWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                        NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                        NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
            }
            else if (NandPara.ubPlaneNumPerLun == 4)
            {

                llfFCCmdQuadWrite_DRAM(fcMode, testbank, 0, blockNo, pageNo,
                                       NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                       NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
            }
        }
        ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmdStart[testbank]);
    }

    ret = NandTestPollCmp(isAllBank);
    if (ret != ERR_OK)
    {
        pRespInfo->over_ecc_thrsh_error_bits = 0xffffffff;
    }
    else
    {
        pRespInfo->over_ecc_thrsh_error_bits = TIMER_GAP(gulCmpDone[startbank],
                                               gulTxDone[startbank]); //write busy time
        if (gubDisablePrintTag != 0xcd)
        {
            printk("[NT] bank %d, cmd tx-Done %d us\r\n", startbank, TIMER_GAP(gulCmpDone[startbank],
                    gulTxDone[startbank]));
        }
    }
#endif

    return ret;

}
U32 NandTestPollCmp(U8 isAllBank)
{
    U32 ret = ERR_OK;
    U32 cmp = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0;
    U8 loopcount, cmpCnt, failcnt = 0, bank = 0;
    cmpCnt = 0;
    if (isAllBank)
    {
        loopcount = NandPara.ubBankNum;
    }
    else
    {
        loopcount = 1;
    }
    while (cmpCnt != loopcount)
    {
        ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);

        if((ret != ERR_OK) || (((cmp & BE_COMPLETION_ERROR_MASK) != 0) && (!((cmp & OVER_ECC_THRESHOLD_ERR)
                               && !(cmp & ECC_UNCORRECT_ERR)))))
        {
            printk("[NT] ErrInfo: 0x%x 0x%x ret: 0x%x, cmp: 0x%x\r\n", errInfo0, errInfo1, ret, cmp);
            failcnt ++;
        }
        else
        {
            bank = (cmp >> 16) & 0xff;
            if ((gul_FW_TAG >> 8) == TAG_READ)
            {
                gulTxDone[bank] = errInfo0;
            }
#if 0
            printk("[NT] ret: 0x%x, cmp: 0x%x, bank %d, cmd tx-Done %d us, cmd Start-Done %d us\r\n", ret, cmp,
                   bank, (gulCmpDone[bank] - gulTxDone[bank]) * 10, (gulCmpDone[bank] - gulCmdStart[bank]) * 10);
#endif

        }
        cmpCnt ++;
    }

    if(failcnt > 0)
    {
        return ERR_ECC;
    }
    return ERR_OK;

}
U32 NandTestDispSecondWrite(U32 writeMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo,
                            U32 dataLen, U32 headLen, U8 isAllBank, U8 ubSLCMode)
{
    U32 ret = ERR_OK;
#if defined(FTL_N18A) || defined(FTL_SSV4) || defined(FTL_SSV5)
    U8 endbank, testbank, startbank;
    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }
#endif
#ifdef FTL_N18A
    /* For N18 top page program */
    if(((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON) || (FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL))
            && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A))
    {
        if(ubSLCMode != BS_SLC_MODE)
        {
            WL_INFO info;
            info = Micron_3DQ_N18_WordLine_trans(pageNo);
            if(info.loc_j == Micron3DQ_PAGE_IS_TOP)
            {
                U16 page;
                PageNumFill_IM(WL_DATA_BUF_BASE_CACHE);
                page = Get_N18A_Page(pageNo, WL_DATA_BUF_BASE_CACHE, Micron3DQ_PAGE_IS_EXTRA);

                for (testbank = startbank; testbank < endbank; testbank ++)
                {
                    gul_FW_TAG = llfBETagSetting(TAG_WRITE, testbank);
                    if(writeMode == SINGLE_TLC)
                    {
                        llfFCCmdWrite_DRAM(fcMode, testbank, 0, blockNo, page,
                                           NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                           NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                    }
                    else
                    {
                        if(NandPara.ubPlaneNumPerLun == 1)
                        {
                            llfFCCmdWrite_DRAM(fcMode, testbank, 0, blockNo, page,
                                               NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                               NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                        }
                        else if(NandPara.ubPlaneNumPerLun == 2)
                        {
                            blockNo &= ~((U16)NandPara.ubPlaneNumPerLun - 1);
                            llfFCCmdMultiWrite_DRAM(fcMode, testbank, 0, blockNo, page,
                                                    NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                                    NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                        }
                        else if(NandPara.ubPlaneNumPerLun == 4)
                        {
                            blockNo &= ~((U16)NandPara.ubPlaneNumPerLun - 1);
                            llfFCCmdQuadWrite_DRAM(fcMode, testbank, 0, blockNo, page,
                                                   NAND_TEST_WRITE_BUF_PHY_ADDR, dataLen,
                                                   NAND_TEST_WRITE_HEAD_PHY_ADDR, headLen);
                        }
                    }
                }

                ret = NandTestPollCmp(isAllBank);

                if(ret != ERR_OK)
                {
                    printk("[NT] N18 write extra page normal fail ret: 0x%x\r\n", ret);
                }
            }
        }
    }
#endif

#if defined (FTL_SSV4) || defined (FTL_SSV5)
    U32 LMH, WL, Group;
    WL_INFO wlInfo;
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4
                 || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4_64G)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5_64G)))
    {
        if(ubSLCMode != BS_SLC_MODE)
        {
            U16 curPage = pageNo;
            wlInfo = WordLineTrans(curPage);
            WL = wlInfo.loc_k;
            LMH = wlInfo.loc_j;
            Group = (WL * 4) + wlInfo.loc_i;
            if(LMH == Micron3DT_PAGE_IS_EXTRA)
            {
                //DISP
                for (testbank = startbank; testbank < endbank; testbank ++)
                {
                    gul_FW_TAG = llfBETagSetting(TAG_WRITE, testbank);
                    // EDGE_SLC, 1 page per shot, if curPage is the last page of this shot
                    if(writeMode == SINGLE_TLC)
                        FCWriteConfirm( fcMode, testbank, 0, blockNo, curPage);
                    //llfFcCmdWriteConfirm(fcMode, testbank, blockNo, curPage, 1);
                    else
                        FCMultiWriteConfirm( fcMode, testbank, 0, blockNo, curPage);
                    //llfFcCmdWriteConfirm(fcMode, testbank, blockNo, Group, NandPara.ubPlaneNumPerLun);
                }

                //CMP
                ret = NandTestPollCmp(isAllBank);

                if(ret != ERR_OK)
                {
                    printk("[NT] SSV4 write confirm fail, Group %d\r\n", Group);
                }
                else
                {
                    printk("[NT] SSV4 write confirm done, Group %d\r\n", Group);
                }
            }
        }

    }
#endif
    return ret;

}

void ProgramMarkByBank(U32 * tmpWbusy, U32 * tmpPFmark, U8 bankNo, U16 blockNo, U16 pageNo,
                       U8 isAllBank)
{

    U8 endbank, testbank, startbank;
    if (tmpWbusy == ((U32*)0xffffffff))
    {
        return;
    }
    if (isAllBank)
    {
        testbank = 0;
        startbank = 0;
        endbank = NandPara.ubBankNum;
    }
    else
    {
        testbank = bankNo;
        startbank = bankNo;
        endbank = bankNo + 1;
    }


    for (testbank = startbank; testbank < endbank; testbank ++)
    {

        tmpWbusy[testbank] = TIMER_GAP(gulCmpDone[testbank],
                                       gulTxDone[testbank]) * 10; //* report write busy time
        if (gubDisablePrintTag != 0xcd)
        {
            printk("[dbg_write] b %d %x %x %x %d\r\n", testbank, gulCmdStart[testbank], gulTxDone[testbank],
                   gulCmpDone[testbank], tmpWbusy[testbank]);
        }
        if( gulCmp[testbank] & FLASH_ERR)
        {
            tmpPFmark[testbank] = 1; //* report program fail
            printk("[PF] bank %d block %d page %d\r\n", testbank, blockNo, pageNo);
        }
    }
    FillOutResult2(WL_DATA_OUT2_BASE_CACHE, bankNo, pageNo, tmpWbusy, isAllBank);

}



U32 NandTestDispWrite(U32 writeMode, U8 fcMode, U8 bankNo, U16 blockNo, U16 pageNo, U16 pageCount,
                      U8 eccState,
                      U8 aesState, U8 scrambleState, U32 * tmpWbusy, U32 * tmpPFmark, U8 isAllBank)
{
    U32 ret = ERR_OK;
    U32 dataLen = 0;
    U32 headLen = 0;
    U32 i = 0, j = 0, failcnt = 0;
    U8 testPlaneNo = 1;
    U8  ubSLCMode = 0;
    //FC_TOP_REG(FR_SEED_AGITATION_EN) = 1;

    FcCmdBypass.bits.ecc_bypass = (eccState == 0);
    FcCmdBypass.bits.aes_bypass = (aesState == 0);
    FcCmdBypass.bits.scramble_bypass = (scrambleState == 0);

    if(writeMode == SINGLE_SLC || writeMode == SINGLE_TLC)
    {
        dataLen = DRAM_DATA_SIZE;
        headLen = DRAM_HEAD_SIZE;
        testPlaneNo = 1;
    }
    else if(writeMode == MULTI_SLC || writeMode == MULTI_TLC)
    {
        dataLen = DRAM_DATA_SIZE * NandPara.ubPlaneNumPerLun;
        headLen = DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun;
        blockNo &= ~((U16)NandPara.ubPlaneNumPerLun - 1);
        testPlaneNo = NandPara.ubPlaneNumPerLun;
    }

#if 1
    NandInitBuff(NAND_TEST_WRITE_BUF_VA_ADDR, NAND_TEST_WRITE_BUF_SIZE, 0);
#else
    NandTestGenData(NAND_TEST_WRITE_BUF_VA_ADDR, dataLen, 0x12345678);
#endif

    if(writeMode == SINGLE_SLC || writeMode == MULTI_SLC)
    {
        ubSLCMode = BS_SLC_MODE;
        gubNandFlashType = 1;
    }
    else
    {
        ubSLCMode = 0;
        gubNandFlashType = 0;
    }



    if(eccState)
    {
        for(i = 0; i < pageCount; i++)
        {
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[NT] bank: %d, Blk: %d, Page: %d\r\n", bankNo, blockNo, pageNo + i);
            }

            ret = NandTestDispFirstWrite(writeMode, fcMode, bankNo, blockNo, pageNo + i, dataLen, headLen,
                                         isAllBank, ubSLCMode);
            if (ret != ERR_OK)
            {
                failcnt ++;
            }
            ProgramMarkByBank(tmpWbusy, tmpPFmark, bankNo, blockNo, pageNo + i, isAllBank);
            ret = NandTestDispSecondWrite(writeMode, fcMode, bankNo, blockNo, pageNo + i, dataLen, headLen,
                                          isAllBank, ubSLCMode);
            if (ret != ERR_OK)
            {
                failcnt ++;
            }
        }
    }
    else
    {
        for (j = 0; j < testPlaneNo; j++)
        {
            for(i = 0; i < pageCount; i++)
            {
                printk("[NT] bank: %d, Blk: %d, Page: %d\r\n", bankNo, blockNo, pageNo + i);
                ret = NandTestDispWriteRedundant(writeMode, fcMode, bankNo, blockNo, pageNo + i, isAllBank, j,
                                                 ubSLCMode);
                if (ret != ERR_OK)
                {
                    failcnt ++;
                }
                ProgramMarkByBank(tmpWbusy, tmpPFmark, bankNo, blockNo + j, pageNo + i, isAllBank);
            }
        }
    }

#if 0
    llfFCCmdStatusPolling(fcMode, bankNo);
    ret = llfFCCmdCompletionPolling(&cmp, gul_FW_TAG);
    if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        DBGPRINTK(ALWAYS_MSG, "[NT] Polling normal fail ret: 0x%x, cmp: 0x%x\r\n", ret, cmp);
    }
#endif
    if (failcnt > 0)
    {
        return ERR_ECC;
    }
    return ERR_OK;
}
#endif

U32 NandWrite()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_WRITE
    U8 eccState;
    U8 aesState;
    U8 scrambleState;
    U8 ch;
    U8 ce;
    U8 lun;
    U16 blockNo;
    U16 pageNo;
    U16 pageCount;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U32 mappingAddr = 0;
    U8 bankNo = 0;
    U8 writeMode = 0;
    U8 DisableTimingRecording = 0;
    U8 DisablePringLog = 0;

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_WRITE) && (subCommand == TEST_NAND_WRITE));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    blockNo = pNandTestCmd->lbaLowExp;
    blockNo = (blockNo << 8) + pNandTestCmd->lbaHigh;
    pageNo = pNandTestCmd->lbaMid;
    pageNo = (pageNo << 8) + pNandTestCmd->lbaLow;
    pageCount = pNandTestCmd->sectorExp;
    pageCount = (pageCount << 8) + pNandTestCmd->sector;
    eccState = (pNandTestCmd->lbaMidExp) & 0x01;
    aesState = (pNandTestCmd->lbaMidExp >> 1) & 0x01;
    scrambleState = (pNandTestCmd->lbaMidExp >> 2) & 0x01;
    writeMode = (pNandTestCmd->lbaMidExp >> 3) & 0x03;
    DisableTimingRecording = (pNandTestCmd->lbaMidExp >> 5) & 0x01; //bit5
    DisablePringLog = (pNandTestCmd->lbaMidExp >> 6) & 0x01; //bit6
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    if (DisablePringLog)
    {
        gubDisablePrintTag = 0xcd;
    }
    else
    {
        gubDisablePrintTag = 0;
    }


    printk("[NT] Write: (ch %d ce %d lun %d blk %d pg %d) pgcnt %d writemode %d (ecc %d aes %d scr %d) b: %d, fc %d (Dis time %d print %d)\r\n",
           ch, ce, lun, blockNo, pageNo,
           pageCount, writeMode, eccState, aesState, scrambleState, bankNo, fcMode,
           DisableTimingRecording, DisablePringLog);
    //no slc cache program

    NandTestChangeLdpc(blockNo, pageNo);
    ret = NandTestDispWrite(writeMode, fcMode, bankNo, blockNo, pageNo, pageCount, eccState, aesState,
                            scrambleState, (U32*) 0xffffffff, (U32*) 0xffffffff, 0);
    gubDisablePrintTag = 0;
#endif
    return ret;
}

U32 NandSetFeature()
{
    U32 ret = ERR_OK;
#ifdef NAND_TEST_SETFEATURE
    U8 ch;
    U8 ce;
    U8 lun;
    U8 bankNo = 0;
    U16 featureAddr, featureValue;
    PNandTestCmd pNandTestCmd = NULL;
    U8 command = 0xff;
    U8 subCommand = 0xff;
    U8 fcMode = ONFI_SDR;
    U8 clkMode = FC_PLL_CLK_10M;
    U32 cmp = 0;
    U32 mappingAddr = 0;
    U16 fcCycleNum[14] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266};

    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    command = pNandTestCmd->command;
    subCommand = pNandTestCmd->feature;
    ASSERT(ALWAYS_MSG, (command == BE_VEN_WRITE) && (subCommand == TEST_NAND_SETFEATURE));

    ch = (pNandTestCmd->featureExp >> 5) & 0x07;
    ce = (pNandTestCmd->featureExp >> 2) & 0x07;
    lun = pNandTestCmd->featureExp & 0x03;
    featureValue = pNandTestCmd->sectorExp;
    featureAddr = pNandTestCmd->lbaLow;
    printk("[NT] Set feature: (%d %d %d) addr: %d\r\n", ch, ce, lun, featureAddr);

    clkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    bankNo = *(U8 *)(mappingAddr + ch * 8 + ce);
    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    printk("[NT] FC: %d, bank: %d, addr: %d\r\n", fcMode, bankNo, featureAddr);

    ChangeFCClk(fcMode, FC_PLL_CLK_10M);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0x500);
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | 0xa00);
#endif

    gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bankNo);
    cache_area_dwbinval(NAND_TEST_WRITE_BUF_VA_ADDR, 0x20);
    cache_dummy_update_read();
    FCSetfeature(fcMode, bankNo, featureAddr, featureValue/*_REG32(NAND_TEST_WRITE_BUF_VA_ADDR)*/);
    ret = FCCompletionPolling(&cmp, gul_FW_TAG);
    if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
    {
        DBGPRINTK(ALWAYS_MSG, "[NT] Set feature fail bank: %d, cmp: 0x%x\r\n", bankNo, cmp);
    }
    else
    {
        printk("[NT] Set feature: 0x%x 0x%x\r\n",
               featureAddr/*_MEM32(NAND_TEST_WRITE_BUF_VA_ADDR)*/,
               featureValue/*_MEM32(NAND_TEST_WRITE_BUF_VA_ADDR + 4)*/);
    }

    ChangeFCClk(fcMode, clkMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG,
                                  gubStartCH) & 0xFFFC00FF) | ((fcCycleNum[clkMode] >> 1) << 8));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG,
                                  gubStartCH) & 0xFFFC00FF) | (fcCycleNum[clkMode] << 8));
#endif
#endif
    return ret;
}

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
void NandTestWriteRedundant(U8 ulMode, U8 ubBankNo, U8 ubLunNo, U16 uwBlock, U16 uwPage,
                            U32 dataAddr, U32 dataLen, U32 headAddr, U32 headLen, U8 ubSLCMode)
{

}
#else
void NandTestWriteRedundant(U8 ulMode, U8 ubBankNo, U8 ubLunNo, U16 uwBlock, U16 uwPage,
                            U32 dataAddr, U32 dataLen, U32 headAddr, U32 headLen, U8 ubSLCMode)
{
    Write_Para writePara;
    FC_Cmd_Bypass bypass;
    U32 ulMulNum = 0;
    U32 ulDcnt = 0;
    U16 parserIndex = 0;
    ulMulNum = (ulMode != 0) ? (4) : (8);
    bypass.bits.aes_bypass = 1;
    bypass.bits.ecc_bypass = 1;
    bypass.bits.scramble_bypass = 1;
    writePara.bank = ubBankNo;
    writePara.lun = ubLunNo;
    writePara.block = uwBlock;
    writePara.page = uwPage;
    writePara.fc_data_len = gub_Total_len_per_2K * ulMulNum - 1;//ecc_bypass_mode = 0x0

    if (ubSLCMode)
    {
        parserIndex = g_parser.index.SlcWrite;
    }
    else
    {
        parserIndex = g_parser.index.Write;
    }
    ulDcnt += FCCmdSetCtrlDram(&gsHlcWriteRedundant.control_bit, &bypass, parserIndex, 1, 1);
    ulDcnt += FCCmdSetHead2(&gsHlcWriteRedundant.head2, gul_FW_TAG, ulMode, 0, 0, 0);
    ulDcnt += FCCmdSetEccInfo(&gsHlcWriteRedundant.head3, 0);
    ulDcnt += FCCmdSetDramAddr(&gsHlcWriteRedundant.data_info, &gsHlcWriteRedundant.head_info,
                               dataAddr, dataLen, headAddr, headLen);
    ulDcnt += FCCmdSetParaWriteRedundant(gsHlcWriteRedundant.parameter, ubSLCMode, &writePara);
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    gsHlcWriteRedundant.control_bit.bits.hlcmd_size = ulDcnt - 1;
    gsHlcWriteRedundant.head3.bits.head_chk_lbn_en = 0;
#endif
    while(CheckFcCmdFifo(ubBankNo, ulDcnt) == FC_CMD_FIFO_FULL);
    PushFcCmdFifo(ubBankNo, &gsHlcWriteRedundant.control_bit.AsU32, ulDcnt);
}
#endif
#endif

U32 NandTestCompletionPolling(U32 * pCMP, U16 expectTag, U32 * pInfo0, U32 * pInfo1)
{
    U32 timeCount = 0;
    U8 bank = 0;
    *pCMP = 0xFFFF0000;

    if(expectTag == 0xffff)
    {
        *pCMP = 0x0;
    }

    while((*pCMP >> 24) != (expectTag >> 8) || (*pCMP & PROGRAM_DATA_DONE) != 0)
    {
        while(FC_TOP_REG(FR_CMPQ_BC) != 0)
        {
            *pCMP = NandTestCompletion(pInfo0, pInfo1);
            bank = (*pCMP >> 16) & 0xff;
            if ((*pCMP & PROGRAM_DATA_DONE) != 0)
            {
                ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulTxDone[bank]);
            }
            else
            {
                ACCESS_FAST_TIMER(FAST_TIMER_VALUE, gulCmpDone[bank]);
            }
            break;
        }

        FcBusyWait10us(1);
        timeCount++;
        if(timeCount > 20000)
        {
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[NT] CompletionPolling timeout: 0x%x %x\r\n", expectTag, *pCMP);
            }

            return ERR_FIO_TIMEOUT;
        }
    }
    gulCmp[bank] = *pCMP;
    return ERR_OK;
}

U32 NandTestCompletion(U32 * pInfo0, U32 * pInfo1)
{
    U32 cmp = 0;
    U32 readPtr = 0;
    U32 errInfo0 = 0;
    U32 errInfo1 = 0;

    readPtr = FC_TOP_REG(FR_CMPQ_HDBL) & 0x1FF;
    cmp = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
    readPtr++;

    if((cmp & 0x7fff) != 0)
    {
        if((cmp & OVER_ECC_THRESHOLD_ERR) && !(cmp & ECC_UNCORRECT_ERR))
        {
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            if ((gubDisablePrintTag != 0xcd) && (errInfo0 > 0x28))
            {
                printk(  "[NT] Max ECC error, ErrBit: %d\r\n", errInfo0);
            }
        }

        if(cmp & ECC_UNCORRECT_ERR)
        {
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            errInfo1 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            printk(  "[NT] ECC uncorrec error, loc: %x %x, cmp %x\r\n", errInfo1, errInfo0, cmp);
        }

        if(cmp & LBN_ERR)
        {
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            //printk(  "[NT] LBN error, Head LBN: 0x%x\r\n", errInfo0);
        }

        if(cmp & UNC_ERR)
        {
            ASSERT(ALWAYS_MSG, 0);
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            printk(  "[NT] UNC err, Head UNC: %x cmp %x\r\n", (errInfo0 & 0xFFFF), cmp);
        }

        if(cmp & FLASH_ERR)
        {
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            printk(  "[NT] Flash error, Last Flash status: %x cmp %x\r\n", (errInfo0 & 0xFF), cmp);
        }

        if(cmp & CRC_ERR && ((cmp & ALL_FF) == 0))
        {
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            errInfo1 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            printk(  "[NT] CRC err, info: %x %x cmp %x\r\n", errInfo0, errInfo1, cmp);
        }
        if((cmp & ALL_FF) && (cmp & CRC_ERR))
        {
            errInfo0 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            errInfo1 = _REG32(FC_CMP_FIFO_BASE + 4 * (readPtr & 0xFF));
            readPtr++;
            printk( "[NT] ALL FF, empty loc: %x %x cmp %x\r\n", errInfo0, errInfo1, cmp);
        }
        if (cmp & TIMEOUT_ERR)
        {
            if (gubDisablePrintTag != 0xcd)
            {
                printk("[NT] TIMEOUT_ERR\r\n");
            }
        }
    }

    FC_TOP_REG(FR_CMPQ_HDBL) = readPtr & 0x1FF;

    *pInfo0 = errInfo0;
    *pInfo1 = errInfo1;
    return cmp;
}

#ifdef NAND_TEST_EN
U32 configFAddrAndVth(U8 * ubFAddrStart, U8 * ubFAddrEnd, U32 * vthStart, U32 * vthEnd,
                      U8 * FAddrNum, U8 * vthNum, U8 * ubFAddrInterval, U8 * vthInterval, U8 cellMode, U16 uwPage)
{
    U32 ret = ERR_OK;
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL))
    {
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B17)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B16)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27A)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B))
        {
#if defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B17A) || defined(FTL_B16A)
            if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
            {
                *ubFAddrStart = 0xA4;
                *ubFAddrEnd = 0xA4;
                *ubFAddrInterval = 2;
            }
            else if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
            {
                *ubFAddrStart = 0xA5;
                *ubFAddrEnd = 0xAB;
                *ubFAddrInterval = 2;
            }
            else
            {
                ret = ERR_UNKNOWN;
                return ret;
            }

            *FAddrNum = (*ubFAddrEnd - *ubFAddrStart) / *ubFAddrInterval + 1;
            *vthStart = 0x80;
            *vthEnd = 0x17F;
            *vthInterval = 4;
            *vthNum = (*vthEnd - *vthStart) / *vthInterval + 1;
#endif
        }
        else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
        {
#ifdef FTL_N18A
            if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
            {
                *ubFAddrStart = 0xA4;
                *ubFAddrEnd = 0xA4;
                *ubFAddrInterval = 1;
            }
            else if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
            {
                WL_INFO info = Micron_3DQ_N18_WordLine_trans(uwPage);

                if(info.loc_k == 3 || info.loc_k == 4 || info.loc_k == 5
                        || info.loc_k == 192 || info.loc_k == 193 || info.loc_k == 194) //TLC WL
                {
                    if(info.loc_j == Micron3DQ_PAGE_IS_LOW)
                    {
                        *ubFAddrStart = 0xD3;
                        *ubFAddrEnd = 0xD3;
                        *ubFAddrInterval = 1;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_UP)
                    {
                        *ubFAddrStart = 0xD1;
                        *ubFAddrEnd = 0xD5;
                        *ubFAddrInterval = 4;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_EXTRA)
                    {
                        *ubFAddrStart = 0xD0;
                        *ubFAddrEnd = 0xD6;
                        *ubFAddrInterval = 2;
                    }
                }
                else if(info.loc_k != 0 && info.loc_k != 1 && info.loc_k != 2
                        && info.loc_k != 195 && info.loc_k != 196 && info.loc_k != 197) //QLC WL
                {
                    if(info.loc_j == Micron3DQ_PAGE_IS_LOW)
                    {
                        *ubFAddrStart = 0xC8;
                        *ubFAddrEnd = 0xC8;
                        *ubFAddrInterval = 1;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_UP)
                    {
                        *ubFAddrStart = 0xC4;
                        *ubFAddrEnd = 0xCC;
                        *ubFAddrInterval = 8;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_EXTRA)
                    {
                        *ubFAddrStart = 0xC2;
                        *ubFAddrEnd = 0xCE;
                        *ubFAddrInterval = 4;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_TOP)
                    {
                        *ubFAddrStart = 0xC1;
                        *ubFAddrEnd = 0xCF;
                        *ubFAddrInterval = 2;
                    }
                }
                else
                {
                    DBGPRINTK(ALWAYS_MSG, "[NT] ERR SLC Page but TLC Mode \r\n");
                    ret = ERR_UNKNOWN;
                    return ret;
                }
            }
            else
            {
                DBGPRINTK(ALWAYS_MSG, "[NT] Invalid cellMode %d\r\n", cellMode);
                ret = ERR_UNKNOWN;
                return ret;
            }
            *FAddrNum = (*ubFAddrEnd - *ubFAddrStart) / *ubFAddrInterval + 1;
            *vthStart = 0x80;
            *vthEnd = 0x17F;
            *vthInterval = 4;
            *vthNum = (*vthEnd - *vthStart) / *vthInterval + 1;
#endif
        }
        else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28 ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
        {
#if defined(FTL_N28A) || defined(FTL_N38A)
            if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
            {
                *ubFAddrStart = 0xA4;
                *ubFAddrEnd = 0xA4;
                *ubFAddrInterval = 1;
            }
            else if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
            {
                WL_INFO info = WordLineTrans(uwPage);

                if(info.loc_k == 1 || info.loc_k == 47 || info.loc_k == 49
                        || info.loc_k == 51 || info.loc_k == 53 || info.loc_k == 97
                        || info.loc_k == 100 || info.loc_k == 146 || info.loc_k == 148
                        || info.loc_k == 150 || info.loc_k == 152 || info.loc_k == 196
                        || info.loc_k == 199 || info.loc_k == 245 || info.loc_k == 247
                        || info.loc_k == 249 || info.loc_k == 251 || info.loc_k == 295) //TLC WL
                {
                    if(info.loc_j == Micron3DQ_PAGE_IS_LOW)
                    {
                        *ubFAddrStart = 0xD3;
                        *ubFAddrEnd = 0xD3;
                        *ubFAddrInterval = 1;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_UP)
                    {
                        *ubFAddrStart = 0xD1;
                        *ubFAddrEnd = 0xD5;
                        *ubFAddrInterval = 4;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_EXTRA)
                    {
                        *ubFAddrStart = 0xD0;
                        *ubFAddrEnd = 0xD6;
                        *ubFAddrInterval = 2;
                    }
                }
                else if(info.loc_k != 0 && info.loc_k != 98 && info.loc_k != 99
                        && info.loc_k != 197 && info.loc_k != 198 && info.loc_k != 296) //QLC WL
                {
                    if(info.loc_j == Micron3DQ_PAGE_IS_LOW)
                    {
                        *ubFAddrStart = 0xC8;
                        *ubFAddrEnd = 0xC8;
                        *ubFAddrInterval = 1;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_UP)
                    {
                        *ubFAddrStart = 0xC4;
                        *ubFAddrEnd = 0xCC;
                        *ubFAddrInterval = 8;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_EXTRA)
                    {
                        *ubFAddrStart = 0xC2;
                        *ubFAddrEnd = 0xCE;
                        *ubFAddrInterval = 4;
                    }
                    else if(info.loc_j == Micron3DQ_PAGE_IS_TOP)
                    {
                        *ubFAddrStart = 0xC1;
                        *ubFAddrEnd = 0xCF;
                        *ubFAddrInterval = 2;
                    }
                }
                else
                {
                    DBGPRINTK(ALWAYS_MSG, "[NT] ERR SLC Page but TLC Mode \r\n");
                    ret = ERR_UNKNOWN;
                    return ret;
                }
            }
            else
            {
                DBGPRINTK(ALWAYS_MSG, "[NT] Invalid cellMode %d\r\n", cellMode);
                ret = ERR_UNKNOWN;
                return ret;
            }
            *FAddrNum = (*ubFAddrEnd - *ubFAddrStart) / *ubFAddrInterval + 1;
            *vthStart = 0x80;
            *vthEnd = 0x17F;
            *vthInterval = 4;
            *vthNum = (*vthEnd - *vthStart) / *vthInterval + 1;
#endif
        }
        else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R)
        {
#if defined(FTL_B37R) || defined(FTL_B36R)
            if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
            {
                *ubFAddrStart = 0xA4;
                *ubFAddrEnd = 0xA4;
                *ubFAddrInterval = 1;
            }
            else if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
            {
                WL_INFO info = WordLineTrans(uwPage);

                if(info.loc_k == 64 ||  info.loc_k == 65) //MLC WL
                {
                    if(info.loc_j == Micron3DT_PAGE_IS_LOW)
                    {
                        *ubFAddrStart = 0xA1;
                        *ubFAddrEnd = 0xA1;
                        *ubFAddrInterval = 1;
                    }
                    else if(info.loc_j == Micron3DT_PAGE_IS_UP)
                    {
                        *ubFAddrStart = 0xA0;
                        *ubFAddrEnd = 0xA3;
                        *ubFAddrInterval = 2;
                    }
                }
                else if(info.loc_k != 0 &&  info.loc_k != 129) //TLC WL
                {
                    if(info.loc_j == Micron3DT_PAGE_IS_LOW)
                    {
                        *ubFAddrStart = 0xA5;
                        *ubFAddrEnd = 0xA9;
                        *ubFAddrInterval = 4;
                    }
                    else if(info.loc_j == Micron3DT_PAGE_IS_UP)
                    {
                        *ubFAddrStart = 0xA6;
                        *ubFAddrEnd = 0xAA;
                        *ubFAddrInterval = 2;
                    }
                    else if(info.loc_j == Micron3DT_PAGE_IS_EXTRA)
                    {
                        *ubFAddrStart = 0xA7;
                        *ubFAddrEnd = 0xAB;
                        *ubFAddrInterval = 4;
                    }
                }
                else
                {
                    DBGPRINTK(ALWAYS_MSG, "[NT] ERR SLC Page but TLC Mode \r\n");
                    ret = ERR_UNKNOWN;
                    return ret;
                }
            }
            else
            {
                DBGPRINTK(ALWAYS_MSG, "[NT] Invalid cellMode %d\r\n", cellMode);
                ret = ERR_UNKNOWN;
                return ret;
            }
            *FAddrNum = (*ubFAddrEnd - *ubFAddrStart) / *ubFAddrInterval + 1;
            *vthStart = 0x80;
            *vthEnd = 0x17F;
            *vthInterval = 4;
            *vthNum = (*vthEnd - *vthStart) / *vthInterval + 1;
#endif
        }
        else
        {
            DBGPRINTK(ALWAYS_MSG, "[NT] Not support NAND vendor_serial %d_%d for drawing Vth\r\n",
                      FLASH_VENDOR(gulFlashVendorNum), FLASH_SERIAL_NUM(gulFlashVendorNum));
            ret = ERR_UNKNOWN;
            return ret;
        }
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX
            && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV5
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV6
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV6_1Tb))
    {
        if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
        {
            *ubFAddrStart = 0x03;
            *ubFAddrEnd = 0x03;
            *ubFAddrInterval = 1;
            *FAddrNum = 1;

        }
        else if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
        {
            if(uwPage % 3 == 0)
            {
                *ubFAddrStart = 0x11;
                *ubFAddrEnd = 0x15;
                *ubFAddrInterval = 4;
            }
            else if(uwPage % 3 == 1)
            {
                *ubFAddrStart = 0x10;
                *ubFAddrEnd = 0x14;
                *ubFAddrInterval = 2;
            }
            else
            {
                *ubFAddrStart = 0x0F;
                *ubFAddrEnd = 0x13;
                *ubFAddrInterval = 4;
            }
        }
        else
        {
            DBGPRINTK(ALWAYS_MSG, "[NT] Invalid cellMode %d\r\n", cellMode);
            ret = ERR_UNKNOWN;
            return ret;
        }

        *FAddrNum = (*ubFAddrEnd - *ubFAddrStart) / *ubFAddrInterval + 1;
        *vthStart = 0x20;
        *vthEnd = 0xFF;
        *vthInterval = 1;
        *vthNum = (*vthEnd - *vthStart) / *vthInterval + 2;

    }
    else if ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
             || (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
             || (FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
             || (FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC))
    {
        *ubFAddrStart = 0;
        *ubFAddrEnd = 0;
        *ubFAddrInterval = 0;
        *FAddrNum = 0;
        *vthStart = 0x80;
        *vthEnd = 0x17f;
        *vthInterval = 4;
        *vthNum = (*vthEnd - *vthStart) / *vthInterval + 1;
    }
    else
    {
        DBGPRINTK(ALWAYS_MSG, "[NT] Not support NAND vendor_serial %d_%d for drawing Vth\r\n",
                  FLASH_VENDOR(gulFlashVendorNum), FLASH_SERIAL_NUM(gulFlashVendorNum));
        ret = ERR_UNKNOWN;
    }
    return ret;
}

//mainly for SSV4 FTL_TSB_BICS3 FTL_SANDISK_BICS3
void NanddoDistribution2(void)
{
#ifdef NAND_VTH_DRAW2
    U8  i, bits, cellMode;
    U32 vt, bytes,  units;
    U8 ubMode, ubCh, ubCe, ubLunNo = 0, ubBank;
    U16 uwBlock;
    U16 uwPage;
    U8 featureAddr1 = 0, featureAddr2 = 0, featureAddr = 0; //shiftVal = 4
#if defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
    U8 featureAddr3 = 0;
    U8 featureAddr4 = 0;
#endif

    U8 cycle, len;
    U16 lowpage = 0;
    U32 ulfeatureVal = 0;
    U32 ret, errInfo0, errInfo1, cmp, mappingAddr;

    U8 ubFAddrStart, ubFAddrEnd, ubFAddrInterval;
    U32 vthStart, vthEnd;
    U8 vthInterval;
    U8 vthNum, fAddrNum;
    //U8 ubSLCMode = 0;
    PNandTestCmd pNandTestCmd = NULL;
    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    ubCh = (pNandTestCmd->featureExp >> 5) & 0x07;
    ubCe = (pNandTestCmd->featureExp >> 2) & 0x07;
    ubLunNo = pNandTestCmd->featureExp & 0x03;
    uwBlock = pNandTestCmd->lbaLowExp;
    uwBlock = (uwBlock << 8) + pNandTestCmd->lbaHigh;
    uwPage = pNandTestCmd->lbaMid;
    uwPage = (uwPage << 8) + pNandTestCmd->lbaLow;

    //FcBusyWait10us(500);//FIXME - workaround for sandisk no value


    ubMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    ubBank = *(U8 *)(mappingAddr + ubCh * 8 + ubCe);

#if defined(FTL_SANDISK_BICS3) || defined(FTL_TSB_BICS3) || defined(FTL_SANDISK_BICS4) || defined(FTL_TSB_BICS4)
    units = 18336 /*16448*/; //18336 TO CHECK
#else
    units = _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FULL_PAGE_PER_BLOCK);//18432 18K per page
#endif

    cellMode = (pNandTestCmd->lbaMidExp >> 3) & 0x03;

    printk("[NT] Vth: (m %d ch %d ce %d bank %d lun %d blk %d pg %d %d celm: %d)\r\n", ubMode, ubCh,
           ubCe, ubBank, ubLunNo, uwBlock, uwPage, units, cellMode);

#if defined(FTL_SSV4) || defined(FTL_TSB_BICS3) || defined(FTL_SSV5) || defined(FTL_TSB_BICS4)
    featureAddr1 = 0x89;
    featureAddr2 = 0x8A;
#elif defined(FTL_SANDISK_BICS3)  || defined(FTL_SANDISK_BICS4)
    featureAddr1 = 0x12;
    featureAddr2 = 0x13;
#elif defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
    featureAddr1 = 0xA0;
    featureAddr2 = 0xA1;
    featureAddr3 = 0xA2;
    featureAddr4 = 0xA3;
#endif


    if(cellMode == SINGLE_TLC)//MULTI_TLC not support
    {
#if defined(FTL_SANDISK_BICS3) || defined(FTL_TSB_BICS3) || defined(FTL_SANDISK_BICS4) || defined(FTL_TSB_BICS4)\
    || defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
        lowpage = (3 * (uwPage / 3));
#elif defined(FTL_SSV4)
        lowpage = Get_SSV4_LowerPage(uwPage);
#elif defined(FTL_SSV5)
        lowpage = GetWordLinePage(LOWER_PAGE, uwPage, WL_DATA_BUF_BASE_CACHE);
#endif

        if(lowpage == 0xffff)
        {
            printk("page error\r\n");
            return;
        }
        printk("[NT] Pg: %d, %d len: %d\r\n", uwPage, lowpage, units);

    }


    ret = configFAddrAndVth(&ubFAddrStart, &ubFAddrEnd, &vthStart, &vthEnd,
                            &fAddrNum, &vthNum, &ubFAddrInterval, &vthInterval, cellMode, lowpage);
    if(ret != ERR_OK)
    {
        return;
    }


    for( i = 0; i < 3; i++ )
    {
        len = (i == 1) ? (3) : (2);
        U32 error_cnt[64][len];
        memset(error_cnt, 0, sizeof(U32) * 64 * len);

        for( cycle = 0; cycle < len; cycle++ )
        {
            for( vt = vthStart; vt < vthEnd; vt += vthInterval )
            {
#if 0
                U32 dword;
                for(dword = 0; dword < (units >> 2); dword++) // 32bit/8bit
                {
                    _REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword) = 0xA5ED7B92;
                }

#endif
                FCSetFeatureAndCheckByBank(ubMode, ubBank, featureAddr1, 0 );
                FCSetFeatureAndCheckByBank(ubMode, ubBank, featureAddr2, 0 );
#if defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
                FCSetFeatureAndCheckByBank(ubMode, ubBank, featureAddr3, 0 );
                FCSetFeatureAndCheckByBank(ubMode, ubBank, featureAddr4, 0 );

#endif

#if defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
                if((cellMode == SINGLE_SLC) || (cellMode == MULTI_SLC))
                {
                    ulfeatureVal = ((vt & 0xff) << 0);
                    featureAddr = featureAddr4;
                }
                else
#endif
                {
                    if(i == 0) //low page
                    {
                        if(cycle == 0)
                        {
                            ulfeatureVal = ((vt & 0xff) << 0);
                        }
                        else if(cycle == 1)
                        {
#if defined(FTL_SANDISK_BICS3) || defined(FTL_TSB_BICS3)  || defined(FTL_SANDISK_BICS4) || defined(FTL_TSB_BICS4)
                            ulfeatureVal = ((vt & 0xff) << 16);
#elif defined(FTL_SSV4) || defined(FTL_SSV5) || defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
                            ulfeatureVal = ((vt & 0xff) << 8);
#endif
                        }
                        featureAddr = featureAddr1;
                    }
                    else if(i == 1) //up page
                    {
                        if(cycle == 0)
                        {
                            ulfeatureVal = ((vt & 0xff) << 0);
                        }
                        else if(cycle == 1)
                        {
                            ulfeatureVal = ((vt & 0xff) << 8);
                        }
                        else
                        {
                            ulfeatureVal = ((vt & 0xff) << 16);
                        }
                        featureAddr = featureAddr2;
                    }
                    else //if(i == 2) //extra page
                    {
                        if(cycle == 0)
                        {
#if defined(FTL_SANDISK_BICS3) || defined(FTL_TSB_BICS3)  || defined(FTL_SANDISK_BICS4) || defined(FTL_TSB_BICS4)
                            ulfeatureVal = ((vt & 0xff) << 8);
#elif defined(FTL_SSV4) || defined(FTL_SSV5)
                            ulfeatureVal = ((vt & 0xff) << 16);
#elif defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
                            ulfeatureVal = ((vt & 0xff) << 0);
#endif
                        }
                        else if(cycle == 1)
                        {
#if defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
                            ulfeatureVal = ((vt & 0xff) << 8);
#else
                            ulfeatureVal = ((vt & 0xff) << 24);
#endif
                        }
#if defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
                        featureAddr = featureAddr3;
#else
                        featureAddr = featureAddr1;
#endif

                    }
                }


                FCSetFeatureAndCheckByBank(ubMode, ubBank, featureAddr, ulfeatureVal);

                if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
                {
                    //ubSLCMode = BS_SLC_MODE;
                }
                else
                {
                    //ubSLCMode = 0;
                }



#if defined(FTL_SANDISK_BICS3)  || defined(FTL_SANDISK_BICS4)
                //ASSERT(ALWAYS_MSG, 0);//TBD
                gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, ubBank);
                FCOneCmd(ubMode, ubBank, 0, 0x5D);
                //FCCmdOneCmd(ubMode, ubCh, ubCe, 0x5D);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                //ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
                if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
                {
                    printk("[NT] Err: 0x%x 0x%x 0x%x 0x%x\r\n", errInfo0, errInfo1, ret, cmp);
                }
#endif
                gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

                if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
                {
                    NandTestReadRedundant(ubMode, ubBank, 0, uwBlock, lowpage + i, 0,
                                          NAND_TEST_READ_BUF_PHY_ADDR, units, 1);
                }
                else
                {
                    FCReadRedundant(ubMode, ubBank, 0, uwBlock, lowpage + i, 0x00, NAND_TEST_READ_BUF_PHY_ADDR,
                                    units);
                }


                //ASSERT(ALWAYS_MSG, 0)//TBD.

                ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
                if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
                {
                    printk("[NT] Err: 0x%x 0x%x 0x%x 0x%x\r\n", errInfo0, errInfo1, ret, cmp);
                }

                // FcBusyWait10us(100);
                cache_area_dinval(NAND_TEST_READ_BUF_VA_ADDR, units); //cache invalid

                for(bytes = 0; bytes < (units >> 2); bytes++) // 32bit/8bit=4byte->bytes
                {
#if 0
                    if ((_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * bytes)) != 0xA5ED7B92)
                    {
                        printk("d:%x, 0x%x\r\n", bytes, (_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * bytes)));
                    }
#endif
                    for(bits = 0; bits < 32; bits++)
                    {
                        if((_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * bytes) >> bits) & 0x1)
                        {
                            error_cnt[(vt - vthStart) >> 2][cycle]++;
                        }
                    }
                }


            }

            for(vt = vthStart + vthInterval; vt <= vthEnd; vt += vthInterval) // 0xCE-0x132
            {
                printk("%d, %d, %d, %x_%x\r\n",
                       (vt - vthStart),
                       doAbs(error_cnt[(vt - vthStart) >> 2][cycle], error_cnt[((vt - vthStart) >> 2) - 1][cycle]),
                       error_cnt[((vt - vthStart) >> 2) - 1][cycle], i, cycle);
            }
        }
//#if defined(FTL_SSV4)
//        lowpage++;
//#endif

    }

#endif
}

//mainly for b16/b17/b27/n18/hynix/n28

void NanddoDistribution1(void)
{
#ifdef NAND_VTH_DRAW1
    U8  i, cellMode;
    U32 units;
    U8 ubMode, ubCh, ubCe, ubLunNo = 0, ubBank;
    U16 uwBlock;
    U16 uwPage;
    //U8 LowMiddleUp = 0xff;

    U8 ubFAddrStart, ubFAddrEnd, ubFAddrInterval;
    U32 vthStart, vthEnd;
    U8 vthInterval;
    U8 vthNum, fAddrNum;
    U32 mappingAddr;
    U32 ret, cmp;
    U32 errInfo0, errInfo1;
    U8 ubReadOffsetFeatureAddress;
    U32 vt, dword;
    U8 bits;
    //U8 ubSLCMode = 0;
    PNandTestCmd pNandTestCmd = NULL;
    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    ubCh = (pNandTestCmd->featureExp >> 5) & 0x07;
    ubCe = (pNandTestCmd->featureExp >> 2) & 0x07;
    ubLunNo = pNandTestCmd->featureExp & 0x03;
    uwBlock = pNandTestCmd->lbaLowExp;
    uwBlock = (uwBlock << 8) + pNandTestCmd->lbaHigh;
    uwPage = pNandTestCmd->lbaMid;
    uwPage = (uwPage << 8) + pNandTestCmd->lbaLow;

    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    ubBank = *(U8 *)(mappingAddr + ubCh * 8 + ubCe);
    units = _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FULL_PAGE_PER_BLOCK);
    cellMode = (pNandTestCmd->lbaMidExp >> 3) & 0x03;
    ubMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    printk("[NT] Vth: (%d %d %d %d %d %d %d)\r\n", ubCh, ubCe, ubLunNo, ubBank, uwBlock, uwPage,
           cellMode);
    printk("[NT] FC: %d\r\n", ubMode);
    if(cellMode == MULTI_SLC || cellMode == SINGLE_SLC)
    {
        //gubNandFlashType = 1
    }
    else
    {
        //gubNandFlashType = 0;
    }

#if defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B17A) || defined(FTL_B16A) || defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N38A)

    if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
    {
        U32 tmpPage;
        PageNumFill(WL_DATA_BUF_BASE_CACHE);
        tmpPage = GetWordLinePage(EXTRA_PAGE, uwPage, WL_DATA_BUF_BASE_CACHE);

        if(tmpPage == 0xffff)
        {
            //printk("[NT] Page error\r\n");
            return;
        }
        printk("[NT] Pg: %d, %d len: %d\r\n", uwPage, tmpPage, units);
        uwPage = tmpPage;

    }

#endif

    ret = configFAddrAndVth(&ubFAddrStart, &ubFAddrEnd, &vthStart, &vthEnd,
                            &fAddrNum, &vthNum, &ubFAddrInterval, &vthInterval, cellMode, uwPage);
    FcBusyWait10us(3000);

    if(ret != ERR_OK)
    {
        return;
    }
    i = 0;


#ifdef FTL_HYV5
    for(ubReadOffsetFeatureAddress = ubFAddrStart; ubReadOffsetFeatureAddress <= ubFAddrEnd;
            ubReadOffsetFeatureAddress += ubFAddrInterval)
    {
        FcGetparameter(ubMode, uwBlock, ubBank, uwPage, ubReadOffsetFeatureAddress, GETFEATURE_BY_BANK_PHY,
                       0x10, ubSLCMode);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
        {
            DBGPRINTK(ALWAYS_MSG, "GetFeature or GetParameter Fail bank %d,%x\r\n", ubBank, cmp);
            return;
        }
        else
        {
            U32 para;
            if(ubMode == 0)
            {
                para = _REG32(GETFEATURE_BY_BANK_UC);
            }
            else
            {
                para = (_REG08(GETFEATURE_BY_BANK_UC + 6) << 24) | (_REG08(GETFEATURE_BY_BANK_UC + 4) << 16) |
                       (_REG08(GETFEATURE_BY_BANK_UC + 2) << 8) | _REG08(GETFEATURE_BY_BANK_UC);
            }
            //originVth[i++] = para & 0xFF;
            printk("[NT] Get feature/parameter 0x%x: 0x%x\r\n", ubReadOffsetFeatureAddress, para);
        }
        i++;
    }
#endif


    U32* error_cnt = (U32*)NAND_TEST_ERROR_BIT;
    memset((void*)NAND_TEST_ERROR_BIT, 0, sizeof(U32) * vthNum * fAddrNum);

#if defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B17A) || defined(FTL_B16A) || defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N38A) || defined(FTL_B37R) || defined (FTL_B36R)
    for(ubReadOffsetFeatureAddress = ubFAddrStart; ubReadOffsetFeatureAddress <= ubFAddrEnd;
            ubReadOffsetFeatureAddress += ubFAddrInterval)
    {
        FCSetFeatureAndCheckByBank(ubMode, ubBank, ubReadOffsetFeatureAddress, 0x0);
        FcBusyWait10us(100);
    }
#endif

    i = 0;
    for(ubReadOffsetFeatureAddress = ubFAddrStart; ubReadOffsetFeatureAddress <= ubFAddrEnd;
            ubReadOffsetFeatureAddress += ubFAddrInterval)
    {
#if defined(FTL_H3DTV6) || defined(FTL_H3DTV5)
        if(cellMode == MULTI_TLC || cellMode == SINGLE_TLC)
        {
            if(ubReadOffsetFeatureAddress == 0x11)
            {
                FCSetParameterAndCheckByBank(ubMode, ubBank, 0x15, 0xA1);
                //FCSetParams(ubMode, ubBank, 0, 0x15, 0xA1);
            }
            else if(ubReadOffsetFeatureAddress == 0x15)
            {
                //FCSetParameterAndCheck(ubMode, uwBlock, ubBank, uwPage, 0x11, 0x20, ubSLCMode);
                FCSetParameterAndCheckByBank(ubMode, ubBank, 0x11, 0x20);
                vthStart = 0x4A;
                vthEnd = 0xA1;
            }
            else if(ubReadOffsetFeatureAddress == 0x13 || ubReadOffsetFeatureAddress == 0x14)
            {
                vthStart = 0x00;
                vthEnd = 0xA1;
            }
        }
#endif

        FcBusyWait10us(100);

        for(vt = vthStart; vt <= vthEnd; vt += vthInterval)
        {
#if defined(FTL_H3DTV6) || defined(FTL_H3DTV5)

            FCSetParameterAndCheckByBank(ubMode, ubBank, ubReadOffsetFeatureAddress,
                                         (vt & 0xFF));
#else
            FCSetFeatureAndCheckByBank(ubMode, ubBank, ubReadOffsetFeatureAddress, (vt & 0xff));

#endif

#if 0 ///test
            for(dword = 0; dword < (units >> 2); dword++) // 32bit/8bit
            {
                _REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword) = 0xA5ED7B92;
            }

#if 1
            for(dword = 0; dword < (units >> 2); dword++) // 32bit/8bit
            {
                //printk("data:0x%x\r\n",(_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * words)));
                if((_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword) != _REG32(NAND_TEST_READ_BUF_VA_ADDR)) )
                {
                    printk("idx:%d, data:0x%x\r\n", dword, (_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword)));
                }
            }
#endif
#endif

            gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

#ifdef FTL_HYV5
            LowMiddleUp = uwPage % 3 + 1;
#endif

            FCReadRedundant(ubMode, ubBank, 0, uwBlock, uwPage, 0x00, NAND_TEST_READ_BUF_PHY_ADDR,
                            units);
            //ASSERT(ALWAYS_MSG, 0);//TBD
            ret = NandTestCompletionPolling(&cmp, gul_FW_TAG, &errInfo0, &errInfo1);
            if(ret != ERR_OK || (cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                printk("ErrInfo: 0x%x 0x%x ret %x cmp %x\r\n", errInfo0, errInfo1, ret, cmp);
            }
            //cache_area_dinval(NAND_TEST_READ_BUF_VA_ADDR, units);
            cache_area_dinval(NAND_TEST_READ_BUF_VA_ADDR, ((units / 32) + 1) * 32);
            for(dword = 0; dword < (units >> 2); dword++) // 32bit/8bit
            {
#if 0
                if ((_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword)) != 0xA5ED7B92)
                {
                    printk("d:%x, 0x%x\r\n", dword, (_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword)));
                }
#endif
                for(bits = 0; bits < 32; bits++)
                {
                    if((_REG32(NAND_TEST_READ_BUF_VA_ADDR + 4 * dword) >> bits) & 0x1)
                    {
//                        error_cnt[(vt - vthStart) / vthInterval][i]++;
                        error_cnt[((vt - vthStart) / vthInterval) + (i * vthNum)]++;
                    }
                }

            }
        }

#if defined(FTL_H3DTV6) || defined(FTL_H3DTV5)
        // 0x38 Hard Reset Parameter Address
        gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, ubBank);
        FCOneCmdSetParams(ubMode, ubBank, 0, ubReadOffsetFeatureAddress, 0, 0x38);
        FcBusyWait10us(50);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
        {
            printk( "Hard Reset Fail bank %d,%x\r\n", ubBank, cmp);
        }
#else
        FCSetFeatureAndCheckByBank(ubMode, ubBank, ubReadOffsetFeatureAddress, 0x0);
#endif
        FcBusyWait10us(100);

        for(vt = vthStart + vthInterval; vt <= vthEnd; vt += vthInterval)
        {
            printk("%d, %d, %d %x\r\n",
                   (vt - vthStart),
                   doAbs(error_cnt[((vt - vthStart) / vthInterval) + (i * vthNum)],
                         error_cnt[(((vt - vthStart) / vthInterval) - 1) + (i * vthNum)]),
                   error_cnt[(((vt - vthStart) / vthInterval) - 1) + (i * vthNum)],
                   ubReadOffsetFeatureAddress);

        }
        i++;
    }


#endif
}

U32 NanddoDistribution(void)
{
    printk("Trigger Scan Vth\r\n");

    //FIXME - workaround for sandisk no value
    FcBusyWait10us(500);

    if(((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC))
      )
    {
        NanddoDistribution2();
    }
    else if((((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON)
              || (FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL))
             && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B17)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B16)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27A)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R)
                 || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R))) ||
            (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX &&
             (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV5
              || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV6)
              || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV6_1Tb)))
           )
    {
        NanddoDistribution1();
    }

    return ERR_OK;
}

U32 NANDTestFunction(void)
{
    U8 arg0;
    U32 ret = ERR_OK;
    U16 cycles = 200;
    U32 mappingAddr = 0, cmp = 0, para = 0;
    U8 ubMode, ubBank, ubFAddr;
    U8 ubCh, ubCe;

    struct _LLF_UNI_INFO *pLLF_uni_info;
    PNandTestCmd pNandTestCmd = NULL;
    pNandTestCmd = (PNandTestCmd)NAND_TEST_CMD_BUF_VA_ADDR;
    arg0 = pNandTestCmd->lbaMidExp;
    ubCh = (pNandTestCmd->featureExp >> 5) & 0x07;
    ubCe = (pNandTestCmd->featureExp >> 2) & 0x07;
    mappingAddr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
    ubBank = *(U8 *)(mappingAddr + ubCh * 8 + ubCe);
    ubMode = FR_CONFIG_CH(FR_FC_MODE, 0);
    printk("[NT]function:%d,(%d,%d,%d),\r\n", arg0, ubCh, ubCe, ubMode);

    cycles = pNandTestCmd->lbaLowExp;
    cycles = (cycles << 8) + pNandTestCmd->lbaHigh;
    if(cycles == 0)
        cycles = 200;

    switch(arg0)
    {
    case FUNCTION_RW_CACHE:
        printk("r/w cache repeat:2000\r\n");
        ret = repeated_WriteReadCache(cycles);
        break;
    case FUNCTION_READID:
#if 0 //use NandReadId
        gul_FW_TAG = llfBETagSetting(TAG_READ_ID, ubBank);
        llfFCCmdReadID(ubMode, ubBank, 0x40, 5, NAND_TEST_READ_BUF_PHY_ADDR, 0x10);
        FcBusyWait1ms(1);
        ret = llfFCCmdCompletionPolling(&cmp, (gul_FW_TAG));
        if(((cmp & BE_COMPLETION_ERROR_MASK) != 0))
        {
            printk("read id fail:0x%x,0x%x", ret, cmp);
        }
        else
        {
            printk("nand id:0x%x,0x%x\r\n", READ_REG_32(NAND_TEST_READ_BUF_VA_ADDR),
                   READ_REG_32(NAND_TEST_READ_BUF_VA_ADDR + 4));
        }
#endif
        break;
    case FUNCTION_SCAN_FDB:
        pLLF_uni_info = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
        ret = 0;//llfScanFactoryDefectBlocks(ubBank);TBD
        if(ret == ERR_OK)
        {
            printk("Factory BB:%d\r\n", pLLF_uni_info->ulData5);
        }
        break;
    case FUNCTION_READ_UID:
        printk("not support now\r\n");
        break;
    case FUNCTION_SET_FEATURE:
        ubFAddr = pNandTestCmd->lbaHigh;
        gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, ubBank);
        para = pNandTestCmd->lbaMid + (pNandTestCmd->lbaLow << 8) +
               (pNandTestCmd->sector << 16) + (pNandTestCmd->sectorExp << 24);
        FCSetFeatureAndCheckByBank(ubMode, ubBank, ubFAddr, para);
        if(ret == ERR_OK)
        {
            printk("%x is: 0x%x, 0x%x\r\n", ubFAddr, _REG32(GET_FEATURE_VA_ADDR));
        }
        break;

    case FUNCTION_GET_FEATURE:
        ubFAddr = pNandTestCmd->lbaHigh;
        gul_FW_TAG = llfBETagSetting(TAG_GETFEATURE, ubBank);
        FCGetfeature(ubMode, ubBank, ubFAddr, GET_FEATURE_PHY_ADDR, 0x10);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret == ERR_OK)
        {
            printk("%x is: 0x%x, 0x%x\r\n", ubFAddr, _REG32(GET_FEATURE_VA_ADDR),
                   _REG32(GET_FEATURE_VA_ADDR + 4));
        }
        else
        {
            printk("get feature fail,cmp:0x%x\r\n", cmp);
        }
        break;
    case FUNCTION_READCACHE:
        NandTest_ReadCache(ubMode, ubBank);
        break;
    case FUNCTION_WRITECACHE:
        NandTest_WriteCache(ubMode, ubBank);
        break;
    case FUNCTION_LUNDETECT:
        NandTest_LunDetect();
        break;
    default:
        printk("invalid function test command\r\n");
        ret = ERR_UNKNOWN_VENDOR_CMD;
        break;

    }
    return ret;
}
#endif
#endif

