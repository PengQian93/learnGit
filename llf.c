
#define LLF_GLOBAL
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
#else
#define _COMMEM_GLOBAL
#endif

#include "setup.h"
#include "lib_basetype.h"
#include "system_reg.h"
#include "serial.h"
#include "lib_debug.h"
#include "smp.h"
#include "codebank.h"
#include "spic.h"
#include "memory.h"
#include "scheduler0.h"
#include "fe_misc.h"
#include "lib_fc_public.h"
#include "fc_reg.h"
#include "hlc_driver_a1_public.h"
#include "timer.h"

#include "lib_misc.h"
#include "lib_fc_public.h"
#include "lib_retcode.h"
#include "lib_debug.h"
#include "lib_cpu.h"
#include "lib_fio_public.h"
#include "lib_sblk_config.h"
#include "lib_hlc_com.h"
#include "llf_global.h"
#include "llf_public.h"
#include "llf_public_com.h"
#include "llf_lib_public.h"
#include "llfmp.h"
//#include "fw_version.h"
#include "nand_test.h"
#include "nand_test_public.h"
#include "platform_global.h"
#include "platform.h"
#if defined(RL6577_VA)
#include "../platform/include/i2c.h"
#include "../platform/include/timer.h"
#ifndef BL_ROM
#include "../nvme/include/nvme.h"
#endif
#endif


//const char version1[VERSION_LEN] __attribute__((section(".llf_version"))) = FW_VERSION_TAG;
const unsigned int gold_llf __attribute__((section(".llf_signature1"))) = SIGNATURE_BEGIN;
const unsigned int beef __attribute__((section(".llf_signature2"))) = SIGNATURE_END;

U32 llfCheckDefectThreshold();
U32 llfSearchDynamicSBlock(U32 static_sblk_addr, U32 dynamic_sblk_addr, U8 tag);
#ifdef LLF_AUTO_RMA
extern int vsprintf(char *buf, const char *fmt, const U32 *dp);
#endif

U32 llfFdmInit()
{
    U8 FcClkMode = 0;
    taskInfo_t taskInfo;

    PVENDOR_CMD pVendorCmd;
    PVENDOR_CMD_RESPONSE pVendorCmdResponse;
    PLLF_UNI_INFO pLLFInfo;

    // Point the Vendor command to cmd address
    pVendorCmd = (PVENDOR_CMD)LLF_CMD_BUF_VA_ADDR;// SRAM used for vendor cmd
    pVendorCmdResponse = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;// SRAM used for vendor cmd response
    pLLFInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

#if defined(RL6577_FPGA) || defined(RL6577_VA) || defined(RL6447_VA) || defined(RL6643_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA) || defined(RL6531_VB)
    memcpy((void*)CONFIG_SEED_TABLE_VA_ADDR, (const void*)(SEED_TABLE_VA_ADDR),
           SEED_TABLE_ACTUAL_USE_SIZE);
    //initial ram global vars and sblk
    memset((void*)LLF_RAM_GLOBAL_ADDR, 0, LLF_RAM_GLOBAL_SIZE);
#ifdef MST_MERGE
    // init SBLK area from CONFIG_BASE_VA_ADDR
    memcpy((void*)SBLK_ADDR, (const void*)(CONFIG_BASE_VA_ADDR), SBLK_SIZE >> 1);
    memset((void*)(SBLK_ADDR + (SBLK_SIZE >> 1)), 0, SBLK_SIZE >> 1);
#else
    memset((void*)SBLK_ADDR, 0, SBLK_SIZE);
#endif
#endif
    printk("g_print_lock= %d\r\n", g_print_lock);

    core0_timer_init();
#if defined(RL6577_FPGA) || defined(RL6577_VA)
    config_fasttimer();
#endif
    SllInitListCtrl(&BE2FEAdminCmdListCtrl, HOST_ADMIN_CMD_NUM_MAX, (void *)&gpHostAdminCmd[0],
                    sizeof(HostAdminCommand));
#if defined(RTS5771_FPGA) && defined(NOFE_LLF_FLOW)
    gfTestFlag = 1;
    printk("gfTestFlag = 1\r\n");
#endif
#if defined(RL6577_FPGA) || defined(RL6577_VA) || defined(RL6643_FPGA)||defined(RL6643_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
    if( (pVendorCmd->subcmd) == BE_CLEAR_SPI_LOCK )
#else
    if(gfTestFlag == SPI_WP_CLEAR_TAG)
#endif
    {
        WRITE_REG_32(MMCR_GIC_RESET_MASK0, 0x3fff);  //mask all interrupt
#if !(defined(RTS5771_FPGA)||defined(RTS5771_VA))
        spi_init_llf();
#endif
        spi_flash_set_protection_llf(FALSE);
        WRITE_REG_32(MMCR_GIC_SET_MASK0, 0x3fff);  //unmask all interrupt
        //gfTestFlag = 0;
        return ERR_OK;
    }

#ifdef SBLK_EXPAND
    U32 value_temp;

    if(READ_REG_32(CPU_MODE_REG) & CPU_EA_MODE)
    {
        value_temp = _REG32(SPI_DL_SBLK_START_TAG_ADDR_WHEN_EA);
    }
    else
    {
        value_temp = _REG32(SPI_DL_SBLK_START_TAG_ADDR);
    }
    //printk("[bl_load spi]spi SBLK_EXPAND_ADDR startTag 0x%x\r\n", value_temp);
    if((value_temp & 0xff) == TAG_FOR_SBLK_EXPAND)
    {
        gubSblkStart = (value_temp >> 8) & 0xff;
        gubSblkBankStart = (value_temp >> 16) & 0xff;
        if(READ_REG_32(CPU_MODE_REG) & CPU_EA_MODE)
        {
            gulSblkCHCEMap[0] = _REG32(SPI_DL_SBLK_CH0CEMAP_ADDR_WHEN_EA);
            gulSblkCHCEMap[1] = _REG32(SPI_DL_SBLK_CH1CEMAP_ADDR_WHEN_EA);
        }
        else
        {
            gulSblkCHCEMap[0] = _REG32(SPI_DL_SBLK_CH0CEMAP_ADDR);
            gulSblkCHCEMap[1] = _REG32(SPI_DL_SBLK_CH1CEMAP_ADDR);
        }
    }

    llfprintk("[llfFdmint] v %d sblkstart %d sblkbankstart %d ch0 %x ch1 %x\r\n",
              (value_temp & 0xff), gubSblkStart, gubSblkBankStart, gulSblkCHCEMap[0], gulSblkCHCEMap[1]);
#endif

    if(gfTestFlag == 1) //LLF_INIT
    {
        llfmpInit();
        pVendorCmdResponse->res_state = VENDOR_CMD_IDLE;
        taskInfo.taskId = e_BE_llf_handle_taskid;
        taskInfo.taskFunc = BE_llf_handle_UTask;
        schedulerSetTaskCore0(&taskTable_BE_Cpu, taskInfo);
        /*pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_INIT;
        pVendorCmd->arg0 = LLF_MP_TAG;
        pVendorCmd->arg1 = LLF_MP_TAG;
        llfprintk("llfinit start.\r\n");*/

        gub_fe_mode = 1;

        // Clear DBT  Set default defect table of super block as zero
        memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
        cache_area_dwb(DBT_ADDR, DBT_SIZE);
        cache_dummy_update_read();
        memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
        cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
        cache_dummy_update_read();


        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_CALIBRATE;
        pVendorCmd->arg0 = (U8) IF_ONFI_SDR;//ubIFType
        pVendorCmd->arg1 = (U8) FcClkMode;//ubClkMode 0:DQS 10M, 9:DQS 100M, 12:DQS 200M

        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }


        pVendorCmdResponse->res_state = VENDOR_CMD_ERASE;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ERASENAND;
        pVendorCmd->arg0 = 0;// Erase all blocks

        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }


        pVendorCmdResponse->res_state = VENDOR_CMD_BUILD_DBT;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ODBT;
        pVendorCmd->arg0 = 0;// Erase all blocks
        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }

        llfprintk("execute start.\r\n");
        pVendorCmdResponse->res_state = VENDOR_CMD_WRITE_FW;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_EXECUTE;
        pVendorCmd->arg0 = (U8) 0;//ubIFType
        //pVendorCmd->arg0 = (U8) IF_ONFI_DDR2;//ubIFType
        pVendorCmd->arg1 = 0;//ubClkMode

        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }
        llfprintk("execute\r\n");


    }
    else if(gfTestFlag == 2) //LLF_CALIBRATE
    {
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_CALIBRATE;
        pVendorCmd->arg0 = (U8) IF_TOGGLE_DDR;//ubIFType
        //pVendorCmd->arg0 = (U8) IF_ONFI_DDR2;//ubIFType
        pVendorCmd->arg1 = (U8) FcClkMode;//ubClkMode 0:DQS 10M, 9:DQS 100M, 12:DQS 200M
        gub_fe_mode = 1;
    }
    else if(gfTestFlag == 3) //LLF_EXECUTE
    {
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_EXECUTE;
        pVendorCmd->arg0 = (U8) IF_TOGGLE_DDR;//ubIFType
        //pVendorCmd->arg0 = (U8) IF_ONFI_DDR2;//ubIFType
        pVendorCmd->arg1 = (U8)FcClkMode;//ubClkMode
        gub_fe_mode = 1;
    }
    else if(gfTestFlag == 8) //LLF_ERASE_ALL_NAND
    {
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ERASENAND;
        pVendorCmd->arg0 = 0;// Erase all blocks
        gub_fe_mode = 1;
    }
    else if(gfTestFlag == 4) //LLF_Build DBT
    {
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ODBT;
        pVendorCmd->arg0 = 0;// Erase all blocks
        gub_fe_mode = 1;
    }

    if(gfTestFlag == 5) //LLF_ALL
    {
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ALL;
        pVendorCmd->arg0 = 0;

        while(1)
        {
            BE_llf_handle_UTask();
        }
    }

    if(gfTestFlag == 13) //llfFCSelfBurningTest
    {
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_SELFTEST;
        pVendorCmd->arg0 = 1;// FC Self Burning Test
        pVendorCmd->arg1 = 5; // arg1 SELFTEST_FCSCAN_BLKNUM
        gub_fe_mode = 1;
    }

    gfTestFlag = 0; // reset to no command

    if(pVendorCmd->subcmd == BE_LLF_INIT)
    {
        ASSERT_LLF(pVendorCmd->arg0 == LLF_MP_TAG);
        ASSERT_LLF(pVendorCmd->arg1 == LLF_MP_TAG);
        llfmpInit();
#ifdef RL6643_VA
        pLLFInfo->ubData0 = AUTO_CALIBRATE_VREF_START;
#else
        pLLFInfo->ubData0 = AUTO_CALIBRATE_TX_START;
#endif
        pVendorCmdResponse->res_state = VENDOR_CMD_IDLE;
        taskInfo.taskId = e_BE_llf_handle_taskid;
        taskInfo.taskFunc = BE_llf_handle_UTask;
        schedulerSetTaskCore0(&taskTable_BE_Cpu, taskInfo);
        //gfTestFlag = 2;
    }

    if(gfTestFlag == 2)
    {
        llfprintk("calibrate start.\r\n");
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_CALIBRATE;
        pVendorCmd->arg0 = 2;//interface
        pVendorCmd->arg1 = 0;//timing mode
        gub_fe_mode = 1;

        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }
        llfprintk("calibrate done.\r\n");
        //gfTestFlag = 8;
    }

    if(gfTestFlag == 8)
    {
        llfprintk("erase start.\r\n");
        pVendorCmdResponse->res_state = VENDOR_CMD_ERASE;
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ERASENAND;
        pVendorCmd->arg0 = 0;// Erase all blocks
        gub_fe_mode = 1;
        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }
        FcBusyWait1ms(1);
        //gfTestFlag = 4;
        llfprintk("erase ok.\r\n");
    }

    if(gfTestFlag == 4) //LLF_Build DBT
    {
        llfprintk("build DBT\r\n");
        pVendorCmdResponse->res_state = VENDOR_CMD_START;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_ODBT;
        pVendorCmd->arg0 = 0;// Erase all blocks
        gub_fe_mode = 1;
        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }
        //gfTestFlag = 3;
    }

    if(gfTestFlag == 3) //LLF_EXECUTE
    {
        llfprintk("execute start.\r\n");
        pVendorCmdResponse->res_state = VENDOR_CMD_WRITE_FW;// inform LLF to start llf erase all nand
        pVendorCmd->cmd  = BE_VEN_NONDAT;
        pVendorCmd->subcmd = BE_LLF_EXECUTE;
        pVendorCmd->arg0 = (U8) IF_TOGGLE_DDR;//ubIFType
        //pVendorCmd->arg0 = (U8) IF_ONFI_DDR2;//ubIFType
        pVendorCmd->arg1 = 9;//ubClkMode
        gub_fe_mode = 1;

        while(pVendorCmdResponse->res_state != VENDOR_CMD_IDLE)
        {
            BE_llf_handle_UTask();
        }
        llfprintk("execute\r\n");
    }

    return ERR_OK;
}

void llfFcInitReg()
{
    U32 i;
    U32 reg_v;

    FR_G_CTRL_REG32_W(FR_PAD_SHDN_CTRL, 0x2);
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON) || (FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
            || FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
    {
        if((gubNandDefaultMode) && (0 == vdd_1v8_en))
        {
            reg_v = (4 << 24) | (0x0d << 16) | (0x0d << 8) | (4 << 0);
            FR_G_CTRL_REG32_W(FR_CHN_DELAY_0, reg_v );

            reg_v = (0x00 << 24) | (0x00 << 16) | (0x46 << 8) | (3 << 0);//For DDR2 33M
            FR_G_CTRL_REG32_W(FR_CHN_DELAY_1, reg_v );
        }
        else
        {
            reg_v = (2 << 24) | (3 << 16) | (2 << 8) | (2 << 0);
            FR_G_CTRL_REG32_W(FR_CHN_DELAY_0, reg_v );

            reg_v = (10 << 24) | (6 << 16) | (11 << 8) | (2 << 0);//For SDR
            FR_G_CTRL_REG32_W(FR_CHN_DELAY_1, reg_v );
        }

        reg_v = (3 << 24) | (1 << 16) | (10 << 8) | (6 << 0);
        FR_G_CFG_REG32_W(FR_GLB_DELAY_0, reg_v );

        reg_v = 5;
        FR_G_CFG_REG32_W(FR_GLB_DELAY_1, reg_v );
    }
    else if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA) ||
            (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX))
    {
        reg_v = (3 << 24) | (3 << 16) | (3 << 8) | (1 << 0);
        FR_G_CTRL_REG32_W(FR_CHN_DELAY_0, reg_v );

        reg_v = (0x0a << 24) | (0x06 << 16) | (11 << 8) | (2 << 0);//For SDR
        FR_G_CTRL_REG32_W(FR_CHN_DELAY_1, reg_v );
        FR_G_CFG_REG32_W(FR_GLB_DELAY_0, 0x03010a06 );
        FR_G_CFG_REG32_W(FR_GLB_DELAY_1, 0x05 );
    }

    //default timing seting 10/20 MHz
    if(FLASH_INTERFACE(gulFlashVendorNum) == (FLASH_IF_SUP_DDR2 | FLASH_IF_SUP_TOGGLE))
    {
        FR_G_CFG_REG32_W(FR_PHY_TIME0, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME1, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME2, 0x00020200 );
        FR_G_CFG_REG32_W(FR_PHY_TIME3, 0x00000200 );
        FR_G_CFG_REG32_W(FR_PHY_TIME4, 0x0 );
    }
    else if((FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX) && (gubNandDefaultMode))
    {
        FR_G_CFG_REG32_W(FR_PHY_TIME0, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME1, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME2, 0x00020200 );
        FR_G_CFG_REG32_W(FR_PHY_TIME3, 0x00000200 );
        FR_G_CFG_REG32_W(FR_PHY_TIME4, 0x0 );
    }
    else if( (gubNandDefaultMode) && (0 == vdd_1v8_en) &&
             (( FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON ) || (FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
              || (FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC )) )
    {
        FR_G_CFG_REG32_W(FR_PHY_TIME0, 0x0100 );
        FR_G_CFG_REG32_W(FR_PHY_TIME1, 0x01050000 );
        FR_G_CFG_REG32_W(FR_PHY_TIME2, 0x010100 );
        FR_G_CFG_REG32_W(FR_PHY_TIME3, 0x0601 );
        FR_G_CFG_REG32_W(FR_PHY_TIME4, 0x06000000 );
    }
    else
    {
        FR_G_CFG_REG32_W(FR_PHY_TIME0, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME1, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME2, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME3, 0x0 );
        FR_G_CFG_REG32_W(FR_PHY_TIME4, 0x0 );
    }

    //set cemapping
    for(i = 0; i < 16; i++)
    {
        FC_TOP_REG(FR_PARSER_TO_LOGIC_CMDBUF_MAP0 + i * 4) = _MEM32(CONFIG_BASE_VA_ADDR +
                CONFIG_FC_DIE_MAPPING_CE + i * 4);
    }
    // set CE command buffer in L2MEM
    for(i = 0; i < CH_NUM_MAX * CMD_FIFO_NUM_PER_CH; i++)
    {
        FC_CMD_BUF_REG(FR_CMDQ_ADDR_CE0 + i * FR_CMDQ_CE_SIZE) = FC_CMD_FIFO_BASE_PHY + i * FC_CMDQ_SIZE;
    }
#if defined(RL6577_FPGA)||defined(RL6577_VA)||defined(RL6643_FPGA)||defined(RL6643_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
    if(NandPara.ubLunNumPerCE == 2)
    {
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP0, (~(0xffffff01)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP1, (~(0xffffff02)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP2, (~(0xffffff04)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP3, (~(0xffffff08)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP4, (~(0xffffff01)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP5, (~(0xffffff02)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP6, (~(0xffffff04)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP7, (~(0xffffff08)) );
    }
    else if(NandPara.ubLunNumPerCE == 4)
    {
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP0, (~(0xffffff01)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP1, (~(0xffffff02)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP2, (~(0xffffff01)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP3, (~(0xffffff02)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP4, (~(0xffffff01)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP5, (~(0xffffff02)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP6, (~(0xffffff01)) );
        FR_G_CFG_REG32_W(FR_PARSER_TO_CE_MAP7, (~(0xffffff02)) );
    }
    if (NandPara.ubLunNumPerCE > 1)
    {
        for (i = 0; i < CMD_FIFO_NUM_PER_CH; i++)
        {
            printk("FR_PARSER_TO_CE_MAP%d: %x\r\n", i, FR_G_CFG_REG32(FR_PARSER_TO_CE_MAP0 + (i * 4)));
        }

        FR_G_CFG_REG32_W(FR_CMD_INDEX_SPECIAL_RD, (g_parser.index.MultiLunReadInsert + 1));
        FR_G_CFG_REG32_W(FR_CMD_INDEX_SPECIAL_PG, (g_parser.index.MultiLunWriteInsert + 1));
        FR_G_CFG_REG32_W(FR_MTL_FBD_EN, 0x7);
    }

    FC_TOP_REG(FR_CMPQ_BANK_ADDR) = FC_BANK_CMP_FIFO_BASE_PHY;

    for(i = 0; i < 0x400; i += 4)
    {
        _REG32(FC_BANK_CMP_FIFO_BASE + i) = 0;
    }
#endif

    // init completion queue base address(0x8021a000)
    FC_TOP_REG(FR_CMPQ_ADDR) = FC_CMP_FIFO_BASE_PHY;

    for(i = 0; i < 0x1000; i += 4)
    {
        _REG32(FC_CMD_FIFO_BASE + i) = 0;
    }

    // reset completion fifo to zero, 64byte
    for(i = 0; i < 0x400; i += 4)
    {
        _REG32(FC_CMP_FIFO_BASE + i) = 0;
    }

    // cache index number
    for (i = 0; i < 16; i++)
    {
        FR_G_CFG_REG32_W(FR_CACHE_INDEX_NUM0 + (i * 4), i );
    }
    FR_G_CFG_REG32_W(FR_CACHE_INDEX_NUM0, 16);// Since we have no zero length of cache

    // need to give twice value
    FR_G_CFG_REG32_W(FR_LBN_SIZE, 0 );// the default LBN size=4KB
    FC_TOP_REG(FR_LBN_SIZE) = 0;// the default LBN size=4KB


    //FC_TOP_REG(FC_WRDATA_SYNC_EN) = 0; //Disable write data transfer done after last data send to bus
    //FC_TOP_REG(FR_ECC_ERROUT_EN) = 1;// even ecc error happened, still output error data to dram
    FR_G_CFG_REG32_W(FR_ECC_INFO_OUT_EN, 0);// enable transmit all ecc info to completion fifo

    //FC_TOP_REG(FR_ECC_CFG0) = 0x0;//ECC mode from config file

    FR_G_CFG_REG32_W(FR_FULL_PAGE_FILL, ((0 << 31) | (0)) );
    FR_G_CFG_REG32_W(FR_ECC_THV, gubCacheErrbitLimit);

    //FC_TOP_REG(FR_LBN_UNC_CHECK_CS) = 0x00;//check LBN

    // Sequencer auto insert enable
    FR_G_CFG_REG32_W(FR_SEQ_CTRL, 5);// default is 1: enable, 0: disable
    FR_G_CFG_REG32_W(FR_SEQ_WDT_CFG, 0xffffffff );

    llfprintk("FR_SCR_CTRL %x\r\n", FC_TOP_REG(FR_SCR_CTRL));

    FC_TOP_REG(FR_LBN_UNC_CHECK_CS) = 0x255;//check UNC&LBN
#if !(defined(RTS5771_FPGA)||defined(RTS5771_VA))
    FC_TOP_REG(FR_CRC_EN) = 0x1; //1:crc_enable(defult) 0:crc_disable
#endif
    //Write all FF error info to fc completion enable 1:enable 0:disable(defult)
    FR_G_CFG_REG32_W(FR_ALLFF_ERROUT_EN, 0x1);

#ifdef _DISABLE_SUSPEND
    FR_G_CFG_REG32_W(FR_PAR_PHASE_SUS_DIS, 0x1 );// 1 disable suspend
#else
    FR_G_CFG_REG32_W(FR_PAR_PHASE_SUS_DIS, 0x0 );// o enable suspend
#endif

#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
    FR_G_CFG_REG32_W(FR_PAR_CFG, (0x5 << 8));
#else
    FR_G_CFG_REG32_W(FR_PAR_CFG, (0xa << 8)); //default 10MHz
#endif

    FR_G_CFG_REG32_W(FR_PAR_CFG, (FR_CONFIG_CH(FR_PAR_CFG,
                                  gubStartCH) & (~(1 << 24))));//Disable read remapping
#if defined(RL6577_FPGA)||defined(RL6577_VA)||defined(RL6643_FPGA)||defined(RL6643_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
    //Rona:fix later,xor related function need to be verified later.
    FC_TOP_REG(FC_XOR_LOAD_CFG) = 0; //xor buf index = 0
    FC_TOP_REG(FC_XOR_CFG) = 0; //xor buf size = 16k
    FR_G_CFG_REG32_W(FR_XOR_BUF_SIZE, 7 );//xor buf entry size
    FC_TOP_REG(FC_XOR_RD_MODE) = 3; //xor read 3+1mode
    for( i = 0; i < 16; i++ )
        FC_TOP_REG(FC_XOR_WR_MODE_0 + i * 4) = 0x00300c03; ////xor write 3+1 mode

    FC_TOP_REG(FR_CMPQ_HEAD_SIZE_CONFIG) = 0x1b; //cmp depth:256
#endif
    FR_G_CFG_REG32_W(FR_PHY_SDR_CFG, (FR_CONFIG_CH(FR_PHY_SDR_CFG,
                                      gubStartCH) & 0x87ff));//clear bit11~14
    //pgm busy time unit:10us,ers busy time unit:100us,bit11~12:1,bit13~14:1
    FR_G_CFG_REG32_W(FR_PHY_SDR_CFG, (FR_CONFIG_CH(FR_PHY_SDR_CFG, gubStartCH) | 0x2800));

    FC_TOP_REG(FR_MAC_CFG) = 0x20;
    FC_TOP_REG(FC_BIU_RCMD_OSDIS) = 0x0;
    FC_TOP_REG(FR_RCBIU_TIMER_CG) = 0x80040;


    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_0, 0xe0e0);

#ifdef SLC_CACHE_PROGRAM_EN
    //Default value is 0xe0e0, so no need to write "#else"
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_1, 0xc0c0);
#endif
#ifdef SLC_CACHE_READ_EN
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_2, 0xc0c0);
#endif

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
    {
        FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0400);
    }
    else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
        FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0100);
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) && (gubNandFlashType == 3))
    {
        FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0x0);
        FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0400);
    }

    //NEW TABLE: For error check modify
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
#ifdef SLC_CACHE_PROGRAM_EN
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_1, 0xc0c0);
#else
    //Default value of FR_PAR_READY_CHECK_1 is 0xe0e0
#endif
#ifdef SLC_CACHE_READ_EN
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_2, 0xc0c0);
#else
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_2, 0xe0e0);
#endif
#ifdef SNAP_READ_EN
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_3, 0xc0c0);
#else
    FR_G_CFG_REG32_W(FR_PAR_READY_CHECK_3, 0xe0e0);
#endif
#if defined(FTL_B47R) ||\
    defined(FTL_N38A)
    //Default value of FR_PAR_ERROR_CHECK_0 is 0x0100 (For TLC program)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0x0000);//(For read)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0100);//(SLC program/erase)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0100);//(SLC program)
#elif defined(FTL_SANDISK_BICS3) || defined(FTL_SANDISK_BICS4) || defined(FTL_SANDISK_BICS5)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0x0000);//(For read)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0400);//(SLC program/erase)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0400);//(SLC program)
#endif
#endif

#ifdef LLF_ALONE
#if defined(FTL_TSB_BICS4) || defined(FTL_TSB_BICS5)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0400);
#endif
    CalculationRemapOffset();
#endif

//NEW TABLE: For error check modify
#if defined(FTL_SSV7)
    //check error bit for status polling
    //FR_PAR_ERROR_CHECK_0 default as 0x0100
    //don't check flash error bit of all read operation and before array operation polling
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0);
    //check error bit of all SLC erase/write operation
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0);
    //check error bit of all TLC erase/write operation
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0100);
#elif defined(FTL_SSV2) || defined(FTL_SSV4) || defined(FTL_SSV5) || defined(FTL_SSV6)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0);
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0);
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_3, 0x0100);
#elif defined(FTL_B58R) || defined(FTL_N38A) || defined(FTL_Q5171A) || defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N48R)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0);
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0100);
#elif defined(FTL_N38B)
    FR_G_CFG_REG32(FR_PAR_ERROR_CHECK_0) = 0x0100;
    FR_G_CFG_REG32(FR_PAR_ERROR_CHECK_1) = 0;
    FR_G_CFG_REG32(FR_PAR_ERROR_CHECK_2) = 0x0100;
#elif defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0);
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0100);
#elif defined(RL6643_VA) && defined(FTL_B17A) && defined(MULTI_LUN) && !defined(SIX_ADDR_CYCLE)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0);
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0100);
#elif defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
    //check error bit of all TLC erase/write operation
    //FR_PAR_ERROR_CHECK_0 default as 0x0100
    //don't check flash error bit of all read operation and before array operation polling
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0);
    //check error bit of all SLC erase/write operation
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_2, 0x0100);
#endif

#if defined(RL6643_VA) || defined(RL6531_VB)
#ifdef RL6643_VA
    Init_ocd_odt_config();
    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
    if(IS_6855_VERSION_TAG)
    {
        FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, 0x11e);
        //set delay chain mode
        FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 28);//set bit28~31
    }
    else
    {
        FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, 0x124);
    }
    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1) & (~ (0xc000));//clear bit14/15
    reg_v |= (0x1 << 14);//set bit14
    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
#else
    FR_G_CTRL_REG32_W(FR_PAD_CFG0, 0xd0);
#endif

    for(i = 0; i < CE_NUM_MAX; i++)
    {
        if(IS_6855_VERSION_TAG)
        {
            //before cfg delay chain,need to disable output
            FR_G_CTRL_REG32(FR_PHY_STATE) |= (0xf << 24);
            FR_G_CTRL_REG32(FR_PHY_DELAY_CFG0 + i * 4) = 0x00000052;
            FR_G_CTRL_REG32(FR_PHY_STATE) &= (~(0xf << 24));//enable output
        }
        else
        {
            FR_G_CTRL_REG32_W(FR_PHY_DELAY_CFG0 + i * 4, 0x44 );
        }
    }
#endif

#ifdef RL6531_VB
    FcCmdBypass.bits.aes_bypass = 1;
#else
    FcCmdBypass.bits.aes_bypass = 0;
#endif
    FcCmdBypass.bits.ecc_bypass = 0;
    FcCmdBypass.bits.scramble_bypass = 0;
}

U32 llfinitConfigBE(PVENDOR_CMD_RESPONSE pVendorCmdResponse)
{
    U8 i = 0;
    U8 *Manufacturer = (U8 *)TEMP_HBUF_ADDR;
    U8 test_idx = 0;
    U32 cmp;
    U32 ulmode;
    U8 flag_count = 0, flag_bad = 0;
    U32 ret = ERR_OK;
    U8 LunNum, ubBankNum;
#ifdef AUTO_DETECT_DIE
    U8 ubLun;
#endif

    //Config BE
    _REG32(TEMP_HBUF_ADDR) = 0x0;
    cache_dwbinval(TEMP_HBUF_ADDR);
    cache_dummy_update_read();

    llfInitErrorMessage();

    ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    ubBankNum = UnbalancedGetBankNum();
    // reset all CHs and CEs one by one here,if lun num>1,reset all luns in ce
    for(i = 0; i < ubBankNum; i++)
    {
        gul_FW_TAG = llfBETagSetting(TAG_RESET, i);
        FCReset(ulmode, i);
        FcBusyWait1ms(5);
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret != ERR_OK)
        {
            flag_count++;
            flag_bad++;
            llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: bank = %d  ret = %x\r\n", i, ret);
            AddErrorMessage(i, 0, ERR_FIO_TIMEOUT);
        }
        else
        {
            gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, i);
#ifdef AUTO_DETECT_DIE
            ubLun = CalcRealLun(i, 0);
            FCStatusPollingLun(ulmode, i, ubLun);
#else
            FCStatusPolling(ulmode, i);
#endif
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            flag_count++;
            if(ret != ERR_OK)
            {
                flag_bad++;
                llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: bank = %d  ret = %x\r\n", i, ret);
                AddErrorMessage(i, 0, ERR_FIO_TIMEOUT);
            }
            else
            {
                if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                {
                    flag_bad++;
                    llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: bank = %d cmp = %x\r\n", i, cmp);
                    AddErrorMessage(i, 0, ERR_FC_CMP);
                }
            }
        }
    }
    if((flag_bad != 0) || (flag_count != ubBankNum))
    {
        return ERR_RESET_FLASH;
    }

    if(!vdd_1v8_en)
    {
        if(FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
        {
            if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV5)
                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_H3DTV4))
            {
                llfprintk("Hynix switch to 1.2V\r\n");
                flag_bad = 0;
                flag_count = 0;
                for(i = 0; i < ubBankNum; i++)
                {
                    ret = FCHynixTo12VByBank(i);
                    flag_count++;
                    if(ret != ERR_OK)
                    {
                        flag_bad++;
                        llfDbgPrintk(ALWAYS_MSG, "FC set feature error: bank = %d  ret = %x\r\n", i, ret);
                        //llfAddErrorMessage(i, 0, ERR_SET_FEATURE);
                    }
                }
                if((flag_bad != 0) || (flag_count != ubBankNum))
                {
                    return ERR_RESET_FLASH;
                }
            }
        }
    }

#if defined(RL6577_FPGA) || defined(RL6643_FPGA) || defined(RTS5771_FPGA) || defined(RL6643_VA)
    if(gubNandDefaultMode == ONFI_SDR )
    {
        TsbForceSetSdrMode();
    }
#endif
    //read id one by one
    flag_count = 0;
    flag_bad = 0;
    for(i = 0; i < ubBankNum; i++)
    {
        gul_FW_TAG = llfBETagSetting(TAG_READ_ID, i);
        if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
                && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                    || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
        {
#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
            FCReadID(ulmode, i, 0x20, 7, TEMP_HBUF_PHY_ADDR, 0x10);
#else
            FCReadID(ulmode, i, 0x20, 5, TEMP_HBUF_PHY_ADDR, 0x10);
#endif
        }
        else
        {
#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
            FCReadID(ulmode, i, 0x40, 7, TEMP_HBUF_PHY_ADDR, 0x10);
#else
            FCReadID(ulmode, i, 0x40, 5, TEMP_HBUF_PHY_ADDR, 0x10);
#endif
        }
        FcBusyWait1ms(1);
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        flag_count++;
        if(ret == ERR_OK)
        {
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                flag_bad++;
                llfDbgPrintk(ALWAYS_MSG, "FC read id error: bank %d\r\n", i);
                AddErrorMessage(i, 0, ERR_READ_ID);
            }
        }
        else
        {
            flag_bad++;
            llfDbgPrintk(ALWAYS_MSG, "FC read id timeout : bank %d\r\n", i);
            AddErrorMessage(i, 0, ERR_FIO_TIMEOUT);
        }

        if((FR_CONFIG_CH(FR_FC_MODE, gubStartCH) == ONFI_DDR2_TOGGLE) ||
                (FR_CONFIG_CH(FR_FC_MODE, gubStartCH) == ONFI_DDR))
        {
            if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
                    && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
            {
                if((Manufacturer[0] == 0x4F) && (Manufacturer[2] == 0x4E)
                        && (Manufacturer[4] == 0x46) && (Manufacturer[6] == 0x49))
                {
                    test_idx++;
                }
                else
                {
                    llfprintk("FC read ID mismatch bank=%d, %x %x %x %x\r\n", i,
                              Manufacturer[0], Manufacturer[2], Manufacturer[4], Manufacturer[6]);
                    AddErrorMessage(i, 0, ERR_READ_ID);
                    flag_bad++;
                }
            }
            else
            {
                if((Manufacturer[0] == 0x4a) && (Manufacturer[2] == 0x45) && (Manufacturer[4] == 0x44)
                        && (Manufacturer[6] == 0x45))
                {
                    test_idx++;
                }
                else
                {
                    llfprintk("FC read ID mismatch bank=%d, %x %x %x %x\r\n", i,
                              Manufacturer[0], Manufacturer[2], Manufacturer[4], Manufacturer[6]);
                    AddErrorMessage(i, 0, ERR_READ_ID);
                    flag_bad++;
                }
            }
        }
        else
        {
            if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
                    && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                        || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
            {
                if((Manufacturer[0] == 0x4F) && (Manufacturer[1] == 0x4E) && (Manufacturer[2] == 0x46)
                        && (Manufacturer[3] == 0x49))
                {
                    test_idx++;
                }
                else
                {
                    flag_bad++;
                    llfprintk("FC read ID mismatch bank=%d, %x %x %x %x %x %x \r\n", i,
                              Manufacturer[0], Manufacturer[1], Manufacturer[2], Manufacturer[3],
                              Manufacturer[4], Manufacturer[5]);
                    AddErrorMessage(i, 0, ERR_READ_ID);
                }
            }
            else
            {
                if((Manufacturer[0] == 0x4a) && (Manufacturer[1] == 0x45) && (Manufacturer[2] == 0x44)
                        && (Manufacturer[3] == 0x45) && (Manufacturer[4] == 0x43))
                {
                    test_idx++;
                }
                else
                {
                    flag_bad++;
                    llfprintk("FC read ID mismatch bank=%d, %x %x %x %x %x %x \r\n", i,
                              Manufacturer[0], Manufacturer[1], Manufacturer[2], Manufacturer[3],
                              Manufacturer[4], Manufacturer[5]);
                    AddErrorMessage(i, 0, ERR_READ_ID);
                }
            }
        }

        _REG32(TEMP_HBUF_ADDR) = 0x0;
        cache_dwbinval(TEMP_HBUF_ADDR);
        cache_dummy_update_read();
    }

    if((flag_bad != 0) || (test_idx != ubBankNum))
    {
        return ERR_READ_ID;
    }

    gul_FW_TAG = llfBETagSetting(TAG_READ_ID, 0);
    FCReadID(ulmode, 0, 0x00, 7, TEMP_HBUF_PHY_ADDR, 0x10);
    FcBusyWait1ms(1);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "FC read id error: ret = %x bank0\r\n", ret);
        AddErrorMessage(0, 0, ERR_FIO_TIMEOUT);
        return ERR_READ_ID;
    }
    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "FC read id error: cmp = %x bank0\r\n", cmp);
        AddErrorMessage(0, 0, ERR_FC_CMP);
        return ERR_READ_ID;
    }

    if(ulmode == FLASH_MODE_DDR2)
    {
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_0) = Manufacturer[0];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_1) = Manufacturer[2];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_2) = Manufacturer[4];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_3) = Manufacturer[6];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_4) = Manufacturer[8];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_5) = Manufacturer[10];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_6) = Manufacturer[12];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_7) = Manufacturer[14];
    }
    else
    {
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_0) = Manufacturer[0];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_1) = Manufacturer[1];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_2) = Manufacturer[2];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_3) = Manufacturer[3];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_4) = Manufacturer[4];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_5) = Manufacturer[5];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_6) = Manufacturer[6];
        _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_7) = Manufacturer[7];
    }

    //according to the Nand feature decide FC address cyclenum;
    if((gubNandFlashVendor == FLASH_VENDOR_MiCRON ) || (gubNandFlashVendor == FLASH_VENDOR_INTEL))
    {
        // last 2 bits: 00 for 1LUN ;01 for 2LUN ;10 for 4lun ;11 for 8lun
        LunNum = _MEM08(SBLK_ADDR + SBLK_OFFSET_NAND_ID_2) & 0x03;

        if(((gubNandFlashVendor == FLASH_VENDOR_INTEL)
                && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
                && (LunNum >= 1))
                || ((gubNandFlashVendor == FLASH_VENDOR_MiCRON)
                    && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28)
                    && (LunNum >= 1)))
        {
            // 2-lun N18 needs 6 address cycle
            gubFcAddressCycleNum = 6;
        }
        else if (LunNum >= 2)
        {
            gubFcAddressCycleNum = 6;
        }
        else if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N48R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B58R
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38B
                || FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_Q5171A)
        {
            gubFcAddressCycleNum = 6;
        }
        else
        {
            gubFcAddressCycleNum = 5;
        }
    }
    else if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
    {
        //llfprintk("YMTC gulFlashVendorNum= %x, gubFcAddressCycleNum = %d\r\n", gulFlashVendorNum, gubFcAddressCycleNum);
        gubFcAddressCycleNum = 6;
    }
    else if((FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
            && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV7_512Gb))
    {
        gubFcAddressCycleNum = 6;
    }
    else
    {
        // SanDisk, Toshiba, Hynix and Samsung are 5 address cycle
        gubFcAddressCycleNum = 5;
    }

    if(gubFcAddressCycleNum == 6 )
    {
        FR_G_CFG_REG32_W(FR_MTL_FBD_EN, 0x17);//bit4:1,for 4 row address
    }

    llfprintk("llf gubFcAddressCycleNum:%d\r\n", gubFcAddressCycleNum);

#if defined(MULTI_LUN)
    if(_REG08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LUN_NUM_PER_CE) < 2)
        llfprintk("[WARN] Invalid LunNumPerCE:%d which is expected to be greater than 2\r\n",
                  _REG08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LUN_NUM_PER_CE));
#else
    if(_REG08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LUN_NUM_PER_CE) != 1)
        llfprintk("[WARN] Invalid LunNumPerCE:%d which is expected to be 1\r\n",
                  _REG08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LUN_NUM_PER_CE));
#endif

    return ret;
}

// Since reset takes no effect for Toshiba Flash interface,
// we need to do this function.
U32 TsbForceSetSdrMode()
{

    // TODO:: Remove TsbForceSetSdrMode llf by eric

#if defined(RL6577_FPGA) || defined(RL6643_FPGA) || defined(RTS5771_FPGA)
    U8 bank, ubBankNum;
    U32 ulmode;
    U32 cmp, ret = ERR_OK;

    ubBankNum = UnbalancedGetBankNum();
    if (((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)) &&
            (FLASH_INTERFACE(gulFlashVendorNum) == (FLASH_IF_SUP_SDR | FLASH_IF_SUP_DDR2 | FLASH_IF_SUP_TOGGLE))
       )
    {
        // We can not know what interface now, so do set feature for every interface
        //FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_DDR2 );
        for(bank = 0; bank < ubBankNum; bank++)
        {
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank);
            FCSetfeature(ulmode, bank, 0x80, 1);
            FcBusyWait1ms(1);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK)
            {
                //reset all FC
                FcResetAll();
                llfFcInitReg();
                SetParserSequencerIndex(PARSER_INDEX_ADDR, SEQUENCER_INDEX_ADDR, AUTO_INSERT_INDEX_ADDR);
                FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_SDR );
                //FR_G_CFG_REG32_W(FR_INTERFACE_SEL, FLASH_IF_TOGGLE );

                ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
                gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank);
                FCSetfeature(ulmode, bank, 0x80, 1);
                FcBusyWait1ms(1);
                FCCompletionPolling(&cmp, gul_FW_TAG);
            }
        }
        FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_SDR );
    }
#else

#ifdef RL6643_VA
    U8 bank, ubBankNum;
    U32 ulmode;
    U32 cmp, ret = ERR_OK;
    U32 feature_va = GETFEATURE_BY_BANK_UC;
    U32 feature_pha = GETFEATURE_BY_BANK_PHY;
    U32 ulFailBank = 0;

    ubBankNum = UnbalancedGetBankNum();
    if ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) &&
            (FLASH_INTERFACE(gulFlashVendorNum) == (FLASH_IF_SUP_SDR | FLASH_IF_SUP_DDR2 |
                    FLASH_IF_SUP_TOGGLE)))
    {
        DBGPRINTK(ALWAYS_MSG, "Force Set SDR mode\r\n");

        U32 orig_data, new_data;
        orig_data = 0;
        new_data = 0;
        for(bank = 0; bank < ubBankNum; bank++)
        {
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank);

            FCGetfeature(ulmode, bank, 0x80, feature_pha, 0x10);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                DBGPRINTK(ALWAYS_MSG, "bank %d,%x,%d,%s\r\n", bank, cmp, __LINE__, __FILE__);
                ulFailBank |= (1 << bank);
                continue;
            }
            else
            {
                if((_MEM32(feature_va) & 0xFF) == 1)
                {
                    DBGPRINTK(ALWAYS_MSG, "Bank %d is SDR mode\r\n", bank);
                    continue;
                }
            }

            FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_DDR2);
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

            if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb) ||
                    (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb) ||
                    (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT) ||
                    (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC))
            {
                FCGetfeature(ulmode, bank, 0x02, feature_pha, 0x10);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                {
                    DBGPRINTK(ALWAYS_MSG, "Get Toggle Mode-Specific Setting fail bank %d,%x,%d,%s\r\n", bank, cmp,
                              __LINE__, __FILE__);
                    ulFailBank |= (1 << bank);
                }
                else
                {
                    orig_data = _MEM32(feature_va);
                }

                if(!(orig_data & 0x2))
                {
                    DBGPRINTK(ALWAYS_MSG, "Enable DQSn orig 0x%x\r\n", orig_data);
                    new_data = orig_data | 0x2;
                    FCSetfeature(ulmode, bank, 0x02, new_data);
                    FcBusyWait1ms(1);
                    ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                    if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                    {
                        DBGPRINTK(ALWAYS_MSG, "bank:%d enable DQSn err:%x\r\n", bank, cmp);
                        ulFailBank |= (1 << bank);
                    }

                    fc_diff_setting(ulmode, 0, gubFCDiffEnable);
                }
            }

            FCSetfeature(ulmode, bank, 0x80, 1);
            FcBusyWait1ms(1);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                DBGPRINTK(ALWAYS_MSG, "bank:%d Flash reset command Use SDR err:%x\r\n", bank, cmp);
                ulFailBank |= (1 << bank);
            }

            FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_SDR);
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

            FCGetfeature(ulmode, bank, 0x80, feature_pha, 0x10);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                DBGPRINTK(ALWAYS_MSG, "bank %d,%x,%d,%s\r\n", bank, cmp, __LINE__, __FILE__);
                ulFailBank |= (1 << bank);
            }
            else
            {
                if((_MEM32(feature_va) & 0xFF) != 1)
                {
                    DBGPRINTK(ALWAYS_MSG, " Change Intel Fail%x \r\n", _MEM32(feature_va));
                    ulFailBank |= (1 << bank);
                }
            }

            if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb) ||
                    (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb) ||
                    (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT) ||
                    (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC))
            {
                if(!(orig_data & 0x2))
                {
                    DBGPRINTK(ALWAYS_MSG, "Toggle Mode-Specific Setting set back\r\n");
                    FCSetfeature(ulmode, bank, 0x02, orig_data);
                    FcBusyWait1ms(1);
                    ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                    if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                    {
                        DBGPRINTK(ALWAYS_MSG, "bank:%d set back Toggle Mode-Specific Setting err:%x\r\n", bank, cmp);
                        ulFailBank |= (1 << bank);
                    }

                    fc_diff_setting(ulmode, 0, 0);
                }
            }
        }
        FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_SDR );
        for(bank = 0; bank < ubBankNum; bank++)
        {
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            gul_FW_TAG = llfBETagSetting(TAG_RESET, bank);
            FCReset(ulmode, bank);
            FcBusyWait1ms(5);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK)
            {
                printk(" Force Set SDR mode reset fail\r\n");
                ulFailBank |= (1 << bank);
            }
        }

        if(ulFailBank)
        {
            printk("reset fail SDR bank %x\r\n", ulFailBank);
            //reset all FC
            FcResetAll();
            SetParserSequencerTable(PARSER_TABLE_VA_ADDR, SEQUENCER_TABLE_VA_ADDR);
            SetParserSequencerIndex(PARSER_INDEX_ADDR, SEQUENCER_INDEX_ADDR, AUTO_INSERT_INDEX_ADDR);
            FcCopySeedFromROM();
            HlcParserIndexInit();
            llfFcInitReg();
            Change_ldpc(gubECC_CFG);

            for(bank = 0; bank < ubBankNum; bank++)
            {
                FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_SDR );

                ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
                FCReset(ulmode, bank);
                FcBusyWait1ms(5);
                FCCompletionPolling(&cmp, gul_FW_TAG);

                gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank);
                FCSetfeature(ulmode, bank, 0x80, 1);
                FcBusyWait1ms(1);
                FCCompletionPolling(&cmp, gul_FW_TAG);
            }
        }
    }

#endif

#endif
    return ERR_OK;
}

void SetDefaultTimingInterface()
{
    // Set default FC interface and timing

    //config channel default timing
    FR_G_CTRL_REG32_W(FR_CHN_DELAY_0, 0x20206430 );
    FR_G_CTRL_REG32_W(FR_CHN_DELAY_1, 0x0a060073 );

    //config global default timing
    FR_G_CFG_REG32_W(FR_GLB_DELAY_0, 0x03010a02 );
    FR_G_CFG_REG32_W(FR_GLB_DELAY_1, 0x00000006 );
    FR_G_CFG_REG32_W(FR_PHY_TIME0, 0x00000000 );
    FR_G_CFG_REG32_W(FR_PHY_TIME1, 0x00000000 );

    if(gubNandDefaultMode == 0) //SDR
    {
        FR_G_CFG_REG32_W(FR_PHY_TIME2, 0x00000000 );
        FR_G_CFG_REG32_W(FR_PHY_TIME3, 0x00000000 );
    }
    else //DDR
    {
        FR_G_CFG_REG32_W(FR_PHY_TIME2, 0x00020200 );
        FR_G_CFG_REG32_W(FR_PHY_TIME3, 0x00000200 );
    }

    FR_G_CFG_REG32_W(FR_PHY_TIME4, 0x00000000 );
    FR_G_CFG_REG32_W(FR_PHY_TIME5, 0x00000000 );
    FR_G_CFG_REG32_W(FR_PHY_TIME6, 0x00000000 );
    FR_G_CFG_REG32_W(FR_PHY_TIME7, 0x00000000 );

    //config FC interface and mode
    if(gubNandDefaultMode == 0)
    {
        FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_SDR );
    }
    else
    {
        FR_G_CFG_REG32_W(FR_FC_MODE, FLASH_MODE_DDR2 );
    }
}

U32 llffioDefectRead(U32 ubIFType, U8 ubBankNo, U16 block, U16 page, U16 offset)
{
    U32 ret;
    U32 cmp;
    U32 ecc_len;
    U8 lun_no;

    lun_no = ubBankNo / NandPara.ubBankNumPerLun;
    ecc_len  = FR_CONFIG_CH(FR_ECC_CFG, gubStartCH) & 0xffff;

    memset((void*)TEMP_BUF_ADDR + MAX_DB_NUM_SIZE, 0x0, 0x20);
    cache_area_dwbinval(TEMP_BUF_ADDR + MAX_DB_NUM_SIZE, 0x20);
    cache_dummy_update_read();

    gul_FW_TAG = llfBETagSetting(TAG_DEFECT_READ, ubBankNo);

    cache_area_dinval((TEMP_BUF_ADDR + MAX_DB_NUM_SIZE), 0x20);
    cache_dummy_update_read();
    FCReadRedundant(ubIFType, ubBankNo, lun_no, block, page,
                    TEMP_BUF_PHY_ADDR + MAX_DB_NUM_SIZE,
                    LLF_DATA_SIZE + 8 * (10 + ecc_len), 0);

    FCCompletionPolling(&cmp, (gul_FW_TAG));
    if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
    {
        ret = ERR_OK;
    }
    else
    {
        ret = ERR_DEFECT_BLK_READ;
    }

    return ret;
}

void llfInitParameters()
{
    NandPara.uwBlockNumPerCE = NandPara.uwBlockNumPerLun * NandPara.ubLunNumPerCE;
    NandPara.uwMpBlockNumPerLun = (NandPara.uwBlockNumPerLun / NandPara.ubPlaneNumPerLun);
    NandPara.uwLbnNumPerMpPage = ((NandPara.ubSectorNumPerPage >> gubCacheSectorNumPerLbnShift)
                                  * NandPara.ubPlaneNumPerLun);
    NandPara.ubLunNumPerCE = (1 << NandPara.ubLunNumPerCEShift);

    //NandPara.ubLbnNumPerMpPageShift = gubCacheSectorNumPerLbnShift;

    guwCacheSectorNumPerLbn = (1 << gubCacheSectorNumPerLbnShift); // user page size
    NandPara.ulLastMpPageByteNum = NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun;

    gubLbnNumPerPlane = NandPara.uwLbnNumPerMpPage / NandPara.ubPlaneNumPerLun;
    guwLLFSnapshotGroupEnd = 32 / NandPara.ubBankNum;

#ifdef EXTEND_STATIC_DBT
    gubNeedRebuildRemap = 0;
#endif

    ASSERT_LLF((NandPara.ulLastMpPageByteNum & 0x000001ff) == 0);
    ASSERT_LLF(NandPara.ubBankNum <= (1 << NandPara.ubBankNumShift));
    //ASSERT_LLF( NandPara.uwLbnNumPerMpPage == (1 << NandPara.ubLbnNumPerMpPageShift));
}

#if defined(RL6643_VA)
static U8 findFittedRaidPageNum(U8 ProgramCount, U8 ubSLCMode)
{
    if(ProgramCount == 0)
    {
        DBGPRINTK(ALWAYS_MSG, "[Error] ProgramCount is 0\r\n");
        ASSERT_LLF(0);
    }

    U8 RaidPageNum = 1;
    U16 uwPageNumPerBlock = (ubSLCMode == BS_SLC_MODE) ? NandPara.uwSLCPageNumPerBlock :
                            NandPara.uwPageNumPerBlock;
#ifdef COPYBACK_ALGORITHM
    switch(FLASH_SPCIES(gulFlashVendorNum))
    {
    case IS_SLC:
        ProgramCount = 1;
        break;
    case IS_MLC:
    case IS_3DMLC:
        ProgramCount = 2;
        break;
    case IS_TLC:
    case IS_3DTLC:
        ProgramCount = 3;
        break;
    case IS_3DQLC:
        ProgramCount = 4;
        break;
    }
#endif
    const U8 PlaneNumPerSuperPage = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun;

    while(!(uwPageNumPerBlock % (RaidPageNum * ProgramCount) == 0
            && PlaneNumPerSuperPage * RaidPageNum >= 32) && (PlaneNumPerSuperPage * RaidPageNum <= 96))
    {
        RaidPageNum++;
    }

    if(PlaneNumPerSuperPage * RaidPageNum > 96)
    {
        return 0xFF;
    }

    return RaidPageNum;
}
#endif

U32 llfmpInitParameter(void)
{
    U32 i;
#if defined(RL6577_VA)
    U32 ulTemp64_l, ulTemp64_h, ulTemp32_OUI;
    U32 ullTemp64_WWN_L, ullTemp64_WWN_H;
#endif
#ifndef AUTO_DETECT_DIE
    U32 ulVendor, ulSerialNum;
    U8 ubLunInterleave;
#ifdef MTL_SUPPORT_NAND
    U8 ubExpectBankNum;
#endif

    ulVendor = FLASH_VENDOR(gulFlashVendorNum);
    ulSerialNum = FLASH_SERIAL_NUM(gulFlashVendorNum);
#endif

#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
    gulEraseFailSysblk = 0;
#endif
#ifdef IS_8K_PAGE
    gubHeaderlen = 6;
    L2pPara.ubHdr4BLenPerPage = FC_HEADER_LEN_PER_PAGE >> 1;
#else
    gubHeaderlen = 12;
    L2pPara.ubHdr4BLenPerPage = FC_HEADER_LEN_PER_PAGE;
#endif
    gfDBTInitDone = LLF_DBT_NONE;  // to remind LLF not initialize DBT yet
    gufLastEraseFlag = 0;
    gubLLFALLStep = STEP_CALIBRATE;
    gfSelfTestFlag = 0;
    gubIsSerialMultiLUN = 0;
    gubLLFSubStep.ubKStep = STEP_CALIBRATE_INIT;
    gubLLFSubStep.ubKSpeedCount = 2;
    gubLLFSubStep.ubKSpeedIndex = 2;
    gubLLFSubStep.ubKRx = 0;
    gubLLFSubStep.ubKBankIndex = 0;
    gubLLFSubStep.ubLlfStep = STEP_LLF_INHERIT;
    gubLLFSubStep.ubWriteConfigStep = STEP_WRITE_CONFIG_INIT;
    gubLLFSubStep.ubMstWriteConfigStep = STEP_WRITE_CONFIG_OVER;
#if defined(RL6577_VA) || defined(RTS5771_VA)
    gubThreePointKSetting = 3;
#else
    gubThreePointKSetting = 0;
#endif
    gubRdtImg = 0;
    gubCacheErrbitLimit = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_CHECK_CACHE_ERRBIT_LIMIT);
    if(gubCacheErrbitLimit == 0)
    {
        gubCacheErrbitLimit = 1;
    }
    llfprintk("gubCacheErrbitLimit:%x\r\n", gubCacheErrbitLimit);
    gulFlashVendorNum = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_NANDID_OFFSET);
    ulVendor = FLASH_VENDOR(gulFlashVendorNum);
    ulSerialNum = FLASH_SERIAL_NUM(gulFlashVendorNum);
    gubLLFMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_MODE);
    gubMpModeSelect = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_MP_MODE_SELECT);
    DebugPara.ulDebugLevel = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_DEBUG_LEVEL_SELECT);
#ifdef MST_MERGE
    gubLlfMSTMergeEnable = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_MST_MERGE);
    llfprintk("LlfMSTMergeEnable %d\r\n", gubLlfMSTMergeEnable);
#endif
    NandPara.ubChNum = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CHNUM_OFFSET);
    NandPara.ubChNumShift = (U8)cal_shift((U32)NandPara.ubChNum);
    NandPara.ubCENumPerCh = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CENUM_OFFSET);
    NandPara.ubBankNum = (U8)_MEM16(CONFIG_BASE_VA_ADDR + CONFIG_BANKNUM_OFFSET);
    NandPara.uwBlockNumPerLun = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_TOTAL_BLK_OFFSET);
    guwRealBlockNum = NandPara.uwBlockNumPerLun;
    NandPara.ubBlockNumPerLunShift = (U16)cal_shift((U32)NandPara.uwBlockNumPerLun);
    if(NandPara.uwBlockNumPerLun % (1 << NandPara.ubBlockNumPerLunShift))
    {
        NandPara.ubBlockNumPerLunShift++;
    }
#if defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX2Q) || defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
    gubRowAddrLunShift = NandPara.ubBlockNumPerLunShift + 1;
    printk("According to SPEC,NandPara.ubBlockNumPerLunShift add 1:%d\r\n", gubRowAddrLunShift);
#else
    gubRowAddrLunShift = NandPara.ubBlockNumPerLunShift;
#endif
    NandPara.ubLunNumPerCE = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LUN_NUM_OFFSET);
#if defined(RL6577_VA)
    gubZQKEn = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_ZQ_K_EN);
#endif
    gubRealLunNum = NandPara.ubLunNumPerCE;

    // multi-lun handle: extend or interleave

#ifdef AUTO_DETECT_DIE
    NandPara.ubLunNumPerCE = 1;
#else
    ubLunInterleave = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LUN_INTERLEAVE);

    if(NandPara.ubLunNumPerCE > 1)
    {
        if(ubLunInterleave)
        {
#ifdef MTL_SUPPORT_NAND
            // allow to do MTL
            ubExpectBankNum = (NandPara.ubChNum * NandPara.ubCENumPerCh * gubRealLunNum);
#ifdef RL6577_VA
            // 6577 can't support larger than 16 banks, not use BANK_NUM_MAX here
            if(ubExpectBankNum > 16)
            {
                printk("[WARN][MtLun] 6577 can't support banknum %d do MTL, change to extend\r\n", ubExpectBankNum);
                ubExpectBankNum = (NandPara.ubChNum * NandPara.ubCENumPerCh);
                ubLunInterleave = 0;
            }
#elif defined(RTS5771_FPGA) || defined(RTS5771_VA)
            if(ubExpectBankNum > BANK_NUM_MAX)
            {
                printk("[WARN][MtLun] 6817 can't support banknum %d do MTL, change to extend\r\n", ubExpectBankNum);
                ubExpectBankNum = (NandPara.ubChNum * NandPara.ubCENumPerCh);
                ubLunInterleave = 0;
            }
#endif
            if (NandPara.ubBankNum != ubExpectBankNum)
            {
                printk("[MtLun][WARN] Unbalance? %dx%dx%d Bk:%d,%d\r\n",
                       NandPara.ubChNum, NandPara.ubCENumPerCh, gubRealLunNum,
                       NandPara.ubBankNum, ubExpectBankNum);
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
#else
                NandPara.ubBankNum = ubExpectBankNum;
#endif
            }
#else       // current nand not support interleave
            printk("[WARN][MtLun] Current nand not support MTL, change to extend\r\n");
            ubLunInterleave = 0;
#endif
        }
        if(!ubLunInterleave)
        {
            gubIsSerialMultiLUN = 1;
            NandPara.uwBlockNumPerLun = NandPara.uwBlockNumPerLun * NandPara.ubLunNumPerCE;
            printk("[DBG] uwBlockNumPerLun %d\r\n", NandPara.uwBlockNumPerLun);
            NandPara.ubBlockNumPerLunShift = (U16)cal_shift((U32)NandPara.uwBlockNumPerLun);
            if((IS_INTEL == ulVendor) || (IS_MICRON == ulVendor) ||
                    (IS_HYNIX == ulVendor) || (IS_YMTC == ulVendor)|| (IS_SAMSUNG == ulVendor))
            {
                if(NandPara.uwBlockNumPerLun % (1 << NandPara.ubBlockNumPerLunShift))
                {
                    NandPara.ubBlockNumPerLunShift++;
                }
            }
            NandPara.ubLunNumPerCE = 1;
        }
        llfDbgPrintk(ALWAYS_MSG,
                     "[MtLun] Vendor %x NandType %x Lun:%d,%d Sft:%d,%d Blk:%d,%d Bk:%d,%d\r\n",
                     ulVendor, ulSerialNum,
                     NandPara.ubLunNumPerCE, gubRealLunNum,
                     NandPara.uwBlockNumPerLun, guwRealBlockNum,
                     NandPara.ubBlockNumPerLunShift, gubRowAddrLunShift,
                     NandPara.ubBankNum, NandPara.ubBankNumPerLun);
    }
#endif
    NandPara.ubBankNumShift = (U8)cal_shift((U32)NandPara.ubBankNum);
    if(NandPara.ubBankNum % (1 << NandPara.ubBankNumShift))
    {
        NandPara.ubBankNumShift++;
    }

    NandPara.ubBankNumPerLun = NandPara.ubBankNum / NandPara.ubLunNumPerCE;
    NandPara.ubLunNumPerCEShift = (U8)cal_shift((U32)NandPara.ubLunNumPerCE); // At least one bit
    NandPara.uwPageNumPerBlock = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_PAGE_NUM_OFFSET);
    if(gubNandFlashType == 1)
    {
        NandPara.uwPageNumPerBlock = NandPara.uwPageNumPerBlock / 3;
    }
    NandPara.ubPageNumPerBlockShift = (U8)cal_shift((U32)NandPara.uwPageNumPerBlock);
    if(NandPara.uwPageNumPerBlock % (1 << NandPara.ubPageNumPerBlockShift))
    {
        NandPara.ubPageNumPerBlockShift++;
    }

    if(gubNandFlashType == 0 || gubNandFlashType == 1)
    {
        NandPara.uwSLCPageNumPerBlock = NandPara.uwPageNumPerBlock;
    }
    else if (gubNandFlashType == 4)
    {
        NandPara.uwSLCPageNumPerBlock = NandPara.uwPageNumPerBlock / 4;
    }
    else
    {
        NandPara.uwSLCPageNumPerBlock = NandPara.uwPageNumPerBlock / 3;
    }

#ifdef FTL_YX2Q
    NandPara.uwSLCPageNumPerBlock = 768; // by spec slc = 768 qlc = 3048
#endif

    NandPara.ubSLCPageNumPerBlockShift = (U8)cal_shift((U32)NandPara.uwSLCPageNumPerBlock);
    if(NandPara.uwSLCPageNumPerBlock % (1 << NandPara.ubSLCPageNumPerBlockShift))
    {
        NandPara.ubSLCPageNumPerBlockShift++;
    }

    gubSysBankNo = MIN(SYS_BANK_NUM, NandPara.ubBankNum);

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL) &&
            ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B0K)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B16)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B17)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27A)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B27B)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B37R)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B36R)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_B47R)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N18A)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N28)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38A)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N38B)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_Q5171A)
             || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_N48R)))
    {
        NandPara.ubSLCPageNumPerBlockShift = NandPara.ubPageNumPerBlockShift;
    }

    if(NandPara.ubSLCPageNumPerBlockShift < 8)
        NandPara.ubSLCPageNumPerBlockShift = 8;
    if(NandPara.ubPageNumPerBlockShift < 8)
        NandPara.ubPageNumPerBlockShift = 8;

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV2_128G)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV4_64G)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV5_64G)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_1v8)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_512Gb)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV6_512Gb_1v8)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_SSV7_512Gb)))
    {
        NandPara.ubSLCPageNumPerBlockShift = NandPara.ubPageNumPerBlockShift;
    }
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
    {
        NandPara.ubSLCPageNumPerBlockShift = NandPara.ubPageNumPerBlockShift;
    }

    NandPara.ubPlaneNumPerLun = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_PLANE_NUM_OFFSET);
    NandPara.ubPlaneNumPerLunShift = (U8)cal_shift((U32)NandPara.ubPlaneNumPerLun);
    if(NandPara.ubPlaneNumPerLun % (1 << NandPara.ubPlaneNumPerLunShift))
    {
        NandPara.ubPlaneNumPerLunShift++;
    }

    NandPara.ubSectorNumPerPage = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_PAGE_SECTOR_SIZE);
    NandPara.ubMpPageByteNumShift = (U8)cal_shift((U32)NandPara.ubSectorNumPerPage) + 9 +
                                    NandPara.ubPlaneNumPerLunShift;//9means 512B

    guwCacheSectorNumPerLbn = _MEM16(CONFIG_BASE_VA_ADDR +
                                     CONFIG_LBN_BUF_SIZE_OFFSET); // Buffer size=4K(8 sectors)
    gubCacheSectorNumPerLbnShift = (U8)cal_shift((U32)guwCacheSectorNumPerLbn);

    NandPara.uwLbnNumPerMpPage = ((NandPara.ubSectorNumPerPage >> gubCacheSectorNumPerLbnShift)
                                  * NandPara.ubPlaneNumPerLun);
    NandPara.ubLbnNumPerMpPageShift = (U8)cal_shift((U32)NandPara.uwLbnNumPerMpPage);
    if(NandPara.uwLbnNumPerMpPage % (1 << NandPara.ubLbnNumPerMpPageShift))
    {
        NandPara.ubLbnNumPerMpPageShift++;
    }

    NandPara.uwMpBlockNumPerLun = (NandPara.uwBlockNumPerLun / NandPara.ubPlaneNumPerLun);
    NandPara.ubMpBlockNumPerLunShift = (U8)cal_shift((U32)NandPara.uwMpBlockNumPerLun);
    if(NandPara.uwMpBlockNumPerLun % (1 << NandPara.ubMpBlockNumPerLunShift))
    {
        NandPara.ubMpBlockNumPerLunShift++;
    }

    NandPara.uwFullPageSize = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_FULL_PAGE_PER_BLOCK);
    ASSERT_LLF(NandPara.uwFullPageSize >= NandPara.ubSectorNumPerPage * 512);
    NandPara.uwDrivingSetting = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_DEFAULT_DRIVING_SETTING);
    NandPara.uwMaxPECycle = _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_MAX_PE_CYCLE);
    gubdram_row_bit =  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_DDR_ROW_BIT);
    gubdram_colum_bit =  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_DDR_COL_BIT);
    gubdram_dq_width = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_DDR_DQ_WIDTH);
    ubRDTLLFNormalEn = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_RDT_LLF_NORMAL_EN);
    ubRDTLLFCheckPerDieEn = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_RDT_LLF_PER_DIE_EN);

    gulMaxLBAAddr = _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET);
#ifdef EXTEND_LBA
    if (gulMaxLBAAddr <= 4096)
#else
    if (gulMaxLBAAddr <= 2048)
#endif
    {
        guwHostClaimCapacity = gulMaxLBAAddr;
    }
    else
    {
        guwHostClaimCapacity = gulMaxLBAAddr >> (1 + 10 + 10); // LBA(Sector) to GB
        printk("Calculate capacity as LBA\r\n");
    }
#ifdef EXTEND_LBA
    gulMaxLBAAddr = (U64)97696368 + (U64)guwHostClaimCapacity * (U64)1953504 - 50 * (U64)1953504 - 1;
#else
    gulMaxLBAAddr = 97696368 + guwHostClaimCapacity * 1953504 - 50 * 1953504 - 1;
#endif
    _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET) = gulMaxLBAAddr + 1;
    printk("claim GB %d, LBA %x %x, shift %d\r\n", guwHostClaimCapacity,
           _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET + 4),
           _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET),
           _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_STABLE_ENTRYSIZE_SHIFT));

    gfPureSLC = _REG08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_PESUDO_SLC_MODE);

    // smart log temperature control
    gubTemperatureOffset = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPERATURE_OFFSET);
    gubTemperatureMax = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPERATURE_MAX);
    gubTemperatureMin = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPERATURE_MIN);
#if defined(RL6577_VA) || defined(RTS5771_VA) || defined(RTS5771_FPGA)
    gubTemperatureLimit = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPERATURE_LIMIT);
#else
    guwTemperatureLimit = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_TEMPERATURE_LIMIT);
#endif
    gubFixTemperature = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPERATURE_FIX);
    gubSmartInfo = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_SMART_INFO);
#ifndef RL6643_VA
    if (gubTemperatureMax == 0)
    {
        gubTemperatureMax = 0xff;
    }
#endif

#if (!(defined(RL6577_VA) || defined(RTS5771_VA) || defined(RTS5771_FPGA)))
    if (guwTemperatureLimit == 0 || guwTemperatureLimit > 110 || guwTemperatureLimit < 70)
    {
        guwTemperatureLimit = 110;  // default
    }
#endif

    gubFWFeatureSetting = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FW_FEATURE_SETTING);

#if defined(RL6577_VA)
    if((READ_REG_32(GPIO_EXT_PORTA) & (GPIO10)) == 0)
    {
        llfprintk("RTS5765DL IC\r\n");
#if 0
        // Adjust QoS for 5765/5766
        U16 uwWQOS, uwRQOS;
        uwWQOS = _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_WQOS_LIMIT);
        uwRQOS = _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RQOS_LIMIT);

        if(uwWQOS == 0 || uwWQOS > 2000)
        {
            uwWQOS = 2000;
        }
        if(uwRQOS == 0 || uwRQOS > 2000)
        {
            uwRQOS = 2000;
        }
        _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_WQOS_LIMIT) = uwWQOS;
        _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RQOS_LIMIT) = uwRQOS;
#endif
    }
    else
    {
        llfprintk("RTS5766DL IC\r\n");
    }
#endif

    // === New Config for Driving ===
    gubFCDiffEnable 	= _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FC_DIFF_EN);
    gubNANDVrefEn		= _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_VREF_EN);
    gubNandDriv	        = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_DRISTR);

#ifdef RL6531_VB
    gulFcOcd3            = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG3);
    gulFcOcd0            = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG0);
    gulFcOcd1            = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG1);
    gulFcOcd2            = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG2);
    gubNANDODTEnable	 = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN);
    gubNANDODTCfg        = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN);
    gulFc_dqs_odt_en	 = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FR_PAD_ODT_CTRL);
    gulFc_dqs_odt		 = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FR_PAD_ODT_CFG);
#else
    gulFc_ocd			= _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH);
    gubNANDODTEnable	= _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN);
    gubNANDODTCfg       = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_CFG);

    gulFc_dqs_odt		= _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQS_ODT_CFG);
    gulFc_dq_re_odt		= _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQ_RE_ODT_CFG);
    gulFc_dqs_odt_en	= _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQS_ODT_CTRL);
    gulFc_dq_re_odt_en 	= _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_DQ_RE_ODT_CTRL);
#endif

#ifndef RL6531_VB
#if defined(RL6447_VA)
    if((gubFCDiffEnable & 0xF0) == 0xc0)
    {
        gubFCDiffEnable = gubFCDiffEnable & 0xf;
    }
    if((gubNANDODTEnable & 0xF0) == 0xc0)
    {
        gubNANDODTEnable = gubNANDODTEnable & 0xf;
    }
    if((gubNANDODTCfg & 0xF0) == 0xc0)
    {
        gubNANDODTCfg = gubNANDODTCfg & 0xf;
    }
    if((gubNANDVrefEn & 0xF0) == 0xc0)
    {
        gubNANDVrefEn = gubNANDVrefEn & 0xf;
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG))
    {
        gubNandDriv	= _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_TSB_DRISTR);
    }
    if((gubNandDriv & 0xF0) == 0xc0)
    {
        gubNandDriv = gubNandDriv & 0xf;
    }
#endif

    if((gulFc_ocd & 0xF0) == 0xc0)
    {
        gulFc_ocd = gulFc_ocd & 0xf;
    }
    else
    {
        gulFc_ocd = 3;
    }
    if((gulFc_dqs_odt & 0xF0) == 0xc0)
    {
        gulFc_dqs_odt = gulFc_dqs_odt & 0xf;
    }
    else
    {
        gulFc_dqs_odt = 4;
    }
    if((gulFc_dq_re_odt & 0xF0) == 0xc0)
    {
        gulFc_dq_re_odt = gulFc_dq_re_odt & 0xf;
    }
    else
    {
        gulFc_dq_re_odt = 4;
    }
    if((gulFc_dqs_odt_en & 0xF00) == 0xc00)
    {
        gulFc_dqs_odt_en = gulFc_dqs_odt_en & 0xff;
    }
    else
    {
        gulFc_dqs_odt_en = 0;
    }

    if((gulFc_dq_re_odt_en & 0xF00) == 0xc00)
    {
        gulFc_dq_re_odt_en = gulFc_dq_re_odt_en & 0xff;
    }
    else
    {
        gulFc_dq_re_odt_en = 0;
    }

    llfprintk("Nand: ODTEN %x ODTCfg %x VREF %x NandDriv %x\r\n",
              gubNANDODTEnable, gubNANDODTCfg, gubNANDVrefEn, gubNandDriv);
    llfprintk("FC : Diff %x ocd %x dqs_odt_en %x dq_re_odt_en %x dqs_odt %x dq_re_odt %x\r\n",
              gubFCDiffEnable, gulFc_ocd, gulFc_dqs_odt_en, gulFc_dq_re_odt_en, gulFc_dqs_odt, gulFc_dq_re_odt);
#else
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
            || (FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG))
    {
        gubNandDriv = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_TSB_DRISTR);
    }
    llfprintk("Nand: ODTCfg %x VREF %x NandDriv %x\r\n",
              gubNANDODTCfg, gubNANDVrefEn, gubNandDriv);
    llfprintk("FC : Diff %x ocd3 %x ocd0 %x ocd1 %x ocd2 %x odt_en %x odt %x\r\n",
              gubFCDiffEnable, gulFcOcd3, gulFcOcd0, gulFcOcd1, gulFcOcd2, gulFc_dqs_odt_en, gulFc_dqs_odt);
#endif

    // === End Config for Driving ===

    // Read RDT Nand Log Use
    guwCaclSSGroupBegin = 1;
    guwCaclSSGroupEnd = (64 / NandPara.ubPlaneNumPerLun);
    gsRDTLog.ubBank = 0;
    gsRDTLog.uwPage = 0;
    gsRDTLog.ubBlock = guwCaclSSGroupBegin * NandPara.ubPlaneNumPerLun;
    gsRDTLog.ubGetBlock = 0;
    gsRDTLog.ufFoundBlock = FALSE;

#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RTS5771_VA)

    for(i = 0; i < CH_NUM_MAX; i++)
    {
        gubNvmeTxDelayMin[i] = 0xff;
        gubNvmeTxDelayMax[i] = 0;

        gubNvmeThinDelayMin[i] = 0xff;
        gubNvmeThinDelayMax[i] = 0;
    }

    gubNvmeRxDelayMinTemp = 0xff;
    gubNvmeRxDelayMaxTemp = 0;
#endif
    for(i = 0; i < 8; i++)
    {
        gulAutoCapacity[i] = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_AUTO_CAPACITY_0 + i * 2);
    }
    gubCmpSel = 0;
    U32 ecc_cfg;
#if defined(RL6447_VA) || defined(RL6577_VA) || defined(RTS5771_VA)
    gubStartCH = 0;
    gubCHNum = 4;

    ecc_cfg = FR_CONFIG_CH(FR_ECC_CFG, gubStartCH) & 0xfff;
#else
    ecc_cfg = FR_CONFIG_REG32(FR_ECC_CFG) & 0xfff;
#endif

    gub_ECC_parity_len_per_2K = ( FC_TOP_REG(FR_FIFO_ECC_EN) != 0) ? ecc_cfg : 0;

#if defined(RL5771_FPGA)||defined(RL5771_VA)
    gub_Total_len_per_2K = 16 + (2 * 1024) +
                           gub_ECC_parity_len_per_2K; //+4 for crc, +6 for head, +6 for dammy
#else
    gub_Total_len_per_2K = 10 + (2 * 1024) + gub_ECC_parity_len_per_2K; //+4 for crc, +6 for head
#endif

#if defined(RL6577_VA) || defined(RL5771_VA)
    ulTemp32_OUI = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_OUI);
    ulTemp64_l = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_EUI64L);
    ulTemp64_h = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_EUI64H);
    ullTemp64_WWN_L = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_WWN);
    ullTemp64_WWN_H = _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_WWN + 4);
    if (ulTemp32_OUI == 0)//no value in config
    {
        ulTemp32_OUI = NVME_IDENTIFY_OUI; //default value
    }
    _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_OUI) = ulTemp32_OUI;

    if(ulTemp64_l != 0xffffffff || ulTemp64_h != 0xffffffff) //EUI mode
    {
        if (ulTemp64_l == 0 && ulTemp64_h == 0)
        {
            for (i = SBLK_OFFSET_SERIAL_NUBER; i < SBLK_OFFSET_FW_VERSION; i += 4)
            {
                ulTemp64_l ^= _MEM32(CONFIG_BASE_VA_ADDR + i) + gulTicks;
                ulTemp64_h ^= (ulTemp64_l + _MEM32(CONFIG_BASE_VA_ADDR + i));
            }
            ulTemp64_l = (ulTemp64_l & 0xff000000) | ((ulTemp32_OUI & 0x00ff0000) >> 16) |
                         (ulTemp32_OUI & 0x0000ff00) | ((ulTemp32_OUI & 0x000000ff) << 16); //OUI revert to big edian
        }
        _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_EUI64L) = ulTemp64_l;
        _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_EUI64H) = ulTemp64_h;
    }

    else //WWN mode
    {
        if(ullTemp64_WWN_L == 0 && ullTemp64_WWN_H == 0)
        {
#ifdef FOR_TG
            //nothing to do
#else

            for (i = SBLK_OFFSET_SERIAL_NUBER; i < SBLK_OFFSET_FW_VERSION; i += 4)
            {
                ullTemp64_WWN_L ^= _MEM32(CONFIG_BASE_VA_ADDR + i) + gulTicks;
                ullTemp64_WWN_H ^= (ulTemp64_l + _MEM32(CONFIG_BASE_VA_ADDR + i));
            }
            ullTemp64_WWN_L = ((ulTemp32_OUI & 0x0000000f) << 28) | ((ulTemp32_OUI & 0x00000ff0) << 12) | ((
                                  ulTemp32_OUI & 0x000ff000) >> 4) | ((ulTemp32_OUI & 0x00f00000) >> 20) |
                              (ullTemp64_WWN_L & 0x0f000000) | (0x5 << 4);
#endif

        }

        _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_WWN) = ullTemp64_WWN_L;
        _MEM32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_WWN + 4) = ullTemp64_WWN_H;
    }

    printk("[KEEP RDT RESULT] config setting: %d\r\n",
           _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_EXTEND_RESULT_BLK));
    gubExtendRdtResultBlk = 0;
#endif

#ifdef LLF_AUTO_RMA
    gubSTableEntrySizeShift = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_STABLE_ENTRYSIZE_SHIFT);
#endif

#if defined(RL6643_VA)
    U16 uwPlaneNumPerSuperPage = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun;
    U16 uwDefaultMode = 32;
    gubRaidPagePerRaid = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_PAGE_NUM_PER_RAID);

    // if bit[7] pull high, check if raid mode <= 7+1,
    // if raid mode <= 7+1 then automatically set to 31+1
    if(gubRaidPagePerRaid & 0x80)
    {
        gubRaidPagePerRaid &= 0x7F;
        if((gubRaidPagePerRaid * uwPlaneNumPerSuperPage) <= 8)
        {
            gubRaidPagePerRaid = 0;
        }
    }

    if(!gubRaidPagePerRaid)
    {
        // 0 means force to 31+1 mode or super page mode
        if(uwPlaneNumPerSuperPage >= uwDefaultMode)
        {
            gubRaidPagePerRaid = 1;
        }
        else
        {
            U8 ProgramCount = 1; //SLC
            gubRaidPagePerRaid = findFittedRaidPageNum(ProgramCount, BS_SLC_MODE);
            if(gubRaidPagePerRaid == 0xFF)
            {
                DBGPRINTK(ALWAYS_MSG, "[Error] Can't find a suitable SLCRaidPageNum\r\n");
                return ERR_INVALID_RAID_PAGENUM;
            }
            DBGPRINTK(ALWAYS_MSG, "SLCRaidPage: %d\r\n", gubRaidPagePerRaid);
        }
    }
    gubTLCRaidPagePerRaid = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_PAGE_NUM_PER_RAID_TLC);
    if(!gubTLCRaidPagePerRaid)
    {
        // 0 means force to 31+1 mode or super page mode
        if(uwPlaneNumPerSuperPage >= uwDefaultMode)
        {
            gubTLCRaidPagePerRaid = 1;
        }
        else
        {

            U8 ProgramCount = 0;
            switch(FLASH_SPCIES(gulFlashVendorNum))
            {
            case IS_SLC:
                ProgramCount = 1;
                break;
            case IS_MLC:
            case IS_3DMLC:
                ProgramCount = 2;
                break;
            case IS_TLC:
            case IS_3DTLC:
                ProgramCount = 3;
                break;
            case IS_3DQLC:
                ProgramCount = 4;
                break;
            default:
            {
                DBGPRINTK(ALWAYS_MSG, "[Error] Uncategorized FLASH_SPCIES %x\r\n", FLASH_SPCIES(gulFlashVendorNum));
                return ERR_UNCATEGORIZED_FLASH_SPCIES;
            }
            }
            gubTLCRaidPagePerRaid = findFittedRaidPageNum(ProgramCount, BS_TLC_MODE);
            if(gubTLCRaidPagePerRaid == 0xFF)
            {
                DBGPRINTK(ALWAYS_MSG, "[Error] Can't find a suitable RaidPageNum\r\n");
                return ERR_INVALID_RAID_PAGENUM;
            }
            DBGPRINTK(ALWAYS_MSG, "TLCRaidPage: %d\r\n", gubTLCRaidPagePerRaid);
        }
    }
#endif
#if defined(RL6643_VA) || defined(RL6577_VA)
#ifdef SBLK_EXPAND
    gubSblkBadDetect = 0;
    gubSblkBadDetect = _REG08(CONFIG_BASE_VA_ADDR + CONFIG_SBLK_BAD_DETECT_EN);
    gubDefaultSblkBad = 0;
#endif
#endif
    return ERR_OK;
}

#if defined(RL6643_VA)
//mdio data structure enable bit+r/w bit+addr+data
void llf_mdio_wr(U32 clk_div, U32 phy_addr, U32 data)
{
    U32 data_tmp;

    data_tmp = 0xc0000000 | (clk_div << 26) | (phy_addr << 16) | data ;
    WRITE_REG_32(U_MDIO_HW_CTRL, data_tmp);
    while ( READ_REG_32(U_MDIO_HW_CTRL) == data_tmp) {};	// wait done
}

U32 llf_mdio_rd(U32 clk_div, U32 phy_addr)
{
    U32 data_tmp;

    data_tmp = 0x80000000 | (clk_div << 26) | (phy_addr << 16) ;
    WRITE_REG_32(U_MDIO_HW_CTRL, data_tmp);
    while (READ_REG_32(U_MDIO_HW_CTRL) == data_tmp) {};	// wait done
    data_tmp = READ_REG_32(U_MDIO_HW_CTRL) & 0xffff;
    return(data_tmp);
}

void llf_thermal_setting()
{
    U32 rd2;
    if(IS_6643_VERSION_TAG)
    {
        while(!(READ_REG_32(THERMAL_CTRL_REG) & THERMAL_INT_FLAG))
        {
            SET_REG_32(THERMAL_CTRL_REG, THERMAL_INT_FLAG);
        }

        //reg_a config
        rd2 = llf_mdio_rd( 0, 0x371 );
        rd2 &= ~(0xffff);   //clear 0x371 bit[15:0]
        rd2 |= 0xa843;
        llf_mdio_wr(0, 0x371, rd2);

        rd2 = llf_mdio_rd( 0, 0x372 );
        rd2 &= ~(0xffff);   //clear 0x372 bit[15:0]
        llf_mdio_wr(0, 0x372, rd2);

        //RESET TM
        rd2 = llf_mdio_rd( 0, 0x370 );
        rd2 &= ~(0xffff);   //clear 0x370 bit[15:0]
        rd2 |= 0x10f2;
        llf_mdio_wr(0, 0x370, rd2);

        rd2 = llf_mdio_rd( 0, 0x370 );
        rd2 &= ~(0xffff);   //clear 0x370 bit[15:0]
        rd2 |= 0x10f3;
        llf_mdio_wr(0, 0x370, rd2);
    }
    if(IS_6855_VERSION_TAG)
    {
        //thermal sensor clk speed up
        rd2 = llf_mdio_rd(0, 0x371);//adccksel 0x371[4:2]=7
        llf_mdio_wr(0, 0x371, rd2 | 0x1C);

        rd2 = llf_mdio_rd(0, 0x37C);//dsr 0x37C[7:5]=0
        llf_mdio_wr(0, 0x37C, rd2 & (~0xE0));
        // Fixed value (by Andy)
        llf_mdio_wr(0, 0x37a, 0xb940);
        while(!(READ_REG_32(THERMAL_CTRL_REG) & THERMAL_INT_FLAG))
        {
            SET_REG_32(THERMAL_CTRL_REG, THERMAL_INT_FLAG);
        }
    }

    llfDbgPrintk(ALWAYS_MSG, "llf_thermal_setting\r\n");
}
#endif

void llfInitConfig(void)
{
    int i, j, bank;
    U32 cmp;
    U32 ulmode;
    U8 *Manufacturer = (U8 *)TEMP_HBUF_ADDR;
    //U8 ubEraseCmd[8];
    //U8 ubWriteCmd[8];
    //U8 ubReadCmd[8];
    struct _TSB_RETRY  *pTsb_Retry_Setting;

    //Define IRP use count, this count is defined by MAX_BUF

    //With 2 BE each 4 plane, these 8 plane total cost 8 IRPs from
    //fdm W, GC W, DRAM preservation W.
    //
    //With 2 BE each 8 plane, these 16 plane total cost 16 IRPs from
    //fdm W, GC W, DRAM preservation W.
    //
    //FE costs 9 IRPs by design which make SATA able to get 1 128KB
    //write data
    //
    //Now we totally have 33 IRPs, and write pending queue may costs
    //10 IRPs if gubPendingWriteNumMax == 5. It's a little wasting.
    bank = 0;
    ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    //use 0x00 addr for read id to distinguish flash
    gul_FW_TAG = llfBETagSetting(TAG_READ_ID, bank);
    FCReadID(ulmode, bank, 0x00, 5, TEMP_HBUF_PHY_ADDR, 0x10);
    FcBusyWait1ms(1);
    FCCompletionPolling(&cmp, (gul_FW_TAG));
    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "FC read id error: %d\r\n", bank);
        ASSERT_LLF(0);
    }
    if((Manufacturer[0] == 0x98) && (Manufacturer[1] == 0xDE) && (Manufacturer[2] == 0x94)
            && (Manufacturer[3] == 0x93) && (Manufacturer[4] == 0x76) && (Manufacturer[5] == 0x50))
        gulFlashVendorNum |= IS_6DDK_SDR;
    else if((Manufacturer[0] == 0x98) && (Manufacturer[2] == 0xDE) && (Manufacturer[4] == 0x94)
            && (Manufacturer[6] == 0x93) && (Manufacturer[8] == 0x76) && (Manufacturer[10] == 0xD0))
        gulFlashVendorNum |= IS_7DDK_TOGGLE;
    else if((Manufacturer[0] == 0x98) && (Manufacturer[1] == 0x3A) && (Manufacturer[2] == 0x94)
            && (Manufacturer[3] == 0x93) && (Manufacturer[4] == 0x76) && (Manufacturer[5] == 0x51))
        gulFlashVendorNum |= IS_8DDL_SDR;
    else if((Manufacturer[0] == 0x98) && (Manufacturer[2] == 0x3A) && (Manufacturer[4] == 0x94)
            && (Manufacturer[6] == 0x93) && (Manufacturer[8] == 0x76) && (Manufacturer[10] == 0xD1))
        gulFlashVendorNum |= IS_8DDL_TOGGLE;

    // Toshiba read retry command, address, data setting
    gulTsbARCBaseAddr = TSB_RETRY_SETTING_ADDR; // For LLF
    pTsb_Retry_Setting = (struct _TSB_RETRY *)TSB_RETRY_SETTING_ADDR;

    // 5C C5 command
    pTsb_Retry_Setting->ubTsbRetryPreCmd[0] = 0;
    pTsb_Retry_Setting->ubTsbRetryPreCmd[1] = 0x5c;
    pTsb_Retry_Setting->ubTsbRetryPreCmd[2] = 0xc5;
    pTsb_Retry_Setting->ubTsbRetryPreCmd[3] = 0;

    // 0x55, 0x26, 0x5D
    pTsb_Retry_Setting->ubTsbTriggerRetryCmd[0] = 0x55;
    pTsb_Retry_Setting->ubTsbTriggerRetryCmd[1] = 0x26;
    pTsb_Retry_Setting->ubTsbTriggerRetryCmd[2] = 0x5d;
    pTsb_Retry_Setting->ubTsbTriggerRetryCmd[3] = 0;

    // Initail data
    for (i = 0; i < MAX_TSB_RR_ADDR_NUM; i++)
    {
        pTsb_Retry_Setting->ubTsbRetryAddr[i] = 0;

        for (j = 0; j < MAX_TSB_RR_DATA_NUM; j++)
            pTsb_Retry_Setting->ubData[i][j] = 0;
    }

    pTsb_Retry_Setting->ubTsbRetryEndCmd[0] =  0x55;
    pTsb_Retry_Setting->ubTsbRetryEndCmd[1]  = 0xFF;
    pTsb_Retry_Setting->ubTsbRetryEndCmd[2]  = 0;
    pTsb_Retry_Setting->ubTsbRetryEndCmd[3]  = 0;

    if (  ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
            && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8DDL_SDR)) ||
            ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
             && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8DDL_TOGGLE)) ||
            ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
             && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_6DDL_SDR)) ||
            ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
             && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_7DDL_SDR))
       )
    {
        pTsb_Retry_Setting->ubMaxRetry = 12;

#ifdef NEW_TSB_READ_RETRY
        pTsb_Retry_Setting->ubCmdSetNum = 0;

        pTsb_Retry_Setting->ubData[0][0] = 0x00;
        pTsb_Retry_Setting->ubData[0][1] = 0x00;
        pTsb_Retry_Setting->ubData[0][2] = 0x00;
        pTsb_Retry_Setting->ubData[0][3] = 0x00;
        pTsb_Retry_Setting->ubData[0][4] = 0x00;

        pTsb_Retry_Setting->ubData[1][0] = 0x00;
        pTsb_Retry_Setting->ubData[1][1] = 0x00;
        pTsb_Retry_Setting->ubData[1][2] = 0x00;
        pTsb_Retry_Setting->ubData[1][3] = 0x00;
        pTsb_Retry_Setting->ubData[1][4] = 0x00;

        pTsb_Retry_Setting->ubData[2][0] = 0x02;
        pTsb_Retry_Setting->ubData[2][1] = 0x04;
        pTsb_Retry_Setting->ubData[2][2] = 0x02;
        pTsb_Retry_Setting->ubData[2][3] = 0x00;
        pTsb_Retry_Setting->ubData[2][4] = 0x00;

        pTsb_Retry_Setting->ubData[3][0] = 0x7C;
        pTsb_Retry_Setting->ubData[3][1] = 0x00;
        pTsb_Retry_Setting->ubData[3][2] = 0x7C;
        pTsb_Retry_Setting->ubData[3][3] = 0x7C;
        pTsb_Retry_Setting->ubData[3][4] = 0x00;

        pTsb_Retry_Setting->ubData[4][0] = 0x7A;
        pTsb_Retry_Setting->ubData[4][1] = 0x7C;
        pTsb_Retry_Setting->ubData[4][2] = 0x7A;
        pTsb_Retry_Setting->ubData[4][3] = 0x7C;
        pTsb_Retry_Setting->ubData[4][4] = 0x00;

        pTsb_Retry_Setting->ubData[5][0] = 0x78;
        pTsb_Retry_Setting->ubData[5][1] = 0x00;
        pTsb_Retry_Setting->ubData[5][2] = 0x78;
        pTsb_Retry_Setting->ubData[5][3] = 0x7A;
        pTsb_Retry_Setting->ubData[5][4] = 0x00;

        pTsb_Retry_Setting->ubData[6][0] = 0x7E;
        pTsb_Retry_Setting->ubData[6][1] = 0x02;
        pTsb_Retry_Setting->ubData[6][2] = 0x7E;
        pTsb_Retry_Setting->ubData[6][3] = 0x7A;
        pTsb_Retry_Setting->ubData[6][4] = 0x00;

        pTsb_Retry_Setting->ubData[7][0] = 0x76;
        pTsb_Retry_Setting->ubData[7][1] = 0x04;
        pTsb_Retry_Setting->ubData[7][2] = 0x76;
        pTsb_Retry_Setting->ubData[7][3] = 0x02;
        pTsb_Retry_Setting->ubData[7][4] = 0x00;

        pTsb_Retry_Setting->ubData[8][0] = 0x04;
        pTsb_Retry_Setting->ubData[8][1] = 0x00;
        pTsb_Retry_Setting->ubData[8][2] = 0x04;
        pTsb_Retry_Setting->ubData[8][3] = 0x78;
        pTsb_Retry_Setting->ubData[8][4] = 0x00;

        pTsb_Retry_Setting->ubData[9][0] = 0x06;
        pTsb_Retry_Setting->ubData[9][1] = 0x00;
        pTsb_Retry_Setting->ubData[9][2] = 0x06;
        pTsb_Retry_Setting->ubData[9][3] = 0x76;
        pTsb_Retry_Setting->ubData[9][4] = 0x00;

        pTsb_Retry_Setting->ubData[10][0] = 0x74;
        pTsb_Retry_Setting->ubData[10][1] = 0x7C;
        pTsb_Retry_Setting->ubData[10][2] = 0x74;
        pTsb_Retry_Setting->ubData[10][3] = 0x76;
        pTsb_Retry_Setting->ubData[10][4] = 0x00;

        pTsb_Retry_Setting->ubData[11][0] = 0x00;
        pTsb_Retry_Setting->ubData[11][1] = 0x00;
        pTsb_Retry_Setting->ubData[11][2] = 0x00;
        pTsb_Retry_Setting->ubData[11][3] = 0x00;
        pTsb_Retry_Setting->ubData[11][4] = 0x00;

#else // original NEW_TSB_READ_RETRY
        pTsb_Retry_Setting->ubCmdSetNum = 5;

        pTsb_Retry_Setting->ubTsbRetryAddr[0] = 0x04;
        pTsb_Retry_Setting->ubTsbRetryAddr[1] = 0x05;
        pTsb_Retry_Setting->ubTsbRetryAddr[2] = 0x06;
        pTsb_Retry_Setting->ubTsbRetryAddr[3] = 0x07;
        pTsb_Retry_Setting->ubTsbRetryAddr[4] = 0x0D;
        pTsb_Retry_Setting->ubTsbRetryAddr[5] = 0x00;
        pTsb_Retry_Setting->ubTsbRetryAddr[6] = 0x00;
        pTsb_Retry_Setting->ubTsbRetryAddr[7] = 0x00;


        pTsb_Retry_Setting->ubData[0][0] = 0x00;
        pTsb_Retry_Setting->ubData[0][1] = 0x00;
        pTsb_Retry_Setting->ubData[0][2] = 0x00;
        pTsb_Retry_Setting->ubData[0][3] = 0x00;
        pTsb_Retry_Setting->ubData[0][4] = 0x00;

        pTsb_Retry_Setting->ubData[1][0] = 0x00;
        pTsb_Retry_Setting->ubData[1][1] = 0x00;
        pTsb_Retry_Setting->ubData[1][2] = 0x00;
        pTsb_Retry_Setting->ubData[1][3] = 0x00;
        pTsb_Retry_Setting->ubData[1][4] = 0x00;

        pTsb_Retry_Setting->ubData[2][0] = 0x04;
        pTsb_Retry_Setting->ubData[2][1] = 0x04;
        pTsb_Retry_Setting->ubData[2][2] = 0x00;
        pTsb_Retry_Setting->ubData[2][3] = 0x00;
        pTsb_Retry_Setting->ubData[2][4] = 0x00;

        pTsb_Retry_Setting->ubData[3][0] = 0x08;
        pTsb_Retry_Setting->ubData[3][1] = 0x04;
        pTsb_Retry_Setting->ubData[3][2] = 0x04;
        pTsb_Retry_Setting->ubData[3][3] = 0x04;
        pTsb_Retry_Setting->ubData[3][4] = 0x00;

        pTsb_Retry_Setting->ubData[4][0] = 0x7C;
        pTsb_Retry_Setting->ubData[4][1] = 0x7C;
        pTsb_Retry_Setting->ubData[4][2] = 0x7C;
        pTsb_Retry_Setting->ubData[4][3] = 0x78;
        pTsb_Retry_Setting->ubData[4][4] = 0x00;

        pTsb_Retry_Setting->ubData[5][0] = 0x78;
        pTsb_Retry_Setting->ubData[5][1] = 0x78;
        pTsb_Retry_Setting->ubData[5][2] = 0x78;
        pTsb_Retry_Setting->ubData[5][3] = 0x74;
        pTsb_Retry_Setting->ubData[5][4] = 0x00;

        pTsb_Retry_Setting->ubData[6][0] = 0x78;
        pTsb_Retry_Setting->ubData[6][1] = 0x76;
        pTsb_Retry_Setting->ubData[6][2] = 0x76;
        pTsb_Retry_Setting->ubData[6][3] = 0x74;
        pTsb_Retry_Setting->ubData[6][4] = 0x00;

        pTsb_Retry_Setting->ubData[7][0] = 0x78;
        pTsb_Retry_Setting->ubData[7][1] = 0x76;
        pTsb_Retry_Setting->ubData[7][2] = 0x76;
        pTsb_Retry_Setting->ubData[7][3] = 0x72;
        pTsb_Retry_Setting->ubData[7][4] = 0x00;

        pTsb_Retry_Setting->ubData[8][0] = 0x78;
        pTsb_Retry_Setting->ubData[8][1] = 0x76;
        pTsb_Retry_Setting->ubData[8][2] = 0x76;
        pTsb_Retry_Setting->ubData[8][3] = 0x72;
        pTsb_Retry_Setting->ubData[8][4] = 0x00;

        pTsb_Retry_Setting->ubData[9][0] = 0x78;
        pTsb_Retry_Setting->ubData[9][1] = 0x76;
        pTsb_Retry_Setting->ubData[9][2] = 0x76;
        pTsb_Retry_Setting->ubData[9][3] = 0x72;
        pTsb_Retry_Setting->ubData[9][4] = 0x00;

        pTsb_Retry_Setting->ubData[10][0] = 0x78;
        pTsb_Retry_Setting->ubData[10][1] = 0x76;
        pTsb_Retry_Setting->ubData[10][2] = 0x76;
        pTsb_Retry_Setting->ubData[10][3] = 0x72;
        pTsb_Retry_Setting->ubData[10][4] = 0x00;

        pTsb_Retry_Setting->ubData[11][0] = 0x00;
        pTsb_Retry_Setting->ubData[11][1] = 0x00;
        pTsb_Retry_Setting->ubData[11][2] = 0x00;
        pTsb_Retry_Setting->ubData[11][3] = 0x00;
        pTsb_Retry_Setting->ubData[11][4] = 0x00;
#endif
    }
    else if(  ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
               && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_6DDK_SDR)) ||
              ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
               && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_7DDK_TOGGLE))
           )
    {
        pTsb_Retry_Setting->ubMaxRetry = 9;
        pTsb_Retry_Setting->ubCmdSetNum = 5;

        pTsb_Retry_Setting->ubTsbTriggerRetryCmd[3] = 0xB3; // For 4th read retry

        pTsb_Retry_Setting->ubTsbRetryAddr[0] = 0x04;
        pTsb_Retry_Setting->ubTsbRetryAddr[1] = 0x05;
        pTsb_Retry_Setting->ubTsbRetryAddr[2] = 0x06;
        pTsb_Retry_Setting->ubTsbRetryAddr[3] = 0x07;
        pTsb_Retry_Setting->ubTsbRetryAddr[4] = 0x0D;
        pTsb_Retry_Setting->ubTsbRetryAddr[5] = 0x00;
        pTsb_Retry_Setting->ubTsbRetryAddr[6] = 0x00;
        pTsb_Retry_Setting->ubTsbRetryAddr[7] = 0x00;

        pTsb_Retry_Setting->ubData[0][0] = 0x00;
        pTsb_Retry_Setting->ubData[0][1] = 0x00;
        pTsb_Retry_Setting->ubData[0][2] = 0x00;
        pTsb_Retry_Setting->ubData[0][3] = 0x00;
        pTsb_Retry_Setting->ubData[0][4] = 0x00;

        pTsb_Retry_Setting->ubData[1][0] = 0x04;
        pTsb_Retry_Setting->ubData[1][1] = 0x04;
        pTsb_Retry_Setting->ubData[1][2] = 0x7c;
        pTsb_Retry_Setting->ubData[1][3] = 0x7e;
        pTsb_Retry_Setting->ubData[1][4] = 0x00;

        pTsb_Retry_Setting->ubData[2][0] = 0x00;
        pTsb_Retry_Setting->ubData[2][1] = 0x7c;
        pTsb_Retry_Setting->ubData[2][2] = 0x78;
        pTsb_Retry_Setting->ubData[2][3] = 0x78;
        pTsb_Retry_Setting->ubData[2][4] = 0x00;

        pTsb_Retry_Setting->ubData[3][0] = 0x7c;
        pTsb_Retry_Setting->ubData[3][1] = 0x76;
        pTsb_Retry_Setting->ubData[3][2] = 0x74;
        pTsb_Retry_Setting->ubData[3][3] = 0x72;
        pTsb_Retry_Setting->ubData[3][4] = 0x00;

        pTsb_Retry_Setting->ubData[4][0] = 0x08;
        pTsb_Retry_Setting->ubData[4][1] = 0x08;
        pTsb_Retry_Setting->ubData[4][2] = 0x00;
        pTsb_Retry_Setting->ubData[4][3] = 0x00;
        pTsb_Retry_Setting->ubData[4][4] = 0x00;

        pTsb_Retry_Setting->ubData[5][0] = 0x0B;
        pTsb_Retry_Setting->ubData[5][1] = 0x7E;
        pTsb_Retry_Setting->ubData[5][2] = 0x76;
        pTsb_Retry_Setting->ubData[5][3] = 0x74;
        pTsb_Retry_Setting->ubData[5][4] = 0x00;

        pTsb_Retry_Setting->ubData[6][0] = 0x10;
        pTsb_Retry_Setting->ubData[6][1] = 0x76;
        pTsb_Retry_Setting->ubData[6][2] = 0x72;
        pTsb_Retry_Setting->ubData[6][3] = 0x70;
        pTsb_Retry_Setting->ubData[6][4] = 0x00;

        pTsb_Retry_Setting->ubData[7][0] = 0x02;
        pTsb_Retry_Setting->ubData[7][1] = 0x7c;
        pTsb_Retry_Setting->ubData[7][2] = 0x7e;
        pTsb_Retry_Setting->ubData[7][3] = 0x70;
        pTsb_Retry_Setting->ubData[7][4] = 0x00;

        pTsb_Retry_Setting->ubData[8][0] = 0x00;
        pTsb_Retry_Setting->ubData[8][1] = 0x00;
        pTsb_Retry_Setting->ubData[8][2] = 0x00;
        pTsb_Retry_Setting->ubData[8][3] = 0x00;
        pTsb_Retry_Setting->ubData[8][4] = 0x00;
    }
    else if (FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)  // 19 nm
    {
        // Tsb 19nm read retry
        pTsb_Retry_Setting->ubMaxRetry = 8;
        pTsb_Retry_Setting->ubCmdSetNum = 4;

        pTsb_Retry_Setting->ubTsbRetryAddr[0] = 0x04;
        pTsb_Retry_Setting->ubTsbRetryAddr[1] = 0x05;
        pTsb_Retry_Setting->ubTsbRetryAddr[2] = 0x06;
        pTsb_Retry_Setting->ubTsbRetryAddr[3] = 0x07;
        pTsb_Retry_Setting->ubTsbRetryAddr[4] = 0x08;
        pTsb_Retry_Setting->ubTsbRetryAddr[5] = 0x09;
        pTsb_Retry_Setting->ubTsbRetryAddr[6] = 0x0A;
        pTsb_Retry_Setting->ubTsbRetryAddr[7] = 0x0B;


        pTsb_Retry_Setting->ubData[0][0] = 0x00;
        pTsb_Retry_Setting->ubData[0][1] = 0x00;
        pTsb_Retry_Setting->ubData[0][2] = 0x00;
        pTsb_Retry_Setting->ubData[0][3] = 0x00;

        pTsb_Retry_Setting->ubData[1][0] = 0x00;
        pTsb_Retry_Setting->ubData[1][1] = 0x00;
        pTsb_Retry_Setting->ubData[1][2] = 0x00;
        pTsb_Retry_Setting->ubData[1][3] = 0x00;

        pTsb_Retry_Setting->ubData[2][0] = 0x04;
        pTsb_Retry_Setting->ubData[2][1] = 0x04;
        pTsb_Retry_Setting->ubData[2][2] = 0x04;
        pTsb_Retry_Setting->ubData[2][3] = 0x04;

        pTsb_Retry_Setting->ubData[3][0] = 0x7C;
        pTsb_Retry_Setting->ubData[3][1] = 0x7C;
        pTsb_Retry_Setting->ubData[3][2] = 0x7C;
        pTsb_Retry_Setting->ubData[3][3] = 0x7C;

        pTsb_Retry_Setting->ubData[4][0] = 0x78;
        pTsb_Retry_Setting->ubData[4][1] = 0x78;
        pTsb_Retry_Setting->ubData[4][2] = 0x78;
        pTsb_Retry_Setting->ubData[4][3] = 0x78;

        pTsb_Retry_Setting->ubData[5][0] = 0x74;
        pTsb_Retry_Setting->ubData[5][1] = 0x74;
        pTsb_Retry_Setting->ubData[5][2] = 0x74;
        pTsb_Retry_Setting->ubData[5][3] = 0x74;

        pTsb_Retry_Setting->ubData[6][0] = 0x08;
        pTsb_Retry_Setting->ubData[6][1] = 0x08;
        pTsb_Retry_Setting->ubData[6][2] = 0x08;
        pTsb_Retry_Setting->ubData[6][3] = 0x08;

        pTsb_Retry_Setting->ubData[7][0] = 0x00;
        pTsb_Retry_Setting->ubData[7][1] = 0x00;
        pTsb_Retry_Setting->ubData[7][2] = 0x00;
        pTsb_Retry_Setting->ubData[7][3] = 0x00;
    }
}

#ifdef SBLK_EXPAND
void llfSblkInit()
{
    U8 ch_no = 0;
    gubSblkStart = 0;
    gubSblkBankStart = 0;

    for(ch_no = 0; ch_no < CH_NUM_MAX; ch_no++)
    {
        gulSblkCHCEMap[ch_no] = 0;
    }
}

void SblkExpdMarkSblk(U8 bank_no, U8 blk_no)
{
    U32 ch_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + gubSblkBankStart + bank_no) & 0xFF) >> 4;
    U32 ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + gubSblkBankStart + bank_no) & 0xF);
    gulSblkCHCEMap[ch_no] &= ~((1 << ((ce_no << SYS_BLK_SHIFT) + blk_no)));
}
#endif

#ifdef REPLACE_FW
U32 llfReadConfigFromSblk(U16 (*uwSblkNo)[SYS_BLK], U32 ulCfgVaddr, U32 ulConfigSz)
{
    U8 ubBank;
    U16 uwSpBlk, uwDefectSpBlk;
    U32 ulCmp, ulRet, ulSblkId;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U8 ubLunNo;
    U16 uwPhyBlk = 0xFF, uwPhyBank = 0xFF;

    for(ubBank = 0; ubBank < gubSysBankNo; ubBank++)
    {
        uwDefectSpBlk = 0;

        ubLunNo = ubBank / NandPara.ubBankNumPerLun;

        for (uwSpBlk = 0; uwSpBlk < SYS_BLK; uwSpBlk++)
        {
            if (0xF == uwSblkNo[ubBank][uwSpBlk])
            {
                uwDefectSpBlk++;
                continue;
            }
#ifdef SBLK_EXPAND
            uwPhyBank = gubSblkBankStart + ubBank;
            uwPhyBlk = uwSblkNo[ubBank][uwSpBlk];
#else
            uwPhyBank = ubBank;
            uwPhyBlk = uwSpBlk;
#endif
            cache_area_dinval(TEMP_BUF_ADDR, DRAM_DATA_SIZE);
            cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), DRAM_HEAD_SIZE);
            cache_dummy_update_read();

            gul_FW_TAG = llfBETagSetting(TAG_READ, uwPhyBank);

            // 16k-page read
            llfFCCmdRead_DRAM(ulMode, uwPhyBank, ubLunNo, uwPhyBlk, 0, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                              TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
            ulRet = FCCompletionPolling(&ulCmp, (gul_FW_TAG));

            if ((ERR_OK != ulRet) || ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0))
            {
                printk("[LLF][ERR] Read cfg FAIL %d,%d,0 Ret:%x Cmp:%x\r\n",
                       uwPhyBlk, uwPhyBank, ulRet, ulCmp);
                //uwSblkNo[ubBank][uwSpBlk] = 0xF;
                uwDefectSpBlk++;
                continue;
            }

            ulSblkId = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
            if(ulSblkId != SBLK_BLK_ID)
            {
                printk("[LLF][ERR] cfg id FAIL %d,%d,0 Id:%x!=%x\r\n",
                       uwPhyBlk, uwPhyBank, ulSblkId, SBLK_BLK_ID);
                uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                uwDefectSpBlk++;
                continue;
            }

            printk("[LLF] Get cfg from %d,%d,0\r\n", uwPhyBlk, uwPhyBank);

            memcpy((void *)(ulCfgVaddr), (const void*)(TEMP_BUF_ADDR), ulConfigSz);
            cache_area_dwbinval(ulCfgVaddr, ulConfigSz);
            cache_dummy_update_read();

            printk("[LLF] Ch:%d Ce:%d Bk:%d\r\n",
                   _REG08(ulCfgVaddr + SBLK_OFFSET_CH_NUM),
                   _REG08(ulCfgVaddr + SBLK_OFFSET_CE_NUM),
                   _REG16(ulCfgVaddr + SBLK_OFFSET_BANK_NUM));
            printk("[LLF] RealLun:%d MpBlk:%d Pln:%d Pg:%d\r\n",
                   _REG08(ulCfgVaddr + SBLK_OFFSET_REAL_LUN_NUM),
                   _REG16(ulCfgVaddr + SBLK_OFFSET_MP_BLOCK_NUM),
                   _REG08(ulCfgVaddr + SBLK_OFFSET_PLANE_NUM_PER_LUN),
                   _REG16(ulCfgVaddr + SBLK_OFFSET_PAGE_NUM_PER_BLOCK));
            printk("[LLF] DySblk:%d~%d SsBs:%d~%d L2pBs:%d~%d Sysblk:%x\r\n",
                   _REG16(ulCfgVaddr + SBLK_OFFSET_SYSTEM_BLOCK_BEGIN_INDEX),
                   _REG16(ulCfgVaddr + SBLK_OFFSET_SYSTEM_BLOCK_END_INDEX) - 1,
                   _REG16(ulCfgVaddr + SBLK_OFFSET_SNAP_MP_BLOCK_BEGIN_INDEX),
                   _REG16(ulCfgVaddr + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX) - 1,
                   _REG16(ulCfgVaddr + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX),
                   _REG16(ulCfgVaddr + SBLK_OFFSET_L2P_MP_BLOCK_END_INDEX) - 1,
                   _REG32(ulCfgVaddr + SBLK_OFFSET_SYS_BLK_CFG2));

            gulSysblk = _REG32(ulCfgVaddr + SBLK_OFFSET_SYS_BLK_CFG2);
#ifdef SBLK_EXPAND
            U8 ubTagSblkExpd = (U8)(_REG32(ulCfgVaddr + SBLK_OFFSET_SBLK_START_TAG));
            if(ubTagSblkExpd != TAG_FOR_SBLK_EXPAND)
            {
                llfprintk("[ERR] bank %d blk %d SBLK_EXPAND %x\r\n", uwPhyBank, uwPhyBlk, ubTagSblkExpd);
                uwSblkNo[ubBank][uwSpBlk] = 0xF;
                SblkExpdMarkSblk(ubBank, uwSpBlk);
            }
#else
            for (ubBank = 0; ubBank < gubSysBankNo; ubBank++)
            {
                for(uwSpBlk = 0; uwSpBlk < SYS_BLK; uwSpBlk ++)
                {
                    if(!((gulSysblk >> (ubBank * 4)) & (1 << uwSpBlk)))
                    {
                        uwSblkNo[ubBank][uwSpBlk] = 0xF;
                    }
                }
            }
#endif
            return ERR_OK;
        }

        if (SYS_BLK == uwDefectSpBlk)
        {
            printk("[LLF][ERR] Read cfg all FAIL Bk:%d\r\n", uwPhyBank);
            //return ERR_READ_SBLK_ECC;
        }
    }

    return ERR_READ_SBLK;
}

U32 llfReadDbtFromSblk(U16 (*uwSblkNo)[SYS_BLK], U32 ulDbtVaddr, U32 ulDbtSz)
{
    U8 ubBank;
    U16 uwSpBlk, uwPage, uwDefectSpBlk;
    U32 ulCmp, ulRet, ulCblkId;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U8 ubLunNo;
    U16 uwPhyBlk = 0xFF, uwPhyBank = 0xFF;

    for(ubBank = 0; ubBank < gubSysBankNo; ubBank++)
    {
        uwDefectSpBlk = 0;

        ubLunNo = ubBank / NandPara.ubBankNumPerLun;

        for (uwSpBlk = 0; uwSpBlk < SYS_BLK; uwSpBlk++)
        {
            if (0xF == uwSblkNo[ubBank][uwSpBlk])
            {
                uwDefectSpBlk++;
                continue;
            }
#ifdef SBLK_EXPAND
            uwPhyBank = gubSblkBankStart + ubBank;
            uwPhyBlk = uwSblkNo[ubBank][uwSpBlk];
#else
            uwPhyBank = ubBank;
            uwPhyBlk = uwSpBlk;
#endif
            for (uwPage = DBT_PAGENO; uwPage < (DBT_PAGENO + DBT_PAGENUM); uwPage++)
            {
                cache_area_dinval(TEMP_BUF_ADDR, DRAM_DATA_SIZE);
                cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), DRAM_HEAD_SIZE);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_READ, uwPhyBank);

                // 16k-page read
                llfFCCmdRead_DRAM(ulMode, uwPhyBank, ubLunNo, uwPhyBlk, uwPage, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);

                ulRet = FCCompletionPolling(&ulCmp, (gul_FW_TAG));

                if ((ERR_OK != ulRet) || ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0))
                {
                    printk("[LLF][ERR] Read dbt FAIL %d,%d,%d Ret:%x Cmp:%x\r\n",
                           uwPhyBlk, uwPhyBank, uwPage, ulRet, ulCmp);
                    uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                    SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                    uwDefectSpBlk++;
                    break;
                }

                ulCblkId = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                if(ulCblkId != CODE_BLK_ID)
                {
                    printk("[LLF][ERR] dbt id FAIL %d,%d,%d Id:%x!=%x\r\n",
                           uwPhyBlk, uwPhyBank, uwPage, ulCblkId, CODE_BLK_ID);
                    uwDefectSpBlk++;
                    uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                    SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                    break;
                }

                printk("[LLF] Get dbt from %d,%d,%d\r\n", uwPhyBlk, uwPhyBank, uwPage);

                memcpy((void *)(ulDbtVaddr + (uwPage - DBT_PAGENO) * DRAM_DATA_SIZE),
                       (const void*)(TEMP_BUF_ADDR), DRAM_DATA_SIZE);
                cache_area_dwbinval(ulDbtVaddr, ulDbtSz); // Write back whole DBT area
                cache_dummy_update_read();

                if ((DBT_PAGENO + DBT_PAGENUM - 1) == uwPage)
                {
                    return ERR_OK;
                }
            }
        }

        if (SYS_BLK == uwDefectSpBlk)
        {
            printk("[LLF][ERR] Read dbt all FAIL Bk:%d\r\n", uwPhyBank);
            return ERR_READ_DBT_ECC;
        }
    }

    return ERR_READ_DBT;
}

#ifdef REPLACE_FW_WITH_SBLK_TABLE
U32 llfReadTableFromSblk(U16 (*uwSblkNo)[SYS_BLK], U32 ulTableVaddr, U32 ulTableSz)
{
    U8 ubBank;
    U16 uwSpBlk, uwPage, uwDefectSpBlk;
    U32 ulCmp, ulRet, ulCblkId;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U8 ubLunNo;
    U16 uwPhyBlk = 0xFF, uwPhyBank = 0xFF;

    for(ubBank = 0; ubBank < gubSysBankNo; ubBank++)
    {
        uwDefectSpBlk = 0;

        ubLunNo = ubBank / NandPara.ubBankNumPerLun;

        for (uwSpBlk = 0; uwSpBlk < SYS_BLK; uwSpBlk++)
        {
            if (0xF == uwSblkNo[ubBank][uwSpBlk])
            {
                uwDefectSpBlk++;
                continue;
            }
#ifdef SBLK_EXPAND
            uwPhyBank = gubSblkBankStart + ubBank;
            uwPhyBlk = uwSblkNo[ubBank][uwSpBlk];
#else
            uwPhyBank = ubBank;
            uwPhyBlk = uwSpBlk;
#endif

            for (uwPage = TABLE_PAGENO; uwPage < (TABLE_PAGENO + TABLE_PAGENUM); uwPage++)
            {
                cache_area_dinval(TEMP_BUF_ADDR, DRAM_DATA_SIZE);
                cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), DRAM_HEAD_SIZE);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_READ, uwPhyBank);

                // 16k-page read
                llfFCCmdRead_DRAM(ulMode, uwPhyBank, ubLunNo, uwPhyBlk, uwPage, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);

                ulRet = FCCompletionPolling(&ulCmp, (gul_FW_TAG));

                if ((ERR_OK != ulRet) || ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0))
                {
                    printk("[LLF][RepFw][ERR] Read table FAIL %d,%d,%d Ret:%x Cmp:%x\r\n",
                           uwPhyBlk, uwPhyBank, uwPage, ulRet, ulCmp);
                    uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                    SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                    uwDefectSpBlk++;
                    break;
                }

                ulCblkId = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                if(ulCblkId != CODE_BLK_ID)
                {
                    printk("[LLF][RepFw][ERR] table id FAIL %d,%d,%d Id:%x!=%x\r\n",
                           uwPhyBlk, uwPhyBank, uwPage, ulCblkId, CODE_BLK_ID);
                    uwDefectSpBlk++;
                    uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                    SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                    break;
                }

                printk("[LLF][RepFw] Get table from %d,%d,%d\r\n", uwPhyBlk, uwPhyBank, uwPage);

                memcpy((void *)(ulTableVaddr + (uwPage - TABLE_PAGENO) * DRAM_DATA_SIZE),
                       (const void*)(TEMP_BUF_ADDR), ulTableSz);
                cache_area_dwbinval(ulTableVaddr, ulTableSz); // Write back whole Table area
                cache_dummy_update_read();

                if ((TABLE_PAGENO + TABLE_PAGENUM - 1) == uwPage)
                {
                    return ERR_OK;
                }
            }
        }

        if (SYS_BLK == uwDefectSpBlk)
        {
            printk("[LLF][RepFw][ERR] Read table all FAIL Bk:%d\r\n", uwPhyBank);
            return ERR_READ_TABLE_ECC;
        }
    }

    return ERR_READ_TABLE;
}
#endif

U32 llfReadRemapFromSblk(U16 (*uwSblkNo)[SYS_BLK], U32 ulRemapVaddr, U32 ulRemapSz)
{
    U8 ubBank;
    U16 uwSpBlk, uwPage, uwDefectSpBlk;
    U32 ulCmp, ulRet, ulCblkId;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U8 ubLunNo;
    U16 uwPhyBlk = 0xFF, uwPhyBank = 0xFF;

    for(ubBank = 0; ubBank < gubSysBankNo; ubBank++)
    {
        uwDefectSpBlk = 0;

        ubLunNo = ubBank / NandPara.ubBankNumPerLun;

        for (uwSpBlk = 0; uwSpBlk < SYS_BLK; uwSpBlk++)
        {
            if (0xF == uwSblkNo[ubBank][uwSpBlk])
            {
                uwDefectSpBlk++;
                continue;
            }
#ifdef SBLK_EXPAND
            uwPhyBank = gubSblkBankStart + ubBank;
            uwPhyBlk = uwSblkNo[ubBank][uwSpBlk];
#else
            uwPhyBank = ubBank;
            uwPhyBlk = uwSpBlk;
#endif

            memset((void *)(ulRemapVaddr), 0, BLK_REMAP_PRO_PAGENUM * DRAM_DATA_SIZE);
            for (uwPage = BLK_REMAP_PRO_PAGENO; uwPage < (BLK_REMAP_PRO_PAGENO + BLK_REMAP_PRO_PAGENUM);
                    uwPage++)
            {
                memset((void *)(TEMP_BUF_ADDR), 0, DRAM_DATA_SIZE);
                cache_area_dinval(TEMP_BUF_ADDR, DRAM_DATA_SIZE);
                cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), DRAM_HEAD_SIZE);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_READ, uwPhyBank);

                // 16k-page read
                llfFCCmdRead_DRAM(ulMode, uwPhyBank, ubLunNo, uwPhyBlk, uwPage, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);

                ulRet = FCCompletionPolling(&ulCmp, (gul_FW_TAG));

                if ((ERR_OK != ulRet) || ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0))
                {
                    printk("[LLF][ERR] Read remap FAIL %d,%d,%d Ret:%x Cmp:%x\r\n",
                           uwPhyBlk, uwPhyBank, uwPage, ulRet, ulCmp);
                    uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                    SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                    uwDefectSpBlk++;
                    break;
                }

                ulCblkId = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                if(ulCblkId != CODE_BLK_ID)
                {
                    printk("[LLF][ERR] remap id FAIL %d,%d,%d Id:%x!=%x\r\n",
                           uwPhyBlk, uwPhyBank, uwPage, ulCblkId, CODE_BLK_ID);
                    uwDefectSpBlk++;
                    uwSblkNo[ubBank][uwSpBlk] = 0xF;
#ifdef SBLK_EXPAND
                    SblkExpdMarkSblk(ubBank, uwSpBlk);
#endif
                    break;
                }

                printk("[LLF] Get remap from %d,%d,%d\r\n", uwPhyBlk, uwPhyBank, uwPage);

                memcpy((void *)(ulRemapVaddr + (uwPage - BLK_REMAP_PRO_PAGENO) * DRAM_DATA_SIZE),
                       (const void*)(TEMP_BUF_ADDR), DRAM_DATA_SIZE);
                cache_area_dwbinval(ulRemapVaddr, ulRemapSz); // Write back whole DBT area
                cache_dummy_update_read();

                if ((BLK_REMAP_PRO_PAGENO + BLK_REMAP_PRO_PAGENUM - 1) == uwPage)
                {
                    return ERR_OK;
                }
            }


        }

        if (SYS_BLK == uwDefectSpBlk)
        {
            printk("[LLF][ERR] Read remap all FAIL Bk:%d\r\n", uwPhyBank);
            return ERR_READ_DBT_ECC;
        }
    }

    return ERR_READ_DBT;
}
#endif
U32 llfWriteSBlkFailHandle(U16 (*SBlkNo)[4])
{
    U32 ret = ERR_READ_SBLK;
#if defined(RL6577_VA)
    U8 llfnobadsblk;
    U32 badbank;
    llfnobadsblk = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_NOBAD_SBLK);
    if(llfnobadsblk == 0 || gubLLFMode == LLF_ONLY_UPDATE_FW)
    {
        llfEraseSblk();
        ret = llfWriteSBlk(SBlkNo);
    }
    else
    {
        U32 bank;
        U32 block_no;
        U32 banknum;
        llfInitErrorMessage();
        if(NandPara.ubBankNum > 8)
        {
            banknum = 8;
        }
        else
        {
            banknum = NandPara.ubBankNum;
        }
#ifndef SBLK_EXPAND
        for(bank = 0; bank < banknum; bank++)
#else
        for(bank = gubSblkBankStart; bank < gubSblkBankStart + banknum; bank++)
#endif
        {
            badbank = 0;
#ifndef SBLK_EXPAND
            for(block_no = 0; block_no < SYS_BLK; block_no++)
#else
            for(block_no = gubSblkStart; block_no < gubSblkStart + SYS_BLK; block_no++)
#endif
            {
                if((1 << (block_no + bank * 4)) & gulFailedImageBitMap)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Bank %d block %d SBLK is bad.\r\n", bank, block_no);
                    if(!badbank)
                    {
                        llfAddErrorMessage(bank, 0, ERR_READ_SBLK);
                        badbank = 1;
                    }
                }
            }
        }
        llfEraseSblk();
    }
#else
    llfEraseSblk();
    ret = llfWriteSBlk(SBlkNo);
#endif
    return ret;
}

U32 llfWriteCodeBlockFailHandle()
{
    U32 ret = ERR_READ_SBLK;
#if defined(RL6577_VA)
    U8 llfnobadsblk;
    U32 badbank;
    llfnobadsblk = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_NOBAD_SBLK);
    if(llfnobadsblk == 1 && gubLLFMode != LLF_ONLY_UPDATE_FW)
    {
        U32 bank, block_no;
        U32 banknum;
        llfInitErrorMessage();
        if(NandPara.ubBankNum > 8)
        {
            banknum = 8;
        }
        else
        {
            banknum = NandPara.ubBankNum;
        }
#ifndef SBLK_EXPAND
        for(bank = 0; bank < banknum; bank++)
#else
        for(bank = gubSblkBankStart; bank < gubSblkBankStart + banknum; bank++)
#endif
        {
            badbank = 0;
#ifndef SBLK_EXPAND
            for(block_no = 0; block_no < SYS_BLK; block_no++)
#else
            for(block_no = gubSblkStart; block_no < gubSblkStart + SYS_BLK; block_no++)
#endif
            {
                if((1 << (block_no + bank * 4)) & gulFailedImageBitMap)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Bank %d block %d SBLK is bad.\r\n", bank, block_no);
                    if(!badbank)
                    {
                        llfAddErrorMessage(bank, 0, ERR_READ_SBLK);
                        badbank = 1;
                    }
                }
            }
        }
        llfEraseSblk();
    }
    else
    {
        ret = ERR_OK;
    }
#else
    ret = ERR_OK;
#endif
    return ret;
}
U32 llfGetSBlkFailHandle()
{
    U32 ret = ERR_READ_SBLK;
#if defined(RL6577_VA)
    U8 llfnobadsblk;
    U32 badbank;
    llfnobadsblk = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_NOBAD_SBLK);
    if(llfnobadsblk == 0 || gubLLFMode == LLF_ONLY_UPDATE_FW)
    {
        return ERR_OK;
    }

    U32 bank;
    U32 block_no;
    U32 banknum;
    llfInitErrorMessage();
    if(NandPara.ubBankNum > 8)
    {
        banknum = 8;
    }
    else
    {
        banknum = NandPara.ubBankNum;
    }
#ifndef SBLK_EXPAND
    for(bank = 0; bank < banknum; bank++)
#else
    for(bank = gubSblkBankStart; bank < gubSblkBankStart + banknum; bank++)
#endif
    {
        badbank = 0;
#ifndef SBLK_EXPAND
        for(block_no = 0; block_no < SYS_BLK; block_no++)
#else
        for(block_no = gubSblkStart; block_no < gubSblkStart + SYS_BLK; block_no++)
#endif
        {
            if((1 << (block_no + bank * 4)) & gulFailedImageBitMap)
            {
                llfDbgPrintk(ALWAYS_MSG, "Bank %d block %d SBLK is bad.\r\n", bank, block_no);
                if(!badbank)
                {
                    llfAddErrorMessage(bank, 0, ERR_READ_SBLK);
                    badbank = 1;
                }
            }
        }
    }

#else
    ret = ERR_OK;
#endif
    return ret;

}

#ifdef LLF_AUTO_RMA
void RmaPrint_to_ram(const char *fmt, const U32 *dp, const U8 level, const U8 type)
{
    char *buf;
    int len;
    U32 tx_reg, wr_reg, tx_addr, src_addr, src_len, src_size;
#ifdef BACKEND_FTL
    U32 ticks = gulTicks + gulUartTimeStamp;
#else
    U32 ticks = gulTicks;
#endif
    const char *typed = "%x";
    const U32 *tickp = &ticks;

    SET_REG_32(SYS_SCRATCH_REG1, PAD_UART_TXD_OE);//enable uart txd pad oe
#ifdef UART_HWFIFO_MODE
    CLR_REG_32(0xff50131c, 0x1); //disable uart mask
#endif

    if(level & PRINT_BOTH)
    {
        if(level & PRINT_MEM)
        {
            if(type == PRINT_SLC_ALLPAGE || type == PRINT_TLC_ALLPAGE ||
                    type == PRINT_REMAP || type == PRINT_SNAPSHOT)
            {
                if(gulCurTxLength + AUTORMA_UART_MEM_MAX_LEN > TX_PART1_SIZE)
                {
                    if(gubCurTxPart == TxMemPart1)
                    {
                        tx_reg = TX_PART2_DONE_ADDR;
                        wr_reg = TX_PART1_DONE_ADDR;
                        tx_addr = TX_PART2_ADDR;
                        gubCurTxPart = TxMemPart2;
                    }
                    else //TxMemPart2
                    {
                        tx_reg = TX_PART1_DONE_ADDR;
                        wr_reg = TX_PART2_DONE_ADDR;
                        tx_addr = TX_PART1_ADDR;
                        gubCurTxPart = TxMemPart1;
                    }
                    while(_REG32(tx_reg) != PRINT_IDLE) {} //TODO: Timeout
                    _REG32(wr_reg) = (gulCurTxLength << 8) | type;
                    buf = (char*)(tx_addr);
                    gulCurTxLength = 0;
                }
                else
                {
                    if(gubCurTxPart == TxMemPart1)
                    {
                        buf = (char*)(TX_PART1_ADDR + gulCurTxLength);
                    }
                    else //TxMemPart2
                    {
                        buf = (char*)(TX_PART2_ADDR + gulCurTxLength);
                    }
                }
                memset(buf, '\0', AUTORMA_UART_MEM_MAX_LEN);
                len = vsprintf(buf, fmt, dp);
                len = (len > AUTORMA_UART_MEM_MAX_LEN) ? (AUTORMA_UART_MEM_MAX_LEN) : (len);
                gulCurTxLength += len;
            }
            else if(type == PRINT_TLC_PAGE0 || type == PRINT_SLC_PAGE0 ||
                    type == PRINT_TLC_BLOCK || type == PRINT_SLC_BLOCK ||
                    type == PRINT_UR_FILE)
            {
                if(type == PRINT_TLC_PAGE0 || type == PRINT_SLC_PAGE0)
                {
                    src_addr = TX_PAGE0_ADDR;
                    src_size = TX_PAGE0_SIZE;
                    src_len = gulCurTxPAGE0Length;
                }
                else if(type == PRINT_TLC_BLOCK || type == PRINT_SLC_BLOCK)
                {
                    src_addr = TX_BLOCK_ADDR;
                    src_size = TX_BLOCK_SIZE;
                    src_len = gulCurTxBLOCKLength;
                }
                else if(type == PRINT_UR_FILE)
                {
                    src_addr = TX_UR_ADDR;
                    src_size = TX_UR_SIZE;
                    src_len = gulCurTxURFileLength;
                }

                if(src_len + AUTORMA_UART_MEM_MAX_LEN > src_size) //128KB
                {
                    if(gubCurTxPart == TxMemPart1)
                    {
                        tx_reg = TX_PART2_DONE_ADDR;
                        tx_addr = TX_PART2_ADDR;
                        wr_reg = TX_PART1_DONE_ADDR;
                        gubCurTxPart = TxMemPart2;
                    }
                    else
                    {
                        tx_reg = TX_PART1_DONE_ADDR;
                        tx_addr = TX_PART1_ADDR;
                        wr_reg = TX_PART2_DONE_ADDR;
                        gubCurTxPart = TxMemPart1;
                    }
                    while(_REG32(tx_reg) != PRINT_IDLE) {} //TODO: Timeout
                    if(gubIsSLC)
                    {
                        _REG32(wr_reg) = (gulCurTxLength << 8) | PRINT_SLC_ALLPAGE;
                    }
                    else
                    {
                        _REG32(wr_reg) = (gulCurTxLength << 8) | PRINT_TLC_ALLPAGE;
                    }
                    if(src_len >> 16) //max = 128KB
                    {
                        memcpy((char*)tx_addr, (char*)src_addr, 0x10000);
                        memcpy((char*)(tx_addr + 0x10000), (char*)(src_addr + 0x10000), src_len - 0x10000);
                    }
                    else
                    {
                        memcpy((char*)tx_addr, (char*)src_addr, src_len);
                    }
                    while(_REG32(wr_reg) != PRINT_IDLE) {} //TODO: Timeout
                    _REG32(tx_reg) = (src_len << 8) | type;
                    buf = (char*)(src_addr);
                    gulCurTxLength = 0;
                    src_len = 0;
                }
                else
                {
                    buf = (char*)(src_addr + src_len);
                }
                memset(buf, '\0', AUTORMA_UART_MEM_MAX_LEN);
                len = vsprintf(buf, fmt, dp);
                len = (len > AUTORMA_UART_MEM_MAX_LEN) ? (AUTORMA_UART_MEM_MAX_LEN) : (len);
                src_len += len;
                if(type == PRINT_TLC_PAGE0 || type == PRINT_SLC_PAGE0)
                {
                    gulCurTxPAGE0Length = src_len;
                }
                else if(type == PRINT_TLC_BLOCK || type == PRINT_SLC_BLOCK)
                {
                    gulCurTxBLOCKLength = src_len;
                }
                else if(type == PRINT_UR_FILE)
                {
                    gulCurTxURFileLength = src_len;
                }
            }
            else if(type == PRINT_NANDLOG)
            {
                if(gulCurTxLength + NAND_LOG_LEN > TX_PART1_SIZE)
                {
                    if(gubCurTxPart == TxMemPart1)
                    {
                        tx_reg = TX_PART2_DONE_ADDR;
                        wr_reg = TX_PART1_DONE_ADDR;
                        tx_addr = TX_PART2_ADDR;
                        gubCurTxPart = TxMemPart2;
                    }
                    else //TxMemPart2
                    {
                        tx_reg = TX_PART1_DONE_ADDR;
                        wr_reg = TX_PART2_DONE_ADDR;
                        tx_addr = TX_PART1_ADDR;
                        gubCurTxPart = TxMemPart1;
                    }
                    while(_REG32(tx_reg) != PRINT_IDLE) {} //TODO: Timeout
                    _REG32(wr_reg) = (gulCurTxLength << 8) | type;
                    buf = (char*)(tx_addr);
                    gulCurTxLength = 0;
                }
                else
                {
                    if(gubCurTxPart == TxMemPart1)
                    {
                        buf = (char*)(TX_PART1_ADDR + gulCurTxLength);
                    }
                    else //TxMemPart2
                    {
                        buf = (char*)(TX_PART2_ADDR + gulCurTxLength);
                    }
                }
                memcpy(buf, fmt, NAND_LOG_LEN);
                gulCurTxLength += NAND_LOG_LEN;
            }
        }

        if(level & PRINT_UART)
        {
            (void)vsprintf(0, typed, tickp);
            (void)vsprintf(0, fmt, dp);

            if(type == PRINT_END)
            {
                while(_REG32(TX_PART1_DONE_ADDR) != PRINT_IDLE) {} //TODO: Timeout
                while(_REG32(TX_PART2_DONE_ADDR) != PRINT_IDLE) {} //TODO: Timeout
                _REG32(TX_PART1_DONE_ADDR) = PRINT_END;
                _REG32(TX_PART2_DONE_ADDR) = PRINT_END;
            }
        }
    }
    else if(level & PRINT_MEM_FLUSH)
    {
        if(type == PRINT_SLC_ALLPAGE || type == PRINT_TLC_ALLPAGE ||
                type == PRINT_REMAP || type == PRINT_SNAPSHOT || type == PRINT_NANDLOG)
        {
            if(gubCurTxPart == TxMemPart1)
            {
                while(_REG32(TX_PART2_DONE_ADDR) != PRINT_IDLE) {} //TODO: Timeout
                _REG32(TX_PART1_DONE_ADDR) = (gulCurTxLength << 8) | type;
                while(_REG32(TX_PART1_DONE_ADDR) != PRINT_IDLE) {} //TODO: Timeout
                gubCurTxPart = TxMemPart1;
                gulCurTxLength = 0;
            }
            else //TxMemPart2
            {
                while(_REG32(TX_PART1_DONE_ADDR) != PRINT_IDLE) {} //TODO: Timeout
                _REG32(TX_PART2_DONE_ADDR) = (gulCurTxLength << 8) | type;
                while(_REG32(TX_PART2_DONE_ADDR) != PRINT_IDLE) {} //TODO: Timeout
                gubCurTxPart = TxMemPart2;
                gulCurTxLength = 0;
            }
        }
        else if(type == PRINT_TLC_PAGE0 || type == PRINT_SLC_PAGE0 ||
                type == PRINT_TLC_BLOCK || type == PRINT_SLC_BLOCK ||
                type == PRINT_UR_FILE)
        {
            if(type == PRINT_TLC_PAGE0 || type == PRINT_SLC_PAGE0)
            {
                src_addr = TX_PAGE0_ADDR;
                src_size = TX_PAGE0_SIZE;
                src_len = gulCurTxPAGE0Length;
            }
            else if(type == PRINT_TLC_BLOCK || type == PRINT_SLC_BLOCK)
            {
                src_addr = TX_BLOCK_ADDR;
                src_size = TX_BLOCK_SIZE;
                src_len = gulCurTxBLOCKLength;
            }
            else if(type == PRINT_UR_FILE)
            {
                src_addr = TX_UR_ADDR;
                src_size = TX_UR_SIZE;
                src_len = gulCurTxURFileLength;
            }

            if(gubCurTxPart == TxMemPart1)
            {
                tx_reg = TX_PART2_DONE_ADDR;
                tx_addr = TX_PART2_ADDR;
                wr_reg = TX_PART1_DONE_ADDR;
                gubCurTxPart = TxMemPart2;
            }
            else
            {
                tx_reg = TX_PART1_DONE_ADDR;
                tx_addr = TX_PART1_ADDR;
                wr_reg = TX_PART2_DONE_ADDR;
                gubCurTxPart = TxMemPart1;
            }
            while(_REG32(tx_reg) != PRINT_IDLE) {} //TODO: Timeout
            if(gubIsSLC)
            {
                _REG32(wr_reg) = (gulCurTxLength << 8) | PRINT_SLC_ALLPAGE;
            }
            else
            {
                _REG32(wr_reg) = (gulCurTxLength << 8) | PRINT_TLC_ALLPAGE;
            }
            if(src_len >> 16) //max = 128KB
            {
                memcpy((char*)tx_addr, (char*)src_addr, 0x10000);
                memcpy((char*)(tx_addr + 0x10000), (char*)(src_addr + 0x10000), src_len - 0x10000);
            }
            else
            {
                memcpy((char*)tx_addr, (char*)src_addr, src_len);
            }
            while(_REG32(wr_reg) != PRINT_IDLE) {} //TODO: Timeout
            _REG32(tx_reg) = (src_len << 8) | type;
            while(_REG32(tx_reg) != PRINT_IDLE) {} //TODO: Timeout
            gulCurTxLength = 0;
            src_len = 0;
            if(type == PRINT_TLC_PAGE0 || type == PRINT_SLC_PAGE0)
            {
                gulCurTxPAGE0Length = src_len;
            }
            else if(type == PRINT_TLC_BLOCK || type == PRINT_SLC_BLOCK)
            {
                gulCurTxBLOCKLength = src_len;
            }
            else if(type == PRINT_UR_FILE)
            {
                gulCurTxURFileLength = src_len;
            }
        }
    }

#ifndef UART_HWFIFO_MODE
    while ((READ_REG_32(UART1_BASE + REG_lsr) & LSR_TEMT) == 0) ;//wait uart tx done
    CLR_REG_32(SYS_SCRATCH_REG1, PAD_UART_TXD_OE);//disable uart txd pad oe
#endif
}


void RmaPrintk(const U8 level, const U8 type, const char *fmt, ...)
{
    U32 flag;
#ifdef UART_HWFIFO_MODE
    spin_trylock_irqsave(&g_print_lock, &flag);
#else
    spin_lock_irqsave(&g_print_lock, &flag);
#endif
    RmaPrint_to_ram(fmt, ((const U32 *)&fmt) + 1, level, type);
    spin_unlock_irqrestore(&g_print_lock, &flag);
}

//bubble_sort
void llfSortNandLog(U16 length)
{
    U32 i, j;
    for(i = length - 1; i >= 0; i--)
    {
        U8 has_sorted = false;
        for(j = 0; j < i; j++)
        {
            if(gNLRecIndex[j].Wc < gNLRecIndex[j + 1].Wc)
            {
                //Swap
                NandLogRecIndex NLTemp = gNLRecIndex[j];
                gNLRecIndex[j] = gNLRecIndex[j + 1];
                gNLRecIndex[j + 1] = NLTemp;

                has_sorted = true;
            }
        }

        if(has_sorted == false)
        {
            break;
        }
    }
}

U32 llfRMATitlePrint(U8 isSLC)
{
    U8 ubBankNum = NandPara.ubBankNum;
    U8 ubMemtypeALLPage, ubMemtypePAGE0, ubMemtypeBLOCK;
    U32 i;

    if(isSLC)
    {
        ubMemtypeALLPage = PRINT_SLC_ALLPAGE;
        ubMemtypePAGE0 = PRINT_SLC_PAGE0;
        ubMemtypeBLOCK = PRINT_SLC_BLOCK;
    }
    else
    {
        ubMemtypeALLPage = PRINT_TLC_ALLPAGE;
        ubMemtypePAGE0 = PRINT_TLC_PAGE0;
        ubMemtypeBLOCK = PRINT_TLC_BLOCK;
    }
    RmaPrintk(PRINT_MEM, ubMemtypeALLPage, "Bank,Block,Page,Status(1=ECC/2=Empty)\r\n");
    RmaPrintk(PRINT_MEM, ubMemtypePAGE0, "para[7:6]=isSLCBuff para[5:4]=isSLC para[3:0]=isGC\r\n");
    RmaPrintk(PRINT_MEM, ubMemtypePAGE0, "Bank,Block,WriteCount,para,UNC,LBN0,LBN1,LBN2,LBN3\r\n");
    RmaPrintk(PRINT_MEM, ubMemtypeBLOCK, "DBT[15:0]=DBT_Bank15:0\r\n");

    if(ubBankNum <= 16)
    {
        RmaPrintk(PRINT_MEM, ubMemtypeBLOCK, "Block,");
        for(i = 0; i < ubBankNum; i++)
        {
            RmaPrintk(PRINT_MEM, ubMemtypeBLOCK, "B%d_EPT,", i);
        }
        for(i = 0; i < ubBankNum; i++)
        {
            RmaPrintk(PRINT_MEM, ubMemtypeBLOCK, "B%d_ECC,", i);
        }
        RmaPrintk(PRINT_MEM, ubMemtypeBLOCK, "DBT\r\n");
    }
    else
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                  "[ERR] Not support BankNum %d\r\n", ubBankNum);
        return ERR_CONFIG_MAPPING;
    }
    return ERR_OK;
}


U32 llfRMAResultPrint(U16 uwSpBlk, U8 ubMemtypeBLOCK, U16 *uwEmptyPageNumBySPBlk,
                      U16 *uwECCPageNumBySPBlk, U16 uwRecDBTBySPBLK)
{
    U8 ubBankNum = NandPara.ubBankNum;
    if(ubBankNum == 1)
    {
        RmaPrintk(PRINT_BOTH, ubMemtypeBLOCK, "%d,%d,%d,0x%x\r\n",
                  uwSpBlk,
                  uwEmptyPageNumBySPBlk[0],
                  uwECCPageNumBySPBlk[0],
                  uwRecDBTBySPBLK);
    }
    if(ubBankNum == 2)
    {
        RmaPrintk(PRINT_BOTH, ubMemtypeBLOCK, "%d,%d,%d,%d,%d,0x%x\r\n",
                  uwSpBlk,
                  uwEmptyPageNumBySPBlk[0], uwEmptyPageNumBySPBlk[1],
                  uwECCPageNumBySPBlk[0], uwECCPageNumBySPBlk[1],
                  uwRecDBTBySPBLK);
    }
    else if(ubBankNum == 4)
    {
        RmaPrintk(PRINT_BOTH, ubMemtypeBLOCK, "%d,%d,%d,%d,%d,%d,%d,%d,%d,0x%x\r\n",
                  uwSpBlk,
                  uwEmptyPageNumBySPBlk[0], uwEmptyPageNumBySPBlk[1],
                  uwEmptyPageNumBySPBlk[2], uwEmptyPageNumBySPBlk[3],
                  uwECCPageNumBySPBlk[0], uwECCPageNumBySPBlk[1],
                  uwECCPageNumBySPBlk[2], uwECCPageNumBySPBlk[3],
                  uwRecDBTBySPBLK);
    }
    else if(ubBankNum == 8)
    {
        RmaPrintk(PRINT_BOTH, ubMemtypeBLOCK, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0x%x\r\n",
                  uwSpBlk,
                  uwEmptyPageNumBySPBlk[0], uwEmptyPageNumBySPBlk[1],
                  uwEmptyPageNumBySPBlk[2], uwEmptyPageNumBySPBlk[3],
                  uwEmptyPageNumBySPBlk[4], uwEmptyPageNumBySPBlk[5],
                  uwEmptyPageNumBySPBlk[6], uwEmptyPageNumBySPBlk[7],
                  uwECCPageNumBySPBlk[0], uwECCPageNumBySPBlk[1],
                  uwECCPageNumBySPBlk[2], uwECCPageNumBySPBlk[3],
                  uwECCPageNumBySPBlk[4], uwECCPageNumBySPBlk[5],
                  uwECCPageNumBySPBlk[6], uwECCPageNumBySPBlk[7],
                  uwRecDBTBySPBLK);
    }
    else if(ubBankNum == 16)
    {
        RmaPrintk(PRINT_BOTH, ubMemtypeBLOCK,
                  "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0x%x\r\n",
                  uwSpBlk,
                  uwEmptyPageNumBySPBlk[0], uwEmptyPageNumBySPBlk[1],
                  uwEmptyPageNumBySPBlk[2], uwEmptyPageNumBySPBlk[3],
                  uwEmptyPageNumBySPBlk[4], uwEmptyPageNumBySPBlk[5],
                  uwEmptyPageNumBySPBlk[6], uwEmptyPageNumBySPBlk[7],
                  uwEmptyPageNumBySPBlk[8], uwEmptyPageNumBySPBlk[9],
                  uwEmptyPageNumBySPBlk[10], uwEmptyPageNumBySPBlk[11],
                  uwEmptyPageNumBySPBlk[12], uwEmptyPageNumBySPBlk[13],
                  uwEmptyPageNumBySPBlk[14], uwEmptyPageNumBySPBlk[15],
                  uwECCPageNumBySPBlk[0], uwECCPageNumBySPBlk[1],
                  uwECCPageNumBySPBlk[2], uwECCPageNumBySPBlk[3],
                  uwECCPageNumBySPBlk[4], uwECCPageNumBySPBlk[5],
                  uwECCPageNumBySPBlk[6], uwECCPageNumBySPBlk[7],
                  uwECCPageNumBySPBlk[8], uwECCPageNumBySPBlk[9],
                  uwECCPageNumBySPBlk[10], uwECCPageNumBySPBlk[11],
                  uwECCPageNumBySPBlk[12], uwECCPageNumBySPBlk[13],
                  uwECCPageNumBySPBlk[14], uwECCPageNumBySPBlk[15],
                  uwRecDBTBySPBLK);
    }
    else
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                  "[ERR] not support BankNum %d\r\n", ubBankNum);
        return ERR_CONFIG_MAPPING;
    }
    return ERR_OK;
}

void llfSMARTInfoPrint(U32 ulAddr)
{
    NVME_SMART_INFO_LOG* nvme_smart_log = (NVME_SMART_INFO_LOG*)(ulAddr + SBLK_SIZE + 0x200 +
                                          NVME_ERR_INFO_LOG_SIZE);
    U32 ulSMARTInfo[SMART_INFO_MAX], i;
    ulSMARTInfo[0] = nvme_smart_log->cw;
    ulSMARTInfo[1] = nvme_smart_log->ct;
    ulSMARTInfo[2] = nvme_smart_log->as;
    ulSMARTInfo[3] = nvme_smart_log->ast;
    ulSMARTInfo[4] = nvme_smart_log->pu;
    ulSMARTInfo[5] = (U32)(nvme_smart_log->dur[0]);
    ulSMARTInfo[6] = (U32)(nvme_smart_log->duw[0]);
    ulSMARTInfo[7] = (U32)(nvme_smart_log->hrc[0]);
    ulSMARTInfo[8] = (U32)(nvme_smart_log->hwc[0]);
    ulSMARTInfo[9] = (U32)(nvme_smart_log->cbt[0]);
    ulSMARTInfo[10] = (U32)(nvme_smart_log->pc[0]);
    ulSMARTInfo[11] = (U32)(nvme_smart_log->poh[0]);
    ulSMARTInfo[12] = (U32)(nvme_smart_log->us[0]);
    ulSMARTInfo[13] = (U32)(nvme_smart_log->mdie[0]);
    ulSMARTInfo[14] = (U32)(nvme_smart_log->neile[0]);
    ulSMARTInfo[15] = nvme_smart_log->wctt;
    ulSMARTInfo[16] = nvme_smart_log->cctt;
    ulSMARTInfo[17] = nvme_smart_log->ts[0];
    ulSMARTInfo[19] = nvme_smart_log->ts[1];

    for(i = 0; i < SMART_INFO_MAX; i++)
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[SMART-%d] 0x%x\r\n", (i + 1), ulSMARTInfo[i]);
    }

#if 0
    U32 i;
    ulAddr += SBLK_SIZE + 0x200 + NVME_ERR_INFO_LOG_SIZE;
    for(i = 0; i < NVME_SMART_LOG_SIZE; i += 32)
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[DBG] addr %x data %x_%x_%x_%x %x_%x_%x_%x\r\n",
                  ulAddr + i,
                  _REG32(ulAddr + i + 0), _REG32(ulAddr + i + 4),
                  _REG32(ulAddr + i + 8), _REG32(ulAddr + i + 12),
                  _REG32(ulAddr + i + 16), _REG32(ulAddr + i + 20),
                  _REG32(ulAddr + i + 24), _REG32(ulAddr + i + 28));
    }
#endif
}

void llfReadConfig()
{
    guwDynamicSblockStart = 0;
    guwDynamicSblockEnd = STATIC_SBLK_BLK_UPPER_BOUND + NandPara.ubPlaneNumPerLun - 1;

    guwMpBlkSSStart = (guwDynamicSblockEnd / NandPara.ubPlaneNumPerLun) + 1;
    guwMpBlkSSEnd = guwMpBlkSSStart + 2;

    guwMpBlkL2PStart = guwMpBlkSSEnd + 1;
    U16 uwMpBlkL2pEnd = guwMpBlkL2PStart + 7;
    guwSpBlkUserStart = (uwMpBlkL2pEnd + 1) * NandPara.ubPlaneNumPerLun;

    if(gubConfigTag)
    {
        guwDynamicSblockStart = _REG16(SBLK_ADDR + SBLK_OFFSET_SYSTEM_BLOCK_BEGIN_INDEX);
        guwDynamicSblockEnd = _REG16(SBLK_ADDR + SBLK_OFFSET_SYSTEM_BLOCK_END_INDEX) - 1;

        guwMpBlkSSStart = _REG16(SBLK_ADDR + SBLK_OFFSET_SNAP_MP_BLOCK_BEGIN_INDEX);
        guwMpBlkSSEnd = _REG16(SBLK_ADDR + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX) - 1; //divide

        guwMpBlkL2PStart = _REG16(SBLK_ADDR + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX);
        uwMpBlkL2pEnd = _REG16(SBLK_ADDR + SBLK_OFFSET_L2P_MP_BLOCK_END_INDEX) - 1;
        guwSpBlkUserStart = (uwMpBlkL2pEnd + 1) * NandPara.ubPlaneNumPerLun;

#ifdef BLK_REMAP_PRO
        guwRealMPBlkNum = _MEM32(SBLK_ADDR + SBLK_OFFSET_REMAP_START);
        NandPara.uwMpBlockNumPerLun = _MEM32(SBLK_ADDR + SBLK_OFFSET_REAL_BLOCK_NUM);
        guwReMapTableSizePerBank = _MEM16(SBLK_ADDR + SBLK_OFFSET_REMAP_TABLE_PER_BANK);
#endif
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] use static SBLK config\r\n");
    }

    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] PlaneNum %d\r\n", NandPara.ubPlaneNumPerLun);
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] Dynamic sblock range SpBlk (%d ~ %d)\r\n",
              guwDynamicSblockStart, guwDynamicSblockEnd);
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] Snapshot range MpBlk (%d ~ %d)\r\n", guwMpBlkSSStart,
              guwMpBlkSSEnd);
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] L2P Table range MpBlk (%d ~ %d)\r\n", guwMpBlkL2PStart,
              uwMpBlkL2pEnd);
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] User Data range SpBlk (%d ~ %d)\r\n",
              guwSpBlkUserStart, NandPara.uwBlockNumPerLun - 1);
}


U32 llfScanStaticSysBlk()
{
    U8 ubBank = 0, ubLunNo, ubBankNum;
    U16 uwSpBlk, uwPage;
    U8 isSLC = 1;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 ulCmp, ulRet = ERR_OK;
    U32 ulSblkId;
    U32 err_info0, err_info1;
    U8 ubDispCnt = 0;
    U32 ulTmpTick = 0;

    //record
    U16 uwEmptyPageNumBySPBlk[NandPara.ubBankNum];
    U16 uwECCPageNumBySPBlk[NandPara.ubBankNum];
    U16 uwRecDBTBySPBLK;

    uwPage = 0;
    ubBankNum = MIN(NandPara.ubBankNum, STATIC_SBLK_BANK_UPPER_BOUND);
    uwRecDBTBySPBLK = (1 << ubBankNum) - 1;

    //Scan sblk
    for (uwSpBlk = 0; uwSpBlk < STATIC_SBLK_BLK_UPPER_BOUND; uwSpBlk++) //SpBlk
    {
        //init record for SpBlk
        memset(uwEmptyPageNumBySPBlk, 0, NandPara.ubBankNum * sizeof(U16));
        memset(uwECCPageNumBySPBlk, 0, NandPara.ubBankNum * sizeof(U16));

        //Dispatch
        ubDispCnt = 0;

        for(ubBank = 0; ubBank < ubBankNum; ubBank++) //Bank
        {
            ubLunNo = ubBank / (NandPara.ubChNum * NandPara.ubCENumPerCh);
            gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

            // 16k-page read
            FCSingleReadDRAM(ulMode, ubBank, ubLunNo, uwSpBlk, uwPage,
                             READ_DATA_PHY_ADDR_X(ubBank), DRAM_DATA_SIZE, READ_HEAD_PHY_ADDR_X(ubBank), DRAM_HEAD_SIZE, isSLC);
            ubDispCnt++;
        }

        //check Disp Cnt
        if( (ubDispCnt) != ubBankNum )
        {
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] Disp bank num %d\r\n", ubDispCnt);
            return ERR_DISPATCH_ERR;
        }

        //Poll cmp
        ulTmpTick = GET_TICK;
        while( ubDispCnt )
        {
            ubBank = 0xff;
            ulCmp = 0xffff0000;
            if((FC_TOP_REG(FR_CMPQ_BC) & FC_NORMAL_CMP_QUEUE_MASK) == 0)
            {
                BeFcBusyWait10us(1);
                continue;
            }
            while((FC_TOP_REG(FR_CMPQ_BC) & FC_NORMAL_CMP_QUEUE_MASK) != 0)
            {
                ulCmp = FcPollCompletion(&err_info0, &err_info1);
                if((ulCmp >> 24) == TAG_READ)
                {
                    ubBank = (ulCmp >> 16) & 0xff;
                    if(ubBank > NandPara.ubBankNum)
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] bank %d > BankNum\r\n", ubBank, NandPara.ubBankNum);
                        return ERR_FC_CMP;
                    }
                    ubDispCnt--;
                    break;
                }
                else
                {
                    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] cmp 0x%x cnt %d\r\n", ulCmp, ubDispCnt);
                    return ERR_FC_CMP;
                }
            }

            if(TIMER_GAP(GET_TICK, ulTmpTick) > 1000)
            {
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] poll timeout cnt %d\r\n", ubDispCnt);
                return ERR_READ_SBLK_TIMEOUT;
            }

            if(ubBank == 0xff)
            {
                continue;
            }

            if(ALL_FF & ulCmp) //all_F, CRC_error
            {
                uwEmptyPageNumBySPBlk[ubBank]++;
                RmaPrintk(PRINT_MEM, PRINT_SLC_ALLPAGE, "%d,%d,%d,%d\r\n",
                          ubBank, uwSpBlk, uwPage, TxMemECC);
                continue;
            }
            else if((ECC_UNCORRECT_ERR | CRC_ERR) & ulCmp) //ECC, CRC_error
            {
                uwECCPageNumBySPBlk[ubBank]++;
                RmaPrintk(PRINT_MEM, PRINT_SLC_ALLPAGE, "%d,%d,%d,%d\r\n",
                          ubBank, uwSpBlk, uwPage, TxMemEmpty);
                continue;
            }

            ulSblkId = _REG32(READ_HEAD_ADDR_X(ubBank));
            if((ulSblkId != SBLK_BLK_ID) ||
                    (_REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_LOW) != SBLOCK_GENERAL_CONFIG_TAG_LOW) ||
                    (_REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_HIGH) != SBLOCK_GENERAL_CONFIG_TAG_HIGH)) //SBlk
            {
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                          "[ERR] DBP %d_%d_%d ret %x cmp %x Hdr %x D %x %x\r\n",
                          ubBank, uwSpBlk, uwPage, ulRet, ulCmp, ulSblkId,
                          _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_LOW),
                          _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_HIGH));
            }
            else
            {
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                          "[Info] DBP %d_%d_%d ret %x cmp %x Hdr %x D %x %x\r\n",
                          ubBank, uwSpBlk, uwPage, ulRet, ulCmp, ulSblkId,
                          _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_LOW),
                          _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_HIGH));
            }
        }

        ulRet = llfRMAResultPrint(uwSpBlk, PRINT_SLC_BLOCK, uwEmptyPageNumBySPBlk, uwECCPageNumBySPBlk,
                                  uwRecDBTBySPBLK);
        if(ulRet != ERR_OK)
        {
            return ulRet;
        }
    }
    return ulRet;
}


U32 llfScanSysBlk()
{
    U8 ubBank = 0, ubLunNo;
    U16 uwSpBlk, uwPage;
    U8 isSLC = 1;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 ulCmp, ulRet = ERR_OK;
    U32 ulSblkId;
    U32 err_info0, err_info1;
    U8 ubDispCnt = 0, ubDefectCnt = 0;
    U32 ulTmpTick = 0;

    //record
    U16 uwEmptyPageNumBySPBlk[NandPara.ubBankNum];
    U16 uwECCPageNumBySPBlk[NandPara.ubBankNum];
    U16 uwRecDBTBySPBLK;

    U16 uwMpBlkL2pEnd = NandPara.uwMpBlockNumPerLun;
    U32 ulSnapDataBuffStart = 0;
    U32 ulSnapDataBuffEnd = 0;

    U32 DmemAddrEnd = 0x80240000, DmemAddrStart = 0x80218000;
    gubSSDataPageSize = (((DmemAddrEnd - DmemAddrStart) + (NandPara.ulLastMpPageByteNum - 1)) /
                         NandPara.ulLastMpPageByteNum); //MPP

    U16 uwSpBlkL2PStart = guwMpBlkL2PStart * NandPara.ubPlaneNumPerLun; //L2PR
    //Scan sblk
    for (uwSpBlk = 0; uwSpBlk < uwSpBlkL2PStart; uwSpBlk++) //SpBlk
    {
        //init record for SpBlk
        memset(uwEmptyPageNumBySPBlk, 0, NandPara.ubBankNum * sizeof(U16));
        memset(uwECCPageNumBySPBlk, 0, NandPara.ubBankNum * sizeof(U16));
        uwRecDBTBySPBLK = 0;
        for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
        {
            if(llfIsBlockBad(DBT_ADDR, ubBank, uwSpBlk))
            {
                uwRecDBTBySPBLK |= (1 << ubBank);
            }
        }

        for (uwPage = 0; uwPage < NandPara.uwSLCPageNumPerBlock; uwPage++) //single Page
        {
            //Dispatch
            ubDispCnt = 0;
            ubDefectCnt = 0;

            for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++) //Bank
            {
                if(ubBank >= STATIC_SBLK_BANK_UPPER_BOUND || uwSpBlk >= STATIC_SBLK_BLK_UPPER_BOUND)
                {
#if 0
                    if(uwRecDBTBySPBLK & (1 << ubBank))
                    {
                        ubDefectCnt++;
                        continue;
                    }
#endif
                }
                else if(ubBank < STATIC_SBLK_BANK_UPPER_BOUND && uwSpBlk < STATIC_SBLK_BLK_UPPER_BOUND
                        && uwPage == 0) //Static system block
                {
                    ubDefectCnt++;
                    continue;
                }

                ubLunNo = ubBank / (NandPara.ubChNum * NandPara.ubCENumPerCh);
                gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

                // 16k-page read
                FCSingleReadDRAM(ulMode, ubBank, ubLunNo, uwSpBlk, uwPage,
                                 READ_DATA_PHY_ADDR_X(ubBank), DRAM_DATA_SIZE, READ_HEAD_PHY_ADDR_X(ubBank), DRAM_HEAD_SIZE, isSLC);
                ubDispCnt++;
            }

            //check Disp Cnt
            if( (ubDispCnt + ubDefectCnt) != NandPara.ubBankNum )
            {
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] Disp bank num %d_%d\r\n", ubDispCnt, ubDefectCnt);
                return ERR_DISPATCH_ERR;
            }

            //Poll cmp
            ulTmpTick = GET_TICK;
            while( ubDispCnt )
            {
                ubBank = 0xff;
                ulCmp = 0xffff0000;
                if((FC_TOP_REG(FR_CMPQ_BC) & FC_NORMAL_CMP_QUEUE_MASK) == 0)
                {
                    BeFcBusyWait10us(1);
                    continue;
                }
                while((FC_TOP_REG(FR_CMPQ_BC) & FC_NORMAL_CMP_QUEUE_MASK) != 0)
                {
                    ulCmp = FcPollCompletion(&err_info0, &err_info1);
                    if((ulCmp >> 24) == TAG_READ)
                    {
                        ubBank = (ulCmp >> 16) & 0xff;
                        if(ubBank > NandPara.ubBankNum)
                        {
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] bank %d > BankNum\r\n", ubBank, NandPara.ubBankNum);
                            return ERR_FC_CMP;
                        }
                        ubDispCnt--;
                        break;
                    }
                    else
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] cmp 0x%x cnt %d\r\n", ulCmp, ubDispCnt);
                        return ERR_FC_CMP;
                    }
                }

                if(TIMER_GAP(GET_TICK, ulTmpTick) > 1000)
                {
                    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] poll timeout cnt %d\r\n", ubDispCnt);
                    return ERR_READ_SBLK_TIMEOUT;
                }

                if(ubBank == 0xff)
                {
                    continue;
                }

                if(ALL_FF & ulCmp) //all_F, CRC_error
                {
                    uwEmptyPageNumBySPBlk[ubBank]++;
                    RmaPrintk(PRINT_MEM, PRINT_SLC_ALLPAGE, "%d,%d,%d,%d\r\n",
                              ubBank, uwSpBlk, uwPage, TxMemEmpty);
                    continue;
                }
                else if((ECC_UNCORRECT_ERR | CRC_ERR) & ulCmp) //ECC, CRC_error
                {
                    uwECCPageNumBySPBlk[ubBank]++;
                    RmaPrintk(PRINT_MEM, PRINT_SLC_ALLPAGE, "%d,%d,%d,%d\r\n",
                              ubBank, uwSpBlk, uwPage, TxMemECC);
                    continue;
                }

                ulSblkId = _REG32(READ_HEAD_ADDR_X(ubBank));
                if(ubBank < STATIC_SBLK_BANK_UPPER_BOUND
                        && uwSpBlk < STATIC_SBLK_BLK_UPPER_BOUND) //Code block range
                {
                    if(uwPage == 1)
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] CODE_BLK DBP %d_%d_%d ret %x cmp %x Hdr %x\r\n",
                                  ubBank, uwSpBlk, uwPage, ulRet, ulCmp, ulSblkId);
                    }
                    if(ulSblkId != CODE_BLK_ID)
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] CODE_BLK DBP %d_%d_%d ret %x cmp %x Hdr %x\r\n",
                                  ubBank, uwSpBlk, uwPage, ulRet, ulCmp, ulSblkId);
                    }
                }
                else if ((uwSpBlk >= STATIC_SBLK_BLK_UPPER_BOUND && uwSpBlk <= guwDynamicSblockEnd) ||
                         (uwSpBlk < STATIC_SBLK_BLK_UPPER_BOUND && ubBank >= STATIC_SBLK_BANK_UPPER_BOUND))
                {
                    if(uwPage == 0)
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                                  "[Info] SBLK_BLK DBP %d_%d_%d ret %x cmp %x Hdr %x D %x %x\r\n",
                                  ubBank, uwSpBlk, uwPage,  ulRet, ulCmp, ulSblkId,
                                  _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_LOW),
                                  _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_HIGH));
                    }
                    if((ulSblkId != SBLK_BLK_ID) ||
                            (_REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_LOW) != SBLOCK_GENERAL_CONFIG_TAG_LOW) ||
                            (_REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_HIGH) != SBLOCK_GENERAL_CONFIG_TAG_HIGH))
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] SBLK_BLK DBP %d_%d_%d ret %x cmp %x Hdr %x D %x %x\r\n",
                                  ubBank, uwSpBlk, uwPage,  ulRet, ulCmp, ulSblkId,
                                  _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_LOW),
                                  _REG32(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_TAG_HIGH));
                    }
                    else
                    {
                        if(!gubConfigTag)
                        {
                            ulSnapDataBuffStart = _REG32(READ_DATA_ADDR_X(ubBank) + 0xA74); //0x174
                            ulSnapDataBuffEnd = _REG32(READ_DATA_ADDR_X(ubBank) + 0xA78); //0x178

                            gubSSDataPageSize += (((ulSnapDataBuffEnd - ulSnapDataBuffStart) +
                                                   (NandPara.ulLastMpPageByteNum - 1)) / NandPara.ulLastMpPageByteNum);

                            guwMpBlkSSStart = _REG16(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_SNAP_MP_BLOCK_BEGIN_INDEX);
                            guwMpBlkSSEnd = _REG16(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX) - 1;

                            guwMpBlkL2PStart = _REG16(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX);
                            uwMpBlkL2pEnd = _REG16(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_L2P_MP_BLOCK_END_INDEX) - 1;
                            guwSpBlkUserStart = (uwMpBlkL2pEnd + 1) * NandPara.ubPlaneNumPerLun;

                            guwDynamicSblockStart = _REG16(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_SYSTEM_BLOCK_BEGIN_INDEX);
                            guwDynamicSblockEnd = _REG16(READ_DATA_ADDR_X(ubBank) + SBLK_OFFSET_SYSTEM_BLOCK_END_INDEX) - 1;

                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] use dynamic SBLK config\r\n");
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] PlaneNum %d\r\n", NandPara.ubPlaneNumPerLun);
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] Dynamic sblock range SpBlk (%d ~ %d)\r\n",
                                      guwDynamicSblockStart, guwDynamicSblockEnd);
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] Snapshot range MpBlk (%d ~ %d)\r\n", guwMpBlkSSStart,
                                      guwMpBlkSSEnd);
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] L2P Table range MpBlk (%d ~ %d)\r\n", guwMpBlkL2PStart,
                                      uwMpBlkL2pEnd);
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] User Data range SpBlk (%d ~ %d)\r\n",
                                      guwSpBlkUserStart, NandPara.uwBlockNumPerLun - 1);
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] Snapshot data (0x%x ~ 0x%x) page size %d\r\n",
                                      ulSnapDataBuffStart, ulSnapDataBuffEnd, gubSSDataPageSize);
                        }
                        if(gubConfigTag != DYNAMIC_SBLK_INFO)
                        {
                            gubConfigTag = DYNAMIC_SBLK_INFO;
                        }
                    }
                }
                else if(uwSpBlk >= (guwMpBlkSSStart / NandPara.ubPlaneNumPerLun)
                        && uwSpBlk < ((guwMpBlkSSEnd + 1) * NandPara.ubPlaneNumPerLun))
                {
                    if( uwPage == 0)
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] SNAPSHOT_BLK DBP %d_%d_%d ret %x cmp %x Hdr %x\r\n",
                                  ubBank, uwSpBlk, uwPage, ulRet, ulCmp, ulSblkId);
                    }
                    if(ulSblkId != SNAPSHOT_BLK_ID)
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] SNAPSHOT_BLK DBP %d_%d_%d ret %x cmp %x Hdr %x\r\n",
                                  ubBank, uwSpBlk, uwPage, ulRet, ulCmp, ulSblkId);
                    }
                }
            }
        }

        ulRet = llfRMAResultPrint(uwSpBlk, PRINT_SLC_BLOCK, uwEmptyPageNumBySPBlk, uwECCPageNumBySPBlk,
                                  uwRecDBTBySPBLK);
        if(ulRet != ERR_OK)
        {
            return ulRet;
        }
    }

    if(!gubConfigTag) //no found Config
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                  "[ERR] No Config found! Scan user from block %d\r\n", guwSpBlkUserStart);
    }
    return ulRet;
}

U32 llfScanUserBlk(U8 isSLC) //1:SLC, 0:TLC/QLC
{
    U8 ubBank = 0, ubLunNo;
    U16 uwSpBlk, uwPage, uwPageNum;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 ulCmp, ulRet = ERR_OK;
    U32 err_info0, err_info1;
    U8 ubDispCnt = 0, ubDefectCnt = 0;
    U32 ulTmpTick = 0;
    U8 ubMemtypeALLPage, ubMemtypePAGE0, ubMemtypeBLOCK;

    //record
    U16 uwEmptyPageNumBySPBlk[NandPara.ubBankNum];
    U16 uwECCPageNumBySPBlk[NandPara.ubBankNum];
    U16 uwRecDBTBySPBLK;

    //Header
    U32 ulHeaderAddr;
    head_content* headerContent;
    U32 ulSLBN_old, ulSLBN_n, i, j;
#ifdef EXTEND_LBA
    U64 ulSpecialLBAStartAddr;
#else
    U32 ulSpecialLBAStartAddr;
#endif
    U64 ullWriteCount = 0;
    U32 ulLBN[4] = {0};
    U32 ulHpara = 0;
    U32 ulUNC = 0;

    ulSpecialLBAStartAddr = ((gulMaxLBAAddr + (1 << gubSTableEntrySizeShift) * 8 - 1) & ~((
                                 1 << gubSTableEntrySizeShift) * 8 - 1));
    ulSLBN_old = (ulSpecialLBAStartAddr + (SPLBN_UART_BYTES_OFFSET_OLD >> SECTOR_BYTE_SHIFT)) >>
                 gubCacheSectorNumPerLbnShift;

    ulSLBN_n = (ulSpecialLBAStartAddr + (SPLBN_UART_BYTES_OFFSET_NEW >> SECTOR_BYTE_SHIFT)) >>
               gubCacheSectorNumPerLbnShift;

    U16 uwSpBlkL2PStart = guwMpBlkL2PStart * NandPara.ubPlaneNumPerLun; //L2PR


    if(isSLC)
    {
        uwPageNum = NandPara.uwSLCPageNumPerBlock;
        ubMemtypeALLPage = PRINT_SLC_ALLPAGE;
        ubMemtypePAGE0 = PRINT_SLC_PAGE0;
        ubMemtypeBLOCK = PRINT_SLC_BLOCK;
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] SLBN Old 0x%x New 0x%x\r\n", ulSLBN_old, ulSLBN_n);
    }
    else
    {
        uwPageNum = NandPara.uwPageNumPerBlock;
        ubMemtypeALLPage = PRINT_TLC_ALLPAGE;
        ubMemtypePAGE0 = PRINT_TLC_PAGE0;
        ubMemtypeBLOCK = PRINT_TLC_BLOCK;
    }
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] m %d pageNum %d\r\n", isSLC, uwPageNum);

    //Scan user Spblk
    for (uwSpBlk = uwSpBlkL2PStart; uwSpBlk < NandPara.uwBlockNumPerLun; uwSpBlk++) //SpBlk
    {
        //init record for SpBlk
        memset(uwEmptyPageNumBySPBlk, 0, NandPara.ubBankNum * sizeof(U16));
        memset(uwECCPageNumBySPBlk, 0, NandPara.ubBankNum * sizeof(U16));
        uwRecDBTBySPBLK = 0;
        for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
        {
            if(llfIsBlockBad(DBT_ADDR, ubBank, uwSpBlk))
            {
                uwRecDBTBySPBLK |= (1 << ubBank);
            }
        }

        //scan all page, bank
        for (uwPage = 0; uwPage < uwPageNum; uwPage++) //Page
        {
            //Dispatch
            ubDispCnt = 0;
            ubDefectCnt = 0;
            for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++) //Bank
            {
#if 0
                if(uwRecDBTBySPBLK & (1 << ubBank))
                {
                    ubDefectCnt++;
                    continue;
                }
#endif
                ubLunNo = ubBank / (NandPara.ubChNum * NandPara.ubCENumPerCh);
                gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

                // 16k-page read
                FCSingleReadDRAM(ulMode, ubBank, ubLunNo, uwSpBlk, uwPage,
                                 READ_DATA_PHY_ADDR_X(ubBank), DRAM_DATA_SIZE,
                                 READ_HEAD_PHY_ADDR_X(ubBank), DRAM_HEAD_SIZE, isSLC);
                ubDispCnt++;
            }

            //check Disp Cnt
            if( (ubDispCnt + ubDefectCnt) != NandPara.ubBankNum )
            {
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] Disp bank num %d_%d\r\n", ubDispCnt, ubDefectCnt);
                return ERR_DISPATCH_ERR;
            }

            //Poll cmp
            ulTmpTick = GET_TICK;
            while( ubDispCnt )
            {
                ubBank = 0xff;
                ulCmp = 0xffff0000;
                if((FC_TOP_REG(FR_CMPQ_BC) & FC_NORMAL_CMP_QUEUE_MASK) == 0)
                {
                    BeFcBusyWait10us(1);
                    continue;
                }
                while((FC_TOP_REG(FR_CMPQ_BC) & FC_NORMAL_CMP_QUEUE_MASK) != 0)
                {
                    ulCmp = FcPollCompletion(&err_info0, &err_info1);
                    if((ulCmp >> 24) == TAG_READ)
                    {
                        ubBank = (ulCmp >> 16) & 0xff;
                        if(ubBank > NandPara.ubBankNum)
                        {
                            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] bank %d > BankNum\r\n", ubBank, NandPara.ubBankNum);
                            return ERR_FC_CMP;
                        }
                        ubDispCnt--;
                        break;
                    }
                    else
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] cmp 0x%x cnt %d\r\n", ulCmp, ubDispCnt);
                        return ERR_FC_CMP;
                    }
                }
                if(TIMER_GAP(GET_TICK, ulTmpTick) > 1000)
                {
                    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] poll timeout cnt %d\r\n", ubDispCnt);
                    return ERR_READ_SBLK_TIMEOUT;
                }
                if(ubBank == 0xff)
                {
                    continue;
                }

                if(ALL_FF & ulCmp) //all_F (CRC)
                {
                    uwEmptyPageNumBySPBlk[ubBank]++;
                    RmaPrintk(PRINT_MEM, ubMemtypeALLPage, "%d,%d,%d,%d\r\n",
                              ubBank, uwSpBlk, uwPage, TxMemEmpty);
                    continue;
                }
                else if((ECC_UNCORRECT_ERR | CRC_ERR) & ulCmp) //CRC, ECC_unc
                {
                    uwECCPageNumBySPBlk[ubBank]++;
                    RmaPrintk(PRINT_MEM, ubMemtypeALLPage, "%d,%d,%d,%d\r\n",
                              ubBank, uwSpBlk, uwPage, TxMemECC);
                    continue;
                }

                //Header
                ulHeaderAddr = READ_HEAD_ADDR_X(ubBank);
                headerContent = (struct _head_content*)(ulHeaderAddr);
                ullWriteCount = ulHpara = ulUNC = 0;
                for(i = 0; i < 4; i++) //LbnNumPerSpPage
                {
                    ullWriteCount |= (U16)(headerContent->head_4k[i].bits.WCount_part) << i * 16;
                }
                for(i = 0; i < 4; i++) //LbnNumPerSpPage
                {
                    //record header
                    ulHpara |= (U8)(headerContent->head_4k[i].bits.AsU8_1) << i * 8;
                    ulUNC |= (U8)(headerContent->head_4k[i].bits.UNC) << i * 8;
                    ulLBN[i] = headerContent->head_4k[i].bits.LBN;

                    //record NAND log index
                    if( (ulSLBN_old == (ulLBN[i] & (~0xff)) ) ||
                            ((ulLBN[i] - ulSLBN_n) < SPLBN_UART_MAX_LBN_INDEX_NEW) )
                    {
                        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                                  "[Info] NandLog Rec %d DBPO %d_%d_%d_%d SLBN 0x%x SLC %d WC 0x%x%x\r\n",
                                  guwCurNLRecIndex, ubBank, uwSpBlk, uwPage, i, ulLBN[i], isSLC,
                                  (U32)(ullWriteCount >> 32), (U32)(ullWriteCount));

                        if(guwCurNLRecIndex < NAND_LOG_REC_MAX)
                        {
                            //record BDPO of SLBN.
                            gNLRecIndex[guwCurNLRecIndex].Wc	 = ullWriteCount;
                            gNLRecIndex[guwCurNLRecIndex].Block	 = uwSpBlk;
                            gNLRecIndex[guwCurNLRecIndex].Page	 = uwPage;
                            gNLRecIndex[guwCurNLRecIndex].Bank	 = ubBank & 0xf;
                            gNLRecIndex[guwCurNLRecIndex].Offset = i & 0x3;
                            gNLRecIndex[guwCurNLRecIndex].isSLC  = isSLC & 0x1;
                        }
                        else if(guwCurNLRecIndex == NAND_LOG_REC_MAX)
                        {
                            llfSortNandLog(NAND_LOG_REC_MAX);

                            for(j = 0; j < NAND_LOG_REC_MAX; j++)
                            {
                                gubNLHashTable[j] = j;
                            }
                        }
                        if(guwCurNLRecIndex >= NAND_LOG_REC_MAX)
                        {
                            /*
                            	* HashTable used to rec Wc rank. *
                            	Step1: Find min_Wc by Hash
                            	Step2: If current_Wc > min_Wc => Record current Wc
                            	Step3: Find best rank from Hash
                            	Step4: Shift Hash rank
                            */
                            U8 ubHashMinIndex = NAND_LOG_REC_MAX - 1;
                            for(j = 0; j < NAND_LOG_REC_MAX; j++)
                            {
                                if(gubNLHashTable[j] == (NAND_LOG_REC_MAX - 1))
                                {
                                    ubHashMinIndex = j;
                                    break;
                                }
                            }
                            if(ullWriteCount >= gNLRecIndex[ubHashMinIndex].Wc)
                            {
                                gNLRecIndex[ubHashMinIndex].Wc      = ullWriteCount;
                                gNLRecIndex[ubHashMinIndex].Block	= uwSpBlk;
                                gNLRecIndex[ubHashMinIndex].Page	= uwPage;
                                gNLRecIndex[ubHashMinIndex].Bank	= ubBank & 0xf;
                                gNLRecIndex[ubHashMinIndex].Offset  = i & 0x3;
                                gNLRecIndex[ubHashMinIndex].isSLC   = isSLC & 0x1;

                                U8 ubHashRank = NAND_LOG_REC_MAX - 1;
                                for(j = 0; j < NAND_LOG_REC_MAX ; j++)
                                {
                                    if(j == ubHashMinIndex)
                                    {
                                        continue;
                                    }
                                    if(ullWriteCount >= gNLRecIndex[j].Wc)
                                    {
                                        ubHashRank = MIN(ubHashRank, gubNLHashTable[j]);
                                    }
                                }
                                for(j = 0; j < NAND_LOG_REC_MAX; j++)
                                {
                                    if(gubNLHashTable[j] >= ubHashRank)
                                    {
                                        gubNLHashTable[j] += 1;
                                    }
                                }
                                gubNLHashTable[ubHashMinIndex] = ubHashRank;
                            }
                        }
                        guwCurNLRecIndex++;
                    }
                }

                if(uwPage == 0)
                {
                    RmaPrintk(PRINT_MEM, ubMemtypePAGE0,
                              "%d,%d,0x%x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\r\n",
                              ubBank, uwSpBlk, (U32)(ullWriteCount >> 32), (U32)(ullWriteCount), ulHpara, ulUNC,
                              ulLBN[0], ulLBN[1], ulLBN[2], ulLBN[3]);
                }
            }
        }

        ulRet = llfRMAResultPrint(uwSpBlk, ubMemtypeBLOCK, uwEmptyPageNumBySPBlk, uwECCPageNumBySPBlk,
                                  uwRecDBTBySPBLK);
        if(ulRet != ERR_OK)
        {
            return ulRet;
        }
    }

    //Send last result
    RmaPrintk(PRINT_MEM_FLUSH, ubMemtypeALLPage, " ");
    RmaPrintk(PRINT_MEM_FLUSH, ubMemtypePAGE0, " ");
    RmaPrintk(PRINT_MEM_FLUSH, ubMemtypeBLOCK, " ");

    if(ulRet != ERR_OK)
    {
        return ulRet;
    }

    return ulRet;
}

U32 llfScanSnapshotBlk()
{
    U8 ubBank = 0, ubLunNo, isSLC = 1;
    U16 uwSpBlk, uwPage;
    U32 ulRet = ERR_OK, ret = ERR_OK;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 ulCmp;
    U32 ulSblkId = 0;

    U32 ulHeadSizePerMPP = (DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
    U16 uwSpBlkSSStart = (guwMpBlkSSStart * NandPara.ubPlaneNumPerLun);
    U16 uwSpBlkSSEnd = ((guwMpBlkSSEnd + 1) * NandPara.ubPlaneNumPerLun);

    //printk("[DBG] NandPara.ubMpPageByteNumShift 0x%x\r\n", NandPara.ubMpPageByteNumShift);
    //printk("[DBG] NandPara.ubPlaneNumPerLunShift 0x%x\r\n", NandPara.ubPlaneNumPerLunShift);
    //printk("[DBG] ulHeadSizePerMPP 0x%x\r\n", ulHeadSizePerMPP);

    U32 ulMpPageByte = NandPara.ubSectorNumPerPage * SECTOR_BYTE_SHIFT * NandPara.ubPlaneNumPerLun;

    if( (gubSSDataPageSize * ulMpPageByte) > READ_DATA_SIZE )
    {
        printk("[ERR] Snapshot out of memory 0x%x > 0x%x\r\n",
               (gubSSDataPageSize * ulMpPageByte), READ_DATA_SIZE);
        return ERR_OUT_OF_MEM;
    }

    //Scan Mpblk
    for (uwSpBlk = uwSpBlkSSStart; uwSpBlk < uwSpBlkSSEnd; uwSpBlk += NandPara.ubPlaneNumPerLun) //MpBlk
    {
        for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++) //Bank
        {
            ubLunNo = ubBank / NandPara.ubBankNumPerLun;
            printk("[Result] Snapshot uwMpBlk %d ubBank %d\r\n", uwSpBlk / NandPara.ubPlaneNumPerLun,
                   ubBank);
            for(uwPage = 0; uwPage < gubSSDataPageSize; uwPage++) //Page
            {
                //MPP => 16K * plane data
                gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

                FCMultiReadDRAM(ulMode, ubBank, ubLunNo, uwSpBlk, uwPage,
                                (READ_DATA_PHY_ADDR + (uwPage * ulMpPageByte)),
                                ulMpPageByte,
                                (READ_HEAD_PHY_ADDR + (uwPage * ulHeadSizePerMPP)), ulHeadSizePerMPP, isSLC);

                ret = FCCompletionPolling(&ulCmp, (gul_FW_TAG));

                ulSblkId = _REG32((READ_HEAD_ADDR + (uwPage * ulHeadSizePerMPP)));
                if ((ERR_OK != ret) || ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0) || ulSblkId != 0xFFFFFFF7L)
                {
                    printk("[DBG] FAIL MpPage %d Data_Addr 0x%x Head_Addr 0x%x ID 0x%x ret 0x%x cmp 0x%x\r\n",
                           uwPage,
                           (READ_DATA_ADDR + (uwPage * ulMpPageByte)),
                           (READ_HEAD_ADDR + (uwPage * ulHeadSizePerMPP)), ulSblkId,
                           ret, ulCmp);
                    ulRet = ERR_ECC;
                }
            }
            if(ulRet == ERR_OK)
            {
                printk("[DBG] Pass MpPage %d Data_Addr 0x%x Head_Addr 0x%x ID 0x%x ret 0x%x cmp 0x%x\r\n",
                       uwPage,
                       (READ_DATA_ADDR + (uwPage * ulMpPageByte)),
                       (READ_HEAD_ADDR + (uwPage * ulHeadSizePerMPP)), ulSblkId,
                       ret, ulCmp);
#if 0
                //for print
                U32 ulDWORD = 0;
                for(ulDWORD = 0; ulDWORD < (gubSSDataPageSize << NandPara.ubMpPageByteNumShift); ulDWORD += 4)
                {
                    printk("[DBG] ulDWORD %d Data 0x%x\r\n", ulDWORD, _REG32(READ_DATA_ADDR + ulDWORD));
                }
#endif
                return ulRet;
            }
        }
    }
    //no found snapshot data
    printk("[ERR] cant find snapshot data ulRet 0x%x\r\n", ulRet);

    return ulRet;
}

U32 llfDumpNandLog()
{
    U32 ulRet = ERR_OK, ret = ERR_OK;
    U64 ullWriteCount = 0;
    U16 uwSpBlk = 0, uwPage = 0;
    U8 ubBank = 0, ubOffset = 0, isSLC = 1, ubLunNo;
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 ulCmp;
    U32 ulHeaderAddr;
    head_content* headerContent;
    U32 ulSLBN_old, ulSLBN_n, ulLBN, i, j;
#ifdef EXTEND_LBA
    U64 ulSpecialLBAStartAddr;
#else
    U32 ulSpecialLBAStartAddr;
#endif
    U16 ulDumpNLRecIndex;

    ulSpecialLBAStartAddr = ((gulMaxLBAAddr + (1 << gubSTableEntrySizeShift) * 8 - 1) & ~((
                                 1 << gubSTableEntrySizeShift) * 8 - 1));
    ulSLBN_old = (ulSpecialLBAStartAddr + (SPLBN_UART_BYTES_OFFSET_OLD >> SECTOR_BYTE_SHIFT)) >>
                 gubCacheSectorNumPerLbnShift;

    ulSLBN_n = (ulSpecialLBAStartAddr + (SPLBN_UART_BYTES_OFFSET_NEW >> SECTOR_BYTE_SHIFT)) >>
               gubCacheSectorNumPerLbnShift;

    if(guwCurNLRecIndex <= NAND_LOG_REC_MAX)
    {
        ulDumpNLRecIndex = guwCurNLRecIndex;

        llfSortNandLog(ulDumpNLRecIndex);
        for(i = 0; i < ulDumpNLRecIndex; i++)
        {
            gubNLHashTable[i] = i;
        }
    }
    else
    {
        ulDumpNLRecIndex = NAND_LOG_REC_MAX;
    }

#if 0
    for(j = 0; j < ulDumpNLRecIndex; j++)
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[DBG] idx %d Hash %d WC 0x%x%x\r\n",
                  j, gubNLHashTable[j], (U32)(gNLRecIndex[j].Wc >> 32), (U32)(gNLRecIndex[j].Wc));
    }
#endif

    for(i = ulDumpNLRecIndex; i > 0; i--)
    {
        U32 idx = i - 1;
        for(j = 0; j < ulDumpNLRecIndex; j++)
        {
            if(gubNLHashTable[j] == idx)
            {
                ullWriteCount = gNLRecIndex[j].Wc;
                uwSpBlk = gNLRecIndex[j].Block;
                ubBank  = gNLRecIndex[j].Bank;
                uwPage  = gNLRecIndex[j].Page;
                ubOffset = gNLRecIndex[j].Offset;
                isSLC 	= gNLRecIndex[j].isSLC;
                break;
            }
        }

        ubLunNo = ubBank / NandPara.ubBankNumPerLun;

        gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);

        FCSingleReadDRAM(ulMode, ubBank, ubLunNo, uwSpBlk, uwPage,
                         READ_DATA_PHY_ADDR, DRAM_DATA_SIZE,
                         READ_HEAD_PHY_ADDR, DRAM_HEAD_SIZE, isSLC);

        ret = FCCompletionPolling(&ulCmp, (gul_FW_TAG));

        if ((ERR_OK != ret) || ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0))
        {
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                      "[ERR] NandLog read %d err_ret 0x%x cmp 0x%x DBPO %d_%d_%d_%d SLBN 0x%x SLC %d WC 0x%x%x\r\n",
                      idx, ret, ulCmp, ubBank, uwSpBlk, uwPage, ubOffset, headerContent->head_4k[ubOffset].bits.LBN,
                      isSLC, (U32)(ullWriteCount >> 32), (U32)(ullWriteCount));
            RmaPrintk(PRINT_MEM_FLUSH, PRINT_NANDLOG, " ");
            continue;
        }

        ulHeaderAddr = READ_HEAD_ADDR;
        headerContent = (struct _head_content*)(ulHeaderAddr);
        ulLBN = (headerContent->head_4k[ubOffset].bits.LBN);
        if( (ulSLBN_old != (ulLBN & (~0xff)) ) &&
                ((ulLBN - ulSLBN_n) >= SPLBN_UART_MAX_LBN_INDEX_NEW) )
        {
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                      "[ERR] NandLog read %d wrong SLBN DBPO %d_%d_%d_%d SLBN 0x%x SLC %d WC 0x%x%x\r\n",
                      idx, ubBank, uwSpBlk, uwPage, ubOffset, ulLBN, isSLC,
                      (U32)(ullWriteCount >> 32), (U32)(ullWriteCount));
            RmaPrintk(PRINT_MEM_FLUSH, PRINT_NANDLOG, " ");
            continue;
        }

        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
                  "[Info] NandLog Dump %d DBPO %d_%d_%d_%d SLBN 0x%x SLC %d WC 0x%x%x\r\n",
                  idx, ubBank, uwSpBlk, uwPage, ubOffset, ulLBN, isSLC,
                  (U32)(ullWriteCount >> 32), (U32)(ullWriteCount));

        RmaPrintk(PRINT_MEM, PRINT_NANDLOG, (char *)(READ_DATA_ADDR + NAND_LOG_LEN  * ubOffset));
    }

    RmaPrintk(PRINT_MEM_FLUSH, PRINT_NANDLOG, " ");

    return ulRet;
}

#if defined (BLK_REMAP_PRO)
void llfDumpRemapTable()
{
    U16 uwSpBlk, uwSpBlk_start, uwSpBlk_end, uwSpBlk_remap;
    U8 ubBank;
    U32 i;
    U16 uwCurMPBlkInBitMapTable;
    U8 ubCurMPBlkInIdxTable;
    U16 uwMPBlkInBitMapTableCount = 0;

    if(gubRemapTag && gubConfigTag)
    {
        //check Remap version
        for(i = 0; i < NandPara.uwMpBlockNumPerLun; i++)
        {
            uwCurMPBlkInBitMapTable = _MEM16(BLK_REMAP_BITMAP_TABLE_ADDR + (i * 2));
            ubCurMPBlkInIdxTable = _MEM08(BLK_REMAP_BS_INDEX_TABLE_ADDR + i);
            if(ubCurMPBlkInIdxTable == 0xFF)
            {
                uwMPBlkInBitMapTableCount++;
                if(uwCurMPBlkInBitMapTable != 0)
                {
                    gubRemapTag = Remap_Pro;
                    break;
                }
            }
        }
        if(uwMPBlkInBitMapTableCount == 0)
        {
            gubRemapTag = Remap_Pro;
        }

        uwSpBlk_end = NandPara.uwMpBlockNumPerLun * NandPara.ubPlaneNumPerLun;
        if(gubRemapTag == Remap_New)
        {
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] NewBlkRemap!\r\n");
            uwSpBlk_start = 0;
        }
        else
        {
            uwSpBlk_start = guwRealMPBlkNum * NandPara.ubPlaneNumPerLun;
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] BlkReMapPro! realBlk %d remapBlk %d\r\n",
                      uwSpBlk_start, uwSpBlk_end);
        }
        RmaPrintk(PRINT_MEM, PRINT_REMAP, "Bank,Block,RemapBlk\r\n");

        for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
        {
            for(uwSpBlk = uwSpBlk_start; uwSpBlk < uwSpBlk_end; uwSpBlk++)
            {
                uwSpBlk_remap = BlkRemapProCalc(uwSpBlk, ubBank);
                if(uwSpBlk_remap != uwSpBlk && uwSpBlk_remap != 0)
                {
                    RmaPrintk(PRINT_MEM, PRINT_REMAP, "%d,%d,%d\r\n",
                              ubBank, uwSpBlk, uwSpBlk_remap);
                }
            }
        }
        RmaPrintk(PRINT_MEM_FLUSH, PRINT_REMAP, "");
    }
    else
    {
        RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] DumpRemapTable Fail Tag %d_%d\r\n", gubRemapTag,
                  gubConfigTag);
    }
}
#endif

U32 llfAutoRMAHandle()
{
    U32 ulRet = ERR_OK;

    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start read config\r\n");
    llfReadConfig();

#if defined (BLK_REMAP_PRO)
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start dump remap inform\r\n");
    llfDumpRemapTable();
#endif
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start scan static system block \r\n");
    gubIsSLC = 1;
    llfRMATitlePrint(gubIsSLC);
    Change_ldpc(5);
    ulRet = llfScanStaticSysBlk();
    Change_ldpc(gubECC_CFG);
    if(ulRet != ERR_OK)
    {
        return ulRet;
    }
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start scan system block \r\n");
    ulRet = llfScanSysBlk();
    if(ulRet != ERR_OK)
    {
        return ulRet;
    }
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start search dynamic sblock \r\n");
    if(gubConfigTag == DYNAMIC_SBLK_INFO)
    {
        ulRet = llfSearchDynamicSBlock(SBLK_ADDR, TEMP_BUF_ADDR, STATIC_SBLK_INFO);
        if(ulRet == ERR_OK)
        {
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Info] start dump SMARTInfo:\r\n");
            llfSMARTInfoPrint(TEMP_BUF_ADDR);
        }
        else
        {
            RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] search dynamic sblock Ret:%x\r\n", ulRet);
        }
    }
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start scan user block m %d\r\n", gubIsSLC);
    ulRet = llfScanUserBlk(gubIsSLC);
    if(ulRet != ERR_OK)
    {
        return ulRet;
    }
    gubIsSLC = 0;
    llfRMATitlePrint(gubIsSLC);
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start scan user block m %d\r\n", gubIsSLC);
    ulRet = llfScanUserBlk(gubIsSLC);
    if(ulRet != ERR_OK)
    {
        return ulRet;
    }
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start dump nand log \r\n");
    ulRet = llfDumpNandLog();
    if(ulRet != ERR_OK)
    {
        return ulRet;
    }
    //RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start scan snapshot block \r\n");
    //ulRet = llfScanSnapshotBlk();
    if(ulRet != ERR_OK)
    {
        return ulRet;
    }

    return ulRet;
}
#endif


#if defined(RL6577_VA) || defined(RL6577_FPGA) || defined(RL6447_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
void llfAPBECalibrate(U8 ubIFType, U8 ubClkMode)
{
    U8 ubBankNum, bank = 0;
    PLLF_UNI_INFO pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    U32 ret = ERR_OK, i;

#ifndef NEW_MUL_WR_CACHE
#ifndef NOFE_LLF_FLOW

    entry_t entry = 0;
    U32 lock_flag;
#endif
    PVENDOR_CMD pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
#endif

#ifdef REPLACE_FW
    U16 SBlkNo[8][SYS_BLK] = {{0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, \
        {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}
    };
#endif

    U8 FcSpeedMode[4];
    int SpeedCnt, SpeedIdx;
    U32 cmp, Reg_v;
    U8 ulmode;
    U16 FcCyclenumberMode[2];
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
    ubBankNum = UnbalancedGetBankNum();

    if(gubLLFSubStep.ubKStep == STEP_CALIBRATE_INIT)
    {
        if (pResponseInfo->res_state == VENDOR_CMD_START)
        {
            pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
            pResponseInfo->res_progress = 0;
            pResponseInfo->res_err_code = ERR_OK;
            pResponseInfo->err_msg_num = 0;

            pLLFUniInfo->ubBankNo = 0;
            pLLFUniInfo->block = 0;
            pLLFUniInfo->page = 0;
            pLLFUniInfo->ulWorkFunc = CALIBRATE_FUNC;
        }

        if (FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
        {
            llfprintk("TOSHIBA: IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if (FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON)
        {
            llfprintk("MICRON : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
        {
            llfprintk("HYNIX : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
        {
            llfprintk("SANDISK : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
        {
            llfprintk("SAMSUNG : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
        {
            llfprintk("YMTC : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }

        if(1 == _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_IS_INTEL_B0KB))
        {
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            llfDbgPrintk(ALWAYS_MSG, "Change Intel L06B to B0KB\r\n");
            for(i = 0; i < ubBankNum; i++)
            {
                gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, i);
                FCSetfeature(ulmode, i, 0x85, 0x03);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                {
                    llfDbgPrintk(ALWAYS_MSG, "Set Feature Fail bank %d,%x\r\n", i, cmp);
                    pResponseInfo->res_err_code = ERR_SET_FEATURE;
                    AddErrorMessage(i, 0, ERR_SET_FEATURE);
                    continue;
                }

                gul_FW_TAG = llfBETagSetting(TAG_RESET, i);
                FCReset(ulmode, i);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret != ERR_OK)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Calibrate: Reset hang, bank %d\r\n", i);
                    pResponseInfo->res_state = VENDOR_CMD_IDLE;
                    pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                    AddErrorMessage(i, 0, ERR_FLASH_CALIBRATE);
                    continue;
                }
                FcBusyWait1ms(10);//reset busy wait 10ms
                gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, i);
                FCStatusPolling(ulmode, i);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret != ERR_OK)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Calibrate: Status polling hang, bank %d\r\n", i);
                    pResponseInfo->res_state = VENDOR_CMD_IDLE;
                    pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                    AddErrorMessage(i, 0, ERR_FLASH_CALIBRATE);
                    continue;;
                }

                FCGetfeature(ulmode, i, 0x85, GET_FEATURE_PHY_ADDR, 0x10);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                {
                    llfDbgPrintk(ALWAYS_MSG, "Get Feature Fail bank %d,%x\r\n", i, cmp);
                    pResponseInfo->res_state = VENDOR_CMD_IDLE;
                    pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                    AddErrorMessage(i, 0, ERR_FLASH_CALIBRATE);
                    continue;
                }
                else
                {
                    if((_MEM32(GET_FEATURE_VA_ADDR) & 0xFF) != 0x03)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "Get Feature Value Fail %x -> %x\r\n", 0x03,
                                     _MEM32(GET_FEATURE_VA_ADDR));
                        pResponseInfo->res_state = VENDOR_CMD_IDLE;
                        pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                        AddErrorMessage(i, 0, ERR_FLASH_CALIBRATE);
                        continue;
                    }
                }
            }
            if(pResponseInfo->err_msg_num > 0)
            {
                return;
            }
        }
    }

    SpeedCnt = 2;
#if defined(RL6577_FPGA) || defined(RTS5771_FPGA)
    FcSpeedMode[0] = ubClkMode;
    FcSpeedMode[1] = ubClkMode;
    FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
    FcCyclenumberMode[1] = FcCycleNum[ubClkMode];

#else // ASIC
    if (ubIFType == ONFI_SDR)
    {
        FcSpeedMode[0] = FC_PLL_CLK_50M;
        FcSpeedMode[1] = FC_PLL_CLK_33M;
        FcSpeedMode[2] = FC_PLL_CLK_20M;
        FcSpeedMode[3] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = FcCycleNum[ubClkMode];
    }
    else if (ubIFType == ONFI_DDR)
    {
        FcSpeedMode[0] = FC_PLL_CLK_100M;
        FcSpeedMode[1] = FC_PLL_CLK_66M;
        FcSpeedMode[2] = FC_PLL_CLK_20M;
        FcSpeedMode[3] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = FcCycleNum[ubClkMode];
    }
    else if (ubIFType == ONFI_DDR2_TOGGLE)
    {
        FcSpeedMode[0] = ubClkMode;
        FcSpeedMode[1] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = 10;
    }
    else
    {
        FcSpeedMode[0] = ubClkMode;
        FcSpeedMode[1] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = 10;
    }
#endif

    if(gubLLFSubStep.ubKStep == STEP_CALIBRATE_INIT)
    {
#ifdef MATRIX_EN
#ifdef FTL_B47R
        Matrix_cfg_setting();
#endif
#endif

        ret = ERR_OK;
        for (bank = 0; bank < ubBankNum; bank++)
        {
            //llfprintk("bank write is %d\r\n",bank);
            ret = WriteReadFlashCache(bank, FC_PLL_CLK_10M);
            if(ret != ERR_OK)
            {
                AddErrorMessage(bank, 0, ret);
            }
        }
        if (pResponseInfo->err_msg_num != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Failed to r/w with 10Mhz\r\n");
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
            return;
        }
        gubLLFSubStep.ubKStep = STEP_START_CALIBRTE;
        gubLLFSubStep.ubKSpeedCount = SpeedCnt;
        gubLLFSubStep.ubKSpeedIndex = gubLLFSubStep.ubKSpeedCount;
        return;
    }

    SpeedIdx = gubLLFSubStep.ubKSpeedIndex - 1;
    while (gubLLFSubStep.ubKSpeedIndex > 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "SPEED--------%x\r\n",  FcSpeedMode[SpeedIdx]);
        //change FC interface and clock mode
        if ((SpeedIdx == (SpeedCnt - 1)))
        {
            ret = FCInterfaceChange(ubIFType, FcSpeedMode[SpeedIdx]);
        }
        else
        {
            ret = FCTimingModeChange(ubIFType, FcSpeedMode[SpeedIdx]);
        }
        if(ret != ERR_OK)
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ret;
            return;
        }

        ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
#if defined(RL6577_VA)||defined(RL6447_VA)||defined(RTS5771_VA)
        if(SpeedIdx == 0)
        {
            FCGetNandODTDiffVrefValue();
            ret = SetWarmUpOdtDrvDiff(gulNandODTDiffVrefValue, gubNandDriv,
                                      GETFEATURE_BY_BANK_UC,  GETFEATURE_BY_BANK_PHY);
            if(ret != ERR_OK)
            {
                llfprintk("Set odt drive diff fail\r\n");
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_progress = 100;
                pResponseInfo->res_err_code = ret;
                return;
            }
        }

        ChangeFCClk(ulmode, FcSpeedMode[SpeedIdx]);
        if(SpeedIdx == 0)
        {
            onfi4_change_setting(ulmode, FcSpeedMode[SpeedIdx],
                                 gulFc_ocd, gulFc_dqs_odt, gulFc_dq_re_odt, gubFCDiffEnable);
        }
        else
        {
            onfi4_change_setting(ulmode, FcSpeedMode[SpeedIdx],
                                 FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG, 0);
        }
        if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX))
        {
            fc_diff_setting(ulmode, FcSpeedMode[SpeedIdx], 1);
        }
#endif

        llfLoadTimingFromConfig();
        //bit 8-17:the cycle number of 1us
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
        Reg_v = ((FR_CONFIG_CH(FR_PAR_CFG,
                               gubStartCH) & 0xFFFC00FF) | ((FcCyclenumberMode[SpeedIdx] >> 1) << 8));
#else
        Reg_v = ((FR_CONFIG_CH(FR_PAR_CFG,
                               gubStartCH) & 0xFFFC00FF) | (FcCyclenumberMode[SpeedIdx] << 8));
#endif
        FR_G_CFG_REG32_W(FR_PAR_CFG, Reg_v);
#ifndef RTS5771_FPGA

        if(SpeedIdx == 0)
        {
            ret = FcGetDiffFeature(ulmode, gulNandODTDiffVrefValue);
            if(ret != ERR_OK)
            {
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_progress = 100;
                pResponseInfo->res_err_code = ret;
                return;
            }
        }
#endif

#ifdef RL6577_VA
#if defined(FTL_B27B) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DTV7)
        if(FcSpeedMode[SpeedIdx] > FC_PLL_CLK_400M)
        {
            //Nand_ZQ_Calibration(FR_CONFIG_CH(FR_FC_MODE, 0));//nand zq k,temp mask
            DCC_Training(FR_CONFIG_CH(FR_FC_MODE, 0), FcSpeedMode[SpeedIdx]);
            //reset onfi fifo pointer
            Reg_v = FC_PHY_DLL_CH(ONFI_DPI_CTRL_0, 0) | 0xc;
            FR_G_PHY_REG32_W(ONFI_DPI_CTRL_0, Reg_v );
            Reg_v = FC_PHY_DLL_CH(ONFI_DPI_CTRL_0, 0) & 0xfffffff3;
            FR_G_PHY_REG32_W(ONFI_DPI_CTRL_0, Reg_v );
        }
#endif
#endif

        for(i = 0; i < CH_NUM_MAX; i++)
        {
            gubNvmeTxDelayMin[i] = 0xff;
            gubNvmeTxDelayMax[i] = 0;

            gubNvmeThinDelayMin[i] = 0xff;
            gubNvmeThinDelayMax[i] = 0;
        }

#ifdef FC_FULL_CALIBRATE
        // Calibrate TX DQ PI
        for (bank = 0; bank < NandPara.ubBankNumPerLun; bank++)
        {
            ret = llfCalibrateTxDelayChain(bank, ulmode, FcSpeedMode[SpeedIdx],
                                           FcCyclenumberMode[SpeedIdx]);
            if(ret != ERR_OK)
            {
                AddErrorMessage(bank, 0, ERR_FLASH_CALIBRATE);
            }
        }
        if (pResponseInfo->err_msg_num != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Failed to r/w: Frequency: %d\r\n", FcCyclenumberMode[SpeedIdx]);
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
            return;
        }

        for (bank = 0; bank < NandPara.ubBankNumPerLun; bank++)
        {
            if(llfWriteCache(bank) != ERR_OK)
            {
                AddErrorMessage(bank, 0, ERR_FLASH_CALIBRATE);
            }
        }
        if (pResponseInfo->err_msg_num != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Failed to r/w: Frequency: %d\r\n", FcCyclenumberMode[SpeedIdx]);
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
            return;
        }
#endif

        // Calibrate DQS input delay
        for (bank = 0; bank < ubBankNum; bank++)
        {
            //llfprintk("bank read is %d\r\n",bank);
            ret = llffioBECalibrate(bank, 0, ubIFType, FcSpeedMode[SpeedIdx]);
            if (ret != ERR_OK)
            {
                AddErrorMessage(bank, 0, ERR_FLASH_CALIBRATE);
            }
        }
        if (pResponseInfo->err_msg_num != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Failed to r/w: Frequency: %d\r\n", FcCyclenumberMode[SpeedIdx]);
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
            return;
        }
#ifndef RTS5771_FPGA
        thin_rx_delay_setting();
#endif
        llfSettingPerCh(SBLK_ADDR, SpeedIdx, FcSpeedMode[SpeedIdx]);
        gubLLFSubStep.ubKSpeedIndex--;
        if(gubLLFALLStep == STEP_CALIBRATE)
            return;
    }

    FR_G_CFG_REG32_W(FR_ECC_THV, 0x50);

#ifndef NEW_MUL_WR_CACHE
#ifndef NOFE_LLF_FLOW
    entry = pResponseInfo->entry;
#endif
    pResponseInfo->res_state = VENDOR_CMD_IDLE;
    pResponseInfo->res_err_code = ret;
    pResponseInfo->res_progress = 100;
    if(pVendorCmd->subcmd == BE_LLF_ALL)
    {
        if(ret != ERR_OK)
        {
            pResponseInfo->err_msg_num = 1;
            _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = bank;
        }
        else
        {
            if(gubLLFMode == LLF_ONLY_UPDATE_FW)
            {
#if defined(RL6577_FPGA)||defined(RL6577_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
                Change_ldpc(5);
#endif

                printk("[LLF][RepFw] Start LlfMode:%d\r\n", gubLLFMode);
                pResponseInfo->res_progress = 1;
                ret = llfReadConfigFromSblk(SBlkNo, SBLK_ADDR, SBLK_SIZE);
#if defined(RL6577_VA) || defined(RTS5771_VA)
                U32 flag = 0;
                if(ret != ERR_OK)
                {
                    printk("[LLF][RepFw] llfReadConfigFromSblk fail and trying to read new config.\r\n");
#ifdef BLK_REMAP_PRO
                    guwRealMPBlkNum = NandPara.uwMpBlockNumPerLun;
#endif
                    flag = 1;
                    ret = ERR_OK;
                }
#endif
                _REG08(SBLK_ADDR + SBLK_OFFSET_LLF_MODE) = gubLLFMode;
#if defined(RL6577_FPGA)||defined(RL6577_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
                Change_ldpc(gubECC_CFG);
#endif
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 2;
                    ret = llfReadDbtFromSblk(SBlkNo, DBT_ADDR, DBT_SIZE);
                }
#ifdef REPLACE_FW_WITH_SBLK_TABLE
                if (ERR_OK == ret)
                {
                    ret = llfReadTableFromSblk(SBlkNo, TABLE_ADDR, TABLE_SIZE);
                }
#endif
#ifndef RL6447_VA
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 3;
                    ret = llfReadRemapFromSblk(SBlkNo, BLK_REMAP_TABLE_ADDR, BLK_REMAP_TABLE_SIZE);
                }
#endif

                memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
                cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
                cache_dummy_update_read();
#if defined(RL6577_VA) || defined(RTS5771_VA)
                if (ERR_OK == ret && flag)
                {
                    gulFailedImageBitMap = 0xFFFFFFFF;
                    gulSysblk = 0x00000000;
                    U32 banknum, block;
                    if(NandPara.ubBankNum > 8)
                        banknum = 8;
                    else
                        banknum = NandPara.ubBankNum;
                    for(bank = 0; bank < banknum; bank++)
                    {
                        for(block = 0; block < SYS_BLK; block++)
                        {
#ifndef SBLK_EXPAND
                            if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block))
#else
                            if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank + gubSblkBankStart, block + gubSblkStart))
#endif
                            {
                                continue;
                            }
                            gulFailedImageBitMap &= ~(1 << (block + bank * 4));
                            gulSysblk |= 1 << (block + bank * 4);
                        }
                    }
                    llfprintk("Sysblk = %x\r\n", gulSysblk);
                    llfprintk("gulFailedImageBitMap = %x\r\n", gulFailedImageBitMap);
                }
#endif
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 4;
                    ret = llfEraseSblk();
                }

                CalculateSnapshotArea();

#if defined(RL6577_VA) || defined(RTS5771_VA)
                if(ERR_OK == ret && flag)
                {
                    llfCheckDefectThreshold();
                    llfSetSBlockHWConfig(SBLK_ADDR, ubIFType, ubClkMode);
                    llfSetSBlockFWConfig(SBLK_ADDR);
                    llfSetSBlockFEConfig(SBLK_ADDR);
                    ret = llfReadOffsetSetting(SBLK_ADDR);
                }
#endif
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 5;
                    ret = llfWriteSBlk(SBlkNo);
                }

                if( ret == ERR_READ_SBLK )
                {
                    ret = llfWriteSBlkFailHandle(SBlkNo);
                }

                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 6;
                    ret = llfSetSblkToDBT(DBT_ADDR, SBlkNo);
                }
#if (defined(RL6577_VA) ||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 7;
                    ret = llfSetExtendRdtBlkToDBT(DBT_ADDR);
                }
#endif
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 8;
                    ret = llfWriteSysGroup(SBlkNo);
                }

                printk("[LLF][RepFw] Finished Step:%d Ret:%x\r\n", pResponseInfo->res_progress, ret);
                llfEndResponce(ret);

                gfDBTInitDone = LLF_DBT_NONE;
                gufLastEraseFlag = 0;
                gubLLFALLStep = STEP_CALIBRATE;
                SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
                if (0)// IS_EA_MODE )
                {

                    WRITE_REG_32(MMCR_GIC_RESET_MASK0, 0x3fff);  //mask all interrupt
                    U32 tmpForRomsetting;
                    tmpForRomsetting = 0x7 | (NandPara.ubSLCPageNumPerBlockShift << 8);
#if defined(RL6577_VA) || defined(RL6577_FPGA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
                    WRITE_REG_32(SPI_DL_FLASH_SETTING2_WHEN_EA, tmpForRomsetting);
#elif defined(RL6447_VA)
                    WRITE_REG_32(SPI_DL_SHIFT_SETTING_WHEN_EA, tmpForRomsetting);
#endif
                }
                if ( IS_EA_MODE )
                {
#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
                    WRITE_REG_32(0x8003ff40, 0x0);

                    WriteBackInvalidateDcahe();
                    cache_dummy_update_read();
                    InvalidateIcache();
                    cache_dummy_update_read();
                    FcBusyWait1ms(5);
#endif
                    spi_init_llf();
                    spi_flash_set_protection_llf(TRUE);
                }
                if ( IS_EA_MODE )
                {
                    WRITE_REG_32(MMCR_GIC_SET_MASK0, 0x3fff);  //unmask all interrupt
                }
                return;
            }
            else if(gubLLFMode == LLF_CHECK_BS)
            {
#ifdef LLF_AUTO_RMA
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start LlfMode:%d\r\n", gubLLFMode);
                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] start init \r\n");
                memset((void*)TX_PART1_ADDR, '\0', TX_PART1_SIZE);
                memset((void*)TX_PART2_ADDR, '\0', TX_PART2_SIZE);
                gubCurTxPart = TxMemPart1;
                gulCurTxLength = 0;
                gulCurTxPAGE0Length = 0;
                gulCurTxBLOCKLength = 0;
                gulCurTxURFileLength = 0;
                gubConfigTag = 0;
                gubRemapTag = Remap_Null;
                guwCurNLRecIndex = 0;
                pResponseInfo->res_progress = 1;

                Change_ldpc(5);
                ret = llfReadConfigFromSblk(SBlkNo, SBLK_ADDR, SBLK_SIZE);
                Change_ldpc(gubECC_CFG);
                if(ERR_OK == ret )
                {
                    gubConfigTag = STATIC_SBLK_INFO;
                }
                else
                {
                    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] Read Config Ret:%x\r\n", ret);
                }
                ret = llfReadDbtFromSblk(SBlkNo, DBT_ADDR, DBT_SIZE);
                if(ERR_OK != ret )
                {
                    memset((void*)DBT_ADDR, 0, DBT_SIZE);
                    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] Read DBT Ret:%x\r\n", ret);
                    ret = ERR_OK;
                }
                ret = llfReadRemapFromSblk(SBlkNo, BLK_REMAP_TABLE_ADDR, BLK_REMAP_TABLE_SIZE);
                if (ERR_OK == ret)
                {
                    gubRemapTag = Remap_New;
                    gubBlkRemapProFlag = 1;
                }
                else
                {
                    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[ERR] Read Remap Ret:%x\r\n", ret);
                }
                ret = llfAutoRMAHandle();

                RmaPrintk(PRINT_BOTH, PRINT_UR_FILE, "[Action] finished Step:%d Ret:%x\r\n",
                          pResponseInfo->res_progress, ret);
                RmaPrintk(PRINT_MEM_FLUSH, PRINT_UR_FILE, " ");

                RmaPrintk(PRINT_UART, PRINT_END, "[Action] send print end tag\r\n");
#else
                ret = ERR_UNSUPPORT_FLASH;
#endif
                llfEndResponce(ret);

                gfDBTInitDone = LLF_DBT_NONE;
                gufLastEraseFlag = 0;
                gubLLFALLStep = STEP_CALIBRATE;
                SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);

                if ( IS_EA_MODE )
                {
                    spi_init_llf();
                    spi_flash_set_protection_llf(TRUE);
                    WRITE_REG_32(MMCR_GIC_SET_MASK0, 0x3fff);  //unmask all interrupt
                }
                return;
            }
            else
            {
                pResponseInfo->err_msg_num = 0;
                pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
                pResponseInfo->res_err_code = ret;
                pResponseInfo->res_progress = 30;
                if(gubLLFMode <= LLF_FORCE_INHERIT || (gubLLFMode == LLF_DEFECT_BAD_BLOCK))
                {
                    gubLLFALLStep = STEP_LOAD_RDT_START;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                }
                else if(gubLLFMode == LLF_FORCE_FORMAT)
                {
                    gubLLFALLStep = STEP_FORMAT_INIT;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                }
                else
                {
                    gubLLFALLStep = STEP_FORMAT_INIT;
                    pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
                }
                _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = 0;
                printk("gubLLFALLStep = %d\r\n", gubLLFALLStep);
            }
        }
    }
    else if(pVendorCmd->subcmd == BE_LLF_CALIBRATE)
    {
#ifndef NOFE_LLF_FLOW
        SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
        PrintDone();
        gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
        spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
        SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
        spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
#endif
    }
#endif
}
#elif defined(RL6643_VA) || defined(RL6643_FPGA) || defined(RL6531_VB)
void llfAPBECalibrate(U8 ubIFType, U8 ubClkMode)
{
    U8 ubBankNum, bank = 0;
    PLLF_UNI_INFO pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    U32 ret = ERR_OK, i;
    U32 reg_v;
#ifndef NEW_MUL_WR_CACHE
    entry_t entry = 0;
    U32 lock_flag;
    PVENDOR_CMD pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
#endif

#ifdef REPLACE_FW
#ifdef SBLK_EXPAND
    U16 SBlkNo[SYS_BANK_NUM][SYS_BLK];
    if(pVendorCmd->subcmd == BE_LLF_ALL && gubLLFMode == LLF_ONLY_UPDATE_FW)
    {
        U16 bank_no, blk_no;
        if(!gulSblkCHCEMap[0] && !gulSblkCHCEMap[1])
        {
            U8 ubCHCE, ubCH, ubCE;
            llfprintk("[WARN] repFw sblk bit map all zero!\r\n");
            for(bank_no = 0; bank_no < SYS_BANK_NUM; bank_no++)
            {
                if(bank_no >= gubSysBankNo)
                {
                    break;
                }
                ubCHCE = _MEM08(BANK_IMAPPING_TABLE_ADDR + gubSblkBankStart + bank_no);
                ubCH = (ubCHCE & 0xff) >> 4;
                ubCE = ubCHCE & 0xf;
                gulSblkCHCEMap[ubCH] |= (0xF << (ubCE << SYS_BLK_SHIFT));
            }
        }
        for(bank_no = 0; bank_no < SYS_BANK_NUM; bank_no++)
        {
            if(bank_no >= gubSysBankNo)
            {
                for(blk_no = 0; blk_no < SYS_BLK; blk_no++)
                    SBlkNo[bank_no][blk_no] = 0xF;
                continue;
            }
            for(blk_no = 0; blk_no < SYS_BLK; blk_no++)
            {
                if(llfIsGoodSblk(gubSblkBankStart + bank_no, gubSblkStart + blk_no))
                {
                    SBlkNo[bank_no][blk_no] = gubSblkStart + blk_no;
                }
                else
                {
                    SBlkNo[bank_no][blk_no] = 0xF;
                }
            }
        }
    }
#else
    U16 SBlkNo[8][SYS_BLK] = {{0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, \
        {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}, {0x0, 0x1, 0x2, 0x3}
    };
#endif
#endif

    U8 FcSpeedMode[4];
    int SpeedCnt, SpeedIdx;
    U32 cmp, Reg_v;
    U8 ulmode;
    U16 FcCyclenumberMode[2];
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};
    U8  VrefTryFlag = 0;
    U32 VrefCfg[3] = {0x124, 0x011, 0x106};//50%,65%,35%
    if(IS_6855_VERSION_TAG)
    {
        VrefCfg[0] = 0x11e;
        VrefCfg[1] = 0x00a;
        VrefCfg[2] = 0x100;
    }

#ifdef RL6643_VA
    if(gubNandDefaultMode == 1)
    {
        VrefTryFlag = 1;
    }
#endif

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
    ubBankNum = UnbalancedGetBankNum();

    if(gubLLFSubStep.ubKStep == STEP_CALIBRATE_INIT)
    {
        if (pResponseInfo->res_state == VENDOR_CMD_START)
        {
            pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
            pResponseInfo->res_progress = 0;
            pResponseInfo->res_err_code = ERR_OK;
            pResponseInfo->err_msg_num = 0;

            pLLFUniInfo->ubBankNo = 0;
            pLLFUniInfo->block = 0;
            pLLFUniInfo->page = 0;
            pLLFUniInfo->ulWorkFunc = CALIBRATE_FUNC;
        }

        if (FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
        {
            llfprintk("TOSHIBA: IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if (FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON)
        {
            llfprintk("MICRON : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
        {
            llfprintk("HYNIX : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
        {
            llfprintk("SANDISK : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
        {
            llfprintk("SAMSUNG : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }
        else if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
        {
            llfprintk("YMTC : IF %d Clk %d\r\n", ubIFType, ubClkMode);
        }

        if(1 == _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LLF_IS_INTEL_B0KB))
        {
            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            llfDbgPrintk(ALWAYS_MSG, "Change Intel L06B to B0KB\r\n");
            for(i = 0; i < ubBankNum; i++)
            {
                gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, i);
                FCSetfeature(ulmode, i, 0x85, 0x03);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                {
                    llfDbgPrintk(ALWAYS_MSG, "Set Feature Fail bank %d,%x\r\n", i, cmp);
                    pResponseInfo->res_err_code = ERR_SET_FEATURE;
                    AddErrorMessage(i, 0, ERR_SET_FEATURE);
                    continue;
                }

                gul_FW_TAG = llfBETagSetting(TAG_RESET, i);
                FCReset(ulmode, i);
                FCCompletionPolling(&cmp, (gul_FW_TAG));
                FcBusyWait1ms(10);//reset busy wait 10ms
                gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, i);
                FCStatusPolling(ulmode, i);
                FCCompletionPolling(&cmp, (gul_FW_TAG));

                FCGetfeature(ulmode, i, 0x85, GET_FEATURE_PHY_ADDR, 0x10);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
                {
                    llfDbgPrintk(ALWAYS_MSG, "Get Feature Fail bank %d,%x\r\n", i, cmp);
                    pResponseInfo->res_state = VENDOR_CMD_IDLE;
                    pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                    AddErrorMessage(i, 0, ERR_FLASH_CALIBRATE);
                    continue;
                }
                else
                {
                    if((_MEM32(GET_FEATURE_VA_ADDR) & 0xFF) != 0x03)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "Get Feature Value Fail %x -> %x\r\n", 0x03,
                                     _MEM32(GET_FEATURE_VA_ADDR));
                        pResponseInfo->res_state = VENDOR_CMD_IDLE;
                        pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                        AddErrorMessage(i, 0, ERR_FLASH_CALIBRATE);
                        continue;
                    }
                }
            }
            if(pResponseInfo->err_msg_num > 0)
            {
                return;
            }
        }
    }

    SpeedCnt = 2;
#if defined(RL6643_FPGA)
    FcSpeedMode[0] = ubClkMode;
    FcSpeedMode[1] = ubClkMode;
    FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
    FcCyclenumberMode[1] = FcCycleNum[ubClkMode];

#else // ASIC
    if (ubIFType == ONFI_SDR)
    {
        FcSpeedMode[0] = FC_PLL_CLK_50M;
        FcSpeedMode[1] = FC_PLL_CLK_33M;
        FcSpeedMode[2] = FC_PLL_CLK_20M;
        FcSpeedMode[3] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = FcCycleNum[ubClkMode];
    }
    else if (ubIFType == ONFI_DDR)
    {
        FcSpeedMode[0] = FC_PLL_CLK_100M;
        FcSpeedMode[1] = FC_PLL_CLK_66M;
        FcSpeedMode[2] = FC_PLL_CLK_20M;
        FcSpeedMode[3] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = FcCycleNum[ubClkMode];
    }
    else if (ubIFType == ONFI_DDR2_TOGGLE)
    {
        FcSpeedMode[0] = ubClkMode;
        FcSpeedMode[1] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = 10;
    }
    else
    {
        FcSpeedMode[0] = ubClkMode;
        FcSpeedMode[1] = FC_PLL_CLK_10M;
        FcCyclenumberMode[0] = FcCycleNum[ubClkMode];
        FcCyclenumberMode[1] = 10;
    }
#endif

    if(gubLLFSubStep.ubKStep == STEP_CALIBRATE_INIT)
    {
#ifndef RL6643_VA
        Fc_ocd_odt_setting(FC_OCD_DRIVE, FC_ODT_CFG);
#endif

        ret = ERR_OK;
        if(VrefTryFlag)
        {
            for(i = 0; i < 3; i++)
            {
                FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, VrefCfg[i]);
                for (bank = 0; bank < ubBankNum; bank++)
                {
                    ret = WriteReadFlashCache(bank, FC_PLL_CLK_10M);
                    if(ret != ERR_OK)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "vref%d fail\r\n", i);
                        llfResetAndRecoverFC10M(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M);
                        break;
                    }
                }

                if(ret == ERR_OK)
                {
                    gulVrefCfg = FR_G_CTRL_REG32(FR_RX_VREF_CFG);
                    llfDbgPrintk(ALWAYS_MSG, "vref%d pass\r\n", i);
                    break;
                }
            }
            if(ret != ERR_OK)
            {
                AddErrorMessage(bank, 0, ret);
            }
        }
        else
        {
            for (bank = 0; bank < ubBankNum; bank++)
            {
                //llfprintk("bank write is %d\r\n",bank);
                ret = WriteReadFlashCache(bank, FC_PLL_CLK_10M);
                if(ret != ERR_OK)
                {
                    AddErrorMessage(bank, 0, ret);
                }
            }
        }
        if (pResponseInfo->err_msg_num != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Failed to r/w with 10Mhz\r\n");
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
            return;
        }
        gubLLFSubStep.ubKStep = STEP_START_CALIBRTE;
        gubLLFSubStep.ubKSpeedCount = SpeedCnt;
        gubLLFSubStep.ubKSpeedIndex = SpeedCnt;
        return;
    }

    SpeedIdx = gubLLFSubStep.ubKSpeedIndex - 1;
    while (gubLLFSubStep.ubKSpeedIndex > 0)
    {
        if(gubLLFSubStep.ubKBankIndex == 0 && gubLLFSubStep.ubKRx == 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "SPEED--------%x Bank %d\r\n",  FcSpeedMode[SpeedIdx],
                         gubLLFSubStep.ubKBankIndex);
            //change FC interface and clock mode
            if ((SpeedIdx == (SpeedCnt - 1)))
            {
                ret = FCInterfaceChange(ubIFType, FcSpeedMode[SpeedIdx]);

                if(!VrefTryFlag)//default SDR
                {
                    if(ret != ERR_OK)
                    {
                        pResponseInfo->res_state = VENDOR_CMD_IDLE;
                        pResponseInfo->res_progress = 100;
                        pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                        return;
                    }
                    for(i = 0; i < 3; i++)
                    {
                        FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, VrefCfg[i]);
                        for (bank = 0; bank < ubBankNum; bank++)
                        {
                            ret = WriteReadFlashCache(bank, FC_PLL_CLK_10M);
                            if(ret != ERR_OK)
                            {
                                llfDbgPrintk(ALWAYS_MSG, "vref%d fail\r\n", i);
                                llfResetAndRecoverFC10M(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), FC_PLL_CLK_10M);
                                FCSDRToDDRSetting(ubIFType, FcSpeedMode[SpeedIdx]);
                                break;
                            }
                        }

                        if(ret == ERR_OK)
                        {
                            gulVrefCfg = FR_G_CTRL_REG32(FR_RX_VREF_CFG);
                            llfDbgPrintk(ALWAYS_MSG, "vref%d pass\r\n", i);
                            break;
                        }
                    }
                    if(ret != ERR_OK)
                    {
                        AddErrorMessage(bank, 0, ret);
                    }
                }
            }
            else
            {
                ret = FCTimingModeChange(ubIFType, FcSpeedMode[SpeedIdx]);
            }
            if(ret != ERR_OK)
            {
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_progress = 100;
                pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                return;
            }

            ulmode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
#if defined(RL6643_VA) || defined(RL6531_VB)
            if(SpeedIdx == 0)
            {
                FCGetNandODTDiffVrefValue();
                ret = SetWarmUpOdtDrvDiff(gulNandODTDiffVrefValue, gubNandDriv,
                                          GETFEATURE_BY_BANK_UC,  GETFEATURE_BY_BANK_PHY);
                if(ret != ERR_OK)
                {
                    llfprintk("Set odt drive diff fail\r\n");
                    pResponseInfo->res_state = VENDOR_CMD_IDLE;
                    pResponseInfo->res_progress = 100;
                    pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                    return;
                }
            }
            if(IS_6855_VERSION_TAG)
            {
                if(SpeedIdx == 0)
                {
                    reg_v = FR_G_CTRL_REG32(FR_PAD_CFG1);
                    reg_v &= (~(0xf << 24));//clear bit24/25/26/27
                    FR_G_CTRL_REG32_W(FR_PAD_CFG1, reg_v);
                }
            }
            ChangeFCClk(ulmode, FcSpeedMode[SpeedIdx]);
            if(SpeedIdx == 0)
            {
#ifdef RL6643_VA
                onfi4_ocd_odt_setting(gulFc_ocd, gulFc_ocd, gulFc_dq_re_odt, gulFc_dqs_odt);
#else
                SetOcdOdtFromSblk();
#endif
                fc_diff_setting(ulmode, FcSpeedMode[SpeedIdx], gubFCDiffEnable);
            }
            else
            {
#ifdef RL6643_VA
                if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb && NandPara.ubBankNum == BANK_NUM_MAX)
                {
                    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, 3, 3);//for SDK BICS5 2x8 case
                }
                else
                {
                    onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
                }
#else
                Fc_ocd_odt_setting(FC_OCD_DRIVE, FC_ODT_CFG);
#endif
                fc_diff_setting(ulmode, FcSpeedMode[SpeedIdx], 0);
            }

            if((FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) || (FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX))
            {
                fc_diff_setting(ulmode, FcSpeedMode[SpeedIdx], gubFCDiffEnable);
            }
#endif

            llfLoadTimingFromConfig();
            //bit 8-17:the cycle number of 1us
            Reg_v = ((FR_CONFIG_CH(FR_PAR_CFG,
                                   gubStartCH) & 0xFFFC00FF) | (FcCyclenumberMode[SpeedIdx] << 8));
            FR_G_CFG_REG32_W(FR_PAR_CFG, Reg_v);

            if(SpeedIdx == 0)
            {
                ret = FcGetDiffFeature(ulmode, gulNandODTDiffVrefValue);
                if(ret != ERR_OK)
                {
                    pResponseInfo->res_state = VENDOR_CMD_IDLE;
                    pResponseInfo->res_progress = 100;
                    pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                    return;
                }
            }
        }

#ifdef FC_FULL_CALIBRATE
        if(gubLLFSubStep.ubKRx == 0)
        {
            // Calibrate DQS/DQ output delay
            for (bank = gubLLFSubStep.ubKBankIndex; bank < ubBankNum; bank++)
            {
                ret = llfCalibrateTxDelayChain(bank, ubIFType, FcSpeedMode[SpeedIdx],
                                               FcCyclenumberMode[SpeedIdx]);
                if(ret != ERR_OK)
                {
                    AddErrorMessage(bank, 0, ERR_FLASH_CALIBRATE);
                }
                else
                {
                    gubLLFSubStep.ubKBankIndex++;
                    if(gubLLFSubStep.ubKBankIndex == NandPara.ubBankNum)
                    {
                        gubLLFSubStep.ubKBankIndex = 0;
                        gubLLFSubStep.ubKRx = 1;
                    }
                    else
                    {
                        return;
                    }
                }
            }
            if (pResponseInfo->err_msg_num != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "Failed to r/w: Frequency: %d\r\n", FcCyclenumberMode[SpeedIdx]);
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_progress = 100;
                pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                return;
            }

            for (bank = 0; bank < ubBankNum; bank++)
            {
                if(llfWriteCache(bank) != ERR_OK)
                {
                    AddErrorMessage(bank, 0, ERR_FLASH_CALIBRATE);
                }
            }
            if (pResponseInfo->err_msg_num != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "Failed to r/w: Frequency: %d\r\n", FcCyclenumberMode[SpeedIdx]);
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_progress = 100;
                pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
                return;
            }
        }
#endif

        if(SpeedIdx == 0)
        {
            if(IS_6855_VERSION_TAG)
                FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, 0x11e);
            else
                FR_G_CTRL_REG32_W(FR_RX_VREF_CFG, 0x124);//keep 50% high frequency
        }
        // Calibrate DQS input delay
        for (bank = gubLLFSubStep.ubKBankIndex; bank < ubBankNum; bank++)
        {
            //llfprintk("bank read is %d\r\n",bank);
            ret = llffioBECalibrate(bank, 0, ubIFType, FcSpeedMode[SpeedIdx]);
            if (ret != ERR_OK)
            {
                AddErrorMessage(bank, 0, ERR_FLASH_CALIBRATE);
            }
            else
            {
                gubLLFSubStep.ubKBankIndex++;
                if(gubLLFSubStep.ubKBankIndex == NandPara.ubBankNum)
                {
                    gubLLFSubStep.ubKBankIndex = 0;
                    gubLLFSubStep.ubKRx = 0;
                }
                else
                {
                    return;
                }
            }
        }
        if (pResponseInfo->err_msg_num != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Failed to r/w: Frequency: %d\r\n", FcCyclenumberMode[SpeedIdx]);
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ERR_FLASH_CALIBRATE;
            return;
        }

        llfSettingPerCh(SBLK_ADDR, SpeedIdx, FcSpeedMode[SpeedIdx]);
        gubLLFSubStep.ubKSpeedIndex--;
        if(gubLLFALLStep == STEP_CALIBRATE)
            return;
    }

    FR_G_CFG_REG32_W(FR_ECC_THV, 0x50);

#ifndef NEW_MUL_WR_CACHE
    entry = pResponseInfo->entry;
    pResponseInfo->res_state = VENDOR_CMD_IDLE;
    pResponseInfo->res_err_code = ret;
    pResponseInfo->res_progress = 100;
    if(pVendorCmd->subcmd == BE_LLF_ALL)
    {
        if(ret != ERR_OK)
        {
            pResponseInfo->err_msg_num = 1;
            _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = bank;
        }
        else
        {
            if(gubLLFMode == LLF_ONLY_UPDATE_FW)
            {
#if defined(RL6643_VA) || defined(RL6643_FPGA)
                Change_ldpc(3);
#endif

                printk("[LLF][RepFw] Start LlfMode:%d\r\n", gubLLFMode);
                pResponseInfo->res_progress = 1;
                ret = llfReadConfigFromSblk(SBlkNo, SBLK_ADDR, SBLK_SIZE);
                _REG08(SBLK_ADDR + SBLK_OFFSET_LLF_MODE) = gubLLFMode;
#if defined(RL6643_VA) || defined(RL6643_FPGA)
                Change_ldpc(gubECC_CFG);
#endif
                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 2;
                    ret = llfReadDbtFromSblk(SBlkNo, DBT_ADDR, DBT_SIZE);
                }

                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 3;
                    ret = llfReadRemapFromSblk(SBlkNo, BLK_REMAP_TABLE_ADDR, BLK_REMAP_TABLE_SIZE);
                }

                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 4;
                    ret = llfEraseSblk();
                }

                memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
                cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
                cache_dummy_update_read();
                CalculateSnapshotArea();

                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 5;
                    ret = llfWriteSBlk(SBlkNo);
                }
                if( ret == ERR_READ_SBLK )
                {
                    ret = llfWriteSBlkFailHandle(SBlkNo);
                }

                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 6;
                    ret = llfSetSblkToDBT(DBT_ADDR, SBlkNo);
                }

                if (ERR_OK == ret)
                {
                    pResponseInfo->res_progress = 7;
                    ret = llfWriteSysGroup(SBlkNo);
                }

                printk("[LLF][RepFw] Finished Step:%d Ret:%x\r\n", pResponseInfo->res_progress, ret);
                llfEndResponce(ret);

                gfDBTInitDone = LLF_DBT_NONE;
                gufLastEraseFlag = 0;
                gubLLFALLStep = STEP_CALIBRATE;
                SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
                if ( IS_EA_MODE )
                {
                    HAL_DISABLE_INTERRUPTS();
                    llfSaveInfoToSpi();

#if defined(RL6643_VA)
                    U32 VrefDefaultSet = 0x124;

                    if(IS_6855_VERSION_TAG)
                        VrefDefaultSet = 0x11e;
                    gubRealCapacity *= NandPara.ubBankNum * NandPara.ubLunNumPerCE;
                    WRITE_REG_32(SPI_DL_SSD_INFO_MODEL_NAME, gubRdtImg);
                    WRITE_REG_32(SPI_DL_SSD_INFO_FW_VERSION, _REG32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FW_VERSION));
                    WRITE_REG_32(SPI_DL_SSD_INFO_FW_VERSION + 4,
                                 _REG32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FW_VERSION + 4));
                    WRITE_REG_32(SPI_DL_SSD_INFO_REAL_CAPACITY, gubRealCapacity);
                    WRITE_REG_32(SPI_DL_SSD_INFO_USING_CAPACITY, guwHostClaimCapacity);
                    if((gulVrefCfg != VrefDefaultSet) && (gubNandDefaultMode == 1))
                    {
                        if(IS_6855_VERSION_TAG)
                            WRITE_REG_32(SPI_DL_VREF_ROM_DRIVE_SET_EA,
                                         ((0x52 << 20) | (gulVrefCfg << 8) | 0 << 4 | TAG_FOR_VREF_ODT_CFG));
                        else
                            WRITE_REG_32(SPI_DL_VREF_ROM_DRIVE_SET_EA,
                                         ((0x44 << 20) | (gulVrefCfg << 8) | 3 << 4 | TAG_FOR_VREF_ODT_CFG));
                    }
#endif
                    spi_init_llf();
                    spi_flash_set_protection_llf(TRUE);
                    llfDbgPrintk(ALWAYS_MSG, "Add spi lock after llf all succeed\r\n");

                    HAL_ENABLE_INTERRUPTS();
                }
#if defined(RL6643_VA)
                else
                {
                    if(IS_6643_VERSION_TAG)
                    {
                        if(gubNandDefaultMode == 1)
                            llfUpdateVrefToEFUSE(gulVrefCfg);
                    }
                }
#endif
                return;
            }
            else
            {
                pResponseInfo->err_msg_num = 0;
                pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
                pResponseInfo->res_err_code = ret;
                pResponseInfo->res_progress = 30;
                if(gubLLFMode <= LLF_FORCE_INHERIT || (gubLLFMode == LLF_DEFECT_BAD_BLOCK))
                {
                    gubLLFALLStep = STEP_LOAD_RDT_START;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                }
                else if(gubLLFMode == LLF_FORCE_FORMAT)
                {
                    gubLLFALLStep = STEP_FORMAT_INIT;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                }
                else
                {
                    gubLLFALLStep = STEP_FORMAT_INIT;
                    pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
                }
                _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = 0;
                printk("gubLLFALLStep = %d\r\n", gubLLFALLStep);
            }
        }
    }
    else if(pVendorCmd->subcmd == BE_LLF_CALIBRATE)
    {
        SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
        PrintDone();
        gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
        spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
        SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
        spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
    }
#endif
}

#endif

#if 0
// Calibrate DQS delay output
U32 llfCalibrateDqsOut(void)
{
    U32 pAddr;
    U32 head_pAddr;
    U32 ret = ERR_OK;
    U32 cmp;
    U32 ulMode;
    U8 bank;
    U8 be_no;
    U8 ce = 0;
    U16 i;
    U32 ulPhyCfg;
    U8 Min = 0xff;
    U8 Max = 0;
    U32 ret_data_compare = ERR_OK;

    //set header
#ifdef SRAM_DMA
    for (i = 0; i < (HEADER_DMA_MAX_LEN * 2); i++)
        _REG32(0xA0130000 + (i * 4)) = ((i << 24) | (i << 16) | (i << 8) | i);
#else // DRAM mode
    /*for (i = 0; i < (HEADER_DMA_MAX_LEN * 2); i++)
        _REG32(TEMP_HBUF_ADDR + (i * 4)) = ((i << 24) | (i << 16) | (i << 8) | i);*/

    for (i = 0; i < (CACHE_DATA_SIZE); i++)
        _REG32(CACHE_DATA_BASE + (i * 4)) = i;

    cache_area_dwbinval(CACHE_DATA_BASE, CACHE_DATA_SIZE);
    cache_dummy_update_read();

    for (i = 0; i < (CACHE_HEAD_SIZE); i++)
        _REG32(CACHE_HEAD_BASE + (i * 4)) = (0xaabbcc00 + i);

    cache_area_dwbinval(CACHE_HEAD_BASE, CACHE_HEAD_SIZE);
    cache_dummy_update_read();

    // DWB that cache line
    cache_area_dwbinval(TEMP_HBUF_ADDR, (HEADER_MAX_LEN * 2));
    cache_dummy_update_read();
#endif

    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        be_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xFF) >> 4;
        ce = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xF);
        FR_REG_BASE_X = FR_REG_BASE + be_no * FR_REG_SIZE;;

        ulPhyCfg = 0;

        for(i = 0; i < 10; i++)
        {
            ulPhyCfg = i;

            FR_REG32_X(FR_PHY_DELAY_CFG0 + ce * 4) |= ((ulPhyCfg & 0x7F) << 8); // DQS delay output bits 8~14

            //---------write----------

#ifdef SRAM_DMA
            // Set Header
            head_pAddr = 0x00130000;

            //set data DMA
            pAddr = DMEM1_RAM_BASE_PHY;

#else // DRAM mode
            //head_pAddr = TEMP_HBUF_PHY_ADDR; //TEMP_HBUF_ADDR 0x49400
            head_pAddr = CACHE_HEAD_BASE_PHY;

            //set data DMA
            //pAddr = TEMP_BUF_PHY_ADDR;
            pAddr = CACHE_DATA_BASE_PHY;
#endif
            ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            gul_FW_TAG = llfBETagSetting(TAG_WRITE_CACHE, bank);
            llfFCCmdWriteCache_DRAM(ulMode, bank, 0, 0, pAddr, NandPara.ubSectorNumPerPage * 512,
                                    head_pAddr, DRAM_HEAD_SIZE);
            FcBusyWait1ms(1);
            FCCompletionPolling(&cmp, (gul_FW_TAG));
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "write cache error: %d\r\n", bank);
                //return ERR_WRITE_SBLK;
                ASSERT_LLF(0);
            }


            //---------read----------
            _REG32(TEMP_HBUF_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun)) = 0xBadBad;
            _REG32(TEMP_HBUF_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun) + 32) = 0xBadBad;
            cache_area_dwbinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512); // 16k Data
            cache_area_dwbinval((TEMP_HBUF_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun)),
                                L2pPara.ubHdr4BLenPerPage * 4);
            cache_dummy_update_read();



#ifdef SRAM_DMA
            // set header DMA
            head_pAddr = (0x00130000 + (NandPara.ubSectorNumPerPage * 512 * 2) + HEADER_MAX_LEN);

            // set data DMA
            pAddr = (DMEM1_RAM_BASE_PHY + NandPara.ubSectorNumPerPage * 512);
#else
            // set header DMA
            head_pAddr = TEMP_HBUF_PHY_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun);

            // set data DMA
            pAddr = (TEMP_BUF_PHY_ADDR + NandPara.ubSectorNumPerPage * 512);
#endif

            ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

            llfFCCmdReadCache_DRAM(ulMode, bank, pAddr, NandPara.ubSectorNumPerPage * 512,
                                   head_pAddr, DRAM_HEAD_SIZE); // read data only
            FcBusyWait1ms(1);
            FCCompletionPolling(&cmp, (gul_FW_TAG));
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "read cache error: %d\r\n", bank);
                //return ERR_WRITE_SBLK;
                ASSERT_LLF(0);
            }



            // Clean read memory cache
            cache_area_dinval((TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512),
                              NandPara.ubSectorNumPerPage * 512); // 16k Data
            cache_area_dinval((TEMP_HBUF_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun)),
                              L2pPara.ubHdr4BLenPerPage * 4);
            cache_dummy_update_read();

            // Compare header and data
#ifdef SRAM_DMA
            ret_data_compare = llfCompareData(0xA0130000,
                                              (0xA0130000 + (NandPara.ubSectorNumPerPage * 512 * 2) + HEADER_MAX_LEN),
                                              HEADER_CRC_OFFSET);
            ret_data_compare |= llfCompareData(DMEM1_RAM_BASE,
                                               (DMEM1_RAM_BASE + NandPara.ubSectorNumPerPage * 512),
                                               (NandPara.ubSectorNumPerPage * 512));
#else
            ret_data_compare = llfCompareData(TEMP_HBUF_ADDR,
                                              (TEMP_HBUF_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun)),
                                              HEADER_CRC_OFFSET);
            ret_data_compare |= llfCompareData(TEMP_BUF_ADDR,
                                               (TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512),
                                               (NandPara.ubSectorNumPerPage * 512));
#endif


            if (ret == ERR_OK)
            {
                if(i < Min)
                    Min = i;

                if(i > Max)
                    Max = i;
            }
            else if (Min != 0xFF)
                break;

        }

        if(Min == 0xff)
        {
            ret = ERR_FLASH_CALIBRATE;
        }
        else
        {
            ret = ERR_OK;

            FR_REG32_X(FR_PHY_DELAY_CFG0 + ce * 4) |= ((Min & 0x7F) << 8);
        }

        FcBusyWait1ms(1);

        //FR_REG32_X(FR_PHY_DELAY_CTRL)  = 0x1;
        //FR_PHY_CFG1_REG_X |= FC_DCO_MODE;
    }
    return ret;
}

U32 writeDQ_tx_delay_calibration(U32 iface, U32 ulMode, U32 bank)
{
    U32 pAddr;
    //U32 head_pAddr;
    U32 cmp;
    U32 ret = ERR_OK;
    U8 lun_no, i;

    lun_no = bank / NandPara.ubBankNumPerLun;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    //head_pAddr = TEMP_HBUF_PHY_ADDR + (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun);
    pAddr = (TEMP_BUF_PHY_ADDR + NandPara.ubSectorNumPerPage * 512);
    for(i = 0; i < 16; i ++)
    {
        _REG32(TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512 + i * 4) = 0;
    }
    gul_FW_TAG = llfBETagSetting(0xa0, bank);
    FCWriteTxTraining(ulMode, bank, lun_no, TEMP_BUF_PHY_ADDR, 0x40, TEMP_HBUF_PHY_ADDR, 0x8);
    FcBusyWait1ms(1);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "bank %d writeTx fail cmp %x\r\n", bank, cmp);
        return ERR_FIO_TIMEOUT;
    }
    llfprintk("write tx dq trainging\r\n");
    gul_FW_TAG = llfBETagSetting(0xb0, bank);
    FCWriteTxTrainReadCheck(ulMode, bank, lun_no, pAddr, 0x40);
    FcBusyWait1ms(1);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "bank %d writeTxCheck fail cmp %x\r\n", bank, cmp);
        return ERR_FIO_TIMEOUT;
    }
    else
    {
        cache_area_dinval((TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512), 0x40);
        cache_dummy_update_read();

        ret = llfCompareData(TEMP_BUF_ADDR,  (TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512),
                             0x40);
    }

    return ret;
}

#endif


void llfAPEraseAll(U8 ubEraseMode)
{
    U8 i, ubLunNo = 0;
    U8 bank_no, defectTag = 0, ubSblkStart = 0;
    U32 ulMode;
    U32 total_bank_num;
    U16 uwPage = 0;
    U32 Block_end;
    PLLF_UNI_INFO pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
#ifndef NOFE_LLF_FLOW
    U32 lock_flag;
    entry_t entry = 0;
    PVENDOR_CMD pVendorCmd;

    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
#endif
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    total_bank_num = NandPara.ubBankNum;

    if((gfDBTInitDone == LLF_DBT_RDT) && (gubLLFMode != LLF_DEFECT_BAD_BLOCK)
#ifdef EXTEND_STATIC_DBT
            && (gubNeedRebuildRemap == 0)
#endif
      )
        Block_end = SYS_BLK + ubSblkStart;
#ifdef BLK_REMAP_PRO
    else if((gubBlkRemapProFlag == 1) && (gfDBTInitDone == LLF_DBT_SYSTEM)) //Inherit from whole system
    {
        Block_end = guwRealMPBlkNum * NandPara.ubPlaneNumPerLun;
    }
#endif
    else
    {
        Block_end = NandPara.uwBlockNumPerLun;
    }

    if (pResponseInfo->res_state == VENDOR_CMD_ERASE)
    {
        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pResponseInfo->res_progress = 0;
        pResponseInfo->res_err_code = ERR_OK;

        pLLFUniInfo->ubBankNo = 0;
        pLLFUniInfo->uwEraseLastBlock = 0;
        gufLastEraseFlag = 0;

        for(i = 0; i < CH_NUM_MAX * CMD_FIFO_NUM_PER_CH; i++)
        {
            pLLFUniInfo->uwBlock[i] = 0;
            gubEraseCmpNum[i] = 0;
#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A) || defined(FTL_N48R)
            pLLFUniInfo->ubEraseSection[i] = 0;
#endif
        }

        if (ubEraseMode == ERASE_ALL_WITH_DBT)
        {
            // the DBT should be initialized first if we choose erase refer to DBT
            if (gfDBTInitDone == LLF_DBT_NONE)
            {
                llfEndResponce(ERR_ERASE_WITHOUT_INIT);
                return;
            }
        }
        else if (ubEraseMode != ERASE_ALL)
        {
            llfEndResponce(ERR_UNKNOWN_VENDOR_CMD);
            return;
        }

        llfDbgPrintk(ALWAYS_MSG, "LLF ERASE begin, %d\r\n", Block_end);
    }

    if((pLLFUniInfo->uwEraseLastBlock == total_bank_num) && (gufLastEraseFlag == 0))
    {
        if(gubLLFMode != LLF_FORMAT_ERASE_NONDEFECT)//need to check
        {
            pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
            pResponseInfo->res_progress = 50;
        }
        else
        {
            pResponseInfo->res_state = VENDOR_CMD_WRITE_FW;
            pResponseInfo->res_progress = 80;
        }

        pResponseInfo->res_err_code = ERR_OK;
        gufLastEraseFlag = 0;
        llfDbgPrintk(ALWAYS_MSG, "Erase 100/100,res_state=%x\r\n", pResponseInfo->res_state);
        PrintDone();

#ifndef NOFE_LLF_FLOW //send msg to FE
        if(pVendorCmd->subcmd != BE_LLF_ALL)
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            entry = pResponseInfo->entry;
            gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
            spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
            SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
            spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
        }
#else
        if(gfTestFlag == 1)
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE; //for llf with gfTestFlag
        }
#endif
        return;
    }

    for(bank_no = 0; bank_no < total_bank_num; bank_no++)
    {
#if (defined(RL6577_VA) ||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
        if((gubExtendRdtResultBlk == RDT_RESULT_BLK_EXTEND_NUM) && (Block_end > SYS_BLK + ubSblkStart)
                && (gubRdtImg != 1))
        {
            while((pLLFUniInfo->uwBlock[bank_no] >= ubSblkStart + SYS_BLK) &&
                    (pLLFUniInfo->uwBlock[bank_no] < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK))
            {
                printk("erase jump bank %d blk %d\r\n", bank_no, pLLFUniInfo->uwBlock[bank_no]);
                pLLFUniInfo->uwBlock[bank_no] ++;
            }
        }
#elif defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
        if((Block_end > SYS_BLK + ubSblkStart) && (gubRdtImg != 1) && gubLLFMode != LLF_FORCE_FORMAT)
        {
            while((pLLFUniInfo->uwBlock[bank_no] >= ubSblkStart + SYS_BLK) &&
                    (pLLFUniInfo->uwBlock[bank_no] < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK))
            {
                printk("erase jump bank %d blk %d\r\n", bank_no, pLLFUniInfo->uwBlock[bank_no]);
                pLLFUniInfo->uwBlock[bank_no] ++;
            }
        }
#endif

        ubLunNo = bank_no / NandPara.ubBankNumPerLun;
        if(pLLFUniInfo->uwBlock[bank_no] < Block_end)
        {
#if 0
            mpblock_no = pLLFUniInfo->uwBlock[bank_no] / NandPara.ubPlaneNumPerLun;
            if(mpblock_no >= 4096)
            {
                llfprintk("DBT size is not enough!!!");//maybe the whole DBT size is enough, but the mpblock num of every bank is not enough
                ASSERT(ALWAYS_MSG, (mpblock_no < 4096));
            }
#endif
            if(pLLFUniInfo->uwBlock[bank_no] < 4096)
            {
                gubEraseNum = 1;
                gul_FW_TAG = ((pLLFUniInfo->uwBlock[bank_no] << 4) | (bank_no & 0x0f));
            }
            else if((pLLFUniInfo->uwBlock[bank_no] >= 4096) && (pLLFUniInfo->uwBlock[bank_no] < 8192))
            {
                gubEraseNum = 2;
                gul_FW_TAG = (((pLLFUniInfo->uwBlock[bank_no] - 4096) << 4) | (bank_no & 0x0f));

            }
            else if((pLLFUniInfo->uwBlock[bank_no] >= 8192) && (pLLFUniInfo->uwBlock[bank_no] < 12288))
            {
                gubEraseNum = 3;
                gul_FW_TAG = (((pLLFUniInfo->uwBlock[bank_no] - 8192) << 4) | (bank_no & 0x0f));
            }
            else if((pLLFUniInfo->uwBlock[bank_no] >= 12288) && (pLLFUniInfo->uwBlock[bank_no] < 16384))
            {
                gubEraseNum = 4;
                gul_FW_TAG = (((pLLFUniInfo->uwBlock[bank_no] - 12288) << 4) | (bank_no & 0x0f));
            }
            else if((pLLFUniInfo->uwBlock[bank_no] >= 16384) && (pLLFUniInfo->uwBlock[bank_no] < 20480))
            {
                gubEraseNum = 5;
                gul_FW_TAG = (((pLLFUniInfo->uwBlock[bank_no] - 16384) << 4) | (bank_no & 0x0f));
            }
            else if((pLLFUniInfo->uwBlock[bank_no] >= 20480) && (pLLFUniInfo->uwBlock[bank_no] < 24576))
            {
                gubEraseNum = 6;
                gul_FW_TAG = (((pLLFUniInfo->uwBlock[bank_no] - 20480) << 4) | (bank_no & 0x0f));
            }
            else
            {
                ASSERT(ALWAYS_MSG, 0);
            }

#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A)
            uwPage = pLLFUniInfo->ubEraseSection[bank_no] * INTELQ_PAGE_PER_SECTION;
#endif
            if (ubEraseMode == ERASE_ALL)
            {
                FCSingleErase(ulMode, bank_no, ubLunNo, pLLFUniInfo->uwBlock[bank_no], uwPage, 0);
                gubEraseCmpNum[bank_no]++;
                gufLastEraseFlag++;
#ifdef ERASEALL_TWICE
                if((gubFWFeatureSetting & 0x20) != 0)
                {
                    llfBECheckStatus();
                }
#endif
                if(pLLFUniInfo->uwBlock[bank_no] % (Block_end / 4) == 0)
                {
                    pResponseInfo->res_progress = pLLFUniInfo->uwBlock[bank_no] * 100 /
                                                  Block_end;
                    llfDbgPrintk(ALWAYS_MSG, "bank%d Erase %d/100\r\n", bank_no, pResponseInfo->res_progress);
                }
            }
            else if (ubEraseMode == ERASE_ALL_WITH_DBT)
            {
                defectTag = 0;
                if(pLLFUniInfo->uwBlock[bank_no] < SYSTEM_BLOCK_MAX_NUM)
                {
                    if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, pLLFUniInfo->uwBlock[bank_no]))
                    {
                        defectTag = 1;
                    }
                }
                else
                {
                    if(llfIsBlockBad(DBT_ADDR, bank_no, pLLFUniInfo->uwBlock[bank_no]))
                    {
                        defectTag = 1;
                    }
                }
                if((!defectTag) || ((pLLFUniInfo->uwBlock[bank_no] < ubSblkStart + SYS_BLK)
                                    && (bank_no < NandPara.ubBankNum)))
                {
                    FCSingleErase(ulMode, bank_no, ubLunNo, pLLFUniInfo->uwBlock[bank_no], uwPage, 0);
                    gubEraseCmpNum[bank_no]++;
                    gufLastEraseFlag++;
#ifdef ERASEALL_TWICE
                    if((gubFWFeatureSetting & 0x20) != 0)
                    {
                        llfBECheckStatus();
                    }
#endif
                    if((pLLFUniInfo->uwBlock[bank_no] < ubSblkStart + SYS_BLK)
                            && (bank_no < NandPara.ubBankNum))
                    {
                        llfUnMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, pLLFUniInfo->uwBlock[bank_no]);
                        llfDbgPrintk(ALWAYS_MSG, "unmark bank %d blk %d\r\n", bank_no, pLLFUniInfo->uwBlock[bank_no]);
                    }
                    if(pLLFUniInfo->uwBlock[bank_no] % (Block_end / 4) == 0)
                    {
                        pResponseInfo->res_progress = pLLFUniInfo->uwBlock[bank_no] * 100 /
                                                      Block_end;
                        llfDbgPrintk(ALWAYS_MSG, "bank%d Erase %d/100\r\n", bank_no, pResponseInfo->res_progress);
                    }
                }
            }
#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A)
            pLLFUniInfo->ubEraseSection[bank_no]++;
            if(pLLFUniInfo->ubEraseSection[bank_no] == INTELQ_SECTION_PER_BLOCK)
            {
                pLLFUniInfo->ubEraseSection[bank_no] = 0;
                pLLFUniInfo->uwBlock[bank_no]++;
            }
#else
            pLLFUniInfo->uwBlock[bank_no]++;
#endif
        }
        else if(pLLFUniInfo->uwBlock[bank_no] == Block_end)
        {
            pLLFUniInfo->uwEraseLastBlock++;
            pLLFUniInfo->uwBlock[bank_no]++;
        }
    }
}

#ifdef ERR_INJECT_SBLK
U8 llfInjectSblkBad(U8 mode, U8 bank_no, U8 blk_no)
{
    U8 inject = 0;// ret = ERR_OK;
    switch(mode)
    {
    case 0:
    {
        break;
    }
    case 1:
    {
        if(blk_no < 4)
        {
            inject = 1;
        }
    }
    case 2:
    {
        if((blk_no < 5) && (bank_no < 3))
        {
            inject = 1;
        }
    }
    default:
    {
        break;
    }
    }

    if(inject)
    {
        llfprintk("inject bank%d, blk%d, mode%d\r\n", bank_no, blk_no, mode);
    }

    return inject;
}
#endif

U32 llfChkDefectBlk(U8 bank, U16 blk_no)
{
    U16 page;
    U32 ulMode;
    U32 odbt_id;
    U32 ret;

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    //check out the first byte of defect area in the first page of the checked block
    page = 0;
    ret = llffioDefectRead(ulMode, bank, blk_no, page, 0);
    if(ret == ERR_OK)
    {
        odbt_id = llfIsDefectBlk((TEMP_BUF_ADDR + MAX_DB_NUM_SIZE), ulMode);
        if(odbt_id == ERR_DEFECT_BLOCK)
        {
            return 0;
        }
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "error = %d, page = %d\r\n", ERR_DEFECT_BLK_READ, page);
        //ASSERT_LLF(0);
        return 0;
    }
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
    {
    }
    else
    {
        ret = llffioDefectRead(ulMode, bank, blk_no, page, NandPara.ubSectorNumPerPage * 512);
        if(ret == ERR_OK)
        {
            odbt_id = llfIsDefectBlk((TEMP_BUF_ADDR + MAX_DB_NUM_SIZE + NandPara.ubSectorNumPerPage *
                                      SECTOR_BYTE_SIZE), ulMode);
            if(odbt_id == ERR_DEFECT_BLOCK)
            {
                return 0;
            }
        }
        else
        {
            llfDbgPrintk(ALWAYS_MSG, "error = %d, page = %d\r\n", ERR_DEFECT_BLK_READ, page);
            //ASSERT_LLF(0);
            return 0;
        }
    }

    //check out the first byte of defect area in the last page of the checked block
#if defined(FTL_N38B) || defined(FTL_Q5171A)
    page = INTELQ_PAGE_PER_SECTION - 1;
#else
    page = NandPara.uwPageNumPerBlock - 1;
#endif
    ret = llffioDefectRead(ulMode, bank, blk_no, page, 0);
    if(ret == ERR_OK)
    {
        odbt_id = llfIsDefectBlk((TEMP_BUF_ADDR + MAX_DB_NUM_SIZE), ulMode);
        if(odbt_id == ERR_DEFECT_BLOCK)
        {
            return 0;
        }
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "error = %d, page = %d\r\n", ERR_DEFECT_BLK_READ, page);
        //ASSERT_LLF(0);
        return 0;
    }
    if((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
            && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS)
                || (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS)))
    {
    }
    else
    {
        ret = llffioDefectRead(ulMode, bank, blk_no, page, NandPara.ubSectorNumPerPage * 512);
        if(ret == ERR_OK)
        {
            odbt_id = llfIsDefectBlk((TEMP_BUF_ADDR + MAX_DB_NUM_SIZE + NandPara.ubSectorNumPerPage *
                                      SECTOR_BYTE_SIZE), ulMode);
            if(odbt_id == ERR_DEFECT_BLOCK)
            {
                return 0;
            }
        }
        else
        {
            llfDbgPrintk(ALWAYS_MSG, "error = %d, page = %d\r\n", ERR_DEFECT_BLK_READ, page);
            //ASSERT_LLF(0);
            return 0;
        }
    }

#ifdef ERR_INJECT_SBLK
    if(llfInjectSblkBad(1, bank, blk_no))
    {
        return 0;
    }
#endif

    return 1;
}

U32 llfScanFactoryDefectBlocks(U32 blk_no)
{
    /***
    * Start to build a new ODBT after getting a good Phy Block
    * defect_cnt[][] should contain all defect counts of each chip
    * gpabDBlkNo[][] should contains all ODBT (suitable for DBT) block address of each chip
    ***/

    U8 defect_flg;
    U32 ret = ERR_OK;
    U16 uwDefectCnt = 0;
    U8 ubtype;
    U8 bank;
    LLF_UNI_INFO *pLLFUniInfo;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

#if (defined(RL6577_VA) ||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
    U8 ubSblkStart = 0;
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif
#elif defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    U8 ubSblkStart = 0;
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif
#endif

    if(((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR  ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC)
            && FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK) ||
            ((FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC) &&
             ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T) ||
              (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2) ||
              (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2T) ||
              (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX2Q) ||
              (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WYS) ||
              (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YX3T_WDS))))
    {
        ubtype = 1;
    }
    else
    {
        ubtype = 0;
    }

    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        if(blk_no < SYSTEM_BLOCK_MAX_NUM)
        {
            if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, blk_no))
            {
                continue;
            }
#if (defined(RL6577_VA)||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
            if((gubExtendRdtResultBlk == 4) && (blk_no >= ubSblkStart + SYS_BLK) &&
                    (blk_no < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK) && (gubRdtImg != 1))
            {
                continue;
            }
#elif defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
            if((blk_no >= ubSblkStart + SYS_BLK) &&
                    (blk_no < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK) && (gubRdtImg != 1))
            {
                continue;
            }
#endif
        }
        else
        {
            if(llfIsBlockBad(DBT_ADDR, bank, blk_no))
            {
                continue;
            }
        }

        if(ubtype == 0)
        {
            defect_flg = llfChkDefectBlk(bank, blk_no);//return 0 is bad
        }
        else
        {
            defect_flg = llfChkDefectBlkForSandisk(bank, blk_no);
        }
        if(defect_flg == ERR_DEFECT_BLK_READ || defect_flg == ERR_FIO_TIMEOUT)
        {
            llfprintk("defect read fail\r\n");
            llfAddErrorMessage(bank, 0, ERR_ALLOCATE_DBT);
            return ERR_ALLOCATE_DBT;
        }

        // Mark defect if system block
        if((!defect_flg) && (blk_no < SYSTEM_BLOCK_MAX_NUM))
        {
            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, blk_no);
        }

        if(!defect_flg) //if this block is defect or it's the ODBT
        {
            uwDefectCnt++;
#ifdef SPECIAL_SYSTEM_BLOCK
            ret = llfSetSpDBT(bank, blk_no, SP_DBT_ADDR);
#endif
            ret = llfSetDBT(bank, blk_no, DBT_ADDR); //build DBT
            pLLFUniInfo->ulData5++; //check bad count
        }
    }

    cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);

    return ret;
}

U32 llfSearchDBT(U8 bank)
{
    U16 blk_no, page;
    U32 ret = ERR_OK;
    U32 cmp;
    U32 ulMode;
    U32 ulODBTBid;
    U8 ubLunNo;

    ubLunNo = bank / NandPara.ubBankNumPerLun;

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
    blk_no = 0;
    page = DBT_PAGENO;//DBT put in block0 page 65 66

    llfFCCmdRead_DRAM(ulMode, bank, ubLunNo, blk_no, page, DBT_PHY_ADDR, DRAM_DATA_SIZE,
                      TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
    FCCompletionPolling(&cmp, (gul_FW_TAG));
    if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
    {
        ulODBTBid = _REG32(TEMP_HBUF_PHY_ADDR);
        if(ulODBTBid == DBT_BLK_ID)
        {
            ret = ERR_OK;//found DBT
        }
    }
    else
    {
        ret = ERR_SEARCH_DBT;
        return ret;
    }

    if(ret == ERR_OK)
    {
        page = DBT_PAGENO + 1;
        llfFCCmdRead_DRAM(ulMode, bank, ubLunNo, blk_no, page, DBT_PHY_ADDR + DRAM_DATA_SIZE,
                          DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR + DRAM_HEAD_SIZE, DRAM_HEAD_SIZE);
        FCCompletionPolling(&cmp, (gul_FW_TAG));
        if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
        {
            ulODBTBid = _REG32(TEMP_HBUF_PHY_ADDR);
            if(ulODBTBid == DBT_BLK_ID)
            {
                ret |= ERR_OK;//found DBT
            }
        }
        else
        {
            ret = ERR_SEARCH_DBT;
        }
    }
    return ret;
}

void llfAPBuildDBT(void)
{
    U8 bank;
    U16 blk_no;
    U16 group_no;
    U32 defect_flag;
    U32 ret = ERR_OK;
    PVENDOR_CMD pVendorCmd;
    LLF_UNI_INFO *pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (struct _LLF_UNI_INFO *)LLF_UNI_INFO_ADDR;

    if (pResponseInfo->res_state == VENDOR_CMD_BUILD_DBT)
    {
        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pResponseInfo->res_progress = 0;
        pResponseInfo->res_err_code = 0;

        pLLFUniInfo->ubBankNo = 0;
        pLLFUniInfo->block = 0;
        pLLFUniInfo->ulWorkFunc = BUILD_ODBT_FUNC;

        //clean DBT to 0x0
        if((pVendorCmd->subcmd != BE_LLF_ALL) ||
                ((pVendorCmd->subcmd == BE_LLF_ALL) && (gubLLFMode == LLF_FORMAT_ERASE_NONDEFECT)))
        {
#ifdef NOFE_LLF_FLOW
            printk("NOFE_LLF_FLOW skip clear DBT\r\n");
#else
            memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
            cache_area_dwbinval(DBT_ADDR, DBT_SIZE);
            cache_dummy_update_read();
#endif
        }
    }
    ret = ERR_SEARCH_DBT;
    if(((pVendorCmd->subcmd != BE_LLF_ALL) || ((pVendorCmd->subcmd == BE_LLF_ALL)
            && (gubLLFMode == LLF_FORMAT_ERASE_NONDEFECT))) && (gfTestFlag != 1))
        // well because when erase first, it will mark defect block, so we should not search DBT, or it will cover DBT_ADDR
    {
        if(pLLFUniInfo->ubBankNo == 0)
        {
#ifndef SBLK_EXPAND
            ret = llfSearchDBT(pLLFUniInfo->ubBankNo + 1); //Search the DBT of this chip in bank1
            if(ret != ERR_OK)
            {
                ret = llfSearchDBT(pLLFUniInfo->ubBankNo); //Search the DBT of this chip in bank0
            }
#else
            U8 banknum;
            if(NandPara.ubBankNum > SYS_BANK_NUM)
                banknum = SYS_BANK_NUM;
            else
                banknum = NandPara.ubBankNum;

            for(bank = gubSblkBankStart + banknum - 1; bank >= gubSblkBankStart; bank--)
            {
                ret = llfSearchDBT(bank);
                if(ret == ERR_OK)
                {
                    break;
                }
            }
#endif
            if(ret == ERR_OK)
            {
                for(bank = 0; bank < NandPara.ubBankNum; bank++)
                {
                    for(blk_no = 0; blk_no < SYSTEM_BLOCK_MAX_NUM; blk_no++)
                    {
                        group_no = blk_no / NandPara.ubPlaneNumPerLun;
                        defect_flag = _REG32(DBT_ADDR + bank * DEFECT_USER_MP_BLK_TABLE_SIZE_PER_BANK
                                             + (group_no >> _5BIT_SHIFT) * WORD_BYTE_SIZE);
                        if((defect_flag & (1 << (group_no & _5BIT_MASK))) || (group_no == 0))
                        {
                            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, blk_no);
                        }
                    }
                }
                pLLFUniInfo->ubBankNo = NandPara.ubBankNum;
                gfDBTInitDone = LLF_DBT_SYSTEM;//need to check well
            }
            else
            {
                //clean DBT to 0x0
                memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
                cache_area_dwbinval(DBT_ADDR, DBT_SIZE);
                cache_dummy_update_read();
                gfDBTInitDone = LLF_DBT_INIT;
            }
        }
    }

    if(((ret == ERR_SEARCH_DBT) && (gubRdtImg != 1)) //DBT was not found
            || ((gubRdtImg == 1) && (_MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_LLF_NORMAL_EN) == 1)))
    {
        ret = ERR_OK;
#ifndef SORTING_FW_FOR_WK
        llfprintk("enter llfScanFactoryDefectBlocks\r\n");
        for(blk_no = 0; blk_no < NandPara.uwBlockNumPerLun; blk_no++)
        {
            ret |= llfScanFactoryDefectBlocks(blk_no);
        }
#endif
    }
    else if(gubRdtImg == 1)
    {
        ret = ERR_OK;
    }

    if(ret != ERR_OK)
    {
        llfEndResponce(ret);
        return;
    }
    else
    {
        gfDBTInitDone = LLF_DBT_FACTORY;
        {
            if(gubLLFMode != LLF_FORMAT_ERASE_NONDEFECT)
            {
                pResponseInfo->res_state = VENDOR_CMD_WRITE_FW;
                pResponseInfo->res_progress = 80;
            }
            else
            {
                pResponseInfo->res_state = VENDOR_CMD_ERASE;
                pResponseInfo->res_progress = 50;
            }

            pResponseInfo->res_err_code = 0;
            pLLFUniInfo->ulWorkFunc = NONE_FUNC;
#ifdef NOFE_LLF_FLOW
            if(gfTestFlag == 1)
            {
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
            }
#endif
            PrintDone();
            SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
        }
        if(pVendorCmd->subcmd != BE_LLF_ALL)
        {
            schedulerScheduleUTaskCore0(&taskTable_BE_Cpu, e_BE_llf_handle_taskid);
        }
    }
}

void llfAPBuildDBTandErase(void)
{
    U8 bank;
    U16 blk_no;
    U16 group_no;
    U32 defect_flag;
    U32 ret = ERR_OK;

    struct _LLF_UNI_INFO *pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;


    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;// SRAM used for responding to host
    pLLFUniInfo = (struct _LLF_UNI_INFO *)LLF_UNI_INFO_ADDR;

    if (pResponseInfo->res_state == VENDOR_CMD_START)
    {
        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pResponseInfo->res_progress = 0;
        pResponseInfo->res_err_code = 0;

        pLLFUniInfo->ubBankNo = 0;
        pLLFUniInfo->block = 0;
        pLLFUniInfo->ulWorkFunc = BUILD_ODBT_FUNC;

        //Initialize all DBIK No of chips as zero
        WRITE_REG_32(0xff0ff048, 0); // all zero
        WRITE_REG_32(0xff0ff04c, 1); // trigger
        while(!(READ_REG_32(0xff0ff050) & 0x1)); // wait for done
    }
    ret = ERR_SEARCH_DBT;

    if(pLLFUniInfo->ubBankNo == 0)
    {
        ret = llfSearchDBT(pLLFUniInfo->ubBankNo + 1); //Search the DBT of this chip in bank1

        if(ret != ERR_OK)
            ret = llfSearchDBT(pLLFUniInfo->ubBankNo); //Search the DBT of this chip in bank0

        if(ret == ERR_OK)
        {
            for(bank = 0; bank < CH_NUM_MAX * CE_NUM_MAX; bank++)
            {
                for(blk_no = 0; blk_no < SYSTEM_BLOCK_MAX_NUM; blk_no++)
                {
                    group_no = blk_no / NandPara.ubPlaneNumPerLun;
                    defect_flag = _REG32(DBT_ADDR + bank * DEFECT_USER_MP_BLK_TABLE_SIZE_PER_BANK
                                         + (group_no >> _5BIT_SHIFT) * WORD_BYTE_SIZE);
                    if((defect_flag & (1 << (group_no & _5BIT_MASK))) || (group_no == 0))
                    {
                        MarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, blk_no);
                    }
                }
            }
            pLLFUniInfo->ubBankNo = NandPara.ubBankNum;
        }
    }
    if(ret == ERR_SEARCH_DBT)  //DBT was not found
    {

    }

    if(ret != ERR_OK)
    {
        pResponseInfo->res_state = VENDOR_CMD_IDLE;
        pResponseInfo->res_progress = 100;
        pResponseInfo->res_err_code = ret;

        pLLFUniInfo->ulWorkFunc = NONE_FUNC;
        SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy); //InitROMJumpTable();
    }
    else
    {
        pLLFUniInfo->ubBankNo ++;
        pResponseInfo->res_progress = ((pLLFUniInfo->ubBankNo * 100) / NandPara.ubBankNum);

        if(pLLFUniInfo->ubBankNo >= NandPara.ubBankNum)
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = 0;

            pLLFUniInfo->ulWorkFunc = NONE_FUNC;
            PrintDone();
            SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy); //InitROMJumpTable();
        }
        schedulerScheduleUTaskCore0(&taskTable_BE_Cpu, e_BE_llf_handle_taskid);
    }
}

#ifndef SBLK_EXPAND
U32 llfGetSBlk(U16 (*pSBlkNo)[SYS_BLK])
{
    U8 bank_no;
    U16 blk_no;
    U8 isSandisk = 0;
    U8 count, banknum, ubBankNumChk, err_num = 0;
    U8 badsblkcnt = 0;

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;

    if(gfSelfTestFlag)
    {
        // TODO: self test already, erase sblock
    }

    if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC)
            && (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK))
    {
        isSandisk = 1;
    }
    // find 4 blocks (bank0~3)to save SBlk Config
    for(bank_no = 0; bank_no < banknum; bank_no++)
    {
        for (blk_no = 0; blk_no < SYS_BLK; blk_no++)
        {
            if(isSandisk == 1)
            {
                if (llfChkDefectBlkForSandisk(bank_no, blk_no) == 1)
                {
                    pSBlkNo[bank_no][blk_no] = blk_no;   // Get a good block for S block
                    llfprintk("bank%d block%d is sblock\r\n", bank_no, blk_no);

                }
            }
            else if (llfChkDefectBlk(bank_no, blk_no) == 1)
            {
                pSBlkNo[bank_no][blk_no] = blk_no;   // Get a good block for S block
                llfprintk("bank%d block%d is sblock\r\n", bank_no, blk_no);
            }
        }
    }

    //check can we get a good block to save Sblock
    for(bank_no = 0; bank_no < banknum; bank_no++)
    {
        count = 0;
        for (blk_no = 0; blk_no < SYS_BLK; blk_no++)
        {
            if(pSBlkNo[bank_no][blk_no] == 0xff)
            {
                count++;
                badsblkcnt++;
                gulFailedImageBitMap |= (1 << (blk_no + bank_no * 4));
            }
            else
            {
                gulSysblk |= 1 << (blk_no + 4 * bank_no);
            }
        }
        llfprintk("Sysblk = %x\r\n", gulSysblk);
        if(count == SYS_BLK)
        {
            AddErrorMessage(bank_no, 0, ERR_BAD_SBLOCK);
            err_num++;
            continue;
        }
    }
    llfprintk("gulFailedImageBitMap = 0x%x\r\n", gulFailedImageBitMap);
    if(err_num > 0)
    {
        return ERR_BAD_SBLOCK;
    }
    else if(badsblkcnt > 0)
    {
        return ERR_READ_SBLK;
    }
    return ERR_OK;
}
#else
U32 llfGetSBlk(U16 (*pSBlkNo)[SYS_BLK])
{
    U8 bank_no, i, j, ch_no, ce_no;
    U16 blk_no;
    U8 isSandisk = 0;
    U8 banknum, totalBank, sblkCnt = 0, sblkBankCnt = 0;
    U8 lun_no;
    U32 cmp, ret = ERR_OK, ulMode, pAddr, head_pAddr;

    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    if(NandPara.ubBankNum > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = NandPara.ubBankNum;

    totalBank = NandPara.ubBankNum;

    if(gfSelfTestFlag)
    {
        // TODO: self test already, erase sblock
    }

    if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC)
            && (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK))
    {
        isSandisk = 1;
    }


    for(gubSblkBankStart = 0; gubSblkBankStart + banknum <= totalBank; gubSblkBankStart++)
    {
        for(gubSblkStart = 0; gubSblkStart + SYS_BLK <= (SYS_BLK + EXTEND_SYS_BLK); gubSblkStart += SYS_BLK)
        {
            sblkBankCnt = 0;
            for(bank_no = gubSblkBankStart; bank_no < gubSblkBankStart + banknum; bank_no++)
            {
                lun_no = bank_no / (NandPara.ubChNum * NandPara.ubCENumPerCh);
                for(blk_no = gubSblkStart; blk_no < gubSblkStart + SYS_BLK; blk_no++)
                {
                    if (!llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no))
                    {
                        llfProgramSeqData(TEMP_BUF_ADDR, DRAM_DATA_SIZE);

                        //---------write----------
                        head_pAddr = TEMP_HBUF_PHY_ADDR;
                        pAddr = TEMP_BUF_PHY_ADDR;
                        gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank_no);
                        llfFCCmdWrite_DRAM(ulMode, bank_no, lun_no, blk_no, 0, pAddr, DRAM_DATA_SIZE,
                                           head_pAddr, DRAM_HEAD_SIZE);
                        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                        if(ret == ERR_OK)
                        {
                            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                            {
                                llfDbgPrintk(ALWAYS_MSG, "write error cmp = %x: %d_%d\r\n", cmp, bank_no, blk_no);
                                llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no);
                                continue;
                            }
                        }
                        else
                        {
                            llfprintk("write error %x\r\n", ret);
                            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no);
                            continue;
                        }

                        //---------read----------
                        head_pAddr = TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN;
                        pAddr = TEMP_BUF_PHY_ADDR + DRAM_DATA_SIZE;
                        gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);
                        llfFCCmdRead_DRAM(ulMode, bank_no, lun_no, blk_no, 0, pAddr, DRAM_DATA_SIZE,
                                          head_pAddr, DRAM_HEAD_SIZE );
                        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                        if(ret == ERR_OK)
                        {
                            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                            {
                                llfDbgPrintk(ALWAYS_MSG, "read error cmp = %x: %d_%d\r\n", cmp, bank_no, blk_no);
                                llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no);
                                continue;

                            }
                        }
                        else
                        {
                            llfprintk("read error %x\r\n", ret);
                            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no);
                            continue;
                        }

                        // Clean read memory cache
                        cache_area_dinval((TEMP_BUF_ADDR + DRAM_DATA_SIZE), DRAM_DATA_SIZE);
                        cache_dummy_update_read();

                        // Compare	data
                        ret = llfCompareData(TEMP_BUF_ADDR, TEMP_BUF_ADDR + DRAM_DATA_SIZE, DRAM_DATA_SIZE);
                        if(ret != ERR_OK)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "Compare data error cmp = %x: %d_%d\r\n", cmp, bank_no, blk_no);
                            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no);
                            continue;
                        }
                        llfEraseOneBlk(bank_no, blk_no);
                    }
                }
                sblkCnt = 0;
                for(blk_no = gubSblkStart; blk_no < gubSblkStart + SYS_BLK; blk_no++)
                {
                    if(isSandisk == 1)
                    {
                        if (!llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no)
                                && llfChkDefectBlkForSandisk(bank_no, blk_no))
                        {
                            pSBlkNo[bank_no - gubSblkBankStart][blk_no - gubSblkStart] =
                                blk_no;   // Get a good block for S block
                            llfprintk("bank%d block%d is sblock\r\n", bank_no, blk_no);
                            sblkCnt++;
                        }
                    }
                    else if (!llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no)
                             && (llfChkDefectBlk(bank_no, blk_no) == 1))
                    {
                        pSBlkNo[bank_no - gubSblkBankStart][blk_no - gubSblkStart] =
                            blk_no;   // Get a good block for S block
                        llfprintk("bank%d block%d is sblock\r\n", bank_no, blk_no);
                        sblkCnt++;
                    }
                }
#if defined(RL6643_VA) || defined(RL6577_VA)
#ifdef SBLK_EXPAND
                if(sblkCnt != SYS_BLK )
                {
                    gubDefaultSblkBad = 1 ;
                }
#endif
#endif
                if(0 != sblkCnt)
                {
                    sblkBankCnt++;
                }
                else
                {
                    AddErrorMessage(bank_no, 0, ERR_READ_SBLK);
                }
            }

            if(sblkBankCnt == banknum)
            {
                for(bank_no = 0; bank_no < banknum; bank_no++)
                {
                    for (blk_no = 0; blk_no < SYS_BLK; blk_no++)
                    {
                        if(pSBlkNo[bank_no][blk_no] != 0xff)
                        {
                            ch_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank_no + gubSblkBankStart) & 0xFF) >> 4;
                            ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank_no + gubSblkBankStart) & 0xF);
                            gulSblkCHCEMap[ch_no] |= 1 << ((ce_no << SYS_BLK_SHIFT) + blk_no);
                        }
                    }
                }
                llfprintk("[sblk]Find gubSblkBankStart %d, gubSblkStart %d ch0 %x ch1 %x !!\r\n", gubSblkBankStart,
                          gubSblkStart, gulSblkCHCEMap[0], gulSblkCHCEMap[1]);
                for(bank_no = 0; bank_no < banknum; bank_no++)
                {
                    llfprintk("bank %d sblk %d %d %d %d\r\n", gubSblkBankStart + bank_no, pSBlkNo[bank_no][0],
                              pSBlkNo[bank_no][1], pSBlkNo[bank_no][2], pSBlkNo[bank_no][3]);
                }
                break;
            }
            else
            {
                for(i = 0 ; i < banknum; i++)
                {
                    for(j = 0 ; j < SYS_BLK; j++)
                    {
                        pSBlkNo[i][j] = 0xff;
                    }
                }
            }
        }

        if(sblkBankCnt == banknum)
        {
            break;
        }
    }

    if(sblkBankCnt == banknum)
    {
        for(bank_no = 0; bank_no < banknum; bank_no++)
        {
            for(blk_no = 0; blk_no < gubSblkStart + SYS_BLK; blk_no++)
            {
                if((blk_no < gubSblkStart) || ((bank_no < gubSblkBankStart) && (blk_no < gubSblkStart + SYS_BLK)))
                {
                    if(!(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no)))
                    {
                        llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_no);
                        llfSetDBT(bank_no, blk_no, DBT_ADDR);
                        llfprintk("[sblk]mark bank %d blk %d sys defect and DBT\r\n", bank_no, blk_no);
                    }
                }
            }
        }
        return ERR_OK;
    }
    else
    {
        llfprintk("ERR!!! Can't find enough sblock in block 0-%d\r\n", SYS_BLK + EXTEND_SYS_BLK);
        return ERR_READ_SBLK;
    }
}

BOOL llfIsGoodSblk(U8 ubBank, U16 uwBlock)
{
#if 0
    U8 ubMapIdx;
    ubMapIdx = bank_no << SYS_BLK_SHIFT >> _5BIT_SHIFT;
    return (gulSblkBankMap[ubMapIdx] >> (bank_no << SYS_BLK_SHIFT & 0x1f)) & blk_no;
#endif
    U8 ubCH, ubCE, ubCHCE, ubIsSblk = FALSE;

    if(uwBlock < gubSblkStart)
    {
        return ubIsSblk;
    }
    else
    {
        uwBlock -= gubSblkStart;
    }

    ubCHCE = _MEM08(BANK_IMAPPING_TABLE_ADDR + ubBank);
    ubCH = (ubCHCE & 0xff) >> 4;
    ubCE = ubCHCE & 0xf;
    ubIsSblk = ((gulSblkCHCEMap[ubCH] >> (ubCE << SYS_BLK_SHIFT)) >> uwBlock) & 0x1;
    return ubIsSblk;
}

U32 llfGetInheritSBlk(U16 (*pSBlkNo)[SYS_BLK])
{
    U8 bank_no, ch_no, ce_no, blk_map, ubSblkBankTh, ubSblkBlkTh;
    U16 blk_idx;
    U8 sblkCnt = 0, sblkBankCnt = 0;
    U32 table_addr;

    ASSERT(ALWAYS_MSG, gubSblkStart + SYS_BLK <= SYSTEM_BLOCK_MAX_NUM);
    sblkBankCnt = 0;
    sblkCnt = 0;

    if(NandPara.ubBankNum < SBLK_BANK_CNT_MIN_END)
    {
        ubSblkBankTh = NandPara.ubBankNum;
        ubSblkBlkTh = SBLK_BLK_CNT_MIN_END;
    }
    else
    {
        ubSblkBankTh = SBLK_BANK_CNT_MIN_END;
        ubSblkBlkTh = SBLK_BLK_CNT_MIN_END;
    }

    table_addr = FC_TOP_REG_BASE + FR_PARSER_TO_LOGIC_CMDBUF_MAP0;

    for(ch_no = 0; ch_no < CH_NUM_MAX; ch_no++)
    {
        for(ce_no = 0; ce_no < CE_NUM_MAX; ce_no++)
        {
            blk_map = (gulSblkCHCEMap[ch_no] >> (ce_no << SYS_BLK_SHIFT)) & 0xf;

            if(blk_map != 0)
            {
                sblkBankCnt++;
                bank_no = *(U8 *)(table_addr + ch_no * 8 + ce_no);


                for(blk_idx = 0; blk_idx < SYS_BLK; blk_idx++)
                {
                    if((blk_map >> blk_idx) & 0x1)
                    {
                        pSBlkNo[bank_no - gubSblkBankStart][blk_idx] = blk_idx + gubSblkStart;
                        sblkCnt++;
                    }
                    else
                    {
                        llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no,
                                               blk_idx + gubSblkStart); //if modify, please sync calculatesnapshot function
                        llfSetDBT(bank_no, blk_idx + gubSblkStart, DBT_ADDR);
                    }
                }
            }
        }
    }

    for(bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
    {
        for(blk_idx = 0; blk_idx < gubSblkStart + SYS_BLK; blk_idx++)
        {
            if((blk_idx < gubSblkStart) || ((bank_no < gubSblkBankStart) && (blk_idx < gubSblkStart + SYS_BLK)))
            {
                if(!(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_idx)))
                {
                    llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, blk_idx);
                    llfSetDBT(bank_no, blk_idx, DBT_ADDR);
                    llfprintk("[sblk]mark bank %d blk %d sys defect\r\n", bank_no, blk_idx);
                }
            }
        }
    }


    for(bank_no = 0; bank_no < SYS_BANK_NUM; bank_no++)
    {
        llfprintk("SBlkNo bank%d: %d_%d_%d_%d\r\n", bank_no, pSBlkNo[bank_no][0], pSBlkNo[bank_no][1],
                  pSBlkNo[bank_no][2], pSBlkNo[bank_no][3]);
    }
    llfprintk("gubSblkStart = %d, gubSblkBankCnt = %d\r\n", gubSblkStart, gubSblkBankStart);
    llfprintk("gulSblkCHCEMap[0] = %x, [1] = %x\r\n", gulSblkCHCEMap[0], gulSblkCHCEMap[1]);

    if((sblkBankCnt >= ubSblkBankTh) && (sblkCnt >= ubSblkBlkTh))
    {
        return ERR_OK;
    }

    return ERR_READ_SBLK;
}


U32 llfCheckSblock(U16 (*pSBlkNo)[SYS_BLK])
{
    U8 bank, block, banknum;
    U8 is_good_bank;
    U8 pass_block = 0;
    U8 pass_bank = 0;
    U8 ubSblkBankTh, ubSblkBlkTh;
    U8 ch_no, ce_no;

    if(NandPara.ubBankNum > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = NandPara.ubBankNum;

    if(NandPara.ubBankNum < SBLK_BANK_CNT_MIN_END)
    {
        ubSblkBankTh = NandPara.ubBankNum;
        ubSblkBlkTh = SBLK_BLK_CNT_MIN_END;
    }
    else
    {
        ubSblkBankTh = SBLK_BANK_CNT_MIN_END;
        ubSblkBlkTh = SBLK_BLK_CNT_MIN_END;
    }

    for(ch_no = 0; ch_no < CH_NUM_MAX; ch_no++)
    {
        gulSblkCHCEMap[ch_no] = 0;
    }

    for(bank = 0; bank < banknum ; bank ++)
    {
        for(block = 0; block < SYS_BLK; block ++)
        {
            if(pSBlkNo[bank][block] != 0xff)
            {
                pass_block ++;
                is_good_bank = 1;
                ch_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank + gubSblkBankStart) & 0xFF) >> 4;
                ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank + gubSblkBankStart) & 0xF);
                gulSblkCHCEMap[ch_no] |= 1 << ((ce_no << SYS_BLK_SHIFT) + block);
            }
        }
        if(is_good_bank == 1)
        {
            pass_bank ++;
            is_good_bank = 0;
        }
    }

    llfprintk("pass_block is %d,pass_bank is %d \r\n", pass_block, pass_bank);

    if(pass_bank >= ubSblkBankTh && pass_block >= ubSblkBlkTh)
    {
        for(bank = 0; bank < banknum; bank++)
        {
            llfprintk("bank %d sblk %d %d %d %d\r\n", gubSblkBankStart + bank, pSBlkNo[bank][0],
                      pSBlkNo[bank][1],
                      pSBlkNo[bank][2], pSBlkNo[bank][3]);
        }
        llfprintk("gubSblkStart = %d, gubSblkBankCnt = %d\r\n", gubSblkStart, gubSblkBankStart);
        llfprintk("gulSblkCHCEMap[0] = %x, [1] = %x\r\n", gulSblkCHCEMap[0], gulSblkCHCEMap[1]);
        return ERR_OK;
    }
    else
    {
        llfprintk("New bad system block !! No enough sblock.\r\n");
        return ERR_BAD_SBLOCK;
    }

}
#endif


void llfSaveInfoToSpi()
{
#ifdef AUTO_DETECT_DIE
    U8 ubCh;
    U32 ulSetting;
#endif

#ifdef SBLK_EXPAND
    WRITE_REG_32(SPI_DL_SBLK_START_TAG_ADDR_WHEN_EA,
                 (gubSblkBankStart << 16) | (gubSblkStart << 8) | TAG_FOR_SBLK_EXPAND);
    WRITE_REG_32(SPI_DL_SBLK_CH0CEMAP_ADDR_WHEN_EA, gulSblkCHCEMap[0]);
    WRITE_REG_32(SPI_DL_SBLK_CH1CEMAP_ADDR_WHEN_EA, gulSblkCHCEMap[1]);
#endif

#ifdef AUTO_DETECT_DIE
    //write reseult in SPI
    ulSetting = TAG_FOR_AUTO_DETECTION | (gubTrueBankNum << 8) | (gubRowAddrLunShift << 16);
    WRITE_REG_32(SPI_DL_AUTO_DETECT_SETTING_WHEN_EA, ulSetting);
    printk("[ADD] SPI setting %x\r\n", ulSetting);
    for (ubCh = 0; ubCh < CH_NUM_MAX; ubCh++)
    {
        WRITE_REG_32(SPI_DL_DIE_MAP_WHEN_EA + (ubCh * 8), gulCHDieMap[ubCh][0]);
        WRITE_REG_32(SPI_DL_DIE_MAP_WHEN_EA + (ubCh * 8) + 4, gulCHDieMap[ubCh][1]);
        printk("[ADD] SPI set ch %d map %x_%x\r\n", ubCh, gulCHDieMap[ubCh][1], gulCHDieMap[ubCh][0]);
    }
#endif
}

U32 llfSetSblkToDBT(U32 dbt_addr, U16 (*pSBlkNo)[4])
{
    U8 bank_no, banknum, ubBankNumChk;
    U32 ret;
    U32 block_no;

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;

    // add SBLK blocks into DBT
    for(bank_no = 0; bank_no < banknum; bank_no++)
    {
        for(block_no = 0; block_no < SYS_BLK; block_no++)
        {
            if(pSBlkNo[bank_no][block_no] != 0xff)
            {
                //ce_no = 0;
#ifndef SBLK_EXPAND
                ret = llfSetDBT(bank_no, pSBlkNo[bank_no][block_no], dbt_addr);
#else
                ret = llfSetDBT(bank_no + gubSblkBankStart, pSBlkNo[bank_no][block_no], dbt_addr);
#endif
                if(ret != ERR_OK)
                    return ret;
            }
        }
    }

    return ERR_OK;
}
#if (defined(RL6577_VA)||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
U32 llfSetExtendRdtBlkToDBT(U32 dbt_addr)
{
    U8 bank_no;
    U32 ret;
    U32 block_no;

    // add Extend Rdt blocks into DBT
    for(bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
    {
        for(block_no = SYS_BLK; block_no < SYS_BLK + EXTEND_RDT_BLK; block_no++)
        {
#ifndef SBLK_EXPAND
            ret = llfSetDBT(bank_no, block_no, dbt_addr);
#else
            ret = llfSetDBT(bank_no + gubSblkBankStart, block_no, dbt_addr);
#endif
            if(ret != ERR_OK)
                return ret;
        }
    }

    return ERR_OK;
}
#endif

U32 llfWriteCodeBlock(U8 fw_type, U16 (*pSBlkNo)[4])
{
    U32  ulMode;
    U8  i, err_num = 0;
    U8  bank, lun_no, banknum, ubBankNumChk;
    U16 page;
    U16 fw_page_count;
    U32 ulCBlkBid;
    U32 cmp;
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    U32 addr = DUMMY_DATA_BUF_ADDR;
    U32 paddr = DUMMY_DATA_BUF_PHY_ADDR;
#else
    U32 addr, paddr;
#endif
    U32 ret = ERR_OK;
    //U32 pagebyte_shift;
    //U32 page_beef;
    U32 block_no;
    U8 BadSblk_num, BadSblk_flag;
    U16 PageEndNum;
    U32 ulpreFailedImageBitMap = gulFailedImageBitMap;

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    //initial code head data
    for(i = 0; i < DRAM_HEAD_SIZE / 4; i++)
    {
        _REG32(TEMP_HBUF_ADDR + i * 4) = CODE_BLK_ID;
    }
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;

    // initial bank, block, page
    page  = 0;
    //pagebyte_shift = NandPara.ubMpPageByteNumShift - NandPara.ubPlaneNumPerLunShift;
    //page_gold = uwPageNo;
    /*page_beef = page + ((MAX_IMEM_CODE_SIZE & ((1 << pagebyte_shift) - 1)) ?
                        ((MAX_IMEM_CODE_SIZE >> pagebyte_shift) + 1) :
                        (MAX_IMEM_CODE_SIZE >> pagebyte_shift));*/

#ifdef MST_MERGE
    if(((gubLlfMSTMergeEnable != NO_MST_MERGE) && (_MEM32(FW_RDT_TAG) == SIGNATURE_RDT))
            || (fw_type == FW_FOR_MST))
        PageEndNum = FW_MERGE_PAGENUM;
    else
#endif
        PageEndNum = NandPara.uwSLCPageNumPerBlock;
    // checking page number is acceptable
    if(fw_type == FW_IN_DRAM)
    {
        fw_page_count = (MAX_DRAM_CODE_SIZE / (NandPara.ubSectorNumPerPage * 512));
    }
    else
    {
        fw_page_count = (FW_CODE_SIZE
                         + (NandPara.ubSectorNumPerPage * 512) - 1) / (NandPara.ubSectorNumPerPage * 512);
    }
    llfDbgPrintk(ALWAYS_MSG, "fw_page_count = %d\r\n", fw_page_count);

#ifdef MST_MERGE
    if(fw_type != FW_FOR_MST)
#endif
    {
        if(fw_page_count > (PageEndNum - 1))
        {
            llfDbgPrintk(ALWAYS_MSG, "fw_page_count = %d, PageEndNum %d\r\n", fw_page_count, PageEndNum);
            return ERR_CODE_SIZE;
        }
    }
    /*if(_MEM32(FW_RDT_TAG) == SIGNATURE_RDT)
    {
        addr = FW_CODE_ADDR + (page_beef << pagebyte_shift);
        _REG32(addr) = SIGNATURE_END;
    }*/

    memset((void*) DUMMY_DATA_BUF_ADDR, 0x0, DUMMY_DATA_BUF_SIZE);
    cache_area_dwb(DUMMY_DATA_BUF_ADDR, DUMMY_DATA_BUF_SIZE);
    cache_dummy_update_read();

    // start to write FW code.
    addr = DUMMY_DATA_BUF_ADDR;
    paddr = DUMMY_DATA_BUF_PHY_ADDR;
    Change_ldpc(gubECC_CFG);
#ifndef SBLK_EXPAND
    for(bank = 0 ; bank < banknum; bank++)
#else
    for(bank = gubSblkBankStart ; bank < gubSblkBankStart + banknum; bank++)
#endif
    {
        lun_no = bank / NandPara.ubBankNumPerLun;
        BadSblk_num = 0;
#ifndef SBLK_EXPAND
        for(block_no = 0; block_no < SYS_BLK; block_no++)
#else
        for(block_no = gubSblkStart; block_no < gubSblkStart + SYS_BLK; block_no++)
#endif
        {
#ifndef SBLK_EXPAND
            llfprintk("pSblkNo[%d][%d] = %d\r\n", bank, block_no, pSBlkNo[bank][block_no]);
            if(pSBlkNo[bank][block_no] == 0xff)
#else
            llfprintk("pSblkNo[%d][%d] = %d\r\n", bank, block_no,
                      pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
            if(pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] == 0xff)
#endif
            {
                BadSblk_num++;
                gulSysblk &= ~(1 << (block_no + bank * 4));
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }
#if defined LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
            if(!((gulSysblk >> (bank * 4)) & (1 << block_no)))//skip bad block
            {
                llfprintk("WriteCode Jump bad system block %d_%d \r\n", bank, block_no);
                BadSblk_num++;
                gulSysblk &= ~(1 << (block_no + bank * 4));
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }
#else
            U8 ch_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xFF) >> 4;
            U8 ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xF);
            U32 bitloc = 1 << ((ce_no << SYS_BLK_SHIFT) + (block_no - gubSblkStart));
            if(!(gulSblkCHCEMap[ch_no] & bitloc))
            {
                llfprintk("WriteCode Jump bad system block %d_%d \r\n", bank, block_no);
                BadSblk_num++;
                continue;
            }
#endif
#endif
            BadSblk_flag = 0;
            // Write, page=1 (the page0 is sblock)
            for(page = 1; page < NandPara.uwSLCPageNumPerBlock; page++)
            {
                _REG32(TEMP_HBUF_ADDR) = CODE_BLK_ID;
                _REG32(TEMP_HBUF_ADDR + 4) = page;
                cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
                cache_dummy_update_read();
#ifdef MST_MERGE
                if(fw_type != FW_FOR_MST)
#endif
                {
                    if(page < PageEndNum)
                    {
                        if(( page >= TABLE_PAGENO ) && ( page < TABLE_PAGENO + TABLE_PAGENUM ))
                        {
#if defined(RL6643_VA)
                            addr = TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO + 1);
                            paddr = TEMP_BUF_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO + 1);
                            memcpy((void*)addr, (void*)(TABLE_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO)),
                                   1024 * 8);
                            memcpy((void*)(addr + 1024 * 8), (void*)CONFIG_BASE_VA_ADDR, 1024 * 8);
#else
                            addr = TABLE_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO);
                            paddr = TABLE_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO);
#endif
                        }
                        else if((page >= DBT_PAGENO) && (page < DBT_PAGENO + DBT_PAGENUM))
                        {
                            addr = DBT_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - DBT_PAGENO);
                            paddr = DBT_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - DBT_PAGENO);
                        }
                        else if(page <= fw_page_count)
                        {
                            addr = FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * (page - 1) * 512);
                            paddr = FW_CODE_PHY_ADDR + (NandPara.ubSectorNumPerPage * 512) * (page - 1);
                        }
#ifdef BLK_REMAP_PRO
                        else if((page >= BLK_REMAP_PRO_PAGENO)
                                && (page < BLK_REMAP_PRO_PAGENO + BLK_REMAP_PRO_PAGENUM))
                        {
#ifdef RDT_REMAP
                            if(_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
#endif
                            {
                                addr = BLK_REMAP_TABLE_ADDR + NandPara.ubSectorNumPerPage * 512 *
                                       (page - BLK_REMAP_PRO_PAGENO);
                                paddr = BLK_REMAP_TABLE_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 *
                                        (page - BLK_REMAP_PRO_PAGENO);
                            }
                        }
#endif
#ifdef KEEP_ORIG_DBT
                        else if((page >= ORIG_DBT_PAGENO) && (page < ORIG_DBT_PAGENO + ORIG_DBT_PAGENUM))
                        {
                            _REG32(TEMP_HBUF_ADDR) = ODBT_BLK_ID;
                            addr = ORIG_DBT_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - ORIG_DBT_PAGENO);
                            paddr = ORIG_DBT_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - ORIG_DBT_PAGENO);
                        }
#endif
                        else
                        {
                            addr = DUMMY_DATA_BUF_ADDR;
                            paddr = DUMMY_DATA_BUF_PHY_ADDR;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
#ifdef MST_MERGE
                if(fw_type == FW_FOR_MST)
                {
                    if(page < PageEndNum)
                    {
                        continue;
                    }
                    else if(page <= fw_page_count + FW_MERGE_PAGENUM)
                    {
                        addr = FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * (page - FW_MERGE_PAGENUM) * 512);
                        paddr = FW_CODE_PHY_ADDR + (NandPara.ubSectorNumPerPage * 512) * (page - FW_MERGE_PAGENUM);
                    }
                    else
                    {
                        addr = DUMMY_DATA_BUF_ADDR;
                        paddr = DUMMY_DATA_BUF_PHY_ADDR;
                    }
                }
#endif

                cache_area_dwbinval(addr, NandPara.ubSectorNumPerPage * 512);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank);
                llfFCCmdWrite_DRAM(ulMode, bank, lun_no, block_no, page, paddr,
                                   DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret == ERR_OK)
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "write code blk error: %d_%d_%d_0x%x\r\n", bank, block_no, page, cmp);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                }
                else
                {
                    llfprintk("Write code completion timeout: %d_%d_%d\r\n", bank, block_no, page);
                    BadSblk_flag = 1;
                    BadSblk_num++;
                    gulSysblk &= ~(1 << (block_no + bank * 4));
                    gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                    break;
                }
            }

            if( BadSblk_flag )
            {
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                continue;
            }

            // Read back, only compare we need.
            for(page = 0; page < fw_page_count; page++)
            {
                // Read back
                cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
                cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);

                gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
#ifdef MST_MERGE
                if(fw_type != FW_FOR_MST)
#endif
                {
                    llfFCCmdRead_DRAM(ulMode, bank, lun_no, block_no, (page + 1), TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                      TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                }
#ifdef MST_MERGE
                else
                {
                    llfFCCmdRead_DRAM(ulMode, bank, lun_no, block_no, (page + FW_MERGE_PAGENUM),
                                      TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                      TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                }
#endif
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret == ERR_OK)
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "read code blk error: %d_%d_%d_0x%x\r\n", bank, block_no, page, cmp);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }

                    ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                    if(ulCBlkBid != CODE_BLK_ID)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "code blk head invalid: %d_%d_%d_0x%x\r\n", bank, block_no, page,
                                     ulCBlkBid);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                    ret = llfCompareData(FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * page * 512),
                                         TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                    if(ret != ERR_OK)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "code blk data compare error: %d_%d_%d\r\n", bank, block_no, page);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                }
                else
                {
                    llfprintk("read code cmp timeout: %d_%d_%d\r\n", bank, block_no, page);
                    BadSblk_flag = 1;
                    BadSblk_num++;
                    gulSysblk &= ~(1 << (block_no + bank * 4));
                    gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                    break;
                }
            }
            if(BadSblk_flag)
            {
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                continue;
            }
            llfDbgPrintk(ALWAYS_MSG, "bank%d block%d read FW OK!\r\n", bank, block_no);

            //COMPARE DBT data
            for(page = DBT_PAGENO; page < (DBT_PAGENO + DBT_PAGENUM); page++)
            {
                // Read back
                cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
                cache_dummy_update_read();
                cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
                llfFCCmdRead_DRAM(ulMode, bank, lun_no, block_no, page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret == ERR_OK)
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "read DBT error: %d_%d_%d_0x%x\r\n", bank, block_no, page, cmp);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }

                    ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                    if(ulCBlkBid != CODE_BLK_ID)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "DBT id compare error: %d_%d_%d_0x%x\r\n", bank, block_no, page,
                                     ulCBlkBid);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                    ret = llfCompareData(DBT_ADDR + (NandPara.ubSectorNumPerPage * (page - DBT_PAGENO) * 512),
                                         TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                    if(ret != ERR_OK)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "DBT data compare error: %d_%d_%d\r\n", bank, block_no, page);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                }
                else
                {
                    llfprintk("read DBT cmp timeout: %d_%d_%d\r\n", bank, block_no, page);
                    BadSblk_flag = 1;
                    BadSblk_num++;
                    gulSysblk &= ~(1 << (block_no + bank * 4));
                    gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                    break;
                }
            }
            if( BadSblk_flag )
            {
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                continue;
            }
            llfDbgPrintk(ALWAYS_MSG, "bank%d block%d read DBT ok!\r\n", bank, block_no);

#ifdef MST_MERGE
            if(fw_type != FW_FOR_MST)
#endif
            {
                for( page = TABLE_PAGENO; page < TABLE_PAGENO + TABLE_PAGENUM; page++ )
                {
                    // Read back
                    cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
                    cache_dummy_update_read();
                    cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                    cache_dummy_update_read();

                    gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
                    llfFCCmdRead_DRAM(ulMode, bank, lun_no, block_no, page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                      TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                    if(ret == ERR_OK)
                    {
                        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "read FC TABLE error: %d_%d_%d_0x%x\r\n", bank, block_no, page, cmp);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }

                        ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                        if(ulCBlkBid != CODE_BLK_ID)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "FC table blk id compare error: %d_%d_%d_0x%x\r\n", bank, block_no, page,
                                         ulCBlkBid);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }
#if defined(RL6643_VA)
                        ret = llfCompareData(TEMP_BUF_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO + 1),
                                             TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
#else
                        ret = llfCompareData(TABLE_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO),
                                             TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
#endif
                        if(ret != ERR_OK)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "FC table blk data compare error: %d_%d_%d\r\n", bank, block_no, page);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }
                        page ++;
                    }
                    else
                    {
                        llfprintk("read FC table cmp timeout: %d_%d_%d\r\n", bank, block_no, page);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                }
                if( BadSblk_flag )
                {
#ifdef SBLK_EXPAND
                    pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                    continue;
                }
                llfDbgPrintk(ALWAYS_MSG, "bank%d block%d read FC table ok!\r\n", bank, block_no);

#ifdef BLK_REMAP_PRO
#ifdef RDT_REMAP
                if(_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
#endif
                {
                    //COMPARE BLOCKMAPPING data
                    for(page = BLK_REMAP_PRO_PAGENO; page < (BLK_REMAP_PRO_PAGENO + BLK_REMAP_PRO_PAGENUM);
                            page++)
                    {
                        // Read back
                        cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
                        cache_dummy_update_read();
                        cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                        cache_dummy_update_read();

                        gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
                        llfFCCmdRead_DRAM(ulMode, bank, lun_no, block_no, page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                          TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                        if(ret == ERR_OK)
                        {
                            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                            {
                                llfDbgPrintk(ALWAYS_MSG, "read PRO BLOCKMAPPING error: %d_%d_%d_0x%x\r\n", bank, block_no, page,
                                             cmp);
                                BadSblk_flag = 1;
                                BadSblk_num++;
                                gulSysblk &= ~(1 << (block_no + bank * 4));
                                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                                break;
                            }

                            ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                            if(ulCBlkBid != CODE_BLK_ID)
                            {
                                llfDbgPrintk(ALWAYS_MSG, "PRO BLOCKMAPPING blk id compare error: %d_%d_%d_0x%x\r\n",
                                             bank, block_no, page, ulCBlkBid);
                                BadSblk_flag = 1;
                                BadSblk_num++;
                                gulSysblk &= ~(1 << (block_no + bank * 4));
                                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                                break;
                            }

                            ret = llfCompareData(BLK_REMAP_TABLE_ADDR + (NandPara.ubSectorNumPerPage *
                                                 (page - BLK_REMAP_PRO_PAGENO) * 512),
                                                 TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                            if(ret != ERR_OK)
                            {
                                llfDbgPrintk(ALWAYS_MSG, "PRO BLOCKMAPPING data compare error: %d_%d_%d\r\n", bank, block_no, page);
                                BadSblk_flag = 1;
                                BadSblk_num++;
                                gulSysblk &= ~(1 << (block_no + bank * 4));
                                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                                break;
                            }
                        }
                        else
                        {
                            llfprintk("read PRO BLOCKMAPPING cmp timeout: %d_%d_%d\r\n", bank, block_no, page);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }
                    }
                    if( BadSblk_flag )
                    {
#ifdef SBLK_EXPAND
                        pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                        continue;
                    }
                    llfDbgPrintk(ALWAYS_MSG, "bank%d block%d read PRO BLOCKMAPPING ok!\r\n", bank, block_no);
                }
#endif

#ifdef KEEP_ORIG_DBT
                for(page = ORIG_DBT_PAGENO; page < (ORIG_DBT_PAGENO + ORIG_DBT_PAGENUM); page++)
                {
                    // Read back
                    cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
                    cache_dummy_update_read();
                    cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                    cache_dummy_update_read();

                    gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
                    llfFCCmdRead_DRAM(ulMode, bank, lun_no, block_no, page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                      TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                    ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                    if(ret == ERR_OK)
                    {
                        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "read orig dbt error: %d_%d_%d_0x%x\r\n", bank, block_no, page, cmp);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }

                        ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                        if(ulCBlkBid != ODBT_BLK_ID)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "Orig dbt id compare error: %d_%d_%d_0x%x\r\n", bank, block_no, page,
                                         ulCBlkBid);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }

                        ret = llfCompareData(ORIG_DBT_ADDR + (NandPara.ubSectorNumPerPage *
                                                              (page - ORIG_DBT_PAGENO) * 512),
                                             TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
                        if(ret != ERR_OK)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "Orig dbt data compare error: %d_%d_%d\r\n", bank, block_no, page);
                            BadSblk_flag = 1;
                            BadSblk_num++;
                            gulSysblk &= ~(1 << (block_no + bank * 4));
                            gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                            break;
                        }
                    }
                    else
                    {
                        llfprintk("read orig dbt cmp timeout: %d_%d_%d\r\n", bank, block_no, page);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        gulSysblk &= ~(1 << (block_no + bank * 4));
                        gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                        break;
                    }
                }
                if( BadSblk_flag )
                {
#ifdef SBLK_EXPAND
                    pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                    continue;
                }
                llfDbgPrintk(ALWAYS_MSG, "bank%d block%d read Orig dbt ok!\r\n", bank, block_no);
#endif
            }
        }
#ifndef SBLK_EXPAND
        if( BadSblk_num == SYS_BLK )
        {
            llfDbgPrintk(ALWAYS_MSG, "bank_no %d ,block0~3 are all bad!\r\n", bank);
            AddErrorMessage(bank, 0, ERR_BAD_SBLOCK);
            err_num++;
            continue;
        }
#else
        if( BadSblk_num == SYS_BLK + gubSblkStart )
        {
            llfAddErrorMessage(bank, 0, ERR_BAD_SBLOCK);
            llfDbgPrintk(ALWAYS_MSG, "bank_no %d ,badsblk num is %d!\r\n", bank,BadSblk_num);
            err_num++;
            continue;
        }
#endif
    }
    if(err_num > 0 )
    {
        return ERR_BAD_SBLOCK;
    }
    if(ulpreFailedImageBitMap != gulFailedImageBitMap)
    {
        llfprintk("ulpreFailedImageBitMap = %x, gulFailedImageBitMap = %x\r\n", ulpreFailedImageBitMap,
                  gulFailedImageBitMap);
        return ERR_READ_SBLK;
    }
    return ret;
}

#if 0 //#ifdef MST_MERGE
U32 llfWriteFWMerge(U16 pSBlkNo[CH_NUM_MAX * CE_NUM_MAX][SYS_BLK])
{
    U32 ulMode;
    U8  i;
    U8  bank_no, lun_no, banknum, ubBankNumChk;
    U16 page_no;
    U16 fw_page_count;
    U32 ulCBlkBid;
    U32 cmp;
#if defined(RL5771_FPGA) || defined(RL5771_VA)
    U32 addr = DUMMY_DATA_BUF_ADDR;
    U32 paddr = DUMMY_DATA_BUF_PHY_ADDR;
#else
    U32 addr, paddr;
#endif
    U32 ret = ERR_OK;
    U32 block_no;
    U8 BadSblk_num, BadSblk_flag;
    U8 ubSblkStart;
#ifndef SBLK_EXPAND
    U8 SystemBlockBadErr = 0;
    ubSblkStart = 0;
#else
    ubSblkStart = gubSblkStart;
#endif

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);;

    //initial code head data
    for(i = 0; i < DRAM_HEAD_SIZE / 4; i++)
    {
        _REG32(TEMP_HBUF_ADDR + i * 4) = CODE_BLK_ID;
    }
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;

    // initial page_no
    page_no  = 0;

    // checking page number is acceptable
    fw_page_count = (FW_CODE_SIZE +
                     (NandPara.ubSectorNumPerPage * 512) - 1) / (NandPara.ubSectorNumPerPage * 512);
    DBGPRINTK(ALWAYS_MSG, "[Merge]fw_page_count %d\r\n", fw_page_count);
    if(fw_page_count > (NandPara.uwSLCPageNumPerBlock - 1))
    {
        DBGPRINTK(ALWAYS_MSG, "[Merge]fw_page_count = %d\r\n", fw_page_count);
        return ERR_CODE_SIZE;
    }

    memset((void*) DUMMY_DATA_BUF_ADDR, 0x0, DUMMY_DATA_BUF_SIZE);
    cache_area_dwb(DUMMY_DATA_BUF_ADDR, DUMMY_DATA_BUF_SIZE);
    cache_dummy_update_read();

    // start to write FW code.
    Change_ldpc(gubECC_CFG);
#ifndef SBLK_EXPAND
    for (bank_no = 0 ; bank_no < banknum; bank_no++)
#else
    for (bank_no = gubSblkBankStart; bank_no < gubSblkBankStart + banknum; bank_no++)
#endif
    {
        lun_no = bank_no / NandPara.ubBankNumPerLun;
        BadSblk_num = 0;
        for(block_no = 0; block_no < SYS_BLK; block_no++)
        {
            printk("[Merge]pSblkNo[%d][%d] = %d\r\n", bank_no, block_no, pSBlkNo[bank_no][block_no]);
            if(pSBlkNo[bank_no][block_no] == 0xff)
            {
                BadSblk_num++;
                continue;
            }

            BadSblk_flag = 0;

            // Write, page_no=FW_MERGE_PAGENUM to NandPara.uwSLCPageNumPerBlock
            for(page_no = FW_MERGE_PAGENUM; page_no < NandPara.uwSLCPageNumPerBlock; page_no++)
            {
                _REG32(TEMP_HBUF_ADDR + 4) = page_no;//load code function

                if(page_no <= fw_page_count + FW_MERGE_PAGENUM)
                {
                    addr = FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * (page_no - FW_MERGE_PAGENUM) * 512);
                    paddr = FW_CODE_PHY_ADDR + (NandPara.ubSectorNumPerPage * 512) * (page_no - FW_MERGE_PAGENUM);
                }
                else
                {
                    addr = DUMMY_DATA_BUF_ADDR;
                    paddr = DUMMY_DATA_BUF_PHY_ADDR;
                }

                cache_area_dwbinval(addr, NandPara.ubSectorNumPerPage * 512);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank_no);
                llfFCCmdWrite_DRAM(ulMode, bank_no, lun_no, ubSblkStart + block_no, page_no, paddr, DRAM_DATA_SIZE,
                                   TEMP_HBUF_PHY_ADDR,
                                   DRAM_HEAD_SIZE);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));

                if(ret == ERR_OK)
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        DBGPRINTK(ALWAYS_MSG, "[Merge]write code blk error: %d_%d\r\n", bank_no, block_no);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        break;
                    }
                }
                else
                {
                    DBGPRINTK(ALWAYS_MSG, "[Merge]Write code timeout: %d_%d_%d\r\n", bank_no, block_no, page_no);
                    BadSblk_flag = 1;
                    BadSblk_num++;
                    break;
                }
            }
            if( BadSblk_flag )
            {
#ifdef SBLK_EXPAND
                pSBlkNo[bank_no][block_no] = 0xff;
#endif
                continue;
            }

            DBGPRINTK(ALWAYS_MSG, "[Merge]bank%d block%d write FW OK!\r\n", bank_no, block_no);

            // Read back, only compare merge FW
            for(page_no = 0; page_no < fw_page_count; page_no++)
            {
                // Read back
                cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
                cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);

                gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);

                llfFCCmdRead_DRAM(ulMode, bank_no, lun_no, ubSblkStart + block_no, (page_no + FW_MERGE_PAGENUM),
                                  TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));

                if(ret == ERR_OK)
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        DBGPRINTK(ALWAYS_MSG, "[Merge]read code blk error: %d_%d_%d\r\n", bank_no, block_no,
                                  page_no + FW_MERGE_PAGENUM);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        break;
                    }

                    ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
                    if(ulCBlkBid != CODE_BLK_ID)
                    {
                        DBGPRINTK(ALWAYS_MSG, "[Merge]code blk id compare error: %d_%d\r\n", bank_no, block_no);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        break;
                    }
#if 0
                    if(((page_no == 5) || (page_no == 0)) && (bank_no == 0) && (block_no == 0))
                    {
                        gubPrintEnable = 123;
                        printk("---------page %d---------\r\n", page_no + FW_MERGE_PAGENUM);
                    }
#endif
                    ret = llfCompareData(FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * page_no * 512),
                                         TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
//                    gubPrintEnable = 0;
                    if(ret != ERR_OK)
                    {
                        DBGPRINTK(ALWAYS_MSG, "[Merge]code blk data compare error: %d_%d\r\n", bank_no, block_no);
                        BadSblk_flag = 1;
                        BadSblk_num++;
                        break;
                    }
                }
                else
                {
                    llfprintk("[Merge]read code cmp timeout: %d_%d_%d\r\n", bank_no, block_no, page_no);
                    BadSblk_flag = 1;
                    BadSblk_num++;
                    break;
                }
            }
            if( BadSblk_flag )
            {
#ifdef SBLK_EXPAND
                pSBlkNo[bank_no][block_no] = 0xff;
#endif
                continue;
            }
            DBGPRINTK(ALWAYS_MSG, "[Merge]bank%d block%d read FW OK!\r\n", bank_no, block_no);
        }
#ifndef SBLK_EXPAND
        if( BadSblk_num == SYS_BLK )
        {
            DBGPRINTK(ALWAYS_MSG, "bank_no %d ,block0~3 are all bad!\r\n", bank_no);
            llfAddErrorMessage(bank_no, 4, ERR_BAD_SBLOCK);
            SystemBlockBadErr = 1;
        }
        if((bank_no == NandPara.ubBankNum - 1) && (SystemBlockBadErr == 1))
        {
            return ERR_BAD_SBLOCK;
        }
#else
        ret = llfCheckSblock(pSBlkNo);
#endif
    }

    return ret;
}

#if 0
U32 llfCheckFWFlashtype(void)
{
    U32 ret = ERR_LOAD_SYSTEM_IMAGE;

    switch(_MEM32(BE_VERSION_TAG))//need to check
    {
    case SIG_B0KB:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B0K)
        {
            return ret;
        }
        break;
    case SIG_BICS:
    case SIG_SBIC:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_8T22_SDR) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT23_SDR_TOGGLE_64GB) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_9T23_SDR_TOGGLE) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XW23))
        {
            return ret;
        }
        break;
    case SIG_B16A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B16)
        {
            return ret;
        }
        break;
    case SIG_B17A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B17)
        {
            return ret;
        }
        break;
    case SIG_B27A:

        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B27A)
        {
            return ret;
        }
        break;
    case SIG_B27B:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B27B)
        {
            return ret;
        }
        break;
    case SIG_B37R:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B37R)
        {
            return ret;
        }
        break;
    case SIG_N18A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N18A)
        {
            return ret;
        }
        break;
    case SIG_N28A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N28)
        {
            return ret;
        }
        break;
    case SIG_IN28:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N28)
        {
            return ret;
        }
        break;
    case SIG_SSV4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV4
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV4_64G)
        {
            return ret;
        }
        break;
    case SIG_SSV5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV5
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV5_64G)
        {
            return ret;
        }
        break;
    case SIG_H3V5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV5)
        {
            return ret;
        }
        break;
    case SIG_HQV5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DQV5)
        {
            return ret;
        }
        break;
    case SIG_H3V6:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV6)
        {
            return ret;
        }
        break;
    case SIG_YG2T:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YG2T) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YG2T_CS2))
        {
            return ret;
        }
        break;
    case SIG_YX2T:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YX2T)
        {
            return ret;
        }
        break;
    case SIG_TBI4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24_64G)
        {
            return ret;
        }
        break;
    case SIG_SBI4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24_64G)
        {
            return ret;
        }
        break;
    default:
    {
        return ret;
    }
    }
    return ERR_OK;
}
#endif
#endif

#ifdef SPECIAL_SYSTEM_BLOCK
U32 llfWriteExtRdtBlock()
{
    U32 ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 ulRet = ERR_OK;
    U32 i, ulCmp, ulBlkId;
    U32 ulAddr, ulPhyAddr, ulAddrOff;
    U16 uwSblkStart, uwSpBlk, uwPage;
    U8 ubLun, ubBank;
    U8 ubBadBlkCnt;
    U8 ubIsBadBlk;



    for (i = 0; i < DRAM_HEAD_SIZE; i += 4)
    {
        //To indicate the SP DBT is NOT from RDT
        _REG32(TEMP_HBUF_ADDR + i) = SBLK_BLK_ID;
    }
    cache_area_dwbinval(TEMP_HBUF_ADDR, DRAM_HEAD_SIZE);

    memset((void*)DUMMY_DATA_BUF_ADDR, 0x0, DUMMY_DATA_BUF_SIZE);
    cache_area_dwbinval(DUMMY_DATA_BUF_ADDR, DUMMY_DATA_BUF_SIZE);

    cache_dummy_update_read();

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ulAddr = DUMMY_DATA_BUF_ADDR;
    ulPhyAddr = DUMMY_DATA_BUF_PHY_ADDR;
#endif

#ifdef SBLK_EXPAND
    uwSblkStart = gubSblkStart;
#else
    uwSblkStart = 0;
#endif

    Change_ldpc(gubECC_CFG); //Refer to RdtWriteResultToNAND?

    for (ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
    {
        ubLun = ubBank / NandPara.ubBankNumPerLun;
        ubBadBlkCnt = 0;

        for (uwSpBlk = uwSblkStart + SYS_BLK;
                uwSpBlk < (uwSblkStart + SYS_BLK + EXTEND_RDT_BLK); uwSpBlk++)
        {
            ubIsBadBlk = FALSE;

            gul_FW_TAG = llfBETagSetting(TAG_ERASE, ubBank);
            FCSingleErase(ulMode, ubBank, ubLun, uwSpBlk, 0, BS_TLC_MODE);

            ulRet = FCCompletionPolling(&ulCmp, gul_FW_TAG);

            if (ERR_OK == ulRet)
            {
                if (0 != (ulCmp & BE_COMPLETION_ERROR_MASK))
                {
                    printk("[ERR] Ext rdt blk ers %d,%d %x\r\n", ubBank, uwSpBlk, ulCmp);
                    ubIsBadBlk = TRUE;
                    ubBadBlkCnt++;
                }
            }
            else
            {
                printk("[ERR] Ext rdt blk ers timeout %d,%d\r\n", ubBank, uwSpBlk);
                ubIsBadBlk = TRUE;
                ubBadBlkCnt++;
            }

            if (ubIsBadBlk)
            {
                continue;
            }

            for (uwPage = 0; uwPage < NandPara.uwSLCPageNumPerBlock; uwPage++)
            {
                if ((uwPage >= RDT_STATIC_DBT_PAGE_BEGIN)
                        && (uwPage < RDT_STATIC_DBT_PAGE_END))
                {
                    ulAddrOff = ubBank * DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK;

                    if ((DRAM_DATA_SIZE * (uwPage - RDT_STATIC_DBT_PAGE_BEGIN))
                            < DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK)
                    {
                        ulAddrOff += (DRAM_DATA_SIZE * (uwPage - RDT_STATIC_DBT_PAGE_BEGIN));
                    }

                    ulAddr = SP_DBT_ADDR + ulAddrOff;
                    ulPhyAddr = SP_DBT_PHY_ADDR + ulAddrOff;
                }
                else
                {
                    ulAddr = DUMMY_DATA_BUF_ADDR;
                    ulPhyAddr = DUMMY_DATA_BUF_PHY_ADDR;
                }

                cache_area_dinval(ulAddr, DRAM_DATA_SIZE);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_WRITE, ubBank);
                llfFCCmdWrite_DRAM(ulMode, ubBank, ubLun, uwSpBlk, uwPage, ulPhyAddr,
                                   DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);

                ulRet = FCCompletionPolling(&ulCmp, gul_FW_TAG);

                if (ERR_OK == ulRet)
                {
                    if (0 != (ulCmp & BE_COMPLETION_ERROR_MASK))
                    {
                        printk("[ERR] Ext rdt blk wr %d,%d,%d %x\r\n", ubBank, uwSpBlk, uwPage, ulCmp);
                        ubIsBadBlk = TRUE;
                        ubBadBlkCnt++;
                        break;
                    }
                }
                else
                {
                    printk("[ERR] Ext rdt blk wr timeout %d,%d,%d\r\n", ubBank, uwSpBlk, uwPage);
                    ubIsBadBlk = TRUE;
                    ubBadBlkCnt++;
                    break;
                }
            }

            if (ubIsBadBlk)
            {
                continue;
            }

            for (uwPage = RDT_STATIC_DBT_PAGE_BEGIN; uwPage < RDT_STATIC_DBT_PAGE_END; uwPage++)
            {
                cache_area_dinval((TEMP_HBUF_ADDR + DRAM_HEAD_SIZE), DRAM_HEAD_SIZE);
                cache_area_dinval(TEMP_BUF_ADDR, DRAM_DATA_SIZE);
                cache_dummy_update_read();

                gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);
                llfFCCmdRead_DRAM(ulMode, ubBank, ubLun, uwSpBlk, uwPage, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR + DRAM_HEAD_SIZE, DRAM_HEAD_SIZE);

                ulRet = FCCompletionPolling(&ulCmp, gul_FW_TAG);

                if (ERR_OK == ulRet)
                {
                    if ((ulCmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        printk("[ERR] SP DBT rd %d,%d,%d %x\r\n", ubBank, uwSpBlk, uwPage, ulCmp);
                        ubIsBadBlk = TRUE;
                        ubBadBlkCnt++;
                        break;
                    }

                    ulBlkId = _REG32(TEMP_HBUF_ADDR + DRAM_HEAD_SIZE);
                    if (SBLK_BLK_ID != ulBlkId)
                    {
                        printk("[ERR] SP DBT id %d,%d,%d %x\r\n", ubBank, uwSpBlk, uwPage, ulBlkId);
                        ubIsBadBlk = TRUE;
                        ubBadBlkCnt++;
                        break;
                    }

                    ulAddrOff = ubBank * DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK;
                    if ((DRAM_DATA_SIZE * (uwPage - RDT_STATIC_DBT_PAGE_BEGIN))
                            < DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK)
                    {
                        ulAddrOff += (DRAM_DATA_SIZE * (uwPage - RDT_STATIC_DBT_PAGE_BEGIN));
                    }
                    ulAddr = SP_DBT_ADDR + ulAddrOff;
                    ulRet = llfCompareData(ulAddr, TEMP_BUF_ADDR, DRAM_DATA_SIZE);
                    if(ulRet != ERR_OK)
                    {
                        printk("[ERR] SP DBT data %d,%d,%d\r\n", ubBank, uwSpBlk, uwPage);
                        ubIsBadBlk = TRUE;
                        ubBadBlkCnt++;
                        break;
                    }
                }
                else
                {
                    printk("[ERR] SP DBT rd timeout: %d,%d,%d\r\n", ubBank, uwSpBlk, uwPage);
                    ubIsBadBlk = TRUE;
                    ubBadBlkCnt++;
                    break;
                }
            }

            if (ubIsBadBlk)
            {
                continue;
            }

            llfDbgPrintk(ALWAYS_MSG, "Bank%d Block%d rd SP DBT OK\r\n", ubBank, uwSpBlk);
        }

#ifdef SBLK_EXPAND
        ulRet = ERR_OK;
#else
        if (EXTEND_RDT_BLK == ubBadBlkCnt)
        {
            printk("[ERR] Bank%d Block%d~%d all bad\r\n",
                   ubBank, uwSblkStart + SYS_BLK, uwSblkStart + SYS_BLK + EXTEND_RDT_BLK - 1);
        }
#endif
    }

    return ulRet;
}
#endif

// get start & last block of each bank, it is different if bank=2 or bank=1
U32 llfGetBlkRange(U32 ulODBTAddr, U8 bank, U16 *start_blk, U16 *last_blk)
{
    U8 be_no, ce;

    be_no = (bank & BE_NUM_MASK);
    ce = (bank / NandPara.ubLunNumPerCE / NandPara.ubChNum);
    if(NandPara.ubLunNumPerCE == 1) // single LUN per CE
    {
        *last_blk = _REG16(ulODBTAddr + be_no * CE_NUM_MAX * 2 + ce * 2);
        // skip ODBT block
        if(NandPara.ubPlaneNumPerLun == 2)
        {
            if ((*last_blk % 2) == 1) // odd block
                *last_blk -= 3;
            else// even block
                *last_blk -= 2;
        }
        else if(NandPara.ubPlaneNumPerLun == 1)
        {
            *last_blk -= 1;
        }
        else
            return ERR_GET_BANK_BLK_RANGE;

        *start_blk = 0;
    }
    else if(NandPara.ubLunNumPerCE == 2) // double LUN per CE
    {
        if((ce % 2) == 0) // even CE
        {
            *last_blk = (NandPara.uwBlockNumPerCE / 2) - 2;
            *start_blk = 0;
        }
        else// odd CE
        {
            *last_blk = _REG16(ulODBTAddr + be_no * CE_NUM_MAX * 2 + ce * 2);
            if(NandPara.ubPlaneNumPerLun == 2)
            {
                // skip ODBT block
                if (*last_blk % 2 == 1) // odd block
                    *last_blk -= 3;
                else// even block
                    *last_blk -= 2;
            }
            else if(NandPara.ubPlaneNumPerLun == 1)
            {
                *last_blk -= 1;
            }
            else
                return ERR_GET_BANK_BLK_RANGE;

            *start_blk = NandPara.uwBlockNumPerCE / 2;
        }
    }
    else
    {
        return ERR_GET_BANK_BLK_RANGE;
    }

    return ERR_OK;
}


U32 llfCntDBT(U32 dbt_addr)
{
    U32 i, total_DBT_defect;

    for (i = 0, total_DBT_defect = 0; i < (NandPara.ubBankNum * NandPara.uwMpBlockNumPerLun); i++)
    {
        if (_REG08(dbt_addr + i) == 0x7F)
            total_DBT_defect++;
    }
    return total_DBT_defect;
}

U32 llfWriteDBTGrp(U16 *pSBlkNo)
{
    U8 bank, lun_no;
    U8 i;
    U32 ulMode;
    U32 ulDBlkBid;
    U32 cmp;
    U32 page;
    U32 ret = ERR_OK;

#ifdef MICRON
    // Micron Read Retry
    U8 ubReadRetryIdx;
    U8 ubReadRetryTimes = 8;
#endif

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    //Two-plane: initial DBT head data 128byte
    for(i = 0; i < DRAM_HEAD_SIZE / 4; i++)
    {
        _REG32(TEMP_HBUF_ADDR + i * 4) = DBLK_BLK_ID;
    }
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();

    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        lun_no = bank / NandPara.ubBankNumPerLun;
        for(page = DBT_PAGENO; page < (DBT_PAGENO + DBT_PAGENUM); page++)
        {
            gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank);

            llfFCCmdWrite_DRAM(ulMode, bank, lun_no, pSBlkNo[bank], page,
                               DBT_PHY_ADDR + DRAM_DATA_SIZE * (page - DBT_PAGENO), DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR,
                               DRAM_HEAD_SIZE);
            FCCompletionPolling(&cmp, (gul_FW_TAG));

            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "write dbt error: %d_%d\r\n", bank, pSBlkNo[bank]);
                //return ERR_WRITE_SBLK;
                ASSERT_LLF(0);
            }

            // Read back
            cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun),
                              HEADER_MAX_LEN);
            cache_dummy_update_read();
            cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
            cache_dummy_update_read();

#ifdef MICRON
            // Micron Read Retry
            for (ubReadRetryIdx = 0; ubReadRetryIdx < ubReadRetryTimes; ubReadRetryIdx++)
            {

            }

            // reset to default
            if(ubReadRetryIdx > 0)
            {

            }
#else // not micron flash

            gul_FW_TAG = llfBETagSetting(TAG_READ, bank);

            llfFCCmdRead_DRAM(ulMode, bank, lun_no, pSBlkNo[bank], page, TEMP_BUF_PHY_ADDR,
                              DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR +
                              HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun, DRAM_HEAD_SIZE);
            FCCompletionPolling(&cmp, (gul_FW_TAG));

            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "read dbt error: %d_%d\r\n", bank, pSBlkNo[bank]);
                //return ERR_READ_SBLK;
                ASSERT_LLF(0);
            }

#endif // Micron

            ulDBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun);
            if(ulDBlkBid != DBLK_BLK_ID)
            {
                ASSERT_LLF(0);
                return ERR_WRITE_DBTGRP;
            }
            ret = llfCompareData(DBT_ADDR + DRAM_DATA_SIZE * (page - DBT_PAGENO), TEMP_BUF_ADDR,
                                 NandPara.ubSectorNumPerPage * 512 * 2);
            if(ret != ERR_OK)
                return ret;
        }

    }

    return ret;

}

U32 llfWriteTable(U16 *pSBlkNo)
{
    U8 bank, lun_no;
    U8 page;
    U8 i;
    U32 ulMode;
    U32 ulSBlkBid;
    U32 cmp;
    U32 ret = ERR_OK;

#ifdef MICRON
    // Micron Read Retry
    U8 ubReadRetryIdx;
    U8 ubReadRetryTimes = 8;
#endif

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    //initial SBlk head data
    for(i = 0; i < DRAM_HEAD_SIZE / 4; i++)
    {
        _REG32(TEMP_HBUF_ADDR + i * 4) = SBLK_BLK_ID;
    }
    /*cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();
    cache_area_dwbinval(SBLK_ADDR, SBLK_SIZE);
    cache_dummy_update_read();*/
    page = 61;//save table in page 61
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        MarkSystemBlkDefect(SBLK_ADDR + SBLK_OFFSET_SYSTEM_BLOCK_DBT, bank, pSBlkNo[bank]);
    }
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();
    //cache_area_dwbinval(SBLK_ADDR, SBLK_SIZE);
    // cache_dummy_update_read();

    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        lun_no = bank / NandPara.ubBankNumPerLun;
        gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank);

        llfFCCmdWrite_DRAM(ulMode, bank, lun_no, pSBlkNo[bank], page, SBLK_PHY_ADDR, DRAM_DATA_SIZE,
                           TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
        FCCompletionPolling(&cmp, (gul_FW_TAG));

        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "write sblk error: %d_%d\r\n", bank, page);
            //return ERR_WRITE_SBLK;
            ASSERT_LLF(0);
        }

        // Read back
        cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
        cache_dummy_update_read();
        cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
        cache_dummy_update_read();

#ifdef MICRON
        // Micron Read Retry
        for (ubReadRetryIdx = 0; ubReadRetryIdx < ubReadRetryTimes; ubReadRetryIdx++)
        {

        }

        // reset to default
        if(ubReadRetryIdx > 0)
        {

        }
#else // not micron flash

        gul_FW_TAG = llfBETagSetting(TAG_READ, bank);

        llfFCCmdRead_DRAM(ulMode, bank, lun_no, pSBlkNo[bank], page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                          TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
        FCCompletionPolling(&cmp, (gul_FW_TAG));

        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "read sblk error: %d_%d\r\n", bank, page);
            //return ERR_READ_SBLK;
            ASSERT_LLF(0);
        }

#endif // Micron

        ulSBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
        if(ulSBlkBid != SBLK_BLK_ID)
            return ERR_ALLOCATE_SBLK;
        ret = llfCompareData(SBLK_ADDR, TEMP_BUF_ADDR, SBLK_SIZE);
        if(ret != ERR_OK)
            return ret;
    }

    return ret;
}

U32 llfWriteSysGroup(U16 (*pSBlkNo)[4])
{
    U32 ret;
    U32 addr;

    // write FW mpblock
    if(READ_REG_32(CPU_MODE_REG)&CPU_EA_MODE)
        addr = SPI_DL_TAG_ADDR_WHEN_EA;
    else
        addr = SPI_DL_TAG_VA_ADDR;

    if(READ_REG_32(addr) == IMEM1_JUMP_TAG)
    {
        ret = llfWriteCodeBlock(FW_IN_DRAM, pSBlkNo);
    }
    else
    {
        if(gubMpModeSelect)
        {
            ret = llfWriteCodeBlock(FW_IN_DRAM, pSBlkNo);
        }
        else
        {
            ret = llfWriteCodeBlock(FW_IN_IMEM1, pSBlkNo);
        }
    }

    if( ret == ERR_READ_SBLK )
    {
        ret = llfWriteCodeBlockFailHandle();
    }
    if (ret != ERR_OK)
        return ret;

#ifdef SPECIAL_SYSTEM_BLOCK
    if ((!gubRdtImg) && (LLF_FORCE_FORMAT == gubLLFMode)
            && (LLF_DBT_FACTORY == gfDBTInitDone))
    {
        ret = llfWriteExtRdtBlock();

        if (ret != ERR_OK)
        {
            return ret;
        }
    }
#endif

    return ERR_OK;
}


U32 llfSetSBlockGeneralConfig(U32 sblk_addr)
{
    //U8 i = 0;
    memcpy((void *)(sblk_addr + SBLK_OFFSET_GENERAL_CONFIG_BASE),
           (const void *)(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_GENERAL_CONFIG_BASE),
           SBLK_GENERAL_CONFIG_LENGTH);
#if defined(RL6643_VA) || defined(RL6855_VA)
    U8 IDENTIFY_NAME[14] = {'R', 'E', 'A', 'L', 'T', 'E', 'K', '_', 'R', 'L', '6', '6', '4', '3'};
    U8 j;
    if(_REG32(sblk_addr + SBLK_OFFSET_IDENTIFY_NAME) == 0)
    {
        for(j = 0; j < 14; j++)
        {
            _MEM08(sblk_addr + SBLK_OFFSET_IDENTIFY_NAME + j) = IDENTIFY_NAME[j];
        }
    }
#endif
    if(_MEM32(FW_RDT_TAG) == SIGNATURE_RDT)
    {
        memcpy((void *)(sblk_addr + SBLK_OFFSET_FW_VERSION),
               (const void *)(FW_RDT_TAG + 8 - 64),
               (SBLK_OFFSET_EUI64L - SBLK_OFFSET_FW_VERSION));
    }
    else
    {
#if 0
        memcpy((void *)(sblk_addr + SBLK_OFFSET_FW_VERSION),
               (const void *)(BE_VERSION_TAG + 8 - 64),
               (SBLK_OFFSET_EUI64L - SBLK_OFFSET_FW_VERSION));
#endif
    }

    _REG32(SBLK_ADDR + SBLK_OFFSET_FAIL_IMAGE_BITMAP) = gulFailedImageBitMap;
#if 0
    if (_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
    {
        while(_MEM08((BE_VERSION_TAG + 8 - 64 + i)) !=
                0x2f)//serach '/'
        {
            i++;
        }
        i++;
        ASSERT(ALWAYS_MSG, i <= 60);
        memcpy((void *)(sblk_addr + SBLK_OFFSET_EUI64L),
               (const void *)(BE_VERSION_TAG + 8 - 64 + i), 8);
    }
#endif
#ifdef SBLK_EXPAND
    _REG32(sblk_addr + SBLK_OFFSET_SBLK_START_TAG) = ((gubSblkBankStart << 16) |
            (gubSblkStart << 8) | TAG_FOR_SBLK_EXPAND);
    _REG32(sblk_addr + SBLK_OFFSET_SBLK_CHCEMAP_CH0)  = gulSblkCHCEMap[0];
    _REG32(sblk_addr + SBLK_OFFSET_SBLK_CHCEMAP_CH1)  = gulSblkCHCEMap[1];
#endif

    return ERR_OK;
}

U32 llfSetSBlockPHYConfig(U32 sblk_addr)
{
    memcpy((void *)(sblk_addr + SBLK_OFFSET_PCIE_PHY_BASE),
           (const void *)(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_PCIE_PHY_BASE),
           SBLK_NVME_PHY_SETTING_LENGTH);

    return ERR_OK;
}

U32 llfSetSBlockFEConfig(U32 sblk_addr)
{
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RL6643_VA) || defined(RL6531_VB) || defined(RTS5771_VA)
    llfSetSBlockGeneralConfig(sblk_addr);
#endif
    llfSetSBlockPHYConfig(sblk_addr);

    return ERR_OK;
}

U32 llfWriteSBlk(U16 (*pSBlkNo)[4])
{
    U8 bank, lun_no, banknum, ubBankNumChk;
    U8 i;
    U8 err_num = 0;
    U32 ulMode;
    U32 ulSBlkBid;
    U32 cmp;
    U32 ret = ERR_OK;
    U8 ResetFlag = 0;
    // Micron Read Retry
    U32 block_no;
    U8 BadSblk_num;
    U32 ulpreFailedImageBitMap = gulFailedImageBitMap;

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    //initial SBlk head data
    for(i = 0; i < DRAM_HEAD_SIZE / 4; i++)
    {
        _REG32(TEMP_HBUF_ADDR + i * 4) = SBLK_BLK_ID;
    }

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;
#if defined(RL6643_FPGA)||defined(RL6643_VA)
    Change_ldpc(3);
#elif defined(RL6531_VB)
    Change_ldpc(0);
#elif defined(RL6577_FPGA)||defined(RL6577_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
    Change_ldpc(5);
#endif
#ifndef SBLK_EXPAND
    for(bank = 0; bank < banknum; bank++)
#else
    for(bank = gubSblkBankStart; bank < gubSblkBankStart + banknum; bank++)
#endif
    {
#ifndef SBLK_EXPAND
        for(block_no = 0; block_no < SYS_BLK; block_no++)
#else
        for(block_no = gubSblkStart; block_no < gubSblkStart + SYS_BLK; block_no++)
#endif
        {
            llfMarkSystemBlkDefect(SBLK_ADDR + SBLK_OFFSET_SYSTEM_BLOCK_DBT, bank,
                                   block_no);//if modify, please sync calculatesnapshot function
        }
    }
#if (defined(RL6577_VA)||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
#ifndef SBLK_EXPAND
        for(block_no = SYS_BLK; block_no < SYS_BLK + EXTEND_RDT_BLK; block_no++)
#else
        for(block_no = gubSblkStart + SYS_BLK; block_no < gubSblkStart + SYS_BLK + EXTEND_RDT_BLK;
                block_no++)
#endif
        {
            llfMarkSystemBlkDefect(SBLK_ADDR + SBLK_OFFSET_SYSTEM_BLOCK_DBT, bank,
                                   block_no);//if modify, please sync calculatesnapshot function
        }
    }
#endif
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();
#if defined(RL6447_VA)
    cache_area_dwbinval(SBLK_ADDR, SBLK_SIZE);
    cache_dummy_update_read();
#endif

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
            && (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_9T23_SDR_TOGGLE ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XF24 ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_64GB ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_128GB ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
                FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC))
    {
        for(bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if(llfIsMpBlockBad(DBT_ADDR, bank, NandPara.uwMpBlockNumPerLun - 1))
                ResetFlag |= 1;
        }
        if(ResetFlag)
        {
            llfInitErrorMessage();
            ret = TSBToggleResetFlash();
            if(ret != ERR_OK)
                return ret;
        }
    }

#ifndef SBLK_EXPAND
    for(bank = 0; bank < banknum; bank++)
#else
    for(bank = gubSblkBankStart; bank < gubSblkBankStart + banknum; bank++)
#endif
    {
        BadSblk_num = 0;
        lun_no = bank / NandPara.ubBankNumPerLun;
#ifndef SBLK_EXPAND
        for(block_no = 0; block_no < SYS_BLK; block_no++)
#else
        for(block_no = gubSblkStart; block_no < gubSblkStart + SYS_BLK; block_no++)
#endif
        {
#ifndef SBLK_EXPAND
            if(pSBlkNo[bank][block_no] == 0xff)
#else
            if(pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] == 0xff)
#endif
            {
                BadSblk_num++;
                gulSysblk &= ~(1 << (block_no + bank * 4));
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }
#if defined (LLF_CHECK_SYSTEMBLK_ERASE_FAIL)
#if !defined (SBLK_EXPAND)
            if(!((gulSysblk >> (bank * 4)) & (1 << block_no)))//skip bad block
            {
                llfprintk("WriteSBlk Jump bad system block %d_%d \r\n", bank, block_no);
                BadSblk_num++;
                gulSysblk &= ~(1 << (block_no + bank * 4));
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }
#else
            U8 ch_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xFF) >> 4;
            U8 ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank) & 0xF);
            U32 bitloc = 1 << ((ce_no << SYS_BLK_SHIFT) + (block_no - gubSblkStart));
            if(!(gulSblkCHCEMap[ch_no] & bitloc))
            {
                llfprintk("WriteCode Jump bad system block %d_%d \r\n", bank,
                          pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
                BadSblk_num++;
                continue;
            }
#endif
#endif
            gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank);
#ifndef SBLK_EXPAND
            llfFCCmdWrite_DRAM(ulMode, bank, lun_no, pSBlkNo[bank][block_no], 0, SBLK_PHY_ADDR, DRAM_DATA_SIZE,
                               TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
#else
            llfFCCmdWrite_DRAM(ulMode, bank, lun_no, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart],
                               0, SBLK_PHY_ADDR, DRAM_DATA_SIZE,
                               TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
#endif
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            if(ret != ERR_OK)
            {
                gulSysblk &= ~(1 << (block_no + bank * 4));
#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
                llfEraseOneBlk(bank, pSBlkNo[bank][block_no]);
#else
                llfEraseOneBlk(bank, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
#endif
#endif
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                BadSblk_num++;
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }

            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "write sblk error: %d_0x%x\r\n", bank, cmp);
                gulSysblk &= ~(1 << (block_no + bank * 4));
#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
                llfEraseOneBlk(bank, pSBlkNo[bank][block_no]);
#else
                llfEraseOneBlk(bank, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
#endif
#endif
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                BadSblk_num++;
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }

            // Read back
            cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
            cache_dummy_update_read();
            cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
            cache_dummy_update_read();

            gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
#ifndef SBLK_EXPAND
            llfFCCmdRead_DRAM(ulMode, bank, lun_no, pSBlkNo[bank][block_no], 0, TEMP_BUF_PHY_ADDR,
                              DRAM_DATA_SIZE,
                              TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
#else
            llfFCCmdRead_DRAM(ulMode, bank, lun_no, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart],
                              0, TEMP_BUF_PHY_ADDR,
                              DRAM_DATA_SIZE,
                              TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
#endif
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            if(ret != ERR_OK)
            {
                BadSblk_num++;
                gulSysblk &= ~(1 << (block_no + bank * 4));
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
                llfEraseOneBlk(bank, pSBlkNo[bank][block_no]);
#else
                llfEraseOneBlk(bank, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
#endif
#endif
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                continue;
            }

            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "read sblk error cmp = %x: %d_%d\r\n", cmp, bank, block_no);
                gulSysblk &= ~(1 << (block_no + bank * 4));
#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
                llfEraseOneBlk(bank, pSBlkNo[bank][block_no]);
#else
                llfEraseOneBlk(bank, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
#endif
#endif
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                BadSblk_num++;
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }
            ulSBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
            if(ulSBlkBid != SBLK_BLK_ID)
            {
                llfDbgPrintk(ALWAYS_MSG, "sblk id compare error: %d_%d\r\n", bank, block_no);
                gulSysblk &= ~(1 << (block_no + bank * 4));
#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
                llfEraseOneBlk(bank, pSBlkNo[bank][block_no]);
#else
                llfEraseOneBlk(bank, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
#endif
#endif
                BadSblk_num++;
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                continue;
            }
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
            ret = llfCompareData(SBLK_ADDR + SBLK_SIZE, TEMP_BUF_ADDR + SBLK_SIZE, SBLK_SIZE);
#else
            ret = llfCompareData(SBLK_ADDR, TEMP_BUF_ADDR, SBLK_SIZE);
#endif
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "sblk data compare error: %d_%d\r\n", bank, block_no);
                gulSysblk &= ~(1 << (block_no + bank * 4));
#ifdef LLF_CHECK_SYSTEMBLK_ERASE_FAIL
#if !defined (SBLK_EXPAND)
                llfEraseOneBlk(bank, pSBlkNo[bank][block_no]);
#else
                llfEraseOneBlk(bank, pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart]);
#endif
#endif
#ifdef SBLK_EXPAND
                pSBlkNo[bank - gubSblkBankStart][block_no - gubSblkStart] = 0xff;
#endif
                BadSblk_num++;
                gulFailedImageBitMap |= (1 << (block_no + bank * 4));
                continue;
            }
        }
#ifndef SBLK_EXPAND
        if( BadSblk_num == SYS_BLK )
        {
            llfAddErrorMessage(bank, 0, ERR_BAD_SBLOCK);
            llfDbgPrintk(ALWAYS_MSG, "bank_no %d ,block0~3 are all bad!\r\n", bank);
            err_num++;
            continue;
        }
#else
        ret = ERR_OK;
#endif
    }
    if(err_num > 0)
    {
        return ERR_BAD_SBLOCK;
    }
    if( ulpreFailedImageBitMap != gulFailedImageBitMap )
    {
        _MEM32(SBLK_ADDR + SBLK_OFFSET_FAIL_IMAGE_BITMAP) = gulFailedImageBitMap;
        ret = ERR_READ_SBLK;
    }
    return ret;
}

#if defined(COPYBACK_ALGORITHM) && !defined(ZIGZAG)
U32 LlfCountVictimMPB()
{
    U32 count = 0;
    U32 bs, bank;
    U32 temp;
    U16 uwMpBlockPerBank = NandPara.uwMpBlockNumPerLun;
#ifdef BLK_REMAP_PRO
#ifdef RDT_REMAP
    if(_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
#endif
    {
        if(gubBlkRemapProFlag == 1)
        {
            uwMpBlockPerBank = guwRealMPBlkNum;
        }
    }
#endif
    for(bs = guwCacll2PGroupEnd; bs < uwMpBlockPerBank; bs++)
    {
        temp = 0;
        for(bank = 0; bank < NandPara.ubBankNum; bank ++)
        {
            if(llfIsMpBlockBad(DBT_ADDR, bank, bs))
            {
                //do nothing
            }
            else
            {
                temp++;
            }
        }
        if(temp != NandPara.ubBankNum && temp > (BAD_BS_TH - 1))
        {
            count += temp;
        }
    }
    return (count * 3 / 4);
}
#endif
//0: Pass, ERR_LESS_FREE_FOR_GC: Fail
U32 llfCheckCapacityOP(U32 TotalMpBlockNum, U16 BadMpBlockNum, U16 HostClaimMpBlock,
                       U16 ValidBankNum,
                       U8 CapacityOP,
                       U32 ulL2PMpBlock)
{
#if defined(COPYBACK_ALGORITHM) && !defined(ZIGZAG)
    U32 totalvivtimnum = 0;
    totalvivtimnum = LlfCountVictimMPB();
    printk("[OP] total VictimMPB %d\r\n", totalvivtimnum);
#endif
    U32 realOPMPBlock;
    U32 MPBlockUsed;
    U16 uwSysMpBlockNum = guwCaclSSGroupEnd * ValidBankNum;
#ifdef OPAL_EN
    U32 ulOpalInSpecialMPBlock = 0;
    U16 uwPageNum;


    uwPageNum = gfPureSLC ? NandPara.uwSLCPageNumPerBlock : NandPara.uwPageNumPerBlock;

    ulOpalInSpecialMPBlock = (TCG_SPECIAL_SIZE_IN_LBN / (NandPara.uwLbnNumPerMpPage * uwPageNum)) + 1;
#if defined(COPYBACK_ALGORITHM) && !defined(ZIGZAG)
    MPBlockUsed = BadMpBlockNum + HostClaimMpBlock + ulOpalInSpecialMPBlock + uwSysMpBlockNum +
                  ulL2PMpBlock + totalvivtimnum;
#else
    MPBlockUsed = BadMpBlockNum + HostClaimMpBlock + ulOpalInSpecialMPBlock + uwSysMpBlockNum +
                  ulL2PMpBlock;
#endif

    llfprintk("[OP] Total %d bad %d Host %d sys %d l2p %d opal %d\r\n", TotalMpBlockNum,
              BadMpBlockNum, HostClaimMpBlock, uwSysMpBlockNum, ulL2PMpBlock, ulOpalInSpecialMPBlock);
#else
#if defined(COPYBACK_ALGORITHM) && !defined(ZIGZAG)
    MPBlockUsed = BadMpBlockNum + HostClaimMpBlock + uwSysMpBlockNum + ulL2PMpBlock + totalvivtimnum;
#else
    MPBlockUsed = BadMpBlockNum + HostClaimMpBlock + uwSysMpBlockNum + ulL2PMpBlock;
#endif

    llfprintk("[OP] Total %d bad %d Host %d sys %d l2p %d\r\n", TotalMpBlockNum,
              BadMpBlockNum, HostClaimMpBlock, uwSysMpBlockNum, ulL2PMpBlock);
#endif

    if(MPBlockUsed >= TotalMpBlockNum)
    {
        llfprintk("[OP] BAD+SYS+HOST %d MPBlock > Device total blk %d\r\n", MPBlockUsed, TotalMpBlockNum);
        return ERR_LESS_FREE_FOR_GC;
    }
    else
    {
#if defined(RL6643_VA) || defined(RL6643_FPGA) || defined(RL6531_VB)
        U32 uwTargetFree;
        realOPMPBlock = TotalMpBlockNum  - MPBlockUsed;
        uwTargetFree = (14 * ValidBankNum);//unit in MPBLOCK
        llfprintk("[OP] %d OP %d MPBLK. TargetFree %d MPBLK. real has %d Free MPBlock\r\n", CapacityOP,
                  (HostClaimMpBlock * CapacityOP / 1000), uwTargetFree, realOPMPBlock);
#if defined(COPYBACK_ALGORITHM) && !defined(ZIGZAG)
        if (TotalMpBlockNum - BadMpBlockNum - HostClaimMpBlock - uwSysMpBlockNum - ulL2PMpBlock -
                totalvivtimnum <
                (HostClaimMpBlock * CapacityOP / 1000) || (realOPMPBlock < uwTargetFree))
            return ERR_LESS_FREE_FOR_GC;
#else
        if (TotalMpBlockNum - BadMpBlockNum - HostClaimMpBlock - uwSysMpBlockNum - ulL2PMpBlock <
                (HostClaimMpBlock * CapacityOP / 1000) || (realOPMPBlock < uwTargetFree))
            return ERR_LESS_FREE_FOR_GC;
#endif

#else
        realOPMPBlock = TotalMpBlockNum  - MPBlockUsed;
        llfprintk("[OP] %d need %d Free MPBLK, real has %d Free MPBlock\r\n", CapacityOP,
                  (HostClaimMpBlock * CapacityOP / 1000), realOPMPBlock);
#if defined(COPYBACK_ALGORITHM) && !defined(ZIGZAG)
        if (TotalMpBlockNum - BadMpBlockNum - HostClaimMpBlock - uwSysMpBlockNum - ulL2PMpBlock -
                totalvivtimnum <
                (HostClaimMpBlock * CapacityOP / 1000))
            return ERR_LESS_FREE_FOR_GC;
#else
        if (TotalMpBlockNum - BadMpBlockNum - HostClaimMpBlock - uwSysMpBlockNum - ulL2PMpBlock <
                (HostClaimMpBlock * CapacityOP / 1000))
            return ERR_LESS_FREE_FOR_GC;
#endif

#endif
        else
            return ERR_OK;
    }
}

U16 llfCalculateRaidOP()
{
#if defined(RL6643_VA) && ((defined(RAID_EN) && !defined(SLC_RAID)) || (defined(COPYBACK_ALGORITHM) && defined(SLC_RAID)))
#if (defined(COPYBACK_ALGORITHM) && defined(SLC_RAID))
    U16 uwRaidMode = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun * gubRaidPagePerRaid;
#else
    U16 uwRaidMode = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun * gubTLCRaidPagePerRaid;
#endif
    U16 uwRaidOP = 1000 % uwRaidMode;

    if(uwRaidOP)
    {
        uwRaidOP = (1000 + uwRaidMode - uwRaidOP) / uwRaidMode;
    }
    else
    {
        uwRaidOP = 1000 / uwRaidMode;
    }
    llfDbgPrintk(ALWAYS_MSG, "OP for RAID: %d\r\n", uwRaidOP);
    return uwRaidOP;
#elif defined(RL6577_VA) && (defined(TLC_RAID) || defined(SLC_RAID))
    U16 uwRaidMode, uwRaidOP = 0;
#if defined(SLC_RAID) && !defined(TLC_RAID) && !defined(COPYBACK_ALGORITHM)
    if (gfPureSLC)
#endif
    {
        uwRaidMode = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun;
        uwRaidOP = (1000 / uwRaidMode);
        if (1000 % uwRaidMode)
        {
            uwRaidOP++;
        }
        llfDbgPrintk(ALWAYS_MSG, "OP for RAID: %d\r\n", uwRaidOP);
    }
    return uwRaidOP;
#else
    return 0;
#endif
}

U32 llfCheckSuperPBABits()
{
#if defined(RL6577_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA) || defined(RL6643_VA)
    U8 ubSuperPBABits, ubBSShift;

// ========== 6577 ==========
#if defined(RL6577_VA)
    {
#if defined(FTL_B17A) || defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B47R) || defined(FTL_B58R) || defined(FTL_N28A) || \
    defined(FTL_N38A) || defined(FTL_N48R) || defined(FTL_H3DTV7) || defined(FTL_YX2T) || defined(FTL_YX2Q) || defined(FTL_YX3T) || defined(FTL_SSV7) || defined(FTL_YX3T_WDS)
        /* B17A: min: 0+12+4, max: 4+12+4, l2pbs: 6
           B27A: min: 0+13+4, max: 4+13+4, l2pbs: 5
           B27B: min: 0+12+4, max: 4+12+4, l2pbs: 6
           B47R: min: 0+12+4, max: 4+12+4, l2pbs: 6
           B58R: min: 0+12+5, max: 4+12+5, l2pbs: 5
           N28A: min: 0+13+4, max: 4+13+4, l2pbs: 5
           N38A: min: 0+14+4, max: 4+14+4, l2pbs: 4
           N48R: min: 0+12+4, max: 4+12+4, l2pbs: 6
           H3DTV7: min: 0+13+4, max: 4+13+4, l2pbs: 5
           YX2T: min: 0+12+4, max: 4+12+4, l2pbs: 6
           YX2Q: min: 0+12+4, max: 4+12+4, l2pbs: 6 */
        ubSuperPBABits = PBA_NUM_IN_FW_26BIT;
#elif defined(FTL_B37R) || defined(FTL_H3DTV5) || defined(FTL_H3DQV5) || defined(FTL_H3DTV6)
        /* B37R: min: 0+11+4, max: 4+11+4, l2pbs: 6
           H3DTV5: min: 0+11+4, max: 4+11+4, l2pbs: 6
           H3DQV5: min: 0+12+4, max: 4+12+4, l2pbs: 6
           H3DTV6: min: 0+11+4, max: 4+11+4, l2pbs: 6 */
        ubSuperPBABits = PBA_NUM_IN_FW_25BIT;
#elif defined(FTL_B16A) || defined(FTL_SSV6)
        /* B16A: min: 0+12+3, max: 4+12+3, l2pbs: 6
           SSV6: min: 0+12+3, max: 4+12+3, l2pbs: 6 */
        ubSuperPBABits = PBA_NUM_IN_FW_25BIT;
#elif defined(FTL_YG2T) || defined(FTL_TSB_BICS4) || defined(FTL_TSB_BICS5) || \
      defined(FTL_SANDISK_BICS4) || defined(FTL_SANDISK_BICS5) \
      || defined(FTL_SANDISK_BICS4_QLC) || defined(FTL_SANDISK_BICS5_QLC) || defined(FTL_SSV5)
        /* YG2T: min: 0+11+3, max: 4+11+3, l2pbs: 6
           TBiCS4: min: 0+11+3, max: 4+11+3, l2pbs: 6
           TBiCS5: min: 0+11+3, max: 4+11+3, l2pbs: 6
           SBiCS4: min: 0+11+3, max: 4+11+3, l2pbs: 6
           SBiCS5: min: 0+11+3, max: 4+11+3, l2pbs: 6
           SBiCS4Q: min: 0+11+3, max: 4+11+3, l2pbs: 6
           SSV5: min: 0+11+3, max: 4+11+3, l2pbs: 6 */
        ubSuperPBABits = PBA_NUM_IN_FW_24BIT;
#else
        /* TBiCS3: min: 0+10+3, max: 4+10+3, l2pbs: 6
           SBiCS3: min: 0+10+3, max: 4+10+3, l2pbs: 6
           SSV4: min: 0+10+3, max: 4+10+3, l2pbs: 6 */
        ubSuperPBABits = PBA_NUM_IN_FW_23BIT;
#endif
    }
// ========== 5771 FPGA ==========
#elif defined(RTS5771_FPGA)
    {
        // Plz see wiki file: https://wiki.realtek.com/display/PCSSD/RL6817+FW+DOC?preview=/162829627/223792505/SUPER_PBA_BITs%20and%20L2pBs_Limit_with_4TB.xlsx
#if defined(FTL_B16A) || defined(FTL_B17A) || defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B37R)\
     || defined(FTL_B47R) || defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N38A) || defined(FTL_N48R)\
     || defined(FTL_H3DTV5) || defined(FTL_H3DTV6)|| defined(FTL_H3DQV5) || defined(FTL_H3DTV7)\
     || defined(FTL_SSV6) || defined(FTL_YX2T) || defined(FTL_YX3T)
        ubSuperPBABits = PBA_NUM_IN_FW_26BIT;
#elif defined(FTL_SSV5) || defined(FTL_TSB_BICS4) || defined(FTL_TSB_BICS5)\
      || defined(FTL_SANDISK_BICS4) || defined(FTL_SANDISK_BICS5) || defined(FTL_SANDISK_BICS4_QLC)\
      || defined(FTL_YG2T)
        ubSuperPBABits = PBA_NUM_IN_FW_25BIT;
#elif defined(FTL_SSV4) || defined(FTL_TSB_BICS3) || defined(FTL_SANDISK_BICS3)
        ubSuperPBABits = PBA_NUM_IN_FW_24BIT;
#else
#error "need to define SUPER_PBA_BITS"
        ubSuperPBABits = PBA_NUM_IN_FW_23BIT;
#endif
    }
// ========== 5771 ==========
#elif defined(RTS5771_VA)
    {
        // Plz see wiki file: https://wiki.realtek.com/display/PCSSD/RL6817+FW+DOC?preview=/162829627/223792505/SUPER_PBA_BITs%20and%20L2pBs_Limit_with_4TB.xlsx
#if defined(FTL_B16A) || defined(FTL_B17A) || defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B37R)\
     || defined(FTL_B47R) || defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N38A) || defined(FTL_N48R)\
     || defined(FTL_H3DTV5) || defined(FTL_H3DTV6)|| defined(FTL_H3DQV5) || defined(FTL_H3DTV7)\
     || defined(FTL_SSV5) || defined(FTL_SSV6) || defined(FTL_TSB_BICS4) || defined(FTL_TSB_BICS5)\
     || defined(FTL_SANDISK_BICS4) || defined(FTL_SANDISK_BICS5) || defined(FTL_SANDISK_BICS4_QLC)\
     || defined(FTL_YG2T) || defined(FTL_YX2T) || defined(FTL_YX3T) || defined(FTL_YX2Q)
        ubSuperPBABits = PBA_NUM_IN_FW_26BIT;
#elif defined(FTL_SSV4) || defined(FTL_TSB_BICS3) || defined(FTL_SANDISK_BICS3)
        ubSuperPBABits = PBA_NUM_IN_FW_25BIT;
#else
#error "need to define SUPER_PBA_BITS"
        ubSuperPBABits = PBA_NUM_IN_FW_23BIT;
#endif
    }
// ========== 6643/6855 ==========
#elif defined(RL6643_VA)
    ubSuperPBABits = SUPER_PBA_BITS;
#endif

    ubBSShift = NandPara.ubBankNumShift + NandPara.ubPageNumPerBlockShift +
                NandPara.ubLbnNumPerMpPageShift;
#ifdef RL6643_VA
    if (ubBSShift < 16)
    {
        ubBSShift = 16;
    }
#elif defined(MULTI_LUN)
    if (ubSuperPBABits < PBA_TOTAL_USABLE_BITS)
    {
        ubSuperPBABits++; // add 1 for much L2P of serial multi-lun
    }
#endif
    printk("Super PBA bits %d, L2P BS shift %d, right shift %d\r\n", ubSuperPBABits, ubBSShift,
           (PBA_TOTAL_USABLE_BITS - ubSuperPBABits));
    if (((PBA_TOTAL_USABLE_BITS - ubSuperPBABits) + ubBSShift) < 16)
    {
        printk("[ERR] L2P BS bits shift %d+%d not larger than 16, PBA bits %d\r\n",
               ubBSShift, (PBA_TOTAL_USABLE_BITS - ubSuperPBABits), ubSuperPBABits);
        return ERR_UNENOUGH_SNAPSHOT;
    }
    if ((1 << (ubSuperPBABits - ubBSShift)) <= guwCacll2PGroupEnd)
    {
        printk("[ERR] L2PGroupEnd %d excess PBA bits\r\n", guwCacll2PGroupEnd);
        return ERR_UNENOUGH_SNAPSHOT;
    }
// ========== OTHER PROJECT ==========
#else
    if ((1 << (SUPER_PBA_BITS - NandPara.ubBankNumShift - NandPara.ubPageNumPerBlockShift -
               NandPara.ubLbnNumPerMpPageShift )) <= guwCacll2PGroupEnd)
    {
        llfDbgPrintk(ALWAYS_MSG, "L2PGroupEnd excess PBA %dbit %d\r\n", SUPER_PBA_BITS,
                     guwCaclSSGroupEnd);
        return ERR_UNENOUGH_SNAPSHOT;
    }
#endif
    return ERR_OK;
}

U32 llfCheckDefectThresholdAgain()
{
// K2: After inherit ori RDT and Erase all, then will not check defetect thv, so CAP maybe not enough when Erase fail too much
    U32 ret = ERR_OK;
    U8 i = 0;
    // check if defect blocks are out of threshold
    ret = llfCheckDefectThreshold();
    if (ret != ERR_OK)
    {
        for (i = 0; i < 8; i++)
        {
            if (gulAutoCapacity[i] == 0)
            {
                break;
            }
#ifdef EXTEND_LBA
            else if (gulAutoCapacity[i] <= 4096)
#else
            else if (gulAutoCapacity[i] <= 2048)
#endif
            {
                guwHostClaimCapacity = gulAutoCapacity[i];
            }
            else
            {
                guwHostClaimCapacity = gulAutoCapacity[i] >> (1 + 10 + 10); // LBA(Sector) to GB
                printk("Calculate capacity as LBA, gulAutoCapacity[%d] %x\r\n", i, gulAutoCapacity[i]);
            }
            gulMaxLBAAddr = 97696368 + guwHostClaimCapacity * 1953504 - 50 * 1953504 - 1;
            _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET) = gulMaxLBAAddr + 1;
            CalculateSnapshotArea();
            llfDbgPrintk(ALWAYS_MSG, "[AutoCap] Adjust to %d GB\r\n", guwHostClaimCapacity);
            ret = llfCheckDefectThreshold();

            if (ret == ERR_OK)
            {
                break;
            }
        }
        if (ret != ERR_OK)
        {
            return ret;
        }
    }

    return ERR_OK;
}

U32 llfCheckDefectThreshold()
{
    U8 bank_no, plane_no;//isErr;
    U16 mpblock_no, block_no, uwMpBlockPerBank, PerBankMarkNum;
    U16 MpDefectPerBS, guwCacll2PGroupEndshift;
    U32 TotalMpBlockNumInNonSLCArea, HostClaimMpBlock;
    //U32 TotalBlockNumInSystem, TotalMpBlockNumInSnapshot;
    U16 BadBlockNumInSystemArea, BadMpBlockNumInSnapshotArea;
    U16 BadMpBlockNumInSLCArea, BadMpBlockNumInNonSLCArea;
    U32 BadMpBlockPerBankInNonSLCArea[BANK_NUM_MAX];
    U16 BadBlockNumInAll, BadBlockPerBankInAll[BANK_NUM_MAX], temp[BANK_NUM_MAX], uwPageNum;
    U8 isErr;
    U8 CapacityOP;
    U16 uwL2PMpBlock = 0;
#ifdef L2P_SHRINK
    U16 uwL2PGoodMpBlock = 0;
#endif
    // The Number of Block for Dynamic SBlock is depends on the number of Bank is enough to preserved for
    // current, current-backup, pre-current and pre-current-backup, so it must bigger than 4.
    uwPageNum = gfPureSLC ? NandPara.uwSLCPageNumPerBlock : NandPara.uwPageNumPerBlock;
#ifdef EXTEND_LBA
    U32 pLBA = (U32)(&gulMaxLBAAddr);
    U32 tmpLBN = (_MEM32(pLBA) >> gubCacheSectorNumPerLbnShift) +
                 (_MEM32(pLBA + 4) << (32 - gubCacheSectorNumPerLbnShift));

    HostClaimMpBlock = (tmpLBN / (NandPara.uwLbnNumPerMpPage *
                                  (uwPageNum - NandPara.uwLastPageNumPerBlock))) + 1;
#else
#if defined(RL6577_VA)
    HostClaimMpBlock = ((gulMaxLBAAddr + SPECIAL_DEFAULT_ROOM_LBA_SIZE) /
                        (guwCacheSectorNumPerLbn * NandPara.uwLbnNumPerMpPage * (uwPageNum -
                                NandPara.uwLastPageNumPerBlock))) + 1;
#else
    HostClaimMpBlock = (gulMaxLBAAddr / (guwCacheSectorNumPerLbn * NandPara.uwLbnNumPerMpPage *
                                         (uwPageNum - NandPara.uwLastPageNumPerBlock))) + 1;
#endif
#endif
    uwMpBlockPerBank = NandPara.uwMpBlockNumPerLun;
#ifdef BLK_REMAP_PRO
#ifdef RDT_REMAP
    if(_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
#endif
    {
        if(gubBlkRemapProFlag == 1)
        {
            uwMpBlockPerBank = guwRealMPBlkNum;
        }
    }
#endif

    TotalMpBlockNumInNonSLCArea = uwMpBlockPerBank * NandPara.ubBankNum;
    //  llfprintk("TotalMpBlockNumInNonSLCArea = %d  MpBlock = %d bankNum = %d\r\n",
    //      TotalMpBlockNumInNonSLCArea, NandPara.uwMpBlockNumPerLun, NandPara.ubBankNum);

    for (bank_no = 0; bank_no < BANK_NUM_MAX; bank_no++)
    {
        BadBlockPerBankInAll[bank_no] = 0;
    }

    // System Block, Single-plane Block
    BadBlockNumInSystemArea = 0;
    for (mpblock_no = 0; mpblock_no < guwCaclSSGroupBegin; mpblock_no++)
    {
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            block_no = mpblock_no * NandPara.ubPlaneNumPerLun;
            for (plane_no = 0; plane_no < NandPara.ubPlaneNumPerLun; plane_no++)
            {
                if (llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block_no + plane_no))
                {
                    BadBlockNumInSystemArea++;
                    BadBlockPerBankInAll[bank_no]++; // single-plane
                }
            }
        }
    }

    // System block, MpBlock
    BadMpBlockNumInSnapshotArea = 0;
    for (; mpblock_no < guwCacll2PGroupEnd; mpblock_no++)
    {
        MpDefectPerBS = 0;
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            if (llfIsMpBlockBad(DBT_ADDR, bank_no, mpblock_no))
            {
                MpDefectPerBS++;
            }
        }

        U8 ubBadBsThr = BAD_BS_TH;
#ifdef L2P_SHRINK
        if(mpblock_no >= guwCaclSSGroupEnd && mpblock_no < guwCacll2PGroupEnd)
        {
            ubBadBsThr += (BAD_BS_TH >> 1);
            uwL2PMpBlock += NandPara.ubBankNum;
        }
#endif
        if (MpDefectPerBS > ubBadBsThr)
        {
            BadMpBlockNumInSnapshotArea += NandPara.ubBankNum;
        }
        else
        {
            BadMpBlockNumInSnapshotArea += MpDefectPerBS;
#ifdef L2P_SHRINK
            if(mpblock_no >= guwCaclSSGroupEnd && mpblock_no < guwCacll2PGroupEnd)
            {
                uwL2PGoodMpBlock += (NandPara.ubBankNum - MpDefectPerBS);
            }
#endif
        }
    }
#ifdef L2P_SHRINK
    llfprintk("l2p good mp num %d\r\n", uwL2PGoodMpBlock);
#endif

    BadMpBlockNumInSLCArea = 0;

    // NonSLC, User data area, MpBlock
    BadMpBlockNumInNonSLCArea = 0;
    for (bank_no = 0; bank_no < BANK_NUM_MAX; bank_no++)
    {
        BadMpBlockPerBankInNonSLCArea[bank_no] = 0;
    }

    for (; mpblock_no < uwMpBlockPerBank; mpblock_no++)
    {
        MpDefectPerBS = 0;
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            if (llfIsMpBlockBad(DBT_ADDR, bank_no, mpblock_no))
            {
                MpDefectPerBS++;
                BadMpBlockPerBankInNonSLCArea[bank_no]++;
            }
        }

        // check if Block Stripe broken (defect larger than half of blocks)
        if (MpDefectPerBS > BAD_BS_TH)
        {
            BadMpBlockNumInNonSLCArea += NandPara.ubBankNum;
        }
        else
        {
            BadMpBlockNumInNonSLCArea += MpDefectPerBS;
        }
    }

    // Total, all marked block
    BadBlockNumInAll = 0;
    for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
    {
        PerBankMarkNum = 0;
        for (block_no = 0; block_no < NandPara.uwBlockNumPerLun; block_no++)
        {
            if (llfIsBlockBad(DBT_ADDR, bank_no, block_no))
            {
                BadBlockNumInAll++;
                PerBankMarkNum++;
            }
        }
        llfDbgPrintk(ALWAYS_MSG, "bank %d MarkNum %d\r\n", bank_no, PerBankMarkNum);
    }

    // print check/erase bad and whole system estimated bad
    llfDbgPrintk(ALWAYS_MSG, "Mark block : Total Mark %d\r\n", BadBlockNumInAll);
    llfDbgPrintk(ALWAYS_MSG, "Est. bad   : System %d, Snapshot %d, SLC %d, NonSLC %d\r\n",
                 BadBlockNumInSystemArea,
                 (BadMpBlockNumInSnapshotArea * NandPara.ubPlaneNumPerLun),
                 (BadMpBlockNumInSLCArea * NandPara.ubPlaneNumPerLun),
                 (BadMpBlockNumInNonSLCArea * NandPara.ubPlaneNumPerLun));

    // Total, MpBlock per bank
    for (mpblock_no = guwCaclSSGroupBegin; mpblock_no < uwMpBlockPerBank; mpblock_no++)
    {
        MpDefectPerBS = 0;
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            if (llfIsMpBlockBad(DBT_ADDR, bank_no, mpblock_no))
            {
                MpDefectPerBS++;
                temp[bank_no] = NandPara.ubPlaneNumPerLun;
            }
            else
            {
                temp[bank_no] = 0;
            }
        }

        // check if Block Stripe broken (defect larger than half of blocks)
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            if (MpDefectPerBS > BAD_BS_TH)
            {
                BadBlockPerBankInAll[bank_no] += NandPara.ubPlaneNumPerLun;
            }
            else
            {
                BadBlockPerBankInAll[bank_no] += temp[bank_no];
            }
        }
    }

    // show defect message per bank
    if (gfDBTInitDone != LLF_DBT_RDT)
    {
        llfInitErrorMessage();
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            AddErrorMessage(bank_no, BadBlockPerBankInAll[bank_no] -
                            (uwMpBlockPerBank - NandPara.uwMpBlockNumPerLun) * NandPara.ubPlaneNumPerLun,
                            DEFECT_FACTORY);
        }
    }
    for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no += 4)
    {
        llfDbgPrintk(ALWAYS_MSG,
                     "--->        Bank%d: %d,   Bank%d: %d,   Bank%d: %d,   Bank%d: %d   <---\r\n",
                     bank_no,     BadBlockPerBankInAll[bank_no],
                     bank_no + 1, BadBlockPerBankInAll[bank_no + 1],
                     bank_no + 2, BadBlockPerBankInAll[bank_no + 2],
                     bank_no + 3, BadBlockPerBankInAll[bank_no + 3]);
#ifdef NEW_BLK_REMAP
        gubBadBlockNum[bank_no] = BadBlockPerBankInAll[bank_no];
        gubBadBlockNum[bank_no + 1] = BadBlockPerBankInAll[bank_no + 1];
        gubBadBlockNum[bank_no + 2] = BadBlockPerBankInAll[bank_no + 2];
        gubBadBlockNum[bank_no + 3] = BadBlockPerBankInAll[bank_no + 3];
#endif
    }
#if defined(RL6643_VA)
    if(READ_REG_32(SCAN_DBT_CTR) == FACTORY_DEFECT_TAG)
    {
        printk("XOR_SRAM_SCAN_DBT_CTR %x\r\n", READ_REG_32(SCAN_DBT_CTR));
        printk("SCAN_DBT_INFO!\r\n");
        return ERR_READ_DBT;
    }
#endif
#ifdef SORTING_FW_FOR_WK
    return ERR_FOR_SORTING;
#endif

    if (_MEM32(FW_RDT_TAG) == SIGNATURE_RDT && ubRDTLLFCheckPerDieEn == 1)
    {
        // less NonSLC blocks per bank
        isErr = 0;
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            if ((BadMpBlockPerBankInNonSLCArea[bank_no] * 1000)
                    > ((U32)NandPara.uwMpBlockNumPerLun * (U32)gubDefectRatioPerBank))
            {
                llfprintk("Bad in NonSLC area need less than %d/1000 per bank, now bank%d: %d/%d\r\n",
                          gubDefectRatioPerBank, bank_no,
                          BadMpBlockPerBankInNonSLCArea[bank_no], NandPara.uwMpBlockNumPerLun);
                isErr = 1;
                AddErrorMessage(bank_no, 0, ERR_DEFECT_OVER_LIMIT);
//                ch_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank_no) & 0xFF) >> 4;
//                ce_no = (_MEM08(BANK_IMAPPING_TABLE_ADDR + bank_no) & 0xF);
//                _MEM32(LLF_RES_ERRMSG_START_VA_ADDR + WORD_BYTE_SIZE * pResponseInfo->err_msg_num) =
//                    (ch_no << 16 | ce_no);
//                pResponseInfo->err_msg_num++;
            }
        }
        if (isErr)
        {
            llfDbgPrintk(ALWAYS_MSG, "[DBG]badblk in NonSLC area over Limit,Line %d,%s\r\n", __LINE__,
                         __FILE__);
            return ERR_UNENOUGH_USER_NOSLC;
        }
    }

    // the system block should not exceed 64 blocks
    if (guwCaclSSGroupEnd * NandPara.ubPlaneNumPerLun >= SYSTEM_BLOCK_MAX_NUM)
    {
        llfDbgPrintk(ALWAYS_MSG, "Snap blk exceed limit snapEnd%d\r\n", guwCaclSSGroupEnd);
        return ERR_UNENOUGH_SNAPSHOT;
    }
    guwCacll2PGroupEndshift = (U16)cal_shift((U32)guwCacll2PGroupEnd);
    if(guwCacll2PGroupEnd % (1 << guwCacll2PGroupEndshift))
    {
        guwCacll2PGroupEndshift++;
    }

    if (llfCheckSuperPBABits() != ERR_OK)
    {
        return ERR_UNENOUGH_SNAPSHOT;
    }

    if((guwCaclSSGroupEnd * NandPara.ubPlaneNumPerLun) >= SYSTEM_BLOCK_MAX_NUM)
    {
        llfDbgPrintk(ALWAYS_MSG, "guwCaclSSGroupEnd is %d, over SYSTEM_BLOCK_MAX_NUM!!!\r\n",
                     guwCaclSSGroupEnd);
        return ERR_UNENOUGH_SNAPSHOT;
    }

    llfprintk("TotalMpBlockNumInNonSLCArea = %d\r\n", TotalMpBlockNumInNonSLCArea);
    llfprintk("BadMpBlockNumInNonSLCArea = %d\r\n", BadMpBlockNumInNonSLCArea);
    llfprintk("HostClaimMpBlock = %d\r\n", HostClaimMpBlock);
    U16 uwSysMpBlock = (guwCaclSSGroupEnd * NandPara.ubBankNum);
#ifndef L2P_SHRINK
    uwL2PMpBlock = ((guwCacll2PGroupEnd - guwCaclSSGroupEnd) * NandPara.ubBankNum);
#endif
    llfprintk("sys_bank = %d\r\n", uwSysMpBlock);
    llfprintk("l2p_bank = %d\r\n", uwL2PMpBlock);
    // free space per system check (NOT include defect BS, system blocks and capacity)
    if (TotalMpBlockNumInNonSLCArea < (BadMpBlockNumInNonSLCArea + HostClaimMpBlock +
                                       uwSysMpBlock + uwL2PMpBlock))
    {
        llfDbgPrintk(ALWAYS_MSG, "There is not enough space for HostClaimMpBlock,Line %d,%s!!!\r\n",
                     __LINE__, __FILE__);
        return ERR_UNENOUGH_USER_NOSLC;
    }

    // free space per system should more than
    CapacityOP = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_NORMAL_CAPACITY_OP);
    if(CapacityOP == 0)
    {
        CapacityOP = 100;
    }

    CapacityOP += llfCalculateRaidOP(); // calculate op for raid

    if(llfCheckCapacityOP(TotalMpBlockNumInNonSLCArea, BadMpBlockNumInNonSLCArea, HostClaimMpBlock,
                          NandPara.ubBankNum,  CapacityOP, uwL2PMpBlock))
    {
        llfDbgPrintk(ALWAYS_MSG, "Free space are too few for GC, ratio: %d/1000\r\n", CapacityOP);
        return ERR_LESS_FREE_FOR_GC;
    }

    return ERR_OK;
}

U32 llfAPEraseAllSblk(U8 ubBankNum)
{
    LLF_UNI_INFO *pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PVENDOR_CMD pVendorCmd;
    U8 bank, defectnum;
    U32 ret = ERR_OK;
    U32 ulMode;
    U32 cmp;
    U8 block;
    U8 lun_no;
    U16 uwPage = 0;
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    U8 mstdefectBitMap;
#endif
#ifndef SBLK_EXPAND
    U8 blkstart = 0;
#else
    U8 blkstart = gubSblkStart;
#endif
    U8 ubsysblk = SYSTEM_BLOCK_MAX_NUM;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (struct _LLF_UNI_INFO *)LLF_UNI_INFO_ADDR;
    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);

    // initialize erase arguments
    pResponseInfo->res_progress = 0;
    pLLFUniInfo->ulWorkFunc = ERASE_ALL_FUNC;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    // erase maximum all banks block0
    if (ubBankNum > NandPara.ubBankNum)
    {
        ubBankNum = NandPara.ubBankNum;
    }

    llfDbgPrintk(ALWAYS_MSG, "EraseAllSblk, %d\r\n", blkstart + ubsysblk);
    // loop bank to pipeline erase
    for (bank = 0; bank < ubBankNum; bank++)
    {
        defectnum = 0;
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
        mstdefectBitMap = 0;

#endif
        lun_no = bank / NandPara.ubBankNumPerLun;
        for(block = blkstart; block < blkstart + ubsysblk; block ++)
        {
#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A)
            for(uwPage = 0; uwPage < (INTELQ_PAGE_PER_SECTION * INTELQ_SECTION_PER_BLOCK);
                    uwPage += INTELQ_PAGE_PER_SECTION)
#endif
            {
                gul_FW_TAG = llfBETagSetting(TAG_ERASE, bank);
                FCSingleErase(ulMode, bank, lun_no, block, uwPage, 0);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Err%d\r\n", bank);
                    AddErrorMessage(bank, 0, ERR_ERASE_BLK0);
                }
                //whether need to check
                if((cmp & 0x7fff) != 0)
                {
#ifdef ERASEALL_TWICE
                    if((gubFWFeatureSetting & 0x20) != 0)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "bank%d block%d 1st erase fail cmp %x \r\n", bank, block, cmp);
                        FCSingleErase(ulMode, bank, lun_no, block, uwPage, 0);
                        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                        if(ret != ERR_OK)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "Err%d\r\n", bank);
                            AddErrorMessage(bank, 0, ERR_ERASE_BLK0);
                        }
                        //whether need to check
                        if((cmp & 0x7fff) != 0)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "bank%d block%d 2nd erase fail cmp %x \r\n", bank, block, cmp);
                            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block);
                            ret = llfSetDBT(bank, block, DBT_ADDR);
                            if(ret != 0)
                                return ERR_ERASE_BLK0;
                            if(block < blkstart + SYS_BLK)
                                defectnum++;
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
                            else if(block < blkstart + (SYS_BLK + RDT_RESULT_BLK_EXTEND_NUM))
                            {
                                mstdefectBitMap |= 1 << (block - (blkstart + SYS_BLK));
                            }
#endif

#ifndef SBLK_EXPAND
                            if(defectnum > 3)
                                AddErrorMessage(bank, 0, ERR_ERASE_BLK0);
#endif
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
                            if(mstdefectBitMap == (1 << RDT_RESULT_BLK_EXTEND_NUM) - 1)
                            {
                                //AddErrorMessage(bank, 0, ERR_ERASE_BLK0);
                                llfDbgPrintk(ALWAYS_MSG, "[WARN] bank%d Extend BLK%d~%d all bad\r\n", bank, blkstart + SYS_BLK,
                                             blkstart + SYS_BLK + EXTEND_RDT_BLK);
                            }

#endif
                        }
                    }
                    else
#endif
                    {
                        llfDbgPrintk(ALWAYS_MSG, "bank%d block%d erase fail cmp %x \r\n", bank, block, cmp);
                        llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block);
                        ret = llfSetDBT(bank, block, DBT_ADDR);
                        if(ret != 0)
                            return ERR_ERASE_BLK0;
                        if(block < blkstart + SYS_BLK)
                            defectnum++;
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
                        else if(block < blkstart + (SYS_BLK + RDT_RESULT_BLK_EXTEND_NUM))
                        {
                            mstdefectBitMap |= 1 << (block - (blkstart + SYS_BLK));
                        }
#endif

#ifndef SBLK_EXPAND
                        if(defectnum > 3)
                            AddErrorMessage(bank, 0, ERR_ERASE_BLK0);
#endif
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
                        if(mstdefectBitMap == (1 << RDT_RESULT_BLK_EXTEND_NUM) - 1)
                        {
                            //AddErrorMessage(bank, 0, ERR_ERASE_BLK0);
                            llfDbgPrintk(ALWAYS_MSG, "[WARN] bank%d Extend BLK%d~%d all bad\r\n", bank, blkstart + SYS_BLK,
                                         blkstart + SYS_BLK + EXTEND_RDT_BLK);
                        }
#endif
                    }
                }
            }
        }
    }

    if (pVendorCmd->subcmd != BE_LLF_ALL)  // pVendorCmd->subcmd == BE_LLF_ERASENAND
    {
        llfEndResponceNonPrint(ERR_OK);
        return ERR_OK;
    }

    if(pResponseInfo->err_msg_num != 0)
    {
        ret = ERR_ERASE_BLK0;
    }

    // erase end
    pResponseInfo->res_progress = 80;
    pResponseInfo->res_err_code = ERR_OK;
    pLLFUniInfo->ulWorkFunc = NONE_FUNC;
    PrintDone();

    return ret;
}


#ifdef NEW_BLK_REMAP

void llfRemapFlagInsertBlock(U8 bank, U16 uwMpBlock)
{
    U8 ubPlane;
    U16 uwTableSizePerPlane = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun;
    U16 uwRemapFlagNumPerPlane = uwTableSizePerPlane / 2 ; // 2 means 2 Byte for each block
    U16 uwSpBlock = uwMpBlock * NandPara.ubPlaneNumPerLun;

    if(guwRemapFlag[bank][0] < uwRemapFlagNumPerPlane)
    {
        for(ubPlane = 0; ubPlane < NandPara.ubPlaneNumPerLun; ubPlane++)
        {
            _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * bank + uwTableSizePerPlane * ubPlane +
                   guwRemapFlag[bank][ubPlane] * 2) = uwSpBlock + ubPlane;
            //llfprintk("----Push bank%d plane:%d flag:%d block:%d \r\n", bank, plane_idx, guwRemapFlag[bank][plane_idx],
            //		  guwRemapLastBlock[bank][plane_idx] * NandPara.ubPlaneNumPerLun + plane_idx);
            guwRemapFlag[bank][ubPlane]++ ;
        }
        llfSetDBT(bank, uwSpBlock, DBT_ADDR);  //build DBT
    }
    else
    {
        // full
    }
}

void llfRemapFindBrokenBS()
{
#if 1
    U32 plane_idx;
    U16 local_min;
    U32 curBS_bad_bitmap;
    U16 TableSizePerPlane = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun;
    U16 uwRemapFlagNumPerPlane = TableSizePerPlane / 2 ; // 2 means 2 Byte for each block
    U16 Block_selected[64];
    U8 select_count = 0;
    U8 bank;
    U8 count;
    U32 block;
#ifdef L2P_SHRINK
    U8 ubInsertBankCount;
    U16 ubSortedBankIdx[BANK_NUM_MAX];
    U16 uwRmpFlg[BANK_NUM_MAX];
    U16 i, j, k;
#endif

    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        local_min = 0xffff;
        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            if(local_min > guwRemapFlag[bank][plane_idx])
            {
                local_min = guwRemapFlag[bank][plane_idx];
            }
        }
        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            guwRemapFlag[bank][plane_idx] = local_min;
        }
        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            for(count = guwRemapFlag[bank][plane_idx]; count < uwRemapFlagNumPerPlane; count++)
            {
                _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * bank + TableSizePerPlane * plane_idx +
                       count * 2) = 0;
            }
        }
    }

    for(block = NandPara.uwMpBlockNumPerLun - 1; block >= guwCaclSSGroupEnd; block--)
    {
        count = 0;
        curBS_bad_bitmap = 0;
        for(bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if(llfIsMpBlockBad(DBT_ADDR, bank, block))
            {
                count++;
                curBS_bad_bitmap |= (1 << bank);
            }
        }

#ifdef L2P_SHRINK
        if(block < guwCacll2PGroupEnd)
        {
            // Step 1. Sorting by remap flag
            for(bank = 0; bank < NandPara.ubBankNum; bank++)
            {
                ubSortedBankIdx[bank] = bank;
                uwRmpFlg[bank] = guwRemapFlag[bank][0];
            }
            for(i = 0; i < NandPara.ubBankNum - 1; i++)
            {
                for(j = i + 1; j < NandPara.ubBankNum; j++)
                {
                    if(uwRmpFlg[i] > uwRmpFlg[j])
                    {
                        k = uwRmpFlg[j];
                        uwRmpFlg[j] = uwRmpFlg[i];
                        uwRmpFlg[i] = k;

                        k = ubSortedBankIdx[j];
                        ubSortedBankIdx[j] = ubSortedBankIdx[i];
                        ubSortedBankIdx[i] = k;
                    }
                }
            }
            // Step. 2. Pick Good L2P Mp block
            ubInsertBankCount = 0;
            for(i = 0; i < NandPara.ubBankNum; i++)
            {
                if(ubInsertBankCount >= ((NandPara.ubBankNum - count) >> 1))
                {
                    break;
                }
                bank = ubSortedBankIdx[i];
                if((curBS_bad_bitmap & (1 << bank)) == 0)
                {
                    llfRemapFlagInsertBlock(bank, block);
                    ubInsertBankCount++;
                }
            }
            continue;
        }
#endif

        if((count > 0) && (count >= BAD_BS_TH))
        {
            for(bank = 0; bank < NandPara.ubBankNum; bank++)
            {
                if((curBS_bad_bitmap & (1 << bank)) == 0)
                {
                    llfRemapFlagInsertBlock(bank, block);
                }
            }
        }
        else if(count)
        {
            if(select_count < 64)
            {
                Block_selected[select_count] = block;
                select_count++;
            }
        }
    }
    printk("select_count %d\r\n", select_count);
    for(count = 0; count < select_count; count++)
    {
        for(bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if((!llfIsMpBlockBad(DBT_ADDR, bank, Block_selected[count]))
                    && (guwRemapFlag[bank][0] >= uwRemapFlagNumPerPlane))
            {
                break;// means bank != NandPara.ubBankNum  <==> this MP do not remap
            }
        }
        if(bank == NandPara.ubBankNum)
        {
            for(bank = 0; bank < NandPara.ubBankNum; bank++)
            {
                if(!llfIsMpBlockBad(DBT_ADDR, bank, Block_selected[count]))
                {
                    llfRemapFlagInsertBlock(bank, Block_selected[count]);
                }
            }
        }
    }
#else

    U32 plane_idx;
    U16 TableSizePerPlane, Local_min;

    TableSizePerPlane = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun;
    printk("TableSizePerPlane %d\r\n", TableSizePerPlane);

    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            guwRemapLastBlock[bank][plane_idx] = NandPara.uwMpBlockNumPerLun - 1;
            //printk("bank %d plane %d Now location %d\r\n" , bank, plane_idx, guwRemapFlag[bank][plane_idx]);
        }
    }
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            while((guwRemapFlag[bank][plane_idx] < TableSizePerPlane / 2)
                    && (guwRemapLastBlock[bank][plane_idx] >= guwCaclSSGroupEnd))
            {
                while(llfIsMpBlockBad(DBT_ADDR, bank, guwRemapLastBlock[bank][plane_idx]))
                {
                    guwRemapLastBlock[bank][plane_idx]--;
                    if(guwRemapLastBlock[bank][plane_idx] < guwCaclSSGroupEnd)
                    {
                        break;
                    }
                }
                if(guwRemapLastBlock[bank][plane_idx] >= guwCaclSSGroupEnd)
                {
                    _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * bank + TableSizePerPlane * plane_idx +
                           guwRemapFlag[bank][plane_idx] * 2) = guwRemapLastBlock[bank][plane_idx] * NandPara.ubPlaneNumPerLun
                                   + plane_idx;
                    //llfprintk("----Push bank%d plane:%d flag:%d block:%d \r\n", bank, plane_idx, guwRemapFlag[bank][plane_idx],
                    //		  guwRemapLastBlock[bank][plane_idx] * NandPara.ubPlaneNumPerLun + plane_idx);
                    guwRemapFlag[bank][plane_idx] ++ ;
                    guwRemapLastBlock[bank][plane_idx]--;
                }
            }

        }
        Local_min = 0xffff;

        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            if(Local_min > guwRemapLastBlock[bank][plane_idx])
            {
                Local_min = guwRemapLastBlock[bank][plane_idx];
            }
        }

        for(block = Local_min + 1; block < NandPara.uwMpBlockNumPerLun; block++)
        {
            //printk("Mark bank %d BS %d\r\n",bank, block);
            llfSetDBT(bank, block * NandPara.ubPlaneNumPerLun, DBT_ADDR);  //build DBT
        }
    }
#endif

}


void llfRemaptableReDefine()
{
    U8 curBS_bad_count;
    //U16 curBS_bad_bitmap;
    U8 local_Max_remap_index;
    U8 last_bad_bank;
    U8 total_remap_num = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun / 2;
    U8 bank;
    U8 count;
    U32 block;
#ifdef L2P_SHRINK
    U8 ubBadBankCount;
#endif

    if(total_remap_num > 0xfe)
    {
        printk("[ERR] Remap table %d bigger than 0xfe\r\n", total_remap_num);
        while(1);
    }

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    if ((BLK_REMAP_TEMP_ADDR + total_remap_num * 8) > (BLK_REMAP_TABLE_ADDR + BLK_REMAP_TABLE_SIZE))
#else
    if ((BLK_REMAP_TEMP_ADDR + total_remap_num * 2) > (BLK_REMAP_TABLE_ADDR + BLK_REMAP_TABLE_SIZE))
#endif
    {
        printk("[ERR]Remap memory overlap BS %d!!!", NandPara.uwMpBlockNumPerLun);
        while(1);
    }

    for(block = 0; block < NandPara.uwMpBlockNumPerLun; block++)
    {
        _MEM08(BLK_REMAP_BS_INDEX_TABLE_ADDR + block) = 0xff;
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
        _MEM64(BLK_REMAP_BITMAP_TABLE_ADDR + block * 8) = 0;
#else
        _MEM16(BLK_REMAP_BITMAP_TABLE_ADDR + block * 2) = 0;
#endif
    }
    for(count = 0; count < total_remap_num; count++)
    {
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
        _MEM64(BLK_REMAP_TEMP_ADDR + 8 * count) = 0;
#else
        _MEM16(BLK_REMAP_TEMP_ADDR + 2 * count) = 0;
#endif
    }
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        gubCurRemapIndex[bank] = 0;
    }
    for(block = guwCaclSSGroupEnd; block < NandPara.uwMpBlockNumPerLun; block++)
    {
        curBS_bad_count = 0;
        //curBS_bad_bitmap = 0;
        local_Max_remap_index = 0;
        last_bad_bank = 0;
#ifdef L2P_SHRINK
        if(block >= guwCaclSSGroupEnd && block < guwCacll2PGroupEnd)
        {
            ubBadBankCount = 0;
            for(bank = 0; bank < NandPara.ubBankNum; bank++)
            {
                if(llfIsBlockBad(DBT_ADDR, bank, block * NandPara.ubPlaneNumPerLun))
                    ubBadBankCount++;
            }
            if(ubBadBankCount <= BAD_BS_TH)
                continue;
        }
#endif
        for(bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if(llfIsBlockBad(DBT_ADDR, bank, block * NandPara.ubPlaneNumPerLun))
            {
                //printk("-bank %d block %d pre location %d\r\n", bank, block, gubCurRemapIndex[bank]);
                while(gubCurRemapIndex[bank] < guwMpDefectBlock[bank] &&
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
                        (_MEM64(BLK_REMAP_TEMP_ADDR + gubCurRemapIndex[bank] * 8) & ((U64)1 << bank)) != 0
#else
                        (_MEM16(BLK_REMAP_TEMP_ADDR + gubCurRemapIndex[bank] * 2) & (1 << bank)) != 0
#endif
                     )
                {
                    gubCurRemapIndex[bank]++;
                }

                //printk("+bank %d block %d new location %d\r\n", bank, block, gubCurRemapIndex[bank]);
                if(gubCurRemapIndex[bank] < guwMpDefectBlock[bank])
                {
                    if(local_Max_remap_index < gubCurRemapIndex[bank])
                    {
                        local_Max_remap_index = gubCurRemapIndex[bank];
                    }
                    last_bad_bank = bank;
                    curBS_bad_count++;
                    //curBS_bad_bitmap |= (1 << bank);
                }
            }
        }

        if(curBS_bad_count > 1)
        {
            for(bank = 0; bank < NandPara.ubBankNum; bank++)
            {
                if((llfIsBlockBad(DBT_ADDR, bank, block * NandPara.ubPlaneNumPerLun)) &&
                        (local_Max_remap_index < guwMpDefectBlock[bank]) &&
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
                        ((_MEM64(BLK_REMAP_TEMP_ADDR + local_Max_remap_index * 8) & ((U64)1 << bank)) == 0)
#else
                        ((_MEM16(BLK_REMAP_TEMP_ADDR + local_Max_remap_index * 2) & (1 << bank)) == 0)
#endif
                  )
                {
                    //printk("A Bank %d BS %d remap index %d %d\r\n",bank, block, local_Max_remap_index, gubCurRemapIndex[bank]);

                    if(local_Max_remap_index == gubCurRemapIndex[bank])
                    {
                        gubCurRemapIndex[bank]++;
                    }

#ifdef COPYBACK_WITH_MTL
                    if(gubIsSerialMultiLUN
                            && ((_MEM16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * bank + local_Max_remap_index * 2)) ==
                                0))
                    {
                        continue;
                    }
#endif

                    _MEM08(BLK_REMAP_BS_INDEX_TABLE_ADDR + block) = local_Max_remap_index;
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
                    _MEM64(BLK_REMAP_BITMAP_TABLE_ADDR + block * 8) |= ((U64)1 << bank);
                    _MEM64(BLK_REMAP_TEMP_ADDR + local_Max_remap_index * 8) |= ((U64)1 << bank);
#else
                    _MEM16(BLK_REMAP_BITMAP_TABLE_ADDR + block * 2) |= (1 << bank);
                    _MEM16(BLK_REMAP_TEMP_ADDR + local_Max_remap_index * 2) |= (1 << bank);
#endif
                    llfUnSetDBT(bank, block, DBT_ADDR);
#ifdef L2P_SHRINK
                    if(block >= guwCaclSSGroupEnd && block < guwCacll2PGroupEnd)
                    {
                        ubBadBankCount = 0;
                        U8 tmpBank;
                        for(tmpBank = 0; tmpBank < NandPara.ubBankNum; tmpBank++)
                        {
                            if(llfIsBlockBad(DBT_ADDR, tmpBank, block * NandPara.ubPlaneNumPerLun))
                                ubBadBankCount++;
                        }
                        if(ubBadBankCount <= BAD_BS_TH)
                            break;
                    }
#endif
                }
            }
        }
        else if(curBS_bad_count == 1)
        {
            _MEM08(BLK_REMAP_BS_INDEX_TABLE_ADDR + block) = gubCurRemapIndex[last_bad_bank];
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
            _MEM64(BLK_REMAP_BITMAP_TABLE_ADDR + block * 8) |= ((U64)1 << last_bad_bank);
            _MEM64(BLK_REMAP_TEMP_ADDR + gubCurRemapIndex[last_bad_bank] * 8) |= ((U64)1 << last_bad_bank);
#else
            _MEM16(BLK_REMAP_BITMAP_TABLE_ADDR + block * 2) |= (1 << last_bad_bank);
            _MEM16(BLK_REMAP_TEMP_ADDR + gubCurRemapIndex[last_bad_bank] * 2) |= (1 << last_bad_bank);
#endif
            //printk("B Bank %d BS %d remap index %d %d\r\n",last_bad_bank, block, local_Max_remap_index, gubCurRemapIndex[last_bad_bank]);

            llfUnSetDBT(last_bad_bank, block, DBT_ADDR);
        }

    }
    guwRealMPBlkNum = NandPara.uwMpBlockNumPerLun;

}
#endif
#ifdef BLK_REMAP_PRO
#ifdef COPYBACK_WITH_MTL
void llfSortingRemapBSForMultiLUNWithCopyBack(U8 ubNextLun)
{
    U32 plane_idx;
    U32 i, j;
    U16 TableSizePerPlane = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun;
    U16 MaxEntryPerPlane = TableSizePerPlane / 2;
    U8 ubbank;
    U16 lun0maxuse = 0;
    U16 lun0NumInEachBank[FW_BANK_NUM_MAX] = {0};
    U16 zeronum[FW_BANK_NUM_MAX] = {0};

    //printk("Before sorting\r\n");
    //LlfOutPutRemapTable();
    for(ubbank = 0; ubbank < NandPara.ubBankNum; ubbank++)
    {
        if(guwMpDefectBlock[ubbank] > MaxEntryPerPlane)
        {
            printk("[ERR] Bank %d remap table oversize!!!\r\n", ubbank);
            return;
        }

        i = 0;
        if(guwMpDefectBlock[ubbank] == 0) // all 0 in remap table
        {
            lun0NumInEachBank[ubbank] = 0;
            continue;
        }

        while((_REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * ubbank + i * 2)) < guwRealBlockNum *
                ubNextLun)
        {
            i++;
            if(i == guwMpDefectBlock[ubbank])
            {
                break;
            }
        }

        lun0NumInEachBank[ubbank] = i;
        lun0maxuse = ((lun0maxuse) > (i) ? (lun0maxuse):(i));
    }

    for(ubbank = 0; ubbank < NandPara.ubBankNum; ubbank++)
    {
        if(guwMpDefectBlock[ubbank] == 0)
        {
            continue;
        }
        zeronum[ubbank] = lun0maxuse - lun0NumInEachBank[ubbank]; // decide how many zero mpb in this bank
    }

    for(ubbank = 0; ubbank < NandPara.ubBankNum; ubbank++)
    {
        if(zeronum[ubbank] == 0) //Need not to move
        {
            continue;
        }
        if(lun0maxuse > MaxEntryPerPlane - 1) // lun 1 are all abandon
        {
            guwMpDefectBlock[ubbank] = lun0NumInEachBank[ubbank];
        }
        else
        {
            guwMpDefectBlock[ubbank] = MIN(guwMpDefectBlock[ubbank] + zeronum[ubbank], MaxEntryPerPlane);
        }

        for(plane_idx = 0; plane_idx < NandPara.ubPlaneNumPerLun; plane_idx++)
        {
            for(i = (guwMpDefectBlock[ubbank] - 1); i > (lun0maxuse - 1); i--)
            {
                j = i - zeronum[ubbank];
                _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * ubbank + TableSizePerPlane * plane_idx +
                       i * 2) =
                           _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * ubbank + TableSizePerPlane * plane_idx +
                                  j * 2);
            }
            i = lun0maxuse - 1;
            for(j = 0; j < zeronum[ubbank]; j++)
            {
                _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * ubbank + TableSizePerPlane * plane_idx +
                       i * 2) = 0;
                i--;
            }
        }
    }
    //printk("After sorting\r\n");
    //LlfOutPutRemapTable();
}
#endif

void llfSortingRemapBSForMultiLUN()
{
    U32 ulRmpAddrHead, ulRmpAddrTmp;
    U32 i, j;
    U16 uwRmpTblSizePerPln = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun;
    U16 uwSpBlkTmp;
    U8 ubBank, ubPln, ubLun;
    U8 ubIdxCur, ubSpBlkCntMinPerPln, ubMvUpCnt;



    //LlfOutPutRemapTable();

    for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
    {
        for(ubPln = 0; ubPln < NandPara.ubPlaneNumPerLun; ubPln++)
        {
            for (i = 0; i < LUN_NUM_MAX; i++)
            {
                gubSpBlkCntPerBkPlnLun[ubBank][ubPln][i] = 0;
            }

            // 1st sorting, small-->big
            for(i = 0; i < guwRemapFlag[ubBank][ubPln]; i++)
            {
                ulRmpAddrHead = BLK_REMAP_TABLE_ADDR
                                + guwReMapTableSizePerBank * ubBank
                                + uwRmpTblSizePerPln * ubPln
                                + i * 2;

                for(j = i + 1; j < guwRemapFlag[ubBank][ubPln]; j++)
                {
                    ulRmpAddrTmp = BLK_REMAP_TABLE_ADDR
                                   + guwReMapTableSizePerBank * ubBank
                                   + uwRmpTblSizePerPln * ubPln
                                   + j * 2;

                    if(_REG16(ulRmpAddrTmp) < _REG16(ulRmpAddrHead))
                    {
                        uwSpBlkTmp = _REG16(ulRmpAddrTmp);
                        _REG16(ulRmpAddrTmp) = _REG16(ulRmpAddrHead);
                        _REG16(ulRmpAddrHead) = uwSpBlkTmp;
                    }
                }

                if(gubIsSerialMultiLUN && (NandPara.ubLunNumPerCE == 1)) // extend
                {
                    ubLun = _REG16(ulRmpAddrHead) / guwRealBlockNum;

                    if (ubLun < LUN_NUM_MAX)
                    {
                        gubSpBlkCntPerBkPlnLun[ubBank][ubPln][ubLun]++;
                    }
                    else
                    {
                        printk("[ERR] REMAP sort Lun:%d\r\n", ubLun);
                    }
                }
            }
        }

        if(gubIsSerialMultiLUN && (NandPara.ubLunNumPerCE == 1)) // extend
        {
            ubIdxCur = 0;

            for(ubLun = 0; ubLun < LUN_NUM_MAX; ubLun++)
            {
                ubSpBlkCntMinPerPln = 0xff;
                for(ubPln = 0; ubPln < NandPara.ubPlaneNumPerLun; ubPln++)
                {
                    if(gubSpBlkCntPerBkPlnLun[ubBank][ubPln][ubLun] < ubSpBlkCntMinPerPln)
                    {
                        ubSpBlkCntMinPerPln = gubSpBlkCntPerBkPlnLun[ubBank][ubPln][ubLun];
                    }
                }

                ubIdxCur += ubSpBlkCntMinPerPln;
                for(ubPln = 0; ubPln < NandPara.ubPlaneNumPerLun; ubPln++)
                {
                    //Align the block count per LUN of each plane, and move up other blocks in remapping table
                    if(gubSpBlkCntPerBkPlnLun[ubBank][ubPln][ubLun] > ubSpBlkCntMinPerPln)
                    {
                        ubMvUpCnt = gubSpBlkCntPerBkPlnLun[ubBank][ubPln][ubLun] - ubSpBlkCntMinPerPln;
                        for(i = ubIdxCur; i < guwRemapFlag[ubBank][ubPln] - ubMvUpCnt; i++)
                        {
                            _REG16(BLK_REMAP_TABLE_ADDR
                                   + guwReMapTableSizePerBank * ubBank
                                   + uwRmpTblSizePerPln * ubPln
                                   + i * 2) =
                                       _REG16(BLK_REMAP_TABLE_ADDR
                                              + guwReMapTableSizePerBank * ubBank
                                              + uwRmpTblSizePerPln * ubPln
                                              + (i + ubMvUpCnt) * 2);
                        }

                        guwRemapFlag[ubBank][ubPln] -= ubMvUpCnt;
                    }
                }
            }

            for(ubPln = 0; ubPln < NandPara.ubPlaneNumPerLun; ubPln++)
            {
                for(i = guwRemapFlag[ubBank][ubPln]; i < (uwRmpTblSizePerPln / 2); i++)
                {
                    _REG16(BLK_REMAP_TABLE_ADDR
                           + guwReMapTableSizePerBank * ubBank
                           + uwRmpTblSizePerPln * ubPln
                           + i * 2) = 0;
                }
            }
        }

    }

    //printk("After sorting\r\n");

    //LlfOutPutRemapTable();
}
#endif

#ifdef MST_MERGE
U32 llfCheckMSTMergeBlk()
{
    U8 ubBank, ubGoodBankTh, ubSblkStart = 0, ubGoodBankCnt = 0;
    U16 uwBlk, uwCnt;
#if 0
    U8 ubBankNumChk;

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > 8)
        ubBankNum = 8;
    else
        ubBankNum = ubBankNumChk;
#endif

#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif

#ifdef NEW_RDT_RESULT
    ubGoodBankTh = 1;
#else
    ubGoodBankTh = NandPara.ubBankNum;
#endif

    for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
    {
        uwCnt = 0;
        for(uwBlk = ubSblkStart + SYS_BLK; uwBlk < ubSblkStart + SYS_BLK + RDT_RESULT_BLK_EXTEND_NUM;
                uwBlk++)
        {
            if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, ubBank, uwBlk))
            {
                uwCnt++;
            }
        }

        if(uwCnt != RDT_RESULT_BLK_EXTEND_NUM)
        {
            ubGoodBankCnt++;
        }
    }

    if(ubGoodBankCnt < ubGoodBankTh)
    {
        llfDbgPrintk(ALWAYS_MSG, "no enough mst blk!\r\n");
        return ERR_BAD_SBLOCK;
    }
    return ERR_OK;
}
#endif

U32 llfWriteConfig(U8 ubIFType, U8 ubClkMode)
{
    U32 ret = ERR_OK;
    U32 ret_sblk_get = ERR_OK;
    U32 time_tmp = 0;
#ifdef MST_MERGE
    U16 SBlkNo[CH_NUM_MAX * CE_NUM_MAX][SYS_BLK];
#endif
    U8 bank, i;
#ifndef SBLK_EXPAND
    U8 count;
    U8 err_num = 0;
#endif
    U32 block;
    U8 banknum, ubBankNumChk;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;
    if(gubLLFSubStep.ubWriteConfigStep == STEP_WRITE_CONFIG_INIT)
    {
        for( bank = 0; bank < banknum; bank ++ )
            for( block = 0; block < SYS_BLK; block ++ )
                guwSBlkNo[bank][block] = 0xff;
    }
#ifdef MST_MERGE
    if(pResponseInfo->res_state != VENDOR_CMD_WRITE_FW_CT2)
#endif
    {
        //if (_MEM32(FW_RDT_TAG) != SIGNATURE_RDT || ubRDTLLFNormalEn == 1)
        {
            if(gubLLFSubStep.ubWriteConfigStep == STEP_WRITE_CONFIG_INIT)
            {
#ifdef SBLK_EXPAND
                //get sblock start
                if(pResponseInfo->res_state == VENDOR_CMD_EXECUTE)
                {
                    if(gubLLFMode != LLF_FORCE_INHERIT && gubLLFMode != LLF_DEFECT_BAD_BLOCK)//LLF_INHERIT_OR_FORMAT?
                    {
                        /***Get S Block and System Group***/
                        ret_sblk_get = llfGetSBlk(guwSBlkNo);
                        if(ret_sblk_get != ERR_OK)
                        {
                            return ret_sblk_get;
                        }
                    }
                }
#else
                if (_MEM32(FW_RDT_TAG) == SIGNATURE_RDT)
                {
                    llfInitErrorMessage();
                    for(bank = 0; bank < banknum; bank++)
                    {
                        count = 0;
                        for(block = 0; block < SYS_BLK; block++)
                        {
                            if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block))
                            {
                                count++;
                            }
                        }
                        if(count == SYS_BLK)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "bank_no = %d ,block0~3 are all bad!\r\n", bank);
                            AddErrorMessage(bank, 0, ERR_BAD_SBLOCK);
                            err_num++;
                            continue;
                        }
                    }
                    if(err_num > 0)
                    {
                        return ERR_BAD_SBLOCK;
                    }
                }
#endif
#ifdef MST_MERGE
                if((gubLlfMSTMergeEnable != NO_MST_MERGE) && (_MEM32(FW_RDT_TAG) == SIGNATURE_RDT))
                {
                    ret = llfCheckMSTMergeBlk();
                    if(ret != ERR_OK)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "block4~7 are all bad!\r\n");
                        return ERR_BAD_SBLOCK;
                    }
                }
#endif
                CalculateSnapshotArea();
#ifdef BLK_REMAP_PRO
#ifdef EXTEND_STATIC_DBT
                llfDbgPrintk(ALWAYS_MSG, "StaticDBT LLFMode = %d, flag = %d, TAG %x\r\n",
                             gubLLFMode, gubNeedRebuildRemap, _MEM32(FW_RDT_TAG));
                if ((gubLLFMode == LLF_INHERIT_OR_FORMAT || gubLLFMode == LLF_FORCE_INHERIT
                        || gubLLFMode == LLF_DEFECT_BAD_BLOCK) && (gubNeedRebuildRemap == 0))
#else

                if (gubLLFMode == LLF_INHERIT_OR_FORMAT || gubLLFMode == LLF_FORCE_INHERIT
                        || gubLLFMode == LLF_DEFECT_BAD_BLOCK)
#endif
                {
                    ;
                }
#ifdef RDT_REMAP
                else if((_MEM32(FW_RDT_TAG) == SIGNATURE_RDT)
                        && (_MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_LLF_NORMAL_EN) != 1))
                {
                    ;
                }
#endif
                else
                {
                    LlfGetGoodSystemBlock();
#ifndef NEW_BLK_REMAP
                    LlfGetGoodMPBlock();
#endif
                    LlfGetRealMaxBsNum();

                    LlfUpdateRemapBlock();
                    llfSortingRemapBSForMultiLUN();

#ifdef NEW_BLK_REMAP
                    LlfGetRealMaxBsNum();
                    llfRemapFindBrokenBS();
#endif
                    LlfGetRealMaxBsNum();
#ifndef NEW_BLK_REMAP

                    if(gubLLFALLStep == STEP_FORMAT_START)
                    {
                        for(bank = 0; bank < NandPara.ubBankNum; bank++)
                        {
                            printk("MP:%d,remap:%d,ClrAllBadBlk:%d\r\n", gubBadBlockNum[bank], guwMpDefectBlock[bank],
                                   (gubBadBlockNum[bank]  - guwMpDefectBlock[bank]) * NandPara.ubPlaneNumPerLun);
                            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + bank * SHORT_BYTE_SIZE) =
                                (gubBadBlockNum[bank]  - guwMpDefectBlock[bank]) * NandPara.ubPlaneNumPerLun; //FFailNum
                            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + bank * SHORT_BYTE_SIZE) = _MEM16(
                                        SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + bank * SHORT_BYTE_SIZE);  //New BB Count= 0
                        }
                    }
                    LlfUpdateDBT();
#endif

#ifdef NEW_BLK_REMAP
                    if (gubIsSerialMultiLUN) // extend
                    {
                        llfSortingRemapBSForMultiLUN();
#ifdef COPYBACK_WITH_MTL
                        U8 ubExtendLunNum = NandPara.uwBlockNumPerLun / guwRealBlockNum;
                        for(i = 1; i < ubExtendLunNum; i++)
                        {
                            llfSortingRemapBSForMultiLUNWithCopyBack(i);
                        }
#endif
                    }
                    llfRemaptableReDefine();
#endif
                    ret = LlfOutPutRemapTable();
                    if (ret != ERR_OK)
                    {
                        return ERR_BLOCK_MAPPING;
                    }
                }
#endif
                // check if defect blocks are out of threshold
                ret = llfCheckDefectThreshold();
                if (ret != ERR_OK)
                {
                    for (i = 0; i < 8; i++)
                    {
                        if (gulAutoCapacity[i] == 0)
                        {
                            break;
                        }
#ifdef EXTEND_LBA
                        else if (gulAutoCapacity[i] <= 4096)
#else
                        else if (gulAutoCapacity[i] <= 2048)
#endif
                        {
                            guwHostClaimCapacity = gulAutoCapacity[i];
                        }
                        else
                        {
                            guwHostClaimCapacity = gulAutoCapacity[i] >> (1 + 10 + 10); // LBA(Sector) to GB
                            printk("Calculate capacity as LBA, gulAutoCapacity[%d] %x\r\n", i, gulAutoCapacity[i]);
                        }
                        gulMaxLBAAddr = 97696368 + guwHostClaimCapacity * 1953504 - 50 * 1953504 - 1;
                        _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET) = gulMaxLBAAddr + 1;
                        CalculateSnapshotArea();
                        llfDbgPrintk(ALWAYS_MSG, "[AutoCap] Adjust to %d GB\r\n", guwHostClaimCapacity);
                        ret = llfCheckDefectThreshold();

                        if (ret == ERR_OK)
                        {
                            break;
                        }
                    }
                    if (ret != ERR_OK)
                    {
                        return ret;
                    }
                }

#ifdef NEW_BLK_REMAP
                if(gubLLFALLStep == STEP_FORMAT_START)
                {
                    for(bank = 0; bank < NandPara.ubBankNum; bank++)
                    {
                        _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + bank * SHORT_BYTE_SIZE) =
                            gubBadBlockNum[bank]; //FFailNum
                        _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + bank * SHORT_BYTE_SIZE) = _MEM16(
                                    SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + bank * SHORT_BYTE_SIZE);	//New BB Count= 0
                    }
                }
#endif
#ifdef SORTING_FW_FOR_WK
                if(ret != ERR_FOR_SORTING)
#else
                if (ret != ERR_OK)
#endif
                {
                    return ret;
                }
                gubLLFSubStep.ubWriteConfigStep = STEP_WRITE_CONFIG_ERASE;
                pResponseInfo->res_state = VENDOR_CMD_WRITE_FW;
                return ERR_OK;
            }
            if(gfDBTInitDone == LLF_DBT_RDT || gfDBTInitDone == LLF_DBT_SYSTEM)
            {
                if(gubLLFSubStep.ubLlfStep == STEP_LLF_ERASE)
                {
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                    gubLLFSubStep.ubLlfStep = STEP_LLF_EXECUTE;
                }
                else
                {
                    pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
                }
                time_tmp = gulTicks;
                while((pResponseInfo->res_state == VENDOR_CMD_ERASE)
                        || (pResponseInfo->res_state == VENDOR_CMD_EXECUTE))
                {
                    llfAPEraseAll(ERASE_ALL_WITH_DBT);
                    llfBECheckStatus();
#if defined(RL6643_VA)
                    gulTicks = (READ_REG_32(FAST_REG_CNT0) ? READ_REG_32(FAST_REG_CNT0) : READ_REG_32(FAST_REG_CNT0));
#endif
                    if(gulTicks - time_tmp > 1000)
                    {
                        time_tmp = gulTicks;
                        pResponseInfo->res_state = VENDOR_CMD_WRITE_FW;
                        return ERR_OK;
                    }
                }
                for(bank = 0; bank < NandPara.ubBankNum; bank++)
                {
                    llfChkDefectBlk(bank, 0);
                }
            }
        }

#if defined(RL6643_VA)
        if (gubLLFMode == LLF_DEFECT_BAD_BLOCK)
        {
            printk("K2 should check Defect Threshold again after Erase all\r\n");
            ret = llfCheckDefectThresholdAgain();
            if(ret != ERR_OK)
            {
                return ret;
            }
        }
#endif

        /***Get S Block and System MpBlock***/
#ifdef SBLK_EXPAND
        if(gubLLFMode == LLF_FORCE_INHERIT || gubLLFMode == LLF_DEFECT_BAD_BLOCK)//LLF_INHERIT_OR_FORMAT?
#endif
        {
            llfInitErrorMessage();
            ret_sblk_get = llfGetSBlk(guwSBlkNo);
            if(ret_sblk_get == ERR_READ_SBLK)
            {
                ret_sblk_get = llfGetSBlkFailHandle();
            }
            if(ret_sblk_get != ERR_OK)
            {
                return ret_sblk_get;
            }
        }

#ifdef SORTING_FW_FOR_WK
        return ERR_FOR_SORTING;
#endif

        ret = SandiskResetFlash();
        if(ret != ERR_OK)
        {
            return ret;
        }

        // find 2 groups(Multi-Block) in bank0, bank1 to place DBT and 3 groups in each bank to place DP
        if(!gfSelfTestFlag)
        {
            // set and write S block
            llfSetSBlockHWConfig(SBLK_ADDR, ubIFType, ubClkMode);
            llfSetSBlockFWConfig(SBLK_ADDR);
            llfSetSBlockFEConfig(SBLK_ADDR);
        }

        llfInitErrorMessage();
        ret = llfReadOffsetSetting(SBLK_ADDR);
        if(ret != ERR_OK)
        {
            return ret;
        }
        ret = llfWriteSBlk(guwSBlkNo);
        if( ret == ERR_READ_SBLK )
        {
            ret = llfWriteSBlkFailHandle(guwSBlkNo);
        }
        if(ret != ERR_OK)
        {
            return ret;
        }

        // set SBLK block to defect block
        ret = llfSetSblkToDBT(DBT_ADDR, guwSBlkNo);
        if(ret != ERR_OK)
        {
            return ret;
        }

#if (defined(RL6577_VA)||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
        ret = llfSetExtendRdtBlkToDBT(DBT_ADDR);
        if(ret != ERR_OK)
        {
            return ret;
        }
#endif

        //write Code Table and DBT
        ret = llfWriteSysGroup(guwSBlkNo);
        if(ret != ERR_OK)
        {
            return ret;
        }
        else
        {
#ifdef MST_MERGE
            if((gubLlfMSTMergeEnable != NO_MST_MERGE) && (_MEM32(FW_RDT_TAG) == SIGNATURE_RDT))
            {
                pResponseInfo->res_state = VENDOR_CMD_WRITE_FW_CT2;
                pResponseInfo->res_progress = 90;   //information for MP tools
                //clear memory
                printk("res_state is VENDOR_CMD_WRITE_FW_CT2, Loading whole system IMAGE...\r\n");
                printk("before FW_RDT_TAG %x\r\n", _MEM32(FW_RDT_TAG));
                _MEM32(BE_VERSION_TAG) = 0xffffffff;
                printk("after FW_RDT_TAG %x\r\n", _MEM32(FW_RDT_TAG));
                gubLLFALLStep = STEP_FORMAT_END;
                gubLLFSubStep.ubMstWriteConfigStep = STEP_WRITE_CONFIG_MST;
            }
#endif
        }

#ifdef SBLK_EXPAND
        //check sblock statu
        ret = llfCheckSblock(guwSBlkNo);
        if(ret != ERR_OK)
        {
            return ret;
        }
#endif
        gubLLFSubStep.ubWriteConfigStep = STEP_WRITE_CONFIG_OVER;
        return ERR_OK;
    }
#ifdef MST_MERGE
    if (pResponseInfo->res_state == VENDOR_CMD_WRITE_FW_CT2)
    {
        //read TAG
//        printk("BE_VERSION_TAG %x progress %d addr %x\r\n", _MEM32(BE_VERSION_TAG), pResponseInfo->res_progress, (U32)(&pResponseInfo->res_progress));
        if ((_MEM32(BE_VERSION_TAG) != 0xffffffff) && ( pResponseInfo->res_progress == 99))
        {
            ret = llfCheckFlashSignature();
            if( ret == ERR_OK )
            {
                /***Get S Block and System Group***/
#ifndef SBLK_EXPAND
                for(bank = 0; bank < banknum; bank++)
                {
                    count = 0;
                    for (block = 0; block < SYS_BLK; block++)
                    {
                        if((gulSysblk >> (bank * 4)) & (1 << block))
                        {
                            SBlkNo[bank][block] = block;
                            count++;
                        }
                    }
                    if(count == 0)
                    {
                        printk("Merge llf whole system can not find SBlock !!!\r\n");
                        llfEraseSblk();
                        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
                        return ret;
                    }
                }
#else
                ret = llfGetInheritSBlk(SBlkNo);
                if(ret != ERR_OK)
                {
                    return ret;
                }
#endif
                llfprintk("Merge llf write whole system Image\r\n");
                ret = llfWriteCodeBlock(FW_FOR_MST, SBlkNo);
                //ret = llfWriteFWMerge(SBlkNo); //write merge image
                if( ret == ERR_READ_SBLK )
                {
                    ret = llfWriteCodeBlockFailHandle();
                }
                if (ret != ERR_OK)
                {
                    printk("Merge llf whole system Image write fail !!!,ret is %x\r\n", ret);
                    llfEraseSblk();
                    pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
                    return ret;
                }
                else
                {
                    printk("Merge llf whole system Image write OK !!!\r\n");
#ifdef SBLK_EXPAND
                    //check sblock statu
                    ret = llfCheckSblock(SBlkNo);
                    if(ret != ERR_OK)
                    {
                        return ret;
                    }
#endif
                    pResponseInfo->res_state = VENDOR_CMD_WRITE_FW_CT2;
                    gubLLFSubStep.ubMstWriteConfigStep = 0;
                    return ERR_OK;
                }
            }
            else
            {
                printk("Merge llf whole system check flash type fail !!!\r\n");
                llfEraseSblk();
                pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
                return ERR_LOAD_SYSTEM_IMAGE;
            }
        }
        else
        {
            //printk("Try read FW TAG ,wait whole system read... \r\n");
            gubLLFSubStep.ubWriteConfigStep = STEP_WRITE_CONFIG_OVER;
            return ERR_OK;
        }
    }
#endif
    return ERR_OK;
}

U16 llfChkMarkDefectBlk(U8 bank, U16 block_no)
{
    //just for build DBT now
#if 0
    // if the block has been marked, we won't check, directly return 0
    if (llfIsBlockBad(DBT_ADDR, bank, block_no))
    {
        return 0;
    }

    if (llfChkDefectBlk(bank, block_no))
    {
        llfMarkUserMPBlkDefect(DBT_ADDR, bank, block_no);
        return 1;
    }

    return 0;
#else
    if (llfChkDefectBlk(bank, block_no))
    {
        llfMarkUserMPBlkDefect(DBT_ADDR, bank, block_no);
        return 1;
    }

    return 0;
#endif
}

void CheckDefectMark()
{
    struct _LLF_UNI_INFO *pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    U8 bank, progress;
    U16 block_no;
    U8 strBuf[32] = {'C', 'h', 'e', 'c', 'k', ' ', 'd', 'e', 'f', 'e', 'c', 't', ' ', '%',
                     'd', '/', '1', '0', '0', '\r', '\n', '\0'
                    };
    U8 strbuff[64] = {'D', 'E', 'F', 'E', 'C', 'T', '(', 'b', 'a', 'n', 'k', ':', '%', 'd', ',', ' ', ' ', ' ', ',', ' ', 'b', 'l', 'o', 'c', 'k', ':', '%', 'd', ')', '\r', '\n', '\0'};

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (struct _LLF_UNI_INFO *)LLF_UNI_INFO_ADDR;

    pResponseInfo->res_progress = 0;
    pLLFUniInfo->ulData5 = 0;

    for (block_no = 0; block_no < NandPara.uwBlockNumPerLun; block_no++)
    {
        for (bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if(llfChkMarkDefectBlk(bank, block_no))
            {
                llfDbgPrintk(ALWAYS_MSG, (const char*)strbuff, bank, block_no);//first show strbuff
                pLLFUniInfo->ulData5 ++ ;//data5 is check bad num
            }
        }

        // print progress message
        progress = ((block_no + 1) * 99 / NandPara.uwBlockNumPerLun);
        if ((progress != pResponseInfo->res_progress) && (progress == (progress / 10 * 10)))
        {
            pResponseInfo->res_progress = progress;
            // llfDbgPrintk(ALWAYS_MSG, (const char*)strBuf, progress);
            //@@ now show defect Info strbuff, close this strBuf
            //@@ later open thstrBuf then open macro in llfChkMarkDefectBlk and close strbuff
        }
    }

    llfDbgPrintk(ALWAYS_MSG, (const char*)strBuf, 100);
    gfDBTInitDone = LLF_DBT_FACTORY;
}

void CalculateSnapshotArea()
{
    // TODO: check and opens
    U8 ubSblkStart = 0;
    U16 SnapshotGroupNum;
    U16 bank, mpblock, block, plane;
    U16 dynamicSblkNum, TotalDynamicSblkNum, snapBSNum;
    U16 MpDefectPerBS;
    U16 L2PGroupNum_BS, L2PGroupTotalNum_BS, L2PGroupNum_mpblock, L2PGroupTotalNum_mpblock;
    U32 L2P_Mem_capacity, ulCapPerMpBlock, ulCapPerBS;
    U32 Disk_Capacity_LBA;
#ifdef EXTEND_LBA
    U32 Disk_Capacity_LBA_H;
#endif
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif
#if defined(RL6577_VA) || defined(RL6643_VA) ||defined(RTS5771_VA)
    U8 ufIsExtendL2P;
#endif

#if defined(RL6577_VA)||defined(RTS5771_VA)
    U32 ulTemp3 = NandPara.ubPlaneNumPerLun * NandPara.uwSLCPageNumPerBlock * NandPara.ubBankNum *
                  10; // n == 10
    SnapshotGroupNum = (((344064 % ulTemp3) != 0) ? ((344064 / ulTemp3) + 1) : (344064 / ulTemp3));
    SnapshotGroupNum = (SnapshotGroupNum < 20) ? SnapshotGroupNum : 20;
    printk("SnapshotGroupNum %d\r\n", SnapshotGroupNum);
#else
    //SnapshotGroupNum = SNAPSHOT_BS_NUM_PER_DIE;
    U32 temp1;
    // follow 6577 to align Snapshot BS amount
    // SSD6577-3828
    // N = 10, the EC increase N in 1 hours idle.
    temp1 = NandPara.ubPlaneNumPerLun * NandPara.uwSLCPageNumPerBlock * NandPara.ubBankNum * 10;
    // 0x54000 is the number of write page pre 1 hour.
    SnapshotGroupNum = (0x54000 + temp1 - 1) / temp1;
    // Max = 20
    if (SnapshotGroupNum > 20)
    {
        SnapshotGroupNum = 20;
    }
    printk("SnapshotGroupNum %d\r\n", SnapshotGroupNum);
#endif
#ifdef EXTEND_LBA
    Disk_Capacity_LBA = _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET);
    Disk_Capacity_LBA_H = _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET + 4);
    L2P_Mem_capacity = (Disk_Capacity_LBA >> 11) + (Disk_Capacity_LBA_H << 21); // 32 - 11 = 21
    printk("Disk_Capacity_LBA %x_%x\r\n", Disk_Capacity_LBA_H, Disk_Capacity_LBA);
#else
    Disk_Capacity_LBA = _MEM64(CONFIG_BASE_VA_ADDR + CONFIG_CAPACITY_OFFSET);
    L2P_Mem_capacity = (Disk_Capacity_LBA >> 11);
#endif
#if defined(RL6577_VA) || defined(RL6643_VA) ||defined(RTS5771_VA)
    ufIsExtendL2P = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_EXTEND_L2PAREA);
    if(ufIsExtendL2P >= 20) // at least 2 times OP
    {
        // extend L2P OP for random accesss
        L2P_Mem_capacity = L2P_Mem_capacity * ufIsExtendL2P / 10;
        printk("Extend L2P to %d/10 times cap=%d\r\n", ufIsExtendL2P, L2P_Mem_capacity);
    }
    else
    {
        L2P_Mem_capacity = L2P_Mem_capacity * 14 / 5;
    }
#else
    //The unit is KB, 2.8 OP, if Disk Capacity is 120G, we will reserve 2.8 * 120M as L2P_Mem_capacity
    L2P_Mem_capacity = L2P_Mem_capacity * 14 / 5;
#endif
#if defined(RL6577_VA) && (defined(L2P_RAID) || defined(SLC_RAID))
    // extend L2P area size for L2P RAID
    U16 uwRaidMode = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun;
    L2P_Mem_capacity = L2P_Mem_capacity * uwRaidMode / (uwRaidMode - 1);
#elif defined(RL6643_VA) && defined(RAID_EN)
    // extend L2P area size for L2P RAID

#ifdef L2P_SHRINK
    U16 uwRaidMode = (NandPara.ubBankNum >> 1) * NandPara.ubPlaneNumPerLun * gubRaidPagePerRaid;
#else
    U16 uwRaidMode = NandPara.ubBankNum * NandPara.ubPlaneNumPerLun * gubRaidPagePerRaid;
#endif
    printk("L2P Raid Mode %d Rd %d\r\n", uwRaidMode, gubRaidPagePerRaid);
    L2P_Mem_capacity = L2P_Mem_capacity * uwRaidMode / (uwRaidMode - 1);
#endif

    ulCapPerMpBlock = (NandPara.ubSectorNumPerPage << 9) * NandPara.ubPlaneNumPerLun *
                      NandPara.uwSLCPageNumPerBlock;
    ulCapPerBS = ulCapPerMpBlock * NandPara.ubBankNum;
    ulCapPerMpBlock = ulCapPerMpBlock >> 10; // Byte to KB
    ulCapPerBS = ulCapPerBS >> 10; // Byte to KB
    L2PGroupTotalNum_BS = (L2P_Mem_capacity + (ulCapPerBS - 1)) / ulCapPerBS;
    L2PGroupTotalNum_mpblock = (L2P_Mem_capacity + (ulCapPerMpBlock - 1)) / ulCapPerMpBlock;

    if(L2PGroupTotalNum_BS < L2P_GROUP_NUM)
    {
        L2PGroupTotalNum_BS = L2P_GROUP_NUM;
    }

    guwCaclDynSBlockBegin = ubSblkStart + SYS_BLK + EXTEND_RDT_BLK;
    // The Number of Block for Dynamic SBlock is depends on the number of Bank is enough to preserved for
    // current, current-backup, pre-current and pre-current-backup, so it must bigger than 4.

    if (NandPara.ubBankNum <= 4)
    {
        TotalDynamicSblkNum = NandPara.ubPlaneNumPerLun * 2; // 2 * [ number of block per one BlockStripes ]
    }
    else
    {
        TotalDynamicSblkNum = NandPara.ubPlaneNumPerLun;
    }

    if(TotalDynamicSblkNum < 8)
        TotalDynamicSblkNum = 8;

    // get enough space for SBLK with skipping defect blocks
    mpblock = guwCaclDynSBlockBegin / NandPara.ubPlaneNumPerLun;
    dynamicSblkNum = 0;
    do
    {
        for (bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            for (plane = 0; plane < NandPara.ubPlaneNumPerLun; plane++)
            {
                block = (mpblock * NandPara.ubPlaneNumPerLun) + plane;
                if(block < guwCaclDynSBlockBegin)
                {
                    continue;
                }
                if ((!llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block)) && (!((bank < NandPara.ubBankNum)
                        && (block < ubSblkStart + SYS_BLK))))
                {
                    dynamicSblkNum++;
                }
            }
        }
        mpblock++;
    }
    while (dynamicSblkNum < TotalDynamicSblkNum);
    guwCaclDynSBlockEnd = (mpblock * NandPara.ubPlaneNumPerLun);
    guwCaclSSGroupBegin = (guwCaclDynSBlockEnd) / NandPara.ubPlaneNumPerLun;

    snapBSNum = 0;
    do
    {
        MpDefectPerBS = 0;
        for (bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if (llfIsMpBlockBad(DBT_ADDR, bank, mpblock))
            {
                MpDefectPerBS++;
            }
        }

        if (MpDefectPerBS <= BAD_BS_TH)
        {
            snapBSNum++;
        }
        mpblock++;
    }
    while (snapBSNum < SnapshotGroupNum);
    guwCaclSSGroupEnd = mpblock;

    L2PGroupNum_BS = 0;
    L2PGroupNum_mpblock = 0;
    do
    {
        MpDefectPerBS = 0;
        for (bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if (llfIsMpBlockBad(DBT_ADDR, bank, mpblock))
            {
                MpDefectPerBS++;
            }
        }

        if (MpDefectPerBS <= BAD_BS_TH)
        {
            L2PGroupNum_BS++;
            L2PGroupNum_mpblock += (NandPara.ubBankNum - MpDefectPerBS);
        }
        mpblock++;
    }
    while ((L2PGroupNum_BS < L2PGroupTotalNum_BS) || (L2PGroupNum_mpblock < L2PGroupTotalNum_mpblock));
    guwCacll2PGroupEnd = mpblock;

    llfDbgPrintk(ALWAYS_MSG,
                 "DSblkBegin %d DSblkEnd %d SnapshotBSBegin %d SnapshotBSEnd %d guwCacll2PGroupEnd %d\r\n",
                 guwCaclDynSBlockBegin, guwCaclDynSBlockEnd, guwCaclSSGroupBegin, guwCaclSSGroupEnd,
                 guwCacll2PGroupEnd);
}

void llfAPExecuteLLF(U8 ubIfType, U8 ubClkMode)
{
    U32 ret = ERR_OK;
    entry_t entry = 0;
    U32 lock_flag;
#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
    U32 save_bit;
#endif
    PLLF_UNI_INFO pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

    if(gubLLFSubStep.ubWriteConfigStep == STEP_WRITE_CONFIG_INIT)
    {
        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pResponseInfo->res_progress = 0;
        pResponseInfo->res_err_code = 0;

        pLLFUniInfo->ubBankNo = 0;
        pLLFUniInfo->ulWorkFunc = WRITE_CONFIG_FUNC;
    }
    if(pLLFUniInfo->ulWorkFunc == WRITE_CONFIG_FUNC)
    {
        //llfDbgPrintk(ALWAYS_MSG, "LLF execute\r\n");
#ifdef REPLACE_IMAGE
        if((gulReplaceImageNum & 0xF) == TAG_FOR_REPLACE_IMAGE)
        {
            ret = llfWriteConfigForReplaceImage(ubIfType, ubClkMode);
            if(ret != ERR_OK)
            {
                llfprintk("[RI] Execute LLF fail!\r\n");
            }
        }
        else
#endif
        {
            if(gubLLFSubStep.ubWriteConfigStep != STEP_WRITE_CONFIG_OVER
                    || gubLLFSubStep.ubMstWriteConfigStep == STEP_WRITE_CONFIG_MST)
            {
                ret = llfWriteConfig(ubIfType, ubClkMode);
                if(ret == ERR_OK)
                {
#ifdef MST_MERGE
                    if(pResponseInfo->res_state == VENDOR_CMD_WRITE_FW_CT2)
                    {
                        //printk("state: WRITE_FW_CT2\r\n");
                        pLLFUniInfo->ulWorkFunc = WRITE_CONFIG_FUNC;
                        return;
                    }
#endif
                    if(gubLLFSubStep.ubWriteConfigStep != STEP_WRITE_CONFIG_OVER)
                    {
                        pLLFUniInfo->ulWorkFunc = WRITE_CONFIG_FUNC;
                        pResponseInfo->res_state = VENDOR_CMD_WRITE_FW;
                    }
                    return;
                }
            }
#ifndef NEW_MUL_WR_CACHE
#ifndef RTS5771_FPGA
            else
            {
                ret = repeated_WriteReadCache(MUL_WR_CACHE_COUNT);
                if(ret != ERR_OK)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Multi write and read cache error!!!\r\n");
                    llfEraseSblk();
                }
            }
#endif
#endif
        }
        if(ret != 0)
        {
#if defined(RL6643_VA)
            if(ERR_READ_DBT == ret)
            {
                printk("Turn Over ret %x to OK\r\n", ret);
                WRITE_REG_32(SCAN_DBT_CTR, DEFECT_USER_MP_BLK_TABLE_SIZE_PER_BANK);
                printk("SCAN_DBT_CTR %d\r\n", READ_REG_32(SCAN_DBT_CTR));
                ret = ERR_OK;
            }
            else
#endif
            {
                llfDbgPrintk(ALWAYS_MSG, "Execute LLF fail!\r\n");
            }
        }
#if defined(RL6643_VA) || defined(RL6577_VA)
#ifdef SBLK_EXPAND
        if(0 == gubSblkBadDetect)
        {
            llfprintk("SBLK_BAD_DETECT is on \r\n");
            if( (gubDefaultSblkBad == 1) || (gubSblkBankStart != 0 || gubSblkStart != 0))
            {
                llfprintk("gubSblkBankStart has been shift , gubSblkStart = %d, gubSblkBankCnt = %d\r\n",
                          gubSblkStart, gubSblkBankStart);
                pResponseInfo->err_msg_num++;
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_err_code = ERR_SBLK_BAD_DETECT;
                pResponseInfo->res_progress = 100;
                ret = ERR_SBLK_BAD_DETECT;
                return;
            }
        }
#endif
#endif

#ifdef RL6643_VA
        U32 tempPrintValue, tempLimit;
        //add for control temperature
        tempLimit = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CONTROLLER_TEMPERATURE_TH);
        llfprintk("controller tempLimit is %d\r\n", tempLimit);
        if (tempLimit != 0)
        {
            tempPrintValue = (_REG32(THERMAL_CTRL_REG) & 0x003ffff8 ) >> 13;//bit[21:3]
            llfprintk("Temperature is %d \r\n", tempPrintValue);
            // if the temperature value is negative, need to handle 2's complement
            if (tempPrintValue > tempLimit )
            {
                llfprintk("Controller temperature over set value %d ,ERR!!!\r\n", tempLimit);
                _MEM32(LLF_RES_ERRMSG_START_VA_ADDR + WORD_BYTE_SIZE * pResponseInfo->err_msg_num) =
                    tempPrintValue;
                pResponseInfo->err_msg_num++;
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_err_code = ERR_CONTROLLER_OVER_TEMPERATURE;
                return;
            }
        }

        if(IS_EA_MODE)
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ret;
        }
        else if(ret != ERR_OK)//rom mode
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ret;
        }
        else
        {
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_progress = 100;
            pResponseInfo->res_err_code = ret;
        }
#else
        pResponseInfo->res_state = VENDOR_CMD_IDLE;
        pResponseInfo->res_progress = 100;
        pResponseInfo->res_err_code = ret;
#endif

        pLLFUniInfo->ulWorkFunc = NONE_FUNC;
        SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
        PrintDone();
        gfDBTInitDone = LLF_DBT_NONE;
        gufLastEraseFlag = 0;
        gubLLFALLStep = STEP_CALIBRATE;
        entry = pResponseInfo->entry;
        gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
        spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
        SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
        spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);

        if ( ERR_OK == ret )	//llf all succeed
        {
#ifdef CPU_1CORE
            if ( IS_EA_MODE )
            {
                HAL_DISABLE_INTERRUPTS();
                llfSaveInfoToSpi();

#if defined(RL6643_VA)
                U32 VrefDefaultSet = 0x124;

                if(IS_6855_VERSION_TAG)
                    VrefDefaultSet = 0x11e;
                gubRealCapacity *= NandPara.ubBankNum * NandPara.ubLunNumPerCE;
                WRITE_REG_32(SPI_DL_SSD_INFO_MODEL_NAME, gubRdtImg);
                WRITE_REG_32(SPI_DL_SSD_INFO_FW_VERSION, _REG32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FW_VERSION));
                WRITE_REG_32(SPI_DL_SSD_INFO_FW_VERSION + 4,
                             _REG32(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FW_VERSION + 4));
                WRITE_REG_32(SPI_DL_SSD_INFO_REAL_CAPACITY, gubRealCapacity);
                WRITE_REG_32(SPI_DL_SSD_INFO_USING_CAPACITY, guwHostClaimCapacity);
                if((gulVrefCfg != VrefDefaultSet) && (gubNandDefaultMode == 1))
                {
                    if(IS_6855_VERSION_TAG)
                        WRITE_REG_32(SPI_DL_VREF_ROM_DRIVE_SET_EA,
                                     ((0x52 << 20) | (gulVrefCfg << 8) | 0 << 4 | TAG_FOR_VREF_ODT_CFG));
                    else
                        WRITE_REG_32(SPI_DL_VREF_ROM_DRIVE_SET_EA,
                                     ((0x44 << 20) | (gulVrefCfg << 8) | 3 << 4 | TAG_FOR_VREF_ODT_CFG));
                }
#endif
                spi_init_llf();
                spi_flash_set_protection_llf(TRUE);
                llfDbgPrintk(ALWAYS_MSG, "Add spi lock after llf all succeed\r\n");

                HAL_ENABLE_INTERRUPTS();
            }
#if defined(RL6643_VA)
            else
            {
                if(IS_6643_VERSION_TAG)
                {
                    if(gubNandDefaultMode == 1)
                        llfUpdateVrefToEFUSE(gulVrefCfg);
                }
            }
#endif

#else

            if ( IS_EA_MODE )
            {
#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
                save_bit = READ_REG_32(PLIC_INTERRUPT_MASK);
                WRITE_REG_32(PLIC_INTERRUPT_MASK, 0xffffffff);
#else
                WRITE_REG_32(MMCR_GIC_RESET_MASK0, 0x3fff);  //mask all interrupt
#endif
                llfSaveInfoToSpi();

#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
                WRITE_REG_32(0x8003ff40, 0x0);

                WriteBackInvalidateDcahe();
                cache_dummy_update_read();
                InvalidateIcache();
                cache_dummy_update_read();
                FcBusyWait1ms(5);
#endif

                spi_init_llf();
                spi_flash_set_protection_llf(TRUE);
                llfprintk("SPI lock\r\n");
#if defined(RTS5771_FPGA)||defined(RTS5771_VA)
                WRITE_REG_32(PLIC_INTERRUPT_MASK, save_bit);
#else
                WRITE_REG_32(MMCR_GIC_SET_MASK0, 0x3fff);  //unmask all interrupt
#endif
            }
#endif
        }
    }
}

void llfInheritRDTInfoFromSBLK(U32 ulBaseAddr)
{
    U8 ubBank;
    U32 ulPFOffset, ulEFOffset, ulRFOffset;

    // Copy Sys DBT from Static Sblock
    memcpy((void *) SYS_BLK_DBT_ADDR,
           (void *)(ulBaseAddr + SBLK_OFFSET_SYSTEM_BLOCK_DBT),
           SYS_BLK_DBT_SIZE);
    cache_area_dinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_SIZE);
    cache_dummy_update_read();

    _MEM08(SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_ROUND) =
        _REG08(ulBaseAddr + SBLK_OFFSET_RDT_TESTED_ROUND);
    _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_TIME) =
        _REG16(ulBaseAddr + SBLK_OFFSET_RDT_TESTED_TIME);
    _MEM32(SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_TEMPERATURE) =
        _REG32(ulBaseAddr + SBLK_OFFSET_RDT_TESTED_TEMPERATURE);
    _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_ECC_THRESHOLD) =
        _REG16(ulBaseAddr + SBLK_OFFSET_RDT_ECC_THRESHOLD);
    _REG08(SBLK_ADDR + SBLK_OFFSET_RDT_SLCECC_THRESHOLD) =
        _REG08(ulBaseAddr + SBLK_OFFSET_RDT_SLCECC_THRESHOLD);
    _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_DEFECT_THRESHOLD) =
        _REG16(ulBaseAddr + SBLK_OFFSET_RDT_DEFECT_THRESHOLD);
    _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_PFAIL_THRESHOLD) =
        _REG16(ulBaseAddr + SBLK_OFFSET_RDT_PFAIL_THRESHOLD);
    _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_EFAIL_THRESHOLD) =
        _REG16(ulBaseAddr + SBLK_OFFSET_RDT_EFAIL_THRESHOLD);
    _REG08(SBLK_ADDR + SBLK_OFFSET_RDT_PLANE_NUM_PER_LUN) =
        _REG08(ulBaseAddr + SBLK_OFFSET_RDT_PLANE_NUM_PER_LUN);

    // Inherit RDT version
    memcpy((void *)(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_VERSION),
           (const void *)(ulBaseAddr + SBLK_OFFSET_RDT_VERSION), 8);

    for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
    {
        ulPFOffset = ulEFOffset = ulRFOffset = SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO;
        ulPFOffset += ubBank * SBLK_OFFSET_RDT_PER_BANK_SIZE;
        ulEFOffset += ubBank * SBLK_OFFSET_RDT_PER_BANK_SIZE + SHORT_BYTE_SIZE;
        ulRFOffset += ubBank * SBLK_OFFSET_RDT_PER_BANK_SIZE + SHORT_BYTE_SIZE * 2;

        _REG16(SBLK_ADDR + ulPFOffset) = _REG16(ulBaseAddr + ulPFOffset);
        _REG16(SBLK_ADDR + ulEFOffset) = _REG16(ulBaseAddr + ulEFOffset);
        _REG16(SBLK_ADDR + ulRFOffset) = _REG16(ulBaseAddr + ulRFOffset);

        //FFailNum
        _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + ubBank * SHORT_BYTE_SIZE) =
            _REG16(ulBaseAddr + SBLK_OFFSET_RDT_FACTORY_BAD + ubBank * SHORT_BYTE_SIZE);

        //AFailNum
        _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + ubBank * SHORT_BYTE_SIZE) =
            _REG16(ulBaseAddr + SBLK_OFFSET_RDT_BLOCK_BAD + ubBank * SHORT_BYTE_SIZE);

        llfDbgPrintk(ALWAYS_MSG, "bank %d, PF_EF_RF_FF %d_%d_%d_%d\r\n",
                     ubBank, _MEM16(SBLK_ADDR + ulPFOffset),
                     _MEM16(SBLK_ADDR + ulEFOffset), _MEM16(SBLK_ADDR + ulRFOffset),
                     _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + ubBank * SHORT_BYTE_SIZE));
    }
}

U32 llfParseRDTResult(U8 bank, U32 resultAddr)
{
    //	U8 ch, ce;
    U16 block_no, i;
    U16 defectNum, factoryNum, PFailNum, EFailNum, RFailNum;
    U32 EccStatisticAddr, EccLess10, EccLess30, EccLarg30;
    U32 AssertAddr, result, roundNum, runTime, ret;
    U8 assertFile[64], blk0ErrFlag;

    U8 strBuf[128] = {'R', 'D', 'T', ' ', 'B', 'a', 'n', 'k', '%', 'd', ':', ' ', ' ', ' ', 'R', '.',
                      '%', 'd', ' ', 'T', '.', '%', 'd', ' ', 'B', 'l', 'k', '0', 'E', 'r',
                      'r', ':', '%', 'd', ' ', 'B', 'a', 'd', 'N', 'u', 'm', ':', '%', 'd',
                      ' ', 'O', 'r', 'i', 'g', ':', '%', 'd', '/', 'P', ':', '%', 'd', '/',
                      'E', ':', '%', 'd', '/', 'R', ':', '%', 'd', ' ', 'E', 'c', 'c', 'B',
                      'i', 't', '~', '1', '0', '/', '3', '0', '/', '>', '3', '0', ':', '%',
                      'd', '/', '%', 'd', '/', '%', 'd', '\r', '\n', '\0'
                     };

#ifdef EXTEND_STATIC_DBT
    U8 ubSblkStart, ubloadRdtOk;
    U16 page_no, des_page;
    U32 ulMode, cmp;
#ifdef BLK_REMAP_PRO
    U16 MpNum;
#endif
#ifdef NEW_EXTEND_STATIC_DBT
    U8 ubStaticDBTperPage = (DRAM_DATA_SIZE / DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK);
#endif
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#else
    ubSblkStart = 0;
#endif

#else
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    U8 ubSblkStart;
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#else
    ubSblkStart = 0;
#endif
#endif
#endif

    cache_area_dinval(resultAddr, NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);
    cache_dummy_update_read();

    // parse bad block information
    result		= _REG32(resultAddr);
    roundNum	= _REG32(resultAddr + RDT_DATA_OFFSET);//RDT round num executed
    runTime 	= _REG32(resultAddr + RDT_DATA_OFFSET + WORD_BYTE_SIZE);
    AssertAddr	= resultAddr + WORD_BYTE_SIZE;
    // badListAddr = resultAddr + DEFECT_BLK_TABLE_SIZE_PER_BANK;
    EccStatisticAddr = resultAddr + RDT_ECC_STATISTIC_OFFSET;

    if(bank == 0)
    {
        _REG08(SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_ROUND) = (U8)roundNum;
        _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_TIME) = (U16)runTime;
        _REG32(SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_TEMPERATURE) = _REG32(resultAddr +
                RDT_TESTED_TEMPERATURE_OFFSET);

        _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_DEFECT_THRESHOLD) =
            _REG32(resultAddr + RDT_BAD_BLOCK_LIMIT_OFFSET) & 0xFFFF;
        if(_REG16(SBLK_ADDR + SBLK_OFFSET_RDT_DEFECT_THRESHOLD) == 0xFFFF)
            _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_DEFECT_THRESHOLD) = 0;
        _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_ECC_THRESHOLD) =
            _REG32(resultAddr + RDT_ECC_BIT_LIMIT_OFFSET) & 0xFFFF;
        _REG08(SBLK_ADDR + SBLK_OFFSET_RDT_SLCECC_THRESHOLD) =
            _REG32(resultAddr + RDT_SLCECC_BIT_LIMIT_OFFSET) & 0xFF;
        _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_PFAIL_THRESHOLD) =
            _REG32(resultAddr + RDT_PROGRAM_FAIL_LIMIT_OFFSET) & 0xFFFF;
        _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_EFAIL_THRESHOLD) =
            _REG32(resultAddr + RDT_ERASE_FAIL_LIMIT_OFFSET) & 0xFFFF;
        if((_REG32(resultAddr + RDT_MPBLOCK_PERLUN_OFFSET)) != 0)
            _REG08(SBLK_ADDR + SBLK_OFFSET_RDT_PLANE_NUM_PER_LUN) = ((_REG32(resultAddr +
                    RDT_SPBLOCK_PERLUN_OFFSET)) / (_REG32(resultAddr + RDT_MPBLOCK_PERLUN_OFFSET)));

        // Inherit RDT version
        memcpy((void *)(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_VERSION),
               (const void *)(resultAddr + RDT_VERSION_OFFSET), 8);
    }

    // wirte to sblok
    _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO + bank * SBLK_OFFSET_RDT_PER_BANK_SIZE) =
        _REG16(resultAddr + RDT_PF_NUM_INFO_OFFSET) + _REG16(resultAddr +
                RDT_SLC_PF_NUM_INFO_OFFSET); //PFailNum
    _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO + bank * SBLK_OFFSET_RDT_PER_BANK_SIZE +
           SHORT_BYTE_SIZE) = _REG16(resultAddr + RDT_EF_NUM_INFO_OFFSET) + _REG16(
                                  resultAddr + RDT_SLC_EF_NUM_INFO_OFFSET); //EFailNum
    _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO + bank * SBLK_OFFSET_RDT_PER_BANK_SIZE + 2 *
           SHORT_BYTE_SIZE) = _REG16(resultAddr + RDT_RF_NUM_INFO_OFFSET) + _REG16(
                                  resultAddr + RDT_SLC_RF_NUM_INFO_OFFSET); //RFailNum
    _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + bank * SHORT_BYTE_SIZE) = _REG16(
                resultAddr + RDT_FACTORY_NUM_INFO_OFFSET); //FactoryFailNum
    _REG16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + bank * SHORT_BYTE_SIZE) = _REG16(
                resultAddr + RDT_BADBLOCK_NUM_OFFSET); //AllFailNum

#if defined(RL6577_VA)
    PPCIE_AUTOK_MDIO pcie_save_results_to_sblock;
    PPCIE_AUTOK_MDIO pcie_read_results_from_nand;

    pcie_save_results_to_sblock = (PPCIE_AUTOK_MDIO)(SBLK_ADDR + SBLK_OFFSET_FE_CALIBRATION);
    pcie_read_results_from_nand = (PPCIE_AUTOK_MDIO)(resultAddr + RDT_FE_CALIBRATION_OFFSET);

    for(i = 0; i < 12; i++)
    {
        pcie_save_results_to_sblock->OOBS_Summer_DCVS[i] = pcie_read_results_from_nand->OOBS_Summer_DCVS[i];
    }
    for(i = 0; i < 4; i++)
    {
        pcie_save_results_to_sblock->Offset_4Lane[i] = pcie_read_results_from_nand->Offset_4Lane[i];
    }
    pcie_save_results_to_sblock->Cal_results = pcie_read_results_from_nand->Cal_results;
    printk("OOBS / Summer / DCVS\r\n");
    U32 k;
    for(i = 0; i < 3; i++)
    {
        printk("Gen:%d\r\n", i + 1);
        for(k = 0; k < 4; k++)
        {
            printk("lane:%d, 0x%x\r\n", k, pcie_save_results_to_sblock->OOBS_Summer_DCVS[k]);
        }
    }
    printk("OFFSET\r\n");
    for(i = 0; i < 3; i++)
    {
        printk("%x\r\n", pcie_save_results_to_sblock->Offset_4Lane[i]);
    }
#endif

#ifdef AVG_ERASE_COUNT_TEST
    gulDataBsAvgEraseCnt = _REG32(resultAddr + RDT_RESULT_DATA_BS_EC);
    gulL2PBsAvgEraseCnt = _REG32(resultAddr + RDT_RESULT_L2P_BS_EC);
    llfprintk("Get DataBSAvgEC = %x, L2PBSAvgEC = %x(in Parser RDT_result mode)\r\n",
              gulDataBsAvgEraseCnt, gulL2PBsAvgEraseCnt);
#endif

    defectNum	= 0;
    factoryNum	= 0;
    PFailNum	= 0;
    EFailNum	= 0;
    RFailNum	= 0;
    blk0ErrFlag = 0;
    ret = ERR_OK;
#if 0
    for (i = 0; i < DEFECT_BLK_TABLE_SIZE_PER_BANK; i += 4)
    {
        defectType = _REG08(badListAddr + i);
        block_no   = _REG32(badListAddr + i) >> 16;

        if (block_no == 0xFFFF && defectType == 0xFF)
        {
            break;
        }
        if (block_no >= NandPara.uwBlockNumPerLun || defectType > DEFECT_READ)
        {
            llfprintk("Unrecognize block and type: %d %d\r\n", block_no, defectType);
            //ret = ERR_READ_RDT;
            continue;
        }
        if (block_no == 0)	// block0 fail
        {
            blk0ErrFlag = 1;
        }
        switch(defectType)
        {
        case DEFECT_FACTORY:
            factoryNum++;
            pLLFUniInfo->ulData5++;
            break;
        case DEFECT_PROGRAM:
            PFailNum++;
            break;
        case DEFECT_ERASE:
            EFailNum++;
            pLLFUniInfo->ulData6++;
            break;
        case DEFECT_READ:
            RFailNum++;
            break;
        }
        if(block_no < SYSTEM_BLOCK_MAX_NUM)
        {
            MarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block_no);
        }
        llfSetDBT(bank, block_no, DBT_ADDR);
        defectNum++;
    }
#else
#ifdef EXTEND_STATIC_DBT
    if(gubNeedRebuildRemap == 0)
    {
        for(block_no = 0; block_no < SYSTEM_BLOCK_MAX_NUM; block_no++)
        {
            if(llfIsBlockBadInRDTDBT(resultAddr + RDT_SYS_DBT_OFFSET, block_no))
            {
                llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block_no);
            }
        }
        memcpy((void*)(DBT_ADDR + (bank * DEFECT_USER_MP_BLK_TABLE_SIZE_PER_BANK)),
               (void*)(resultAddr + RDT_MP_DBT_OFFSET), RDT_USER_MP_BLK_TABLE_SIZE_PER_BANK);

        for(block_no = ubSblkStart + SYS_BLK; block_no < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK; block_no++)
        {
            if(llfIsBlockBadInRDTDBT(resultAddr + RDT_SYS_DBT_OFFSET, block_no))
            {
                llfprintk("Bank %d, SpBlock %d is RDT Bad\r\n", bank, block_no);
                llfSetDBT(bank, block_no, DBT_ADDR);
            }
        }

#ifdef BLK_REMAP_PRO
        guwRealMPBlkNum = _REG32(resultAddr + RDT_REMAP_OFFSET);
        llfprintk("=REMAP= Inherit RDT guwRealMPBlkNum is %d\r\n", guwRealMPBlkNum);
        for(block_no = 0; block_no < (guwRealMPBlkNum * NandPara.ubPlaneNumPerLun); block_no++)
#else
        for(block_no = 0; block_no < NandPara.uwBlockNumPerLun; block_no++)
#endif
        {
            if(llfIsBlockBad(DBT_ADDR, bank, block_no))
            {
                defectNum++;
                //llfprintk("inherit RDT Bad Bank %d, mp %d\r\n", bank, block_no / NandPara.ubPlaneNumPerLun);
            }
        }
    }
    else
    {
        for(block_no = ubSblkStart + SYS_BLK; block_no < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK; block_no++)
        {
            cache_area_dinval(TEMP_HBUF_ADDR, DRAM_HEAD_SIZE);
            cache_dummy_update_read();
            cache_area_dinval(TEMP_BUF_ADDR, DRAM_DATA_SIZE);
            cache_dummy_update_read();

            // read back 16KB StaticDBT for inherit
            ubloadRdtOk = 0;
            ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            for(page_no = RDT_STATIC_DBT_PAGE_BEGIN; page_no < RDT_STATIC_DBT_PAGE_END;
                    page_no += (RESULT_SIZE_PER_BANK / DRAM_DATA_SIZE))
            {
#ifdef NEW_EXTEND_STATIC_DBT
                des_page = RDT_STATIC_DBT_PAGE_BEGIN + (bank / ubStaticDBTperPage);
#else
                des_page = page_no;
#endif
                gul_FW_TAG = llfBETagSetting(TAG_READ, des_page);
                llfFCCmdRead_DRAM(ulMode, bank, 0, block_no, des_page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);

                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
#ifdef EXTEND_STATIC_DBT_DEBUG
                llfprintk("StaticDBT read %d %d %d pagecnt 1 -> %x, ret %d, cmp %x, header %x\r\n", bank, block_no,
                          page_no, TEMP_BUF_ADDR, ret, cmp, _REG32(TEMP_HBUF_ADDR));

                for (i = 0; i < NandPara.uwBlockNumPerLun; i++)
                {
#ifdef NEW_EXTEND_STATIC_DBT
                    if(llfIsSpBlockBadInStaticDBT(TEMP_BUF_ADDR + (bank % ubStaticDBTperPage) *
                                                  DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK, i))
#else
                    if(llfIsSpBlockBadInStaticDBT(TEMP_BUF_ADDR, i))
#endif
                        DbgPrintk(ALWAYS_MSG, "Parse StaticDBT Bank %d SpBlock %d is bad\r\n", bank, i);
                }
#endif

#ifdef SLC_RRT
                ret = llfReadRetry(bank, block_no, des_page, &cmp,
                                   TEMP_BUF_PHY_ADDR, TEMP_HBUF_PHY_ADDR);
                if (ret != ERR_OK)
                {
                    ret = ERR_READ_SBLK;
                    continue;
                }

#endif

                if((ret == ERR_OK) && ((cmp & BE_COMPLETION_ERROR_MASK) == 0)
                        && (_REG32(TEMP_HBUF_ADDR) == RDT_BLK_ID))
                {
                    ubloadRdtOk = 1;
                    break;
                }
#ifdef NEW_EXTEND_STATIC_DBT
                else
                    break;
#endif
            }
            if(ubloadRdtOk == 1)
                break;
        }
        ASSERT_LLF(block_no < (ubSblkStart + SYS_BLK + EXTEND_RDT_BLK));

#ifdef BLK_REMAP_PRO
        guwMpDefectBlock[bank] = 0xFFFF;
        for (i = 0;  i < NandPara.ubPlaneNumPerLun ; i ++)
        {
            guwRemapTempBlock[bank][i] = 0xFFFF;
        }
#endif
        for(block_no = 0; block_no < NandPara.uwBlockNumPerLun; block_no++)
        {
#ifdef NEW_EXTEND_STATIC_DBT
            if(llfIsSpBlockBadInStaticDBT(TEMP_BUF_ADDR + (bank % ubStaticDBTperPage) *
                                          DEFECT_STATIC_SP_BLK_TABLE_SIZE_PER_BANK, block_no))
#else
            if(llfIsSpBlockBadInStaticDBT(TEMP_BUF_ADDR, block_no))
#endif
            {
                if(block_no < SYSTEM_BLOCK_MAX_NUM)
                {
                    llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block_no);
                }
#if defined(LLF_CHECK_SYSTEMBLK_ERASE_FAIL)
#if !defined(SBLK_EXPAND)
                if(block_no < SYS_BLK)
                {
                    gulEraseFailSysblk |= 1 << (block_no + 4 * bank);
                }
#else
                U8 ubBankNum = (NandPara.ubBankNum > SYS_BANK_NUM) ? SYS_BANK_NUM : NandPara.ubBankNum;
                if(block_no >= gubSblkStart && block_no < (gubSblkStart + SYS_BLK)
                        && bank >= gubSblkBankStart && bank < (gubSblkBankStart + ubBankNum))
                {
                    gulEraseFailSysblk |= (1 << ((block_no - ubSblkStart) + 4 * (bank - gubSblkBankStart)));
                }
#endif
#endif
#ifdef BLK_REMAP_PRO
                if (block_no >= SYSTEM_BLOCK_MAX_NUM)
                {
                    MpNum = block_no / NandPara.ubPlaneNumPerLun;
                    //llfprintk("Try to push some good single block into Remap table! block %d\r\n", block_no);
                    //llfprintk("guwMpDefectBlock[%d] %d\r\n",bank_no, guwMpDefectBlock[bank_no]);
                    if(guwMpDefectBlock[bank] == MpNum)
                    {
                        //second
                        for (i = 0;  i < NandPara.ubPlaneNumPerLun ; i ++)
                        {
                            if ((MpNum * NandPara.ubPlaneNumPerLun) + i >= block_no)
                            {
                                if((MpNum * NandPara.ubPlaneNumPerLun) + i != block_no)
                                    guwRemapTempBlock[bank][i] = (MpNum * NandPara.ubPlaneNumPerLun) + i;
                                else
                                    guwRemapTempBlock[bank][i] = 0xFFFF;
                            }
                        }
                    }
                    else
                    {
                        //push tempblock into table
                        if (guwMpDefectBlock[bank] != 0xFFFF)
                            LlfCreatReMappingTable(bank);

                        guwMpDefectBlock[bank] = MpNum;

                        //First
                        for (i = 0;  i < NandPara.ubPlaneNumPerLun ; i ++)
                        {
                            if((MpNum * NandPara.ubPlaneNumPerLun) + i != block_no)
                                guwRemapTempBlock[bank][i] = (MpNum * NandPara.ubPlaneNumPerLun) + i;
                            else
                                guwRemapTempBlock[bank][i] = 0xFFFF;
                        }
                    }

                    if (block_no == NandPara.uwBlockNumPerLun - 1)
                    {
                        LlfCreatReMappingTable(bank);
                    }
                }
#endif
                llfSetDBT(bank, block_no, DBT_ADDR);
                defectNum++;
                //llfprintk("inherit RDT Bad Bank %d, mp %d\r\n", bank, block_no / NandPara.ubPlaneNumPerLun);
            }
        }
    }
#else
    for(block_no = 0; block_no < SYSTEM_BLOCK_MAX_NUM; block_no++)
    {
        if(llfIsBlockBadInRDTDBT(resultAddr + RDT_SYS_DBT_OFFSET, block_no))
        {
            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank, block_no);
        }
    }
    memcpy((void*)(DBT_ADDR + (bank * DEFECT_USER_MP_BLK_TABLE_SIZE_PER_BANK)),
           (void*)(resultAddr + RDT_MP_DBT_OFFSET), DEFECT_USER_MP_BLK_TABLE_SIZE_PER_BANK);

#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    for(block_no = ubSblkStart + SYS_BLK; block_no < ubSblkStart + SYS_BLK + EXTEND_RDT_BLK; block_no++)
    {
        if(llfIsBlockBadInRDTDBT(resultAddr + RDT_SYS_DBT_OFFSET, block_no))
        {
            llfprintk("Bank %d, SpBlock %d is RDT Bad\r\n", bank, block_no);
            llfSetDBT(bank, block_no, DBT_ADDR);
        }
    }
#endif

#ifdef BLK_REMAP_PRO
    guwRealMPBlkNum = _REG32(resultAddr + RDT_REMAP_OFFSET);
    llfprintk("=REMAP= Inherit RDT guwRealMPBlkNum is %d\r\n", guwRealMPBlkNum);
    for(block_no = 0; block_no < (guwRealMPBlkNum * NandPara.ubPlaneNumPerLun); block_no++)
#else
    for(block_no = 0; block_no < NandPara.uwBlockNumPerLun; block_no++)
#endif
    {
        if(llfIsBlockBad(DBT_ADDR, bank, block_no))
        {
            defectNum++;
            //llfprintk("inherit RDT Bad Bank %d, mp %d\r\n", bank, block_no / NandPara.ubPlaneNumPerLun);
        }
    }
#endif
#endif
    // parse ECC bit statistic
    EccLess10 = 0;
    EccLess30 = 0;
    EccLarg30 = 0;

    for (i = 0; i < RDT_DATA_OFFSET; i++)
    {
        if (i <= 10)
        {
            EccLess10 += _REG32(EccStatisticAddr + (i * 4));
        }
        else if (i > 10 && i <= 30)
        {
            EccLess30 += _REG32(EccStatisticAddr + (i * 4));
        }
        else
        {
            EccLarg30 += _REG32(EccStatisticAddr + (i * 4));
        }
    }
    if(_MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_RESULTFAILINHERITEN))
    {
        printk("Inherit RDTfailresult open!!\r\n");
        result = ERR_OK;  //RDT result fail can inherit llf pass
    }
    // parse ASSERT message
    if (result != ERR_OK)
    {
        strBuf[12] = 'X';
        if ((result & 0xFFFF0000) == RDT_RESULT_ASSERT_TAG)
        {
            for (i = 0; _REG08(AssertAddr + i) != 0 && i < 64; i++)
            {
                assertFile[i] = _REG08(AssertAddr + i);
            }
            assertFile[i] = '\0';
            llfprintk("RDT ASSERT %s, %d\r\n", assertFile, result);
        }
        ret = ERR_READ_RDT;
    }
    else
    {
        strBuf[12] = 'O';
    }

    llfprintk((const char*)strBuf, bank, roundNum, runTime, blk0ErrFlag, defectNum,
              factoryNum, PFailNum, EFailNum, RFailNum, EccLess10, EccLess30, EccLarg30);

    AddErrorMessage(bank, defectNum, ret);

    return ret;
}

U32 llfGetRDTBankResult(U8 isSLC, U8 bank, U8 IsfromOriRDT)
{
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PVENDOR_CMD pVendorCmd;
    U16 page;
    U32 ret = ERR_OK;
    U32 ulMode, i;
    U32 cmp;
    U8 block_no, ubSblkStart = 0, ubSblkEnd = SYS_BLK;
    U8 ubLunNo;
#ifdef SBLK_EXPAND
#if defined(RL6643_VA)
    U32 ulSblkId;
#endif
#endif

#ifdef NEW_RDT_RESULT
    U8 bank_idx;
    U8 RDT_find = 0;
    U8 ch_ce, ch_no, ce_no;
    U8 targetpage;

    ch_ce = _MEM08(BANK_IMAPPING_TABLE_ADDR + bank);
    ch_no = (ch_ce & 0xff) >> 4;
    ce_no = ch_ce & 0xf;

    targetpage = ch_no * 8 + ce_no ;

    llfprintk("new RDT result, bank %d CH_%d CE_%d targetpage is %d \r\n", bank, ch_no, ce_no,
              targetpage);
#endif
    ubLunNo =  bank / NandPara.ubBankNumPerLun;
    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
    llfprintk("LUN number is %d \r\n", ubLunNo);

    if (pVendorCmd->subcmd == BE_LLF_EXECUTE)
    {
        //check whether need to set FC mode by different flow
        pResponseInfo->res_data_addr = TEMP_BUF_ADDR;
        pResponseInfo->res_progress = 0;
        llfInitErrorMessage();
    }

    cache_area_dinval(TEMP_HBUF_ADDR, WORD_BYTE_SIZE << _5BIT_SHIFT);
    cache_dummy_update_read();
    ASSERT_LLF(DRAM_DATA_SIZE == (NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT));
    if(IsfromOriRDT)
    {
#ifdef SBLK_EXPAND
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
#if defined(RL6643_VA)

        ubSblkStart = gubSblkStart;
        ubSblkEnd = gubSblkStart + SYS_BLK + EXTEND_RDT_BLK;
#endif
#else
        ubSblkStart = gubSblkStart;
        ubSblkEnd = gubSblkStart + SYS_BLK;
#endif
#endif

    }
    else
    {
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
#ifdef SBLK_EXPAND
        ubSblkStart = gubSblkStart + SYS_BLK;
        ubSblkEnd = gubSblkStart + SYS_BLK + EXTEND_RDT_BLK;
#else
        ubSblkStart = SYS_BLK;
        ubSblkEnd = SYS_BLK + EXTEND_RDT_BLK;
#endif
#else
        ASSERT_LLF(0);
#endif
    }
#ifndef NEW_RDT_RESULT
    for(block_no = ubSblkStart; block_no < ubSblkEnd; block_no++)
    {
#ifdef IS_8K_PAGE
        for (page = 0; page < RDT_RESULT_PAGE_NUM; page += 2)
#else
        for (page = 0; page < RDT_RESULT_PAGE_NUM; page++)
#endif
        {
            cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);
            cache_dummy_update_read();

            ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
            for(i = 0; i <= ( RESULT_SIZE_PER_BANK / DRAM_DATA_SIZE - 1); i++)
            {
                llfFCCmdRead_DRAM(ulMode, bank, ubLunNo, block_no, page + i,
                                  TEMP_BUF_PHY_ADDR + DRAM_DATA_SIZE * i,
                                  DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            }
#ifdef SBLK_EXPAND
#if defined(RL6643_VA)
            ulSblkId = _REG32(TEMP_HBUF_ADDR);
            if((gubLLFMode != LLF_DEFECT_BAD_BLOCK) && ( ulSblkId == CODE_BLK_ID))
            {
                printk("Find CODE_BLK_ID, So return to inherit blkinfo...\r\n");
                return ERR_READ_RDT;
            }
#endif
#endif
            if (ret != ERR_OK)
            {
                ret = ERR_READ_RDT_TIMEOUT;
                continue;
            }

#ifdef SLC_RRT
            for(i = 0; i <= ( RESULT_SIZE_PER_BANK / DRAM_DATA_SIZE - 1); i++)
            {
                ret = llfReadRetry(bank, block_no, page + i, &cmp,
                                   TEMP_BUF_PHY_ADDR + DRAM_DATA_SIZE * i, TEMP_HBUF_PHY_ADDR);
            }
            if (ret != ERR_OK)
            {
                ret = ERR_READ_RDT;
                continue;
            }
#endif

            if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
            {
                if (_REG32(TEMP_HBUF_ADDR) != RDT_BLK_ID)
                {
                    llfprintk("[ERR] Get RDT result bank %d block %d page %d RDT_BLK_ID mismatch %x!\r\n", bank,
                              block_no, page, _REG32(TEMP_HBUF_ADDR));
                    ret = ERR_READ_RDT;
                    continue;
                }
                break;
            }
            else if(cmp & ECC_UNCORRECT_ERR)
            {
                llfprintk("[ERR] Get RDT result ECC bank %d block %d page %d cmp %x\r\n", bank, block_no, page,
                          cmp);
                ret = ERR_READ_RDT_ECC;
                continue;
            }
            else
            {
                llfprintk("[ERR] Get RDT result fail bank %d block %d page %d cmp %x\r\n", bank, block_no, page,
                          cmp);
                ret = ERR_READ_RDT;
                continue;
            }
        }
        if(((cmp & BE_COMPLETION_ERROR_MASK) == 0) && (_REG32(TEMP_HBUF_ADDR) == RDT_BLK_ID))
        {
#ifdef SBLK_EXPAND
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
#if defined(RL6643_VA)
            if(block_no > gubTSblk)
            {
                gubTSblk = block_no;
            }
#endif
#endif
#endif
            break;
        }
#ifdef NEW_SORTING

        U32 temp_2k_length;
        temp_2k_length = gub_Total_len_per_2K;
        gub_Total_len_per_2K = 2048;

        U32 startpage, endpage;
#if defined(BLK_REMAP_PRO)
        startpage = RDT_RESULT_PAGE_NUM + BLK_REMAP_PRO_PAGENUM;
#else
        startpage = RDT_RESULT_PAGE_NUM;
#endif
        endpage = startpage + RESULT_PAGE_NUM_FOR_NEWSORTING;
        for(page =  startpage; page <  endpage; page = page + 2)
        {
            cache_area_dinval(DUMMY_DATA_BUF_ADDR, NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);
            cache_dummy_update_read();

            ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
            gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
            FCReadRedundant (ulMode, bank, 0, block_no, page, (DUMMY_DATA_BUF_PHY_ADDR), DRAM_DATA_SIZE, 1);
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            memcpy((U32*)TEMP_HBUF_ADDR, (U32*) (DUMMY_DATA_BUF_ADDR + 2052), 4);
            if(ret != ERR_OK)
            {
                ret = ERR_READ_RDT;
                continue;
            }
            if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
            {
                if (_REG32(TEMP_HBUF_ADDR) != 0x03fffff1)
                {
                    ret = ERR_READ_RDT;
                    continue;
                }
            }
            else if(cmp & ECC_UNCORRECT_ERR)
            {
                ret = ERR_READ_RDT_ECC;
                continue;
            }
            else
            {
                ret = ERR_READ_RDT;
                continue;
            }

            gul_FW_TAG = llfBETagSetting(TAG_READ, bank);
            FCReadRedundant (ulMode, bank, 0, block_no, page + 1, (DUMMY_DATA_BUF_PHY_ADDR + 0x2100),
                             DRAM_DATA_SIZE, 1);
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            if(ret != ERR_OK)
            {
                ret = ERR_READ_RDT;
                continue;
            }
            if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
            {
                if (_REG32(TEMP_HBUF_ADDR) != 0x03fffff1)
                {
                    ret = ERR_READ_RDT;
                    continue;
                }
            }
            else if(cmp & ECC_UNCORRECT_ERR)
            {
                ret = ERR_READ_RDT_ECC;
                continue;
            }
            else
            {
                ret = ERR_READ_RDT;
                continue;
            }

            ret = data_16k_dec(TEMP_BUF_ADDR, DUMMY_DATA_BUF_ADDR);

            if(ret != ERR_OK)
            {
                ret = ERR_READ_RDT;
                continue;
            }
            break;
        }
        gub_Total_len_per_2K = temp_2k_length;
        if(((cmp & BE_COMPLETION_ERROR_MASK) == 0) && (_REG32(TEMP_HBUF_ADDR) == 0x03fffff1))
        {
            break;
        }
#endif
    }

    if (block_no == ubSblkEnd)  // ret != ERR_OK
    {
        AddErrorMessage(bank, 3, ret); //3 means block0~block3 are all bad block
        return ret;
    }
#else
    for(bank_idx = 0; bank_idx < NandPara.ubBankNum; bank_idx++)
    {
        for(block_no = ubSblkStart; block_no < ubSblkEnd; block_no++)
        {
#ifdef IS_8K_PAGE
            for(page = (targetpage << 1); page < RDT_RESULT_PAGE_NUM; page += 64)
#else
            for(page = targetpage; page < RDT_RESULT_PAGE_NUM; page += 32)
#endif
            {
                cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);
                cache_dummy_update_read();

                ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
                gul_FW_TAG = llfBETagSetting(TAG_READ, bank_idx);
                for(i = 0; i <= ( RESULT_SIZE_PER_BANK / DRAM_DATA_SIZE - 1); i++)
                {
                    llfFCCmdRead_DRAM(ulMode, bank_idx, ubLunNo, block_no, page + i,
                                      TEMP_BUF_PHY_ADDR + DRAM_DATA_SIZE * i,
                                      DRAM_DATA_SIZE,
                                      TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
                    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                }

                if (ret != ERR_OK)
                {
                    ret = ERR_READ_RDT_TIMEOUT;
                    continue;
                }
                if((cmp & BE_COMPLETION_ERROR_MASK) == 0)
                {
                    if (_REG32(TEMP_HBUF_ADDR) != RDT_BLK_ID)
                    {
                        ret = ERR_READ_RDT;
                        continue;
                    }
                    RDT_find = 1;
                    llfprintk("Bank%d_Block%d_page%d for result bank%d \r\n", bank_idx, block_no, page, bank);
                }
                else if(cmp & ECC_UNCORRECT_ERR)
                {
                    ret = ERR_READ_RDT_ECC;
                    continue;
                }
                else
                {
                    ret = ERR_READ_RDT;
                    continue;
                }

                if(RDT_find)
                {
                    return ERR_OK;
                }
            }

        }

    }

    if(!RDT_find)
    {
        llfAddErrorMessage(bank_idx, 4, ret); //3 means block0~block3 are all bad block
        llfDbgPrintk(ALWAYS_MSG, "bank %d RDT nofound\r\n", bank);
        return ret;
    }
#endif
    // parsing RDT bank result from target address
    return ERR_OK;
}

U32 llfLoadRDTResult()
{
    U8 bank, passNum, existNum, gubLdpcCodeRateTemp, IsfromOriRDT = true;
#ifdef SBLK_EXPAND
#if defined(RL6643_VA)
    U32 value_temp;
#endif
#endif

#ifdef EXTEND_STATIC_DBT
    U8 existStaticDBT = 0;
#endif

    llfInitErrorMessage();
    passNum = 0;
    existNum = 0;
#if defined(RL6531_VB)
    gubLdpcCodeRateTemp = gubECC_CFG;
#else
    gubLdpcCodeRateTemp = gubLdpcCodeRate;
#endif

#if defined(RL6643_FPGA)||defined(RL6643_VA)
    Change_ldpc(3);
#elif defined(RL6531_VB)
    Change_ldpc(0);
#else
    Change_ldpc(5);
#endif
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    if(gubLLFMode == LLF_DEFECT_BAD_BLOCK)
    {
        IsfromOriRDT = false;
        printk("[LLF] Inherit orig RDT result.\r\n");
    }
#endif

#ifdef EXTEND_STATIC_DBT
    for (bank = 0; bank < BANK_NUM_MAX; bank++)
        gulBankRdtResultTag[bank] = 0;
#endif

    for (bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        U32 ret;
        ret = llfGetRDTBankResult(false, bank, IsfromOriRDT);
        printk("[LLF] Get RDT result By Bank%d ret=%x,\r\n", bank, ret);

#ifdef EXTEND_STATIC_DBT
        gulBankRdtResultTag[bank] = _REG32(TEMP_BUF_ADDR + RDT_BANK_RDTRESULTTAG_OFFSET);
        llfprintk("StaticDBT read bank %d RdtResultTag %x in Dword126\r\n", bank,
                  gulBankRdtResultTag[bank]);
        if(((gulBankRdtResultTag[bank] >> RDT_RESULT_TAG_SHIFT) & 0xFF) == 0xAA)
        {
            existStaticDBT++;
        }
        if(ret == ERR_OK)
            llfCheckRDTPlaneAndDBTSize(TEMP_BUF_ADDR);
#endif
    }

#ifdef EXTEND_STATIC_DBT
#ifdef SBLK_EXPAND
#if defined(RL6643_VA)
    value_temp = _REG32(TEMP_BUF_ADDR + RDT_SBLK_START_TAG_OFFSET);
    gulSblkCHCEMap[0] = _REG32(TEMP_BUF_ADDR + RDT_SBLK_CHCE_MAP1_OFFSET);
    gulSblkCHCEMap[1] = _REG32(TEMP_BUF_ADDR + RDT_SBLK_CHCE_MAP2_OFFSET);

    if((value_temp & 0xff) == TAG_FOR_SBLK_EXPAND)
    {
        gubSblkStart = (value_temp >> 8) & 0xff;
        gubSblkBankStart = (value_temp >> 16) & 0xff;
        llfprintk("gubSblkBankStart=%d, gubSblkStart=%d, \r\n", gubSblkBankStart, gubSblkStart);
        llfprintk("gulSblkCHCEMap[0] = %x, gulSblkCHCEMap[0] = %x\r\n", gulSblkCHCEMap[0],
                  gulSblkCHCEMap[1]);
    }
#endif
#endif

    if (existStaticDBT != NandPara.ubBankNum)
    {
        if(gubNeedRebuildRemap == 1)
            return ERR_READ_RDT;
    }
    else
    {
        llfCheckBankRdtResultTag();
        llfprintk("StaticDBT gubNeedRebuildRemap %d\r\n", gubNeedRebuildRemap);
    }
#endif

#ifdef SBLK_EXPAND
#if defined(RL6643_VA)
    value_temp = _REG32(TEMP_BUF_ADDR + RDT_SBLK_START_TAG_OFFSET);
    gulSblkCHCEMap[0] = _REG32(TEMP_BUF_ADDR + RDT_SBLK_CHCE_MAP1_OFFSET);
    gulSblkCHCEMap[1] = _REG32(TEMP_BUF_ADDR + RDT_SBLK_CHCE_MAP2_OFFSET);

    if((value_temp & 0xff) == TAG_FOR_SBLK_EXPAND)
    {
        gubSblkStart = (value_temp >> 8) & 0xff;
        gubSblkBankStart = (value_temp >> 16) & 0xff;
        llfprintk("gubSblkBankStart=%d, gubSblkStart=%d, \r\n", gubSblkBankStart, gubSblkStart);
        llfprintk("gulSblkCHCEMap[0] = %x, gulSblkCHCEMap[0] = %x\r\n", gulSblkCHCEMap[0],
                  gulSblkCHCEMap[1]);
    }
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    if((gubLLFMode != LLF_DEFECT_BAD_BLOCK) && (gubTSblk >= gubSblkStart + SYS_BLK))
    {
        return ERR_READ_RDT;
    }
#endif
#endif
#endif

    for (bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        if (llfGetRDTBankResult(false, bank, IsfromOriRDT) == ERR_OK)
        {
            if(_REG32(TEMP_BUF_ADDR) == ERR_OK || _REG32(TEMP_BUF_ADDR) == 0xFFFFFFFF
                    || _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_RDT_RESULTFAILINHERITEN))
            {
                existNum++;
            }
            if(llfParseRDTResult(bank, TEMP_BUF_ADDR) == ERR_OK)
            {
                passNum++;
            }
        }
    }
    Change_ldpc(gubLdpcCodeRateTemp);

#ifdef KEEP_ORIG_DBT
    memset((void*)ORIG_DBT_ADDR, 0x0, ORIG_DBT_SIZE);
    cache_area_dwb(ORIG_DBT_ADDR, ORIG_DBT_SIZE);
    cache_dummy_update_read();
    memcpy((void*)(ORIG_DBT_ADDR), (void*)(DBT_ADDR), ORIG_DBT_SIZE);
    cache_area_dwb(ORIG_DBT_ADDR, ORIG_DBT_SIZE);
    cache_dummy_update_read();
#endif

    if (passNum != NandPara.ubBankNum)
    {
        return ERR_READ_RDT;
    }
    if (existNum == NandPara.ubBankNum)
    {
        gfDBTInitDone = LLF_DBT_RDT;
    }

    return ERR_OK;
}

U32 llfReadSBlock(U32 sblk_addr, U32 header_addr, U8 bank_no, U16 block_no, U16 page)
{
    U32 ret = 0xAABBCCDD;
    U32 ulSBlkBid;
    U8 *ubSBlkTag = (U8 *)("RTSANSSD");
    U32 ulAddr = sblk_addr;
    U32 ulMode;
    U32 cmp;
    U8 ubLunNo, i;
    U8 ubSblkStart = 0;
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif

    ubLunNo = bank_no / NandPara.ubBankNumPerLun;

    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);
    llfFCCmdRead_DRAM(ulMode, bank_no, ubLunNo, block_no, page, sblk_addr - UNCACHED_PHY_TO_VA_OFFSET,
                      DRAM_DATA_SIZE, header_addr - UNCACHED_PHY_TO_VA_OFFSET, DRAM_HEAD_SIZE);

    cache_area_dinval(header_addr, DRAM_HEAD_SIZE);
    cache_dummy_update_read();
    cache_area_dinval(sblk_addr, DRAM_DATA_SIZE);
    cache_dummy_update_read();
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_FIO_TIMEOUT)
    {
        ret = ERR_READ_SBLK_TIMEOUT;
    }
    else if(ret == ERR_OK)
    {
        if(!(block_no < ubSblkStart + SYS_BLK && block_no >= ubSblkStart))
        {
#ifdef SLC_RRT
            ret = llfReadRetry(bank_no, block_no, page, &cmp,
                               sblk_addr - UNCACHED_PHY_TO_VA_OFFSET, header_addr - UNCACHED_PHY_TO_VA_OFFSET);
            if (ret != ERR_OK)
            {
                ret = ERR_READ_SBLK;
            }
#endif
        }
        if((cmp & BE_COMPLETION_ERROR_MASK) == FC_COMPLETION_NO_ERR)
        {
            ulSBlkBid = _REG32(header_addr);
            llfDbgPrintk(ALWAYS_MSG, "SBlk ID %x bank = %d  block = %d, page = %d\r\n", ulSBlkBid, bank_no,
                         block_no, page);
            if(ulSBlkBid == SBLK_BLK_ID)
            {
                i = 8;
                while(i--)   //check sblk tag "RTSANSSD"
                {
//					printk("tag:%c,sblk:%c\r\n",_REG08(ubSBlkTag),_REG08(ulAddr));
                    if(_REG08(ubSBlkTag++) != _REG08(ulAddr++))
                    {
                        ret = ERR_READ_SBLK;
                        break;
                    }
                    ret = ERR_OK;
                }
            }
            else
            {
                ret = ERR_READ_SBLK;
            }
        }
        else
        {
            //llfprintk("Read SBLK bank = %d block = %d page = %d, cmp = %x\r\n", bank_no, block_no, page, cmp);
            if(cmp & ALL_FF)
            {
                ret = ERR_EMPTY;
            }
            else if(cmp & (ECC_UNCORRECT_ERR | CRC_ERR))
            {
                ret = ERR_ECC;
            }
            else
            {
                ret = ERR_READ_SBLK_ECC;
            }
        }
    }
    else
    {
        //llfprintk("Read SBLK bank = %d block = %d page = %d, ret = %x\r\n", bank_no, block_no, page, ret);
    }

    return ret;
}
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DQV5) || defined(FTL_H3DTV7)
U32 llfHynixReadSBlock(U32 sblk_addr, U32 header_addr, U8 bank_no, U16 block_no, U16 page)
{
    U32 ret = 0xAABBCCDD;
    U32 ulSBlkBid;
    U8 *ubSBlkTag = (U8 *)("RTSANSSD");
    U32 ulAddr = sblk_addr;
    U32 ulMode;
    U32 cmp;
    U8 ubLunNo, i;
    U8 ubSblkStart = 0;
#ifdef SBLK_EXPAND
    ubSblkStart = gubSblkStart;
#endif

    ubLunNo = bank_no / (NandPara.ubChNum * NandPara.ubCENumPerCh);

    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);
    llfFCCmdHynixRead(ulMode, bank_no, ubLunNo, block_no, page, sblk_addr - UNCACHED_PHY_TO_VA_OFFSET,
                      DRAM_DATA_SIZE, header_addr - UNCACHED_PHY_TO_VA_OFFSET, DRAM_HEAD_SIZE);

    cache_area_dinval(header_addr, DRAM_HEAD_SIZE);
    cache_dummy_update_read();
    cache_area_dinval(sblk_addr, DRAM_DATA_SIZE);
    cache_dummy_update_read();

    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_FIO_TIMEOUT)
    {
        ret = ERR_READ_SBLK_TIMEOUT;
    }
    else if(ret == ERR_OK)
    {
        if(!(block_no < ubSblkStart + SYS_BLK && block_no >= ubSblkStart))
        {
#ifdef SLC_RRT
            ret = llfReadRetry(bank_no, block_no, page, &cmp,
                               sblk_addr - UNCACHED_PHY_TO_VA_OFFSET, header_addr - UNCACHED_PHY_TO_VA_OFFSET);
            if (ret != ERR_OK)
            {
                ret = ERR_READ_SBLK;
            }
#endif
        }
        if((cmp & BE_COMPLETION_ERROR_MASK) == FC_COMPLETION_NO_ERR)
        {
            ulSBlkBid = _REG32(header_addr);
            llfDbgPrintk(ALWAYS_MSG, "SBlk ID %x bank = %d  block = %d, page = %d\r\n", ulSBlkBid, bank_no,
                         block_no, page);
            if(ulSBlkBid == SBLK_BLK_ID)
            {
                i = 8;
                while(i--)   //check sblk tag "RTSANSSD"
                {
//					printk("tag:%c,sblk:%c\r\n",_REG08(ubSBlkTag),_REG08(ulAddr));
                    if(_REG08(ubSBlkTag++) != _REG08(ulAddr++))
                    {
                        ret = ERR_READ_SBLK;
                        break;
                    }
                    ret = ERR_OK;
                }
            }
            else
            {
                ret = ERR_READ_SBLK;
            }
        }
        else
        {
            //llfprintk("Read SBLK bank = %d block = %d page = %d, cmp = %x\r\n", bank_no, block_no, page, cmp);
            ret = ERR_READ_SBLK_ECC;
        }
    }
    else
    {
        //llfprintk("Read SBLK bank = %d block = %d page = %d, ret = %x\r\n", bank_no, block_no, page, ret);
    }

    return ret;
}
#endif

U32 llfSearchDynamicSBlock(U32 static_sblk_addr, U32 dynamic_sblk_addr, U8 tag)
{
    U16 bank, targetBank;
    U16 block, targetBlock;
    U16 page = 0, page_down, page_up;
    U32 wCnt, targetWCnt;
    U32 ret = ERR_OK;
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DQV5) || defined(FTL_H3DTV7)
    U32 (*llfReadSBlock_cb)(U32, U32, U8, U16, U16) = &llfHynixReadSBlock;
#else
    U32 (*llfReadSBlock_cb)(U32, U32, U8, U16, U16) = &llfReadSBlock;
#endif
    // get dynamic SBlock location
    if(tag == STATIC_SBLK_INFO)
    {
        guwCaclDynSBlockBegin = _REG16(static_sblk_addr + SBLK_OFFSET_SYSTEM_BLOCK_BEGIN_INDEX);
        guwCaclDynSBlockEnd = _REG16(static_sblk_addr + SBLK_OFFSET_SYSTEM_BLOCK_END_INDEX);
        if(_REG08(static_sblk_addr + SBLK_OFFSET_VTH_ENABLE) == 0x4f)
        {
            memcpy((void *) SBLK_ADDR + SBLK_OFFSET_VTH_BANK0,
                   (void *)(static_sblk_addr + SBLK_OFFSET_VTH_BANK0),
                   BANK_NUM_MAX);
            gubVthEnNum = NandPara.ubBankNum;
        }
        //cache_area_dinval(SBLK_ADDR + SBLK_OFFSET_VTH_BANK0, BANK_NUM_MAX);
        //cache_dummy_update_read();
    }
    else if(tag == DYNAMIC_SBLK_INFO)
    {
        guwCaclDynSBlockBegin = _REG16(static_sblk_addr + SBLK_OFFSET_DYNAMIC_SBLK_BEGIN);
        guwCaclDynSBlockEnd = _REG16(static_sblk_addr + SBLK_OFFSET_DYNAMIC_SBLK_END);
        if(_REG08(static_sblk_addr + SBLK_OFFSET_VTH_ENABLE) == 0x4f)
        {
            memcpy((void *) SBLK_ADDR + SBLK_OFFSET_VTH_BANK0,
                   (void *)(static_sblk_addr + SBLK_OFFSET_VTH_BANK0),
                   BANK_NUM_MAX);
            gubVthEnNum = NandPara.ubBankNum;
        }
        memcpy((void *) (SBLK_ADDR + SBLK_OFFSET_RDT_TESTED_ROUND),
               (void *)(static_sblk_addr + SBLK_OFFSET_RDT_TESTED_ROUND), 8);
    }

    if (guwCaclDynSBlockBegin > guwCaclDynSBlockEnd || guwCaclDynSBlockEnd > NandPara.uwBlockNumPerCE)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Wrong range of dynamic SBlock\r\n");
        return ERR_READ_SBLK;
    }

    targetWCnt = 0;

    // load last dynamic SBlock
    for (block = guwCaclDynSBlockBegin; block < guwCaclDynSBlockEnd; block++)
    {
        for (bank = 0; bank < NandPara.ubBankNum; bank++)
        {
            if (llfIsSystemBlkDefect(static_sblk_addr + SBLK_OFFSET_SYSTEM_BLOCK_DBT, bank, block))
            {
                continue;
            }
            if (llfReadSBlock_cb(dynamic_sblk_addr, TEMP_HBUF_ADDR, bank, block, 0) != ERR_OK)
            {
                continue;
            }
            if (_REG32(TEMP_HBUF_ADDR) != SBLK_BLK_ID)
            {
                llfDbgPrintk(ALWAYS_MSG, "[ERR] SBLK Fail ID %x ret %x bank %d block %d page %d \r\n",
                             _REG32(TEMP_HBUF_ADDR), ret, bank, block, page);
                continue;
            }
            wCnt = _REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLOCK_WIRTE_CNT);
            llfDbgPrintk(ALWAYS_MSG, "Find Dynamic Block %d bank %d Wcnt %x \r\n", block, bank, wCnt);
            if (wCnt >= targetWCnt)
            {
                targetWCnt = wCnt;
                targetBlock = block;
                targetBank = bank;
            }
        }
    }

    if (targetWCnt == 0)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Can't find any dynamic SBlock\r\n");
        return ERR_READ_SBLK;
    }

    llfDbgPrintk(ALWAYS_MSG, "Start Read Target Dynamic SBlk %d bank %d \r\n", targetBlock, targetBank);
    // this should be kind of ASSERT
    ret = llfReadSBlock_cb(dynamic_sblk_addr, TEMP_HBUF_ADDR, targetBank, targetBlock, 0);
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Read target dynamic SBlock failed Bank %d Block %d ret %x\r\n",
                     targetBank, targetBlock, ret);
        return ERR_READ_SBLK;
    }
    U16 sbank, backupbank, sblock, backupblock;
    sbank = _MEM16(dynamic_sblk_addr + SBLK_OFFSET_SBLOCK_BANK);
    backupbank = _MEM16(dynamic_sblk_addr + SBLK_OFFSET_BACKUP_BANK);
    sblock = _MEM16(dynamic_sblk_addr + SBLK_OFFSET_SBLOCK_BLOCK);
    backupblock = _MEM16(dynamic_sblk_addr + SBLK_OFFSET_BACKUP_BLOCK);

    llfprintk("sbank= %d backupbank = %d sblock = %d backupblock = %d\r\n", sbank, backupbank, sblock,
              backupblock);

    if ((targetBank != sbank && targetBank != backupbank) ||
            (targetBlock != sblock && targetBlock != backupblock))
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Mismatch target bank/block in dynamic SBlock\r\n");
        return ERR_READ_SBLK;
    }
    targetBank = sbank;
    targetBlock = sblock;

#ifdef SBLK_EXPAND
    //load new sblk info from dynamic ablk
    gubSblkStart = (_REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLK_START_TAG) >> 8) & 0xff;
    gubSblkBankStart = (_REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLK_START_TAG) >> 16) & 0xff;
    gulSblkCHCEMap[0] = _REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLK_CHCEMAP_CH0);
    gulSblkCHCEMap[1] = _REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLK_CHCEMAP_CH1);

    llfprintk("[Sblk][SearchDynamicSBlock]gubSblkStart = %d, gubSblkBankStart = %d\r\n", gubSblkStart,
              gubSblkBankStart);
    llfprintk("[Sblk][SearchDynamicSBlock]gulSblkCHCEMap[0] = %x, [1] = %x\r\n", gulSblkCHCEMap[0],
              gulSblkCHCEMap[1]);
#endif

    // use binary search to find target page
    page_down = 0;

#ifdef IS_8K_PAGE
    U8 ub8kPageShift = 1;
#else
    U8 ub8kPageShift = 0;
#endif
    page_up = NandPara.uwSLCPageNumPerBlock >> ub8kPageShift;

    while ((page_up - 1) > page_down)
    {
        page = ((page_up + page_down) >> 1) << ub8kPageShift;
        ret = llfReadSBlock_cb(dynamic_sblk_addr, TEMP_HBUF_ADDR, targetBank, targetBlock, page);
        if (ret != ERR_OK)
        {
            page_up = page >> ub8kPageShift;
            continue;
        }

        wCnt = _REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLOCK_WIRTE_CNT); // get SBlock Write counter
        if (wCnt < targetWCnt)  // ASSERT(wCnt >= targetWCnt);
        {
#if defined(FTL_N38B) || defined(FTL_Q5171A) // N38B only erase deck before program
            page_up = (page / INTELQ_PAGE_PER_SECTION_SLC * INTELQ_PAGE_PER_SECTION_SLC) >> ub8kPageShift;
            continue;
#else
            llfDbgPrintk(ALWAYS_MSG, "[ERR] Can't find target dynamic SBlock wCnt 0x%x_0x%x\r\n",
                         wCnt, targetWCnt);
            return ERR_READ_SBLK;
#endif
        }
        targetWCnt = wCnt;
        page_down = page >> ub8kPageShift;
    }

    // page_down is the final answer
    llfDbgPrintk(ALWAYS_MSG, "pageno %d is final target snapshot page\r\n", page_down << ub8kPageShift);
    ret = llfReadSBlock_cb(dynamic_sblk_addr, TEMP_HBUF_ADDR, targetBank, targetBlock,
                           page_down << ub8kPageShift);
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[WARN] target read error %d_%d_%d, use backup %d_%d\r\n",
                     targetBank, targetBlock, page_down << ub8kPageShift, backupbank, backupblock);
        targetBank = backupbank;
        targetBlock = backupblock;
        page = 0;
        page_down = 0;
        page_up = NandPara.uwSLCPageNumPerBlock >> ub8kPageShift;

        while ((page_up - 1) > page_down)
        {
            page = ((page_up + page_down) >> 1) << ub8kPageShift;
            ret = llfReadSBlock_cb(dynamic_sblk_addr, TEMP_HBUF_ADDR, targetBank, targetBlock, page);
            if (ret != ERR_OK)
            {
                page_up = page >> ub8kPageShift;
                continue;
            }

            wCnt = _REG32(dynamic_sblk_addr + SBLK_OFFSET_SBLOCK_WIRTE_CNT); // get SBlock Write counter
            if (wCnt < targetWCnt)  // ASSERT(wCnt >= targetWCnt);
            {
#if defined(FTL_N38B) || defined(FTL_Q5171A) // N38B only erase deck before program
                page_up = (page / INTELQ_PAGE_PER_SECTION_SLC * INTELQ_PAGE_PER_SECTION_SLC) >> ub8kPageShift;
                continue;
#else
                llfDbgPrintk(ALWAYS_MSG, "[ERR] Can't find target dynamic SBlock wCnt 0x%x_0x%x\r\n",
                             wCnt, targetWCnt);
                return ERR_READ_SBLK;
#endif
            }
            targetWCnt = wCnt;
            page_down = page >> ub8kPageShift;
        }
        ret = llfReadSBlock_cb(dynamic_sblk_addr, TEMP_HBUF_ADDR, targetBank, targetBlock,
                               page_down << ub8kPageShift);
    }
#ifdef LLF_AUTO_RMA
    RmaPrintk(PRINT_BOTH, PRINT_UR_FILE,
              "[Info] Dynamic sblk %d_%d target %d_%d backup %d_%d page %d\r\n",
              sbank, sblock, targetBank, targetBlock, backupbank, backupblock, page_down << ub8kPageShift);
#endif

    if (ret != ERR_OK)
    {
        return ERR_READ_SBLK;
    }
    return ERR_OK;
}

U32 llfReadDBTFromStaticSblock(U8 bank_no, U16 block)
{
    U8 page;
    U32 ulMode;
    U32 cmp, ret;
    U32 ulSBlkBid;
    U8 lun_no;

    lun_no = bank_no / NandPara.ubBankNumPerLun;

    for(page = DBT_PAGENO; page < DBT_PAGENO + DBT_PAGENUM; page++)
    {
        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
        gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);
        llfFCCmdRead_DRAM(ulMode, bank_no, lun_no, block, page,
                          (DBT_PHY_ADDR + (page - DBT_PAGENO) * DRAM_DATA_SIZE), DRAM_DATA_SIZE,
                          TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);

        cache_area_dinval(TEMP_HBUF_ADDR, DRAM_HEAD_SIZE);
        cache_dummy_update_read();
        cache_area_dinval(DBT_ADDR, DRAM_DATA_SIZE);

        cache_dummy_update_read();
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret == ERR_FIO_TIMEOUT)
        {
            ret = ERR_READ_SBLK_TIMEOUT;
        }
        else if(ret == ERR_OK)
        {
            if((cmp & BE_COMPLETION_ERROR_MASK) == FC_COMPLETION_NO_ERR)
            {
                ulSBlkBid = _REG32(TEMP_HBUF_ADDR);
                // llfDbgPrintk(ALWAYS_MSG, "SBlk ID %x \r\n", ulSBlkBid);
                if(ulSBlkBid == CODE_BLK_ID)
                {
                    ret = ERR_OK;
                }
                else
                {
                    ret = ERR_READ_DBT;
                }
            }
            else
            {
                ret = ERR_READ_DBT_ECC;
            }
        }

        if(ret != ERR_OK)
            return ret;

    }

    return ERR_OK;
}

#ifdef VIRTUAL_BANK
void llfCalcVirtualNum(U16 uwRealMpBlockNum, U32 DynamicSblkAddr)
{
    guwVirtualBsStart = _MEM16(SBLK_ADDR + SBLK_OFFSET_VIRTUAL_BS_START);
    gubVirtualBankNum = _REG08(SBLK_ADDR + SBLK_OFFSET_VIRTUAL_BANK_NUM);
    if(gubVirtualBankNum == 0 || gubVirtualBankNum < NandPara.ubBankNum)
    {
        gubVirtualBankNum = NandPara.ubBankNum;
        while(gubVirtualBankNum * NandPara.ubPlaneNumPerLun < 32)
        {
            gubVirtualBankNum *= 2;
        }
    }

    U32 ulMulNum = (gubVirtualBankNum / NandPara.ubBankNum);
    if(guwVirtualBsStart == 1)
    {
        guwVirtualBsStart = _REG16(DynamicSblkAddr + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX);
    }
    else if(guwVirtualBsStart == 0)
    {
        guwVirtualBsStart = _REG16(DynamicSblkAddr + SBLK_OFFSET_L2P_MP_BLOCK_END_INDEX);
    }
    guwVirtualBsNum = guwVirtualBsStart + (uwRealMpBlockNum - guwVirtualBsStart +
                                           (ulMulNum - 1)) / ulMulNum;
}
#endif

void llfVirtualBankV2P(U8 ubVirBank, U16 uwVirBs, U8 *pPhyBank, U16 *pPhyBs)
{
#ifdef VIRTUAL_BANK
    if(uwVirBs >= guwVirtualBsStart)
    {
        *pPhyBs = guwVirtualBsStart + (uwVirBs - guwVirtualBsStart) * (gubVirtualBankNum /
                  NandPara.ubBankNum) +
                  (ubVirBank / NandPara.ubBankNum);
        *pPhyBank = ubVirBank % NandPara.ubBankNum;
    }
    else
    {
        *pPhyBank = ubVirBank;
        *pPhyBs = uwVirBs;
    }
#else
    *pPhyBank = ubVirBank;
    *pPhyBs = uwVirBs;
#endif
}

void llfVirtualBankGetBankNum(U8 *ubBankNum, U16 uwBsIndex)
{
#ifdef VIRTUAL_BANK
    if(uwBsIndex >= guwVirtualBsStart)
    {
        *ubBankNum = gubVirtualBankNum;
    }
    else
    {
        *ubBankNum = NandPara.ubBankNum;
    }
#else
    *ubBankNum = NandPara.ubBankNum;
#endif
}

U32 llfReadBlockInfo(U32 blockInfo_addr, U32 SnapshotStart, U8 ubBSInfoPageOffset)
{
    U32 ulMode;
    U32 DataLen;
    U32 cmp, ret;
    U8 bank, i, lun_no;
    U16 page_num, page;
    U16 block, uwRealMpBlockNum;
    U16 SP_page_num, pagelimit;
#ifdef HANDLE_BLK_INFO_BEYOND_64K
    U16 ub64KRealBlk;
    U8 cnt = 0, loop = 0, limitOffset = 0;

    memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
#endif
    for(bank = 0; bank < NandPara.ubBankNum; bank ++)
    {
        _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + bank * SHORT_BYTE_SIZE) = 0;
    }
    uwRealMpBlockNum = NandPara.uwMpBlockNumPerLun;
#ifdef BLK_REMAP_PRO
    if(gubBlkRemapProFlag == 1)
    {
        uwRealMpBlockNum = guwRealMPBlkNum;
    }
#endif
#ifdef VIRTUAL_BANK
    llfCalcVirtualNum(uwRealMpBlockNum, blockInfo_addr);
    printk("Virtual Num bs %d start %d bank %d\r\n", guwVirtualBsNum, guwVirtualBsStart,
           gubVirtualBankNum);
    uwRealMpBlockNum = guwVirtualBsNum;
#endif
    SP_page_num = ((uwRealMpBlockNum * BS_INFO_SIZE)
                   + (NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT) - 1) /
                  (NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);

    pagelimit = (TEMP_BUF_SIZE / (NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT));
#ifdef HANDLE_BLK_INFO_BEYOND_64K
    ub64KRealBlk = TEMP_BUF_SIZE / BS_INFO_SIZE;
#endif
    if(SP_page_num > pagelimit)
    {
        llfDbgPrintk(ALWAYS_MSG, "We can not parsing more than %d page info, need %d\r\n", pagelimit,
                     SP_page_num);
#ifndef HANDLE_BLK_INFO_BEYOND_64K
        return ERR_READ_BLOCKINFO;
#endif
    }
    page_num = (SP_page_num + NandPara.ubPlaneNumPerLun - 1) / NandPara.ubPlaneNumPerLun;


    // blockinfo should NOT larger than a BS!
    if (page_num >= (NandPara.uwPageNumPerBlock * NandPara.ubBankNum))
    {
        llfDbgPrintk(ALWAYS_MSG, "Impossible blkInfo size %d\r\n", BS_INFO_SIZE);
        return ERR_READ_BLOCKINFO;
    }
    cache_area_dinval(blockInfo_addr, (page_num * NandPara.ulLastMpPageByteNum));
    cache_dummy_update_read();


    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    DataLen = NandPara.ubSectorNumPerPage * SECTOR_BYTE_SIZE;
    block = ((SnapshotStart & SNAPSHOT_BLOCK_MASK_LLF) * NandPara.ubPlaneNumPerLun);
    page = (U16)((SnapshotStart >> SNAPSHOT_PAGE_SHIFT_LLF)&
                 SNAPSHOT_PAGE_MASK_LLF) + ubBSInfoPageOffset;
    bank = ((SnapshotStart >> SNAPSHOT_BANK_SHIFT) & SNAPSHOT_BANK_MASK_LLF);
    lun_no = bank / NandPara.ubBankNumPerLun;

    llfDbgPrintk(ALWAYS_MSG, "Read snapshot %d_%d_%d_%d_%d\r\n", block / NandPara.ubPlaneNumPerLun,
                 bank, lun_no, block, page);

// TODO: now we just have one page size space for bsinfo, so need single operation
#ifdef HANDLE_BLK_INFO_BEYOND_64K
    for (i = 0; i < SP_page_num ; i++)
    {
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DQV5) || defined(FTL_H3DTV7)
        llfFCCmdHynixRead(ulMode, bank, lun_no, block + (i % NandPara.ubPlaneNumPerLun),
                          page + (i / NandPara.ubPlaneNumPerLun),
                          blockInfo_addr + (i - limitOffset) * DataLen - UNCACHED_PHY_TO_VA_OFFSET, DataLen,
                          TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
#else
        llfFCCmdRead_DRAM(ulMode, bank, lun_no, block + (i % NandPara.ubPlaneNumPerLun),
                          page + (i / NandPara.ubPlaneNumPerLun),
                          blockInfo_addr + (i - limitOffset) * DataLen - UNCACHED_PHY_TO_VA_OFFSET, DataLen,
                          TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
#endif
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret != ERR_OK)
        {
            return ERR_READ_DBT_TIMEOUT;
        }
#ifndef SLC_RRT
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "read blockInfo error: %d_%d_%d cmp:%x\r\n", bank, i, 0, cmp);
            return ERR_READ_SBLK;
            //ASSERT(LLF_MSG, 0);
        }
#else
        ret = llfReadRetry(bank, block + (i % NandPara.ubPlaneNumPerLun),
                           page + (i / NandPara.ubPlaneNumPerLun), &cmp,
                           blockInfo_addr + (i - limitOffset) * DataLen - UNCACHED_PHY_TO_VA_OFFSET, TEMP_HBUF_PHY_ADDR);
        if (ret != ERR_OK)
        {
            return  ERR_READ_SBLK;
        }
#endif
        cache_area_dinval(TEMP_HBUF_ADDR, DRAM_HEAD_SIZE);
        cache_dummy_update_read();

        // check snapshot header ID
        if (_REG32(TEMP_HBUF_ADDR) != SNAP_BLK_ID)
        {
            return ERR_READ_BLOCKINFO;
        }
        if(cnt == TEMP_BUF_SIZE / (NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT) - 1)
        {
            ret = llfParseBlockInfoToDBT(blockInfo_addr, ub64KRealBlk, ub64KRealBlk * loop);
            printk("parser time %d\r\n", loop);
            if (ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "[ERR] Parse BlockInfo to RDT failed\r\n");
                return ret;
            }
            cnt = 0;
            loop++;
            limitOffset = pagelimit * loop;
        }
        else
        {
            cnt++;
        }
    }
    printk("last parser time %d %d %d\r\n", loop, cnt, i);
    if(cnt != 0 && i == SP_page_num)
    {
        ret = llfParseBlockInfoToDBT(blockInfo_addr, uwRealMpBlockNum - (ub64KRealBlk * loop),
                                     ub64KRealBlk * loop);
        if (ret != ERR_OK)
        {
            llfDbgPrintk(ALWAYS_MSG, "[ERR] Parse BlockInfo to RDT failed\r\n");
            return ret;
        }
        cnt = 0;
    }
#else
    for (i = 0; i < SP_page_num ; i++)
    {
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DQV5) || defined(FTL_H3DTV7)
        llfFCCmdHynixRead(ulMode, bank, lun_no, block + (i % NandPara.ubPlaneNumPerLun),
                          page + (i / NandPara.ubPlaneNumPerLun),
                          blockInfo_addr + i * DataLen - UNCACHED_PHY_TO_VA_OFFSET, DataLen,
                          TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
#else
        llfFCCmdRead_DRAM(ulMode, bank, lun_no, block + (i % NandPara.ubPlaneNumPerLun),
                          page + (i / NandPara.ubPlaneNumPerLun),
                          blockInfo_addr + i * DataLen - UNCACHED_PHY_TO_VA_OFFSET, DataLen,
                          TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
#endif
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret != ERR_OK)
        {
            return ERR_READ_DBT_TIMEOUT;
        }
#ifndef SLC_RRT
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "read blockInfo error: %d_%d_%d cmp:%x\r\n", bank, i, 0, cmp);
            return ERR_READ_SBLK;
            //ASSERT(LLF_MSG, 0);
        }
#else
        ret = llfReadRetry(bank, block + (i % NandPara.ubPlaneNumPerLun),
                           page + (i / NandPara.ubPlaneNumPerLun), &cmp,
                           blockInfo_addr + i * DataLen - UNCACHED_PHY_TO_VA_OFFSET, TEMP_HBUF_PHY_ADDR);
        if (ret != ERR_OK)
        {
            return  ERR_READ_SBLK;

        }
#endif
        cache_area_dinval(TEMP_HBUF_ADDR, DRAM_HEAD_SIZE);
        cache_dummy_update_read();

        // check snapshot header ID
        if (_REG32(TEMP_HBUF_ADDR) != SNAP_BLK_ID)
        {
            return ERR_READ_BLOCKINFO;
        }
    }
#endif

    return ERR_OK;
}

#ifdef HANDLE_BLK_INFO_BEYOND_64K
U32 llfParseBlockInfoToDBT(U32 blockInfo_addr, U16 deltaMpBlockNum, U16 startBlk)
#else
U32 llfParseBlockInfoToDBT(U32 blockInfo_addr)
#endif
{
    U8 bank, planenum, glsysblkNum;
    U16 mpblock, uwRealMpBlockNum, singleBlock;
    U8 ubBankNum = NandPara.ubBankNum;
    U16 uwPhyBs;
    U8 ubPhyBank;
    //U32 tempBS;
    // U32 offset;

    // Clear DBT
#ifndef HANDLE_BLK_INFO_BEYOND_64K
    memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
#endif
    //memset((void*)ECT_ADDR, 0x0, ECT_SIZE);
    //offset = ECT_ADDR;
#ifdef SBLK_EXPAND
    glsysblkNum = SYS_BLK + EXTEND_SYS_BLK;
#else
    glsysblkNum = SYS_BLK;
#endif

    uwRealMpBlockNum = NandPara.uwMpBlockNumPerLun;
#ifdef BLK_REMAP_PRO
    if(gubBlkRemapProFlag == 1)
    {
        uwRealMpBlockNum = guwRealMPBlkNum;
    }
#endif
#ifdef HANDLE_BLK_INFO_BEYOND_64K
    uwRealMpBlockNum = startBlk + deltaMpBlockNum;
    printk("Start MP %d end %d\r\n", startBlk, uwRealMpBlockNum);

    for (mpblock = startBlk; mpblock < uwRealMpBlockNum; mpblock++)
#else
#if defined(VIRTUAL_BANK)
    uwRealMpBlockNum = guwVirtualBsNum;
#endif
    for (mpblock = 0; mpblock < uwRealMpBlockNum; mpblock++)
#endif
    {
        llfVirtualBankGetBankNum(&ubBankNum, mpblock);
        // record Defect Block Table
        for (bank = 0; bank < ubBankNum; bank++)
        {
#ifdef HANDLE_BLK_INFO_BEYOND_64K
            if (llfBSCheckSinglePlaneBadBlock(blockInfo_addr, mpblock - startBlk, bank))
#else
            if (llfBSCheckSinglePlaneBadBlock(blockInfo_addr, mpblock, bank))
#endif
            {
                llfVirtualBankV2P(bank, mpblock, &ubPhyBank, &uwPhyBs);
#ifdef VIRTUAL_BANK
#ifdef BLK_REMAP_PRO
                if(gubBlkRemapProFlag == 1)
                {
                    if(uwPhyBs >= guwRealMPBlkNum)
                        continue;
                }
                else
#endif
                {
                    if(uwPhyBs >= NandPara.uwMpBlockNumPerLun)
                        continue;
                }
#endif
                if(!llfIsMpBlockBad(DBT_ADDR, ubPhyBank, uwPhyBs))//if already mark, don't need to mark twice well
                {
                    llfMarkUserMPBlkDefect(DBT_ADDR, ubPhyBank, uwPhyBs);
                    llfDbgPrintk(ALWAYS_MSG, "Inherit System Defect: bank %d mpblk %d\r\n", ubPhyBank, uwPhyBs);
                    for(planenum = 0; planenum < NandPara.ubPlaneNumPerLun; planenum++)
                    {
                        singleBlock = (uwPhyBs * NandPara.ubPlaneNumPerLun + planenum);
#ifdef SBLK_EXPAND
                        if((singleBlock < glsysblkNum) && (!(llfIsGoodSblk(ubPhyBank, singleBlock))))
#else
                        if((singleBlock < glsysblkNum) && (!((gulSysblk >> (bank * 4)) & (1 << singleBlock))))
#endif
                        {
                            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + ubPhyBank * SHORT_BYTE_SIZE) ++;
                        }
                        else if(singleBlock >= glsysblkNum && singleBlock < (glsysblkNum + EXTEND_RDT_BLK))
                        {
                            if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, ubPhyBank, singleBlock))
                            {
                                _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + ubPhyBank * SHORT_BYTE_SIZE) ++;
                            }
                        }
                        else if (singleBlock >= (glsysblkNum + EXTEND_RDT_BLK))
                        {
                            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + ubPhyBank * SHORT_BYTE_SIZE) ++;
                        }
                    }
                    //cache_area_dwbinval(ALIGN_32_ROUND(dbt_addr + ubBankNo * group_num + group_no), 32);
                    //ALIGN_32_ROUND
                }
            }
        }

        // parse Erase Count Table
#if 0
        tempBS = _MEM32(blockInfo_addr + mpblock * BS_INFO_SIZE + BAD_INFO_OFFSET);

        after LLF, erase count increased
        _REG16(offset) = ((tempBS >> 16) & 0xFFFF) + 1;
        offset += SHORT_BYTE_SIZE;
#endif
    }

    //cache_area_dwb(DBT_ADDR, DBT_SIZE);
    //cache_dummy_update_read();

    //cache_area_dwb(ECT_ADDR, ECT_SIZE);
    //cache_dummy_update_read();

    gfDBTInitDone = LLF_DBT_SYSTEM;
    return ERR_OK;
}

U32 llfLoadBlockInfo()
{
    U8 bank_no, StaticSblkTag = 0;
    U8 aes_bypass, ubBSInfoPageOffset;
    U16 block = 0;
    U32 ret;
    U32 DynamicSblkAddr = TEMP_BUF_ADDR + DRAM_DATA_SIZE;
    U32 BSRangeStart;
    U32 nosblk = 0;
#ifdef AVG_ERASE_COUNT_TEST
    U16 mpBlockNumPerLun;
#endif
    U8 banknum, ubBankNumChk;
    U8 ubSblkStart = 0, ubSblkBankStart = 0;

#ifdef SBLK_EXPAND
    ubSblkBankStart = gubSblkBankStart;
    ubSblkStart = gubSblkStart;
    U8 ubCH, ubCE;
#if defined(RL6643_VA)
    U32 value_temp;
#endif
#endif

    memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
    cache_area_dwb(DBT_ADDR, DBT_SIZE);
    cache_dummy_update_read();
    memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
    cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
    cache_dummy_update_read();

    // get static SBlock
    llfInitErrorMessage();
#if defined(RL6643_FPGA)||defined(RL6643_VA)
    Change_ldpc(3);
#elif defined(RL6531_VB)
    Change_ldpc(0);
#else
    Change_ldpc(5);
#endif

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    ubBankNumChk = NandPara.ubBankNum;
#else
    ubBankNumChk = UnbalancedGetBankNum();
#endif

    // we have no static SBlock data now, so we get one of dynamic SBlock
    if(ubBankNumChk > SYS_BANK_NUM)
        banknum = SYS_BANK_NUM;
    else
        banknum = ubBankNumChk;

    llfprintk("banknum is %d\r\n", banknum);

    llfDbgPrintk(ALWAYS_MSG, "LLF inherit Whole system Info Start...\r\n");
    for(bank_no = ubSblkBankStart; bank_no < ubSblkBankStart + banknum; bank_no++)
    {
        for(block = ubSblkStart; block < ubSblkStart + SYS_BLK; block++)//need to check
        {
            ret = llfReadSBlock(TEMP_BUF_ADDR, TEMP_HBUF_ADDR, bank_no, block, 0);

            //double check SBLK content
            if(ret == ERR_OK)
            {
                if((_REG16(TEMP_BUF_ADDR + SBLK_OFFSET_FC_PARS_TABLE_SIZE) != FC_PARSER_TABLE_SIZE)
                        || (_REG16(TEMP_BUF_ADDR + SBLK_OFFSET_FC_SEQS_TABLE_SIZE) != FC_SENQUENCER_TABLE_SIZE))
                {
                    DBGPRINTK(ALWAYS_MSG, "Read Static not ok, maybe cause by WS overProgram!\r\n");
                    ret = ERR_READ_SBLK;
                }
            }

            llfDbgPrintk(ALWAYS_MSG, "Read Static SBLK ret %x \r\n", ret);
            if (ret == ERR_OK)
            {
                StaticSblkTag = STATIC_SBLK_INFO;
                break;
            }
        }
        if(StaticSblkTag == STATIC_SBLK_INFO)
            break;
    }

    aes_bypass = FcCmdBypass.bits.aes_bypass;
    FcCmdBypass.bits.aes_bypass = AES_BYPASS;//AES_BYPASS;

    Change_ldpc(gubECC_CFG);
#ifndef SBLK_EXPAND
    if (bank_no >= banknum)//Not find static sblk, try to read dynamic sblk
#else
    if (bank_no >= ubSblkBankStart + banknum)
#endif
    {
        for(block = 0; block < SYSTEM_BLOCK_MAX_NUM; block++)
        {
            for(bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
            {
#ifndef SBLK_EXPAND
                if(bank_no < banknum && block < SYS_BLK)
#else
                if((block < ubSblkStart) || ((bank_no < ubSblkBankStart) && (block < ubSblkStart + SYS_BLK)))
#endif
                    ;
                else
                {
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DQV5) || defined(FTL_H3DTV7)
                    if(llfHynixReadSBlock(TEMP_BUF_ADDR, TEMP_HBUF_ADDR, bank_no, block, 0) == ERR_OK)
#else
                    if(llfReadSBlock(TEMP_BUF_ADDR, TEMP_HBUF_ADDR, bank_no, block, 0) == ERR_OK)
#endif
                    {
                        llfDbgPrintk(ALWAYS_MSG, "No static SBlk and read Dsblk bank %d Block %d\r\n", bank_no, block);
                        StaticSblkTag = DYNAMIC_SBLK_INFO;
                        break;
                    }
                }

            }
            if(StaticSblkTag == DYNAMIC_SBLK_INFO)
                break;
        }
        if (block >= SYSTEM_BLOCK_MAX_NUM)
        {
            FcCmdBypass.bits.aes_bypass = aes_bypass;
            llfDbgPrintk(ALWAYS_MSG, "[WARN] Read first dynamic SBlock failed\r\n");
            nosblk = 1;
            StaticSblkTag = STATIC_SBLK_INFO;
#ifndef RL6577_VA
            return ERR_READ_SBLK;
#endif
        }
    }
    if (nosblk == 0)
    {
        gulLLFTypeRecord = _MEM32(TEMP_BUF_ADDR + SBLK_OFFSET_LLF_TYPE_RECORD);
        gulWriteMBRecord[0] = _MEM32(TEMP_BUF_ADDR + SBLK_OFFSET_WRITE_MB);
        gulWriteMBRecord[1] = _MEM32(TEMP_BUF_ADDR + SBLK_OFFSET_WRITE_MB_RECORD_0);
    }

    // get dynamic SBlock
    if (nosblk || llfSearchDynamicSBlock(TEMP_BUF_ADDR, DynamicSblkAddr, StaticSblkTag) != ERR_OK)
    {
        FcCmdBypass.bits.aes_bypass = aes_bypass;
        if(nosblk)
        {
            ret = ERR_DIRTY;
            for(bank_no = 0; bank_no < banknum; bank_no++)
            {
                if(ret == ERR_OK)
                {
                    break;
                }
                for(block = 0; block < SYS_BLK; block++)
                {
                    ret = llfReadDBTFromStaticSblock(bank_no, block);
                    if(ret == ERR_OK)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "LoadDBT from bank:%d block:%d\r\n", bank_no, block);
                        break;
                    }
                }
            }
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "[WARN] Can't find any DBT.\r\n");
            }
        }
        else
        {
            ret = llfReadDBTFromStaticSblock(bank_no, block);
        }
        if((ret == ERR_OK) && (StaticSblkTag == STATIC_SBLK_INFO))
        {
#ifdef KEEP_ORIG_DBT
            ret = llfLoadOriginalDBT(ORIG_DBT_PHY_ADDR);
            if(ret != ERR_OK)
            {
                // UART REDUCE: llfLoadOriginalDBT from static sblock (1) fail! Set DBT
                llfprintk("Load original dbt fail\r\n");
                memcpy((void*)(ORIG_DBT_ADDR), (void*)(DBT_ADDR), ORIG_DBT_SIZE);
                cache_area_dwb(ORIG_DBT_ADDR, ORIG_DBT_SIZE);
                cache_dummy_update_read();
            }
            else if(_MEM08(CONFIG_BASE_VA_ADDR + CONFIG_RESET_TO_ORIG_DBT) == 1)
            {
                // UART REDUCE: Reset DBT from original DBT
                llfprintk("Reset dbt to original dbt\r\n");
                memcpy((void*)(DBT_ADDR), (void*)(ORIG_DBT_ADDR), DBT_SIZE);
                cache_area_dwb(DBT_ADDR, DBT_SIZE);
                cache_dummy_update_read();
            }
#endif
            if(!nosblk)
            {
                llfInheritRDTInfoFromSBLK(TEMP_BUF_ADDR);
                llfDbgPrintk(ALWAYS_MSG, "Inherit from static Sblock\r\n");
            }
            gfDBTInitDone = LLF_DBT_SYSTEM;
#ifdef COM_BANKING

            if(nosblk)
            {
                gulFailedImageBitMap = 0xFFFFFFFF;
                for(bank_no = 0; bank_no < banknum; bank_no++)
                {
                    for(block = 0; block < SYS_BLK; block++)
                    {
#ifndef SBLK_EXPAND
                        if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block))
#else
                        if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no + gubSblkBankStart, block + gubSblkStart))
#endif
                        {
                            continue;
                        }
                        gulFailedImageBitMap &= ~(1 << (block + bank_no * 4));
                    }
                }
            }
            else
            {
                gulFailedImageBitMap = _REG32(TEMP_BUF_ADDR  + SBLK_OFFSET_FAIL_IMAGE_BITMAP) ;
            }


            llfDbgPrintk(ALWAYS_MSG, "gulFailedImageBitMap:0x%x from Static Sblock\r\n", gulFailedImageBitMap);

#endif
#ifdef BLK_REMAP_PRO
            if(!nosblk)
            {
                guwRealMPBlkNum = _REG16(TEMP_BUF_ADDR + SBLK_OFFSET_REAL_BLOCK_NUM);
            }
            else
            {
                guwRealMPBlkNum = NandPara.uwMpBlockNumPerLun;
            }
            llfprintk("=REMAP= Inherit statics sblock guwRealMPBlkNum is %d\r\n", guwRealMPBlkNum);
            if(_MEM16(TEMP_BUF_ADDR + SBLK_OFFSET_REMAP_TABLE_PER_BANK) < 255) //prevent wrong data
            {
                guwReMapTableSizePerBank = _MEM16(TEMP_BUF_ADDR + SBLK_OFFSET_REMAP_TABLE_PER_BANK);
            }
            ret = LlfInheritRemappingTable(BLK_REMAP_TABLE_PHY_ADDR);
            if (ret != ERR_OK)
            {
                llfprintk("LlfInheritRemappingTable fail !\r\n");
                return ERR_READ_RDT;
            }
            gubBlkIheritRemapFlag = 1;

#endif
#ifdef AVG_ERASE_COUNT_TEST
            gulDataBsAvgEraseCnt = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_DATA_BS_AVG_EC);
            gulL2PBsAvgEraseCnt = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_L2P_BS_AVG_EC);
            llfprintk("Get DataBSAvgEC = %x, L2PBSAvgEC = %x from [Static Sblock](in LLF mode)\r\n",
                      gulDataBsAvgEraseCnt, gulL2PBsAvgEraseCnt);
#endif
#ifdef SBLK_EXPAND
#if defined(RL6643_VA)
			value_temp = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_SBLK_START_TAG);
            if((value_temp & 0xff) == TAG_FOR_SBLK_EXPAND)
            {
                gubSblkStart = (value_temp >> 8) & 0xff;
                gubSblkBankStart = (value_temp >> 16) & 0xff;
            }

            gulSblkCHCEMap[0] = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_SBLK_CHCEMAP_CH0);
            gulSblkCHCEMap[1] = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_SBLK_CHCEMAP_CH1);
            gubSblkBankCnt = 0;
            for(ubCH = 0; ubCH < CH_NUM_MAX; ubCH++)
            {
                for(ubCE = 0; ubCE < CE_NUM_MAX; ubCE++)
                {
                    if(gulSblkCHCEMap[ubCH] & (0xf << ubCE))
                    {
                        gubSblkBankCnt++;
                    }
                }
            }
            llfprintk("[SBE] StaticSBlock SblkStart %d, gubSblkBankStart %d, SblkBankCnt %d\r\n", gubSblkStart,
                      gubSblkBankStart, gubSblkBankCnt);
            llfprintk("static_sblock_[SBE] SblkCHCEMap[0] = %x, SblkCHCEMap[1] = %x\r\n", gulSblkCHCEMap[0],
                      gulSblkCHCEMap[1]);
#endif
#endif
            return ERR_OK;
        }
        else
        {
            return ERR_READ_SBLK;
        }
    }
    else
    {
        gulLLFTypeRecord = _MEM32(DynamicSblkAddr + SBLK_OFFSET_LLF_TYPE_RECORD);
        gulWriteMBRecord[0] = _MEM32(DynamicSblkAddr + SBLK_OFFSET_WRITE_MB);
        gulWriteMBRecord[1] = _MEM32(DynamicSblkAddr + SBLK_OFFSET_WRITE_MB_RECORD_0);

        llfInheritRDTInfoFromSBLK(DynamicSblkAddr);

#ifdef BLK_REMAP_PRO
        guwRealMPBlkNum = _REG16(DynamicSblkAddr + SBLK_OFFSET_REAL_BLOCK_NUM);
        if(_MEM16(TEMP_BUF_ADDR + SBLK_OFFSET_REMAP_TABLE_PER_BANK) < 255) //prevent wrong data
        {
            guwReMapTableSizePerBank = _MEM16(TEMP_BUF_ADDR + SBLK_OFFSET_REMAP_TABLE_PER_BANK);
        }
        FcCmdBypass.bits.aes_bypass = aes_bypass;
        llfprintk("=REMAP= Inherit Dynamic sblock guwRealMPBlkNum is %d\r\n", guwRealMPBlkNum);
        gfDBTInitDone = LLF_DBT_SYSTEM;
        //inherit Table from page 8-9
        ret = LlfInheritRemappingTable(BLK_REMAP_TABLE_PHY_ADDR);
        FcCmdBypass.bits.aes_bypass = 0;
        if (ret != ERR_OK)
        {
            llfprintk("LlfInheritRemappingTable fail !\r\n");
            return ERR_READ_RDT;
        }
        gubBlkIheritRemapFlag = 1;
#endif
    }

    BSRangeStart = _REG32(DynamicSblkAddr + SBLK_OFFSET_SNAP_BS_START);
    ubBSInfoPageOffset = 0;
    if((_MEM16(DynamicSblkAddr + SBLK_OFFSET_WS_BS_INFO_OFFSET) & 0xff) == TAG_BS_INFO_PAGE_OFFSET)
    {
        ubBSInfoPageOffset = _MEM16(DynamicSblkAddr + SBLK_OFFSET_WS_BS_INFO_OFFSET) >> 8;
    }
    else
    {
        llfprintk("[WARN] BSInfo page offset tag mismatch\r\n");
    }
    llfprintk("BSRangeStart %x PageOffset %d\r\n", BSRangeStart, ubBSInfoPageOffset);

#ifdef AVG_ERASE_COUNT_TEST
    guwL2PGroupBegin = _REG16(DynamicSblkAddr + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX);
    guwL2PGroupEnd = _REG16(DynamicSblkAddr + SBLK_OFFSET_L2P_MP_BLOCK_END_INDEX);
    mpBlockNumPerLun = _REG16(DynamicSblkAddr + SBLK_OFFSET_MP_BLOCK_NUM);
#endif
#ifdef SBLK_EXPAND
	value_temp = _REG32(DynamicSblkAddr + SBLK_OFFSET_SBLK_START_TAG);
    if((value_temp & 0xff) == TAG_FOR_SBLK_EXPAND)
    {
        gubSblkStart = (value_temp >> 8) & 0xff;
        gubSblkBankStart = (value_temp >> 16) & 0xff;
    }
    gulSblkCHCEMap[0] = _REG32(DynamicSblkAddr + SBLK_OFFSET_SBLK_CHCEMAP_CH0);
    gulSblkCHCEMap[1] = _REG32(DynamicSblkAddr + SBLK_OFFSET_SBLK_CHCEMAP_CH1);
    llfprintk("[SBE] StaticSBlock SblkStart %d, gubSblkBankStart %d, SblkBankCnt %d\r\n", gubSblkStart,
              gubSblkBankStart, gubSblkBankCnt);
    llfprintk("static_sblock_[SBE] SblkCHCEMap[0] = %x, SblkCHCEMap[1] = %x\r\n", gulSblkCHCEMap[0],
              gulSblkCHCEMap[1]);
#else
    gulSysblk = _REG16(DynamicSblkAddr + SBLK_OFFSET_SYS_BLK_CFG2);
#endif

    DynamicSblkAddr = TEMP_BUF_ADDR;
#ifdef COM_BANKING
    gulFailedImageBitMap = _REG32(DynamicSblkAddr + SBLK_OFFSET_FAIL_IMAGE_BITMAP) ;
    llfDbgPrintk(ALWAYS_MSG, "gulFailedImageBitMap:0x%x from Dynamic Sblock\r\n", gulFailedImageBitMap);

#endif
    // read blockInfo
    ret = llfReadBlockInfo(DynamicSblkAddr, BSRangeStart, ubBSInfoPageOffset);
    FcCmdBypass.bits.aes_bypass = aes_bypass;
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Read BlockInfo failed\r\n");
        return ret;
    }
#ifndef HANDLE_BLK_INFO_BEYOND_64K
    ret = llfParseBlockInfoToDBT(DynamicSblkAddr);
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Parse BlockInfo to RDT failed\r\n");
        return ret;
    }
#endif
#ifdef AVG_ERASE_COUNT_TEST
    ret = llfParseBlockInfoToEC(DynamicSblkAddr, mpBlockNumPerLun); // must after llfReadBlockInfo()
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Parse BlockInfo to EC failed\r\n");
        return ret;
    }
#endif

#ifdef KEEP_ORIG_DBT
    ret = llfLoadOriginalDBT(ORIG_DBT_PHY_ADDR);
    if(ret != ERR_OK)
    {
        // UART REDUCE: [ERR] llfLoadOriginalDBT from static sblock (2) fail! Set DBT
        llfprintk("Load original dbt fail\r\n");
        memcpy((void*)(ORIG_DBT_ADDR), (void*)(DBT_ADDR), ORIG_DBT_SIZE);
        cache_area_dwb(ORIG_DBT_ADDR, ORIG_DBT_SIZE);
        cache_dummy_update_read();
        //return ERR_READ_RDT;
    }
    else if(_MEM08(CONFIG_BASE_VA_ADDR + CONFIG_RESET_TO_ORIG_DBT) == 1)
    {
        // UART REDUCE: Reset DBT from original DBT
        llfprintk("Reset dbt to original dbt\r\n");
        memcpy((void*)(DBT_ADDR), (void*)(ORIG_DBT_ADDR), DBT_SIZE);
        cache_area_dwb(DBT_ADDR, DBT_SIZE);
        cache_dummy_update_read();
    }
#endif

    return ERR_OK;
}

U32 llfInherit()
{
    U32 ret;
    // case 1. Rdt inherit
    // Clear DBT  Set default defect table of super block as zero
    memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
    cache_area_dwb(DBT_ADDR, DBT_SIZE);
    cache_dummy_update_read();
    memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
    cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
    cache_dummy_update_read();

    gubVthEnNum = 0;
    // RDT data exist but fail, we need to show error
    ret = llfLoadRDTResult();
    if (ret == ERR_OK)
    {
        if(gubVthEnNum == NandPara.ubBankNum)
        {
            _REG08(SBLK_ADDR + SBLK_OFFSET_VTH_ENABLE) = 0x4f;//'O'
        }
        else
        {
            _REG08(SBLK_ADDR + SBLK_OFFSET_VTH_ENABLE) = 0;
        }
#ifdef EXTEND_STATIC_DBT
        if(gubNeedRebuildRemap == 0)
#endif
        {
#ifdef 	BLK_REMAP_PRO
            U32 gubLdpcCodeRateTemp;
#ifdef RL6531_VB
            gubLdpcCodeRateTemp = gubECC_CFG;
#else
            gubLdpcCodeRateTemp = gubLdpcCodeRate;
#endif

#if defined(RL6643_FPGA)||defined(RL6643_VA)
            Change_ldpc(3);
#elif defined(RL6531_VB)
            Change_ldpc(0);
#else
            Change_ldpc(5);
#endif
            //inherit Table from page 8-11
            llfprintk("LlfInheritRemappingTable... \r\n");
            ret = LlfInheritRemappingTable(BLK_REMAP_TABLE_PHY_ADDR);
            Change_ldpc(gubLdpcCodeRateTemp);
            if (ret != ERR_OK)
            {
                llfprintk("LlfInheritRemappingTable fail !\r\n");
                return ERR_READ_RDT;
            }
            gubBlkIheritRemapFlag = 1;
#endif
        }
        return ERR_OK;
    }
#ifdef EXTEND_STATIC_DBT
    else
    {
        gubNeedRebuildRemap = 0;
        llfprintk("StaticDBT LoadRDTResult fail, change gubNeedRebuildRemap %d\r\n", gubNeedRebuildRemap);
    }
#endif
#if (defined(RL6577_VA)||defined(RTS5771_VA)) && defined(KEEP_RDT_RESULT)
    if(gubLLFMode == LLF_DEFECT_BAD_BLOCK)
    {
        printk("[ERR] inherit original RDT result FAIL.\r\n");
        return ret;
    }
#endif

    gubVthEnNum = 0;
    // case 2. whole system inherit
    ret = llfLoadBlockInfo();
    if (ret == ERR_OK)
    {
        if(gubVthEnNum == NandPara.ubBankNum)
        {
            _REG08(SBLK_ADDR + SBLK_OFFSET_VTH_ENABLE) = 0x4f;//'O'
        }
        else
        {
            _REG08(SBLK_ADDR + SBLK_OFFSET_VTH_ENABLE) = 0;
        }
        return ret;
    }
    else if (ret != ERR_READ_SBLK)  // found SBlock but wrong blockinfo
    {
        return ret;
    }
    return ERR_READ_RDT;
}

U32 llfInheritDriving(U8 ubIFType)
{
#if 0
    U32 ulFR_PAD_DRV_CFG0, ulFR_PAD_DRV_CFG1, ulFR_PAD_DRV_CFG2, ulFR_PAD_DRV_CFG3;
    U32 ulFR_PAD_ODT_CFG, ulFR_PAD_ODT_CTRL, ulAddr, ret;
    U8 ubFC_DIFF_EN, ubNAND_ODT_EN, ubMICRON_DRISTR, ubTSB_DRISTR, ubNAND_VREF_EN;
    U8 ubBank, ufFindSBlock, ubOrigAESBypass;
    U16 uwBlock;

    ulAddr = TEMP_BUF_ADDR;
    ufFindSBlock = FALSE;
    uwBlock = 0;

    // inherit from RDT result
    for (ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
    {
        if (llfGetRDTBankResult(TRUE, ubBank) == ERR_OK)
        {
            break;
        }
    }
    if (ubBank != NandPara.ubBankNum)
    {
        // find RDT result
        // TODO:: currently no need to save FC_DIFF_EN, FR_PAD_ODT_CFG and FR_PAD_ODT_CTRL
        ubFC_DIFF_EN      = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FC_DIFF_EN);
        ulFR_PAD_ODT_CTRL = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_ODT_CTRL);

        ubNAND_ODT_EN     = _REG08(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_NAND_ODT_EN);
        ubMICRON_DRISTR   = _REG08(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_MICRON_DRISTR);
        ubTSB_DRISTR      = _REG08(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_TSB_DRISTR);
        ubNAND_VREF_EN    = _REG08(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_NAND_VREF_EN);
        ulFR_PAD_ODT_CFG  = _REG32(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_FR_PAD_ODT_CFG);
        ulFR_PAD_DRV_CFG0 = _REG32(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_FR_PAD_DRV_CFG0);
        ulFR_PAD_DRV_CFG0 = _REG32(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_FR_PAD_DRV_CFG0);
        ulFR_PAD_DRV_CFG1 = _REG32(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_FR_PAD_DRV_CFG1);
        ulFR_PAD_DRV_CFG2 = _REG32(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_FR_PAD_DRV_CFG2);
        ulFR_PAD_DRV_CFG3 = _REG32(ulAddr + RDT_PROJECT_DEFINE + RDT_PROJ_DEF_FR_PAD_DRV_CFG3);
    }
    else
    {
        // inherit from whole system
        for (uwBlock = 0; uwBlock < SYSTEM_BLOCK_MAX_NUM; uwBlock++)
        {
            for (ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
            {
                ret = llfReadSBlock(TEMP_BUF_PHY_ADDR, TEMP_HBUF_PHY_ADDR,
                                    ubBank, uwBlock, 0, SBLK_BLK_ID);
                if (ret == ERR_OK)
                {
                    ufFindSBlock = TRUE;
                    break;
                }
            }
            if (ufFindSBlock)
            {
                break;
            }
        }
        if (ufFindSBlock)
        {
            // find SBlock
            ubFC_DIFF_EN      = _REG08(ulAddr + SBLK_OFFSET_FC_DIFF_EN);
            ubNAND_ODT_EN     = _REG08(ulAddr + SBLK_OFFSET_NAND_ODT_EN);
            ubMICRON_DRISTR   = _REG08(ulAddr + SBLK_OFFSET_NAND_MICRON_DRISTR);
            ubTSB_DRISTR      = _REG08(ulAddr + SBLK_OFFSET_NAND_TSB_DRISTR);
            ubNAND_VREF_EN    = _REG08(ulAddr + SBLK_OFFSET_NAND_VREF_EN);
            ulFR_PAD_ODT_CFG  = _REG32(ulAddr + SBLK_OFFSET_NORMAL_FR_PAD_ODT_CFG_PER_CH);
            ulFR_PAD_ODT_CTRL = _REG32(ulAddr + SBLK_OFFSET_NORMAL_FR_PAD_ODT_CTRL_PER_CH);
            ulFR_PAD_DRV_CFG0 = _REG32(ulAddr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG0_PER_CH);
            ulFR_PAD_DRV_CFG1 = _REG32(ulAddr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG1_PER_CH);
            ulFR_PAD_DRV_CFG2 = _REG32(ulAddr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG2_PER_CH);
            ulFR_PAD_DRV_CFG3 = _REG32(ulAddr + SBLK_OFFSET_NORMAL_FR_PAD_DRV_CFG3_PER_CH);
        }
        else
        {
            // UART REDUCE: Can't find RDT result or Sblock to inherit driving
            llfprintk("001979\r\n");
            return ERR_EMPTY;
        }
    }

    // UART REDUCE:   %d %d %d %d %d
    llfprintk("001980 %d %d %d %d %d\r\n", ubFC_DIFF_EN, ubNAND_ODT_EN, ubMICRON_DRISTR,
              ubTSB_DRISTR, ubNAND_VREF_EN);
    // UART REDUCE:   %x %x %x %x
    llfprintk("001981 %x %x %x %x\r\n", ulFR_PAD_DRV_CFG0, ulFR_PAD_DRV_CFG1,
              ulFR_PAD_DRV_CFG2, ulFR_PAD_DRV_CFG3);

    // error check
    if ((FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK
            || FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
            && ubTSB_DRISTR != 2 && ubTSB_DRISTR != 4 && ubTSB_DRISTR != 6)
    {
        // UART REDUCE: Invalid TSB NAND DRI
        llfprintk("001982\r\n");
        return ERR_DIRTY;
    }
    else if ((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON
              || FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL)
             && ubMICRON_DRISTR != 1 && ubMICRON_DRISTR != 2 && ubMICRON_DRISTR != 3)
    {
        // UART REDUCE: Invalid IM NAND DRI
        llfprintk("001983\r\n");
        return ERR_DIRTY;
    }
    else if (ulFR_PAD_ODT_CFG > 0x33333333 || ulFR_PAD_ODT_CTRL > 0x33333333
             || ulFR_PAD_DRV_CFG0 > 0x33333333 || ulFR_PAD_DRV_CFG1 > 0x33333333
             || ulFR_PAD_DRV_CFG2 > 0x33333333 || ulFR_PAD_DRV_CFG3 > 0x33333333)
    {
        // UART REDUCE: Invalid driving setting
        llfprintk("001984\r\n");
        return ERR_DIRTY;
    }

    if (ufFindSBlock)
    {
        // UART REDUCE: Inherit driving from SBlock %d_%d
        llfprintk("001985 %d %d\r\n", ubBank, uwBlock);
    }
    else
    {
        // UART REDUCE: Inherit driving from RDT result
        llfprintk("001986\r\n");
    }

    _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FC_DIFF_EN) = ubFC_DIFF_EN;
    _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_ODT_EN) = ubNAND_ODT_EN;
    _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_MICRON_DRISTR) = ubMICRON_DRISTR;
    _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_TSB_DRISTR) = ubTSB_DRISTR;
    _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_NAND_VREF_EN) = ubNAND_VREF_EN;
    _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_ODT_CFG) = ulFR_PAD_ODT_CFG;
    _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FR_PAD_ODT_CTRL) = ulFR_PAD_ODT_CTRL;
    _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG0) = ulFR_PAD_DRV_CFG0;
    _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG1) = ulFR_PAD_DRV_CFG1;
    _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRISTR_PN_CFG2) = ulFR_PAD_DRV_CFG2;
    _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FC_DRIVESTRENGTH) = ulFR_PAD_DRV_CFG3;

    gubFCDiffEnable = ubFC_DIFF_EN;
    gubNANDODTEnable = ubNAND_ODT_EN;
    gubMicronNANDDriv = ubMICRON_DRISTR;
    gubTSBNANDDriv = ubTSB_DRISTR;
    gubNANDVrefEn = ubNAND_VREF_EN;
    ret = onfi4_change_setting();//llfInitDriveStrength();
    if (ret != ERR_OK)
    {
        return ERR_DIRTY;
    }
#endif
    return ERR_OK;
}

void llfInitDefault()
{
    L2pPara.ubHdr4BLenPerPage = FC_HEADER_LEN_PER_PAGE;
    NandPara.ubPageNumPerBlockShift = 0;
    gubNandFlashType = 3;
    gubLdpcCodeRate = 5;
    guwLdpcParityLen = 150;

    //**********start of load default value************
    //gubNandFlashVoltage = FLASH_1_8V;
    gubNandFlashVendor = FLASH_VENDOR_MiCRON;
    gubNandDefaultMode = 0;//0 for SDR,1 for DDR2
    gubFcAddressCycleNum = 5;

    gubBank0CH = 0;
    gubBank0CE = 0;
    gubBank1CH = 1;
    gubBank1CE = 0;
    gubHeaderlen = 12;
}

U32 llfLoadDefaultFromSblk(U32 *cfg_sblk)
{
    cfg_sblk[0] = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FLASH_TYPE);
    cfg_sblk[1] = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_SYS_BANKMAPPING);
    //cfg_sblk[2] = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_SYS_CFG);

    if(((cfg_sblk[0] & 0xF) != TAG_FOR_ECC_HEDDER)\
            || ((cfg_sblk[1] & 0xF) != TAG_FOR_CH_MAPPING)
            //|| ((cfg_sblk[2] & 0xF) != TAG_FOR_SYS_CFG)
      )
    {
        return ERR_EMPTY;
    }

    return ERR_OK;
}

void llfConfigDefault(U32 *cfg)
{

    gubNandFlashType = (cfg[1] >> 20) & 0x0f;
    gubNandFlashVendor = (cfg[1] >> 24) & 0x0f;
    gubNandDefaultMode = (cfg[0] >> 31); //0 for SDR,1 for DDR2

    //vdd_1v8_en = ((cfg[2] >> 4) & 0x1)? 0 : 1;
    //NandPara.ubSLCPageNumPerBlockShift = (cfg[2] >> 8) & 0xF;
    //gubFcAddressCycleNum = (cfg[2] >> 12) & 0xF; //bit[15:12] for address cycle

    gubBank0CH = (cfg[1] >> 16) & 0x0f;
    gubBank0CE = (cfg[1] >> 12) & 0x0f;
    gubBank1CH = (cfg[1] >> 8) & 0x0f;
    gubBank1CE = (cfg[1] >> 4) & 0x0f;

    gubECC_CFG0 = (cfg[0] >> 4) & 0x0f;

    llfprintk("gubECC_CFG0 = %x\r\n", gubECC_CFG0);
    llfprintk("Config=> %d,%d,%d,%d\r\n", gubBank0CH, gubBank0CE, gubBank1CH, gubBank1CE);
    return;
}

U32 llfLoadDefault(void)
{
    U32 ret;
    //U32 cfg[3];
    U32 cfg_sblk[3];

    llfInitDefault();
    llfLoadDefaultFromGPIO();

    //load default from sblk
    ret = llfLoadDefaultFromSblk(&cfg_sblk[0]);
    if(ret != ERR_OK)
        return ERR_CONFIG_HEADER;

    llfConfigDefault(cfg_sblk);

    return ERR_OK;
}
#if defined(RL6447_VA)||defined(RL6531_VB)
void FcCopyLdpcTableToMatrix(U32 d_begin_addr, U32 s_begin_addr, U32 s_end_addr)
{
    U32 i;
    for(i = s_begin_addr; i < s_end_addr; i += 4)
    {
        _REG32(d_begin_addr + i - s_begin_addr) = _REG32(i);
    }
}

void FcCopyLdpcTableFromROM()
{
#if 0
    FC_LDPC_REG(FR_LENC_CTRL) = 0xd2;

    if(gubLdpcCodeRate == 0x24)
    {
        gulLdpcTableAddr[0] = (U32)ldpc_24_encoder_a_begin;
        gulLdpcTableAddr[1] = (U32)ldpc_24_encoder_a_end;
        gulLdpcTableAddr[2] = (U32)ldpc_24_encoder_b_begin;
        gulLdpcTableAddr[3] = (U32)ldpc_24_encoder_b_end;
        gulLdpcTableAddr[4] = (U32)ldpc_24_decoder_a_begin;
        gulLdpcTableAddr[5] = (U32)ldpc_24_decoder_a_end;
        gulLdpcTableAddr[6] = (U32)ldpc_24_decoder_b_begin;
        gulLdpcTableAddr[7] = (U32)ldpc_24_decoder_b_end;
    }
    else if(gubLdpcCodeRate == 0x45)
    {
        gulLdpcTableAddr[0] = (U32)ldpc_45_encoder_a_begin;
        gulLdpcTableAddr[1] = (U32)ldpc_45_encoder_a_end;
        gulLdpcTableAddr[2] = (U32)ldpc_45_encoder_b_begin;
        gulLdpcTableAddr[3] = (U32)ldpc_45_encoder_b_end;
        gulLdpcTableAddr[4] = (U32)ldpc_45_decoder_a_begin;
        gulLdpcTableAddr[5] = (U32)ldpc_45_decoder_a_end;
        gulLdpcTableAddr[6] = (U32)ldpc_45_decoder_b_begin;
        gulLdpcTableAddr[7] = (U32)ldpc_45_decoder_b_end;
    }
    else if(gubLdpcCodeRate == 0x66)
    {
        gulLdpcTableAddr[0] = (U32)ldpc_66_encoder_a_begin;
        gulLdpcTableAddr[1] = (U32)ldpc_66_encoder_a_end;
        gulLdpcTableAddr[2] = (U32)ldpc_66_encoder_b_begin;
        gulLdpcTableAddr[3] = (U32)ldpc_66_encoder_b_end;
        gulLdpcTableAddr[4] = (U32)ldpc_66_decoder_a_begin;
        gulLdpcTableAddr[5] = (U32)ldpc_66_decoder_a_end;
        gulLdpcTableAddr[6] = (U32)ldpc_66_decoder_b_begin;
        gulLdpcTableAddr[7] = (U32)ldpc_66_decoder_b_end;
    }
    else if(gubLdpcCodeRate == 0x87)
    {
        gulLdpcTableAddr[0] = (U32)ldpc_87_encoder_a_begin;
        gulLdpcTableAddr[1] = (U32)ldpc_87_encoder_a_end;
        gulLdpcTableAddr[2] = (U32)ldpc_87_encoder_b_begin;
        gulLdpcTableAddr[3] = (U32)ldpc_87_encoder_b_end;
        gulLdpcTableAddr[4] = (U32)ldpc_87_decoder_a_begin;
        gulLdpcTableAddr[5] = (U32)ldpc_87_decoder_a_end;
        gulLdpcTableAddr[6] = (U32)ldpc_87_decoder_b_begin;
        gulLdpcTableAddr[7] = (U32)ldpc_87_decoder_b_end;

    }
    else if(gubLdpcCodeRate == 0xa8)
    {
        gulLdpcTableAddr[0] = (U32)ldpc_a8_encoder_a_begin;
        gulLdpcTableAddr[1] = (U32)ldpc_a8_encoder_a_end;
        gulLdpcTableAddr[2] = (U32)ldpc_a8_encoder_b_begin;
        gulLdpcTableAddr[3] = (U32)ldpc_a8_encoder_b_end;
        gulLdpcTableAddr[4] = (U32)ldpc_a8_decoder_a_begin;
        gulLdpcTableAddr[5] = (U32)ldpc_a8_decoder_a_end;
        gulLdpcTableAddr[6] = (U32)ldpc_a8_decoder_b_begin;
        gulLdpcTableAddr[7] = (U32)ldpc_a8_decoder_b_end;
    }
    else
    {
        llfprintk("unknown ldpc code rate\r\n");
        ASSERT_LLF(0);
    }

    FcCopyLdpcTableToMatrix(FC_LDPC_ENCODER_A_BASE, gulLdpcTableAddr[0],
                            gulLdpcTableAddr[1]);
    FcCopyLdpcTableToMatrix(FC_LDPC_ENCODER_B_BASE, gulLdpcTableAddr[2],
                            gulLdpcTableAddr[3]);
    FcCopyLdpcTableToMatrix(FC_LDPC_DECODER_A_BASE, gulLdpcTableAddr[4],
                            gulLdpcTableAddr[5]);
    FcCopyLdpcTableToMatrix(FC_LDPC_DECODER_B_BASE, gulLdpcTableAddr[6],
                            gulLdpcTableAddr[7]);
    FC_LDPC_REG(FR_LENC_CTRL) = 0xc2;
#endif


}
#endif

#if 0
void llfResetNand()
{
    U8 bank_no;
    U32 cmp, ret;
    U32 ulNewTimingMode = 0;  //force default low speed
    U32 ulCurIF = ONFI_DDR2_TOGGLE;
    U32 ulNewIF = ONFI_SDR;
    U32 nand_odt_diff_vref_value = 0;
    U32 diff_en = 0;

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
    {
        if (NandPara.ubLunNumPerCE > 1)
        {
            // setup PC bit for lun interleave, not to clear lun cache afer die program
            ulNewTimingMode |= 0x40;
        }
        if (vdd_1v8_en && ((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T) ||
                           (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_YG2T_CS2)))
        {
            // Set back IF to SDR and timing mode to low speed for NAND
            llfprintk("YMTC X1 SDR/NV-DDR2 change IF and timing mode\r\n");
            for(bank_no = 0; bank_no < NandPara.ubBankNumPerLun; bank_no++)
            {
                gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank_no);
                FCSetfeature(ulCurIF, bank_no, 0x01, ((ulNewIF << 4) | ulNewTimingMode));
                FcBusyWait1ms(1);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret != ERR_OK)
                {
                    llfprintk("Reset IF timeout bank %d\r\n", bank_no);
                }
                else
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        llfprintk("Reset IF Error cmp %x bank %d\r\n", cmp, bank_no);
                    }
                }
            }
            // Set back IF to SDR and timing mode to low speed for FC
            FR_G_CFG_REG32_W(FR_FC_MODE, ulNewIF);
            ChangeFCClk(ulNewIF, FC_PLL_CLK_10M);

            // Disable diff/ODT/Vref for NAND
            llfprintk("YMTC X1 SDR/NV-DDR2 change diff\r\n");
            for(bank_no = 0; bank_no < NandPara.ubBankNumPerLun; bank_no++)
            {
                gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank_no);
                FCSetfeature(ulNewIF, bank_no, 0x02, nand_odt_diff_vref_value);
                FcBusyWait1ms(1);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret != ERR_OK)
                {
                    llfprintk("Reset diff timeout bank %d\r\n", bank_no);
                }
                else
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        llfprintk("Reset diff Error cmp %x bank %d\r\n", cmp, bank_no);
                    }
                }
            }
            // Disable diff for FC
            fc_diff_setting(ulNewIF, FC_PLL_CLK_10M, diff_en);
        }
        else
        {
            printk("YMTC NV-DDR3 Only change IF\r\n");
        }
    }
}
#endif

#ifdef BLK_REMAP_PRO
U32 llfBlkRemapInit()
{
    U16 uwTableLenth;
    U32 ulRet;

    gubBlkRemapProFlag = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_REMAP_TAG);
    gubBlkIheritRemapFlag = 0;
    uwTableLenth = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_REMAP_TABLE_PER_BANK);
    llfprintk("gubBlkRemapProFlag is %d,Table_Lenth is %d\r\n", gubBlkRemapProFlag, uwTableLenth);
    if(uwTableLenth == 0)
    {
        gubBlkRemapProFlag = 0;
    }
    if(gubBlkRemapProFlag == 1)
    {
        ulRet = LlfInitReMapTable(BLK_REMAP_TABLE_ADDR);
        if (ulRet != ERR_OK)
        {
            return ulRet;
        }
    }
    else
    {
        _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_REMAP_TABLE_PER_BANK) = 0;
    }

#ifdef NEW_BLK_REMAP
    U16 uwBlock;
    for(uwBlock = 0; uwBlock < NandPara.uwMpBlockNumPerLun; uwBlock++)
    {
        _MEM08(BLK_REMAP_BS_INDEX_TABLE_ADDR + uwBlock) = 0xff;
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
        _MEM64(BLK_REMAP_BITMAP_TABLE_ADDR + uwBlock * 8) = 0;
#else
        _MEM16(BLK_REMAP_BITMAP_TABLE_ADDR + uwBlock * 2) = 0;
#endif
    }
#endif

    return ERR_OK;
}
#endif
#if defined(RL6577_VA)||defined(RTS5771_VA)
U32 llfCheckOriginalRDTResultExist()
{
    U8 bank, existNum, ubLdpcCodeRateTemp;

    llfInitErrorMessage();
    existNum = 0;
#if defined(RL6531_VB)
    ubLdpcCodeRateTemp = gubECC_CFG;
#else
    ubLdpcCodeRateTemp = gubLdpcCodeRate;
#endif

    Change_ldpc(5);

    for (bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        if (llfGetRDTBankResult(false, bank, false) == ERR_OK)
        {
            if (_REG32(TEMP_BUF_ADDR) == ERR_OK || _REG32(TEMP_BUF_ADDR) == 0xFFFFFFFF)
            {
                existNum++;
            }
#ifdef EXTEND_STATIC_DBT
            if(bank == 0)
                llfCheckRDTPlaneAndDBTSize(TEMP_BUF_ADDR);
#endif
        }
    }
    Change_ldpc(ubLdpcCodeRateTemp);

    if (existNum != NandPara.ubBankNum)
    {
        return ERR_EMPTY;
    }

    return ERR_OK;
}
#endif

U32 llf_init(void)
{
    U32 ret = ERR_OK;
    U32 ulRegValue = 0;
#ifdef SLC_RRT
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DTV7) || defined(FTL_SSV2) || defined(FTL_SSV4) || defined(FTL_SSV5) || defined(FTL_SSV6) || defined(FTL_SSV7) || defined(FTL_H3DQV5)
    U32 RRTBaseAddr;
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DTV7) || defined(FTL_H3DQV5)
    U32 bank;
#endif
#endif
#endif

    PVENDOR_CMD_RESPONSE pVendorCmdResponse;
    pVendorCmdResponse = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

#if (defined(RL6577_VA) && defined(FOR_WEIKE)) || defined(RL6643_VA)
    GPIO_init(GPIO8 | GPIO9);
    GPIO_SetHigh(GPIO8 | GPIO9);
#endif

    llfLoadDefault();
    ret = llfmpInitParameter();
    if(ret != ERR_OK)
    {
        return ret;
    }
    llfInitParameters();

#if defined(RL6643_VA)
    llf_thermal_setting();
#if defined(FOR_SP)
    gubResetFromRDT = 0;
#endif
    if(_MEM08(CONFIG_BASE_VA_ADDR + CONFIG_FCCLKTHROT_EN) != 0)
    {
        llfprintk("Enable FC throtting, Super High: _%d_%d, High _%d_%d, LOW _%d_%d, FC CLK %d \r\n",
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPTHROT_UPPER),
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPTHROT_LOWER),
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_HIGHTEMPTHROT_UPPER),
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_HIGHTEMPTHROT_LOWER),
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LOWTEMPTHROT_UPPER),
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_LOWTEMPTHROT_LOWER),
                  _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_TEMPTHROT_FC_CLOCK));
    }
#endif
#if defined(RL6577_VA) || defined(RL6447_VA) ||defined(RTS5771_VA)
    //need to init fc clk, otherwise fc will hang when access fc register
    if(gubNandDefaultMode == 0)
        InitFCClk_onfi4(IF_ONFI_SDR, FC_PLL_CLK_10M);
    else
        InitFCClk_onfi4(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M);
#ifdef RTS5771_VA
    FR_G_PHY_REG32_W(ONFI_PAD_CEN_CTRL_2, 0x15ffff);
#else
    FR_G_PHY_REG32_W(ONFI_PAD_CEN_CTRL_2, 0x4affff);
#endif
#endif

    if(gubNandDefaultMode == 0)
    {
        FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_SDR);
    }
    else
    {
        FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_DDR2_TOGGLE);
    }

#if defined(RL6643_VA)
    pmu_sleep_pull_high();
#endif
    ulRegValue = READ_REG_32(CPU_MODE_REG);
    WRITE_REG_32(CPU_MODE_REG, (ulRegValue | FC_WP_OE | FC_WP_OFF));

#if defined(RL6577_VA)||defined(RL6447_VA)||defined(RTS5771_VA)
    gubDqsFix90 = 1;
#if defined(RL6447_VA)
    gubThreePointKSetting = 0;
#else
    gubThreePointKSetting = 3;
#endif

    if(gubNandDefaultMode == 0)
    {
        ChangeFCClk(ONFI_SDR, FC_PLL_CLK_10M);
        onfi4_change_setting(ONFI_SDR, FC_PLL_CLK_10M, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG, 0);
    }
    else
    {
        ChangeFCClk(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M);
        onfi4_change_setting(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG, 0);
    }
#endif

#if (defined(RL6577_VA)||defined(RTS5771_VA)) && defined(AIPR_EN)
    if(NandPara.ubChNum == 4)
    {
        printk("Check FC_DIE_MAPPING for AIPR.\r\n");
        U32 table_addr = CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE;
        if(*(U8 *)(table_addr + 0 * CMD_FIFO_NUM_PER_CH) == 0x0 &&
                *(U8 *)(table_addr + 2 * CMD_FIFO_NUM_PER_CH) == 0x2)
        {
            printk("PASS.\r\n");
        }
        else
        {
            printk("The other case.\r\n");
            return ERR_CONFIG_MAPPING;
        }
    }
#endif
    if(CheckDuplicateBank(CONFIG_BASE_VA_ADDR))
    {
        llfprintk("Mapping has duplicate bank\r\n");
        return ERR_CONFIG_MAPPING;
    }
    ConvertBankToChCe(CONFIG_BASE_VA_ADDR, BANK_IMAPPING_TABLE_ADDR);
#ifdef FC_1CH
    if((gubBank0CH << 4 | gubBank0CE) != _REG08(BANK_IMAPPING_TABLE_ADDR))
    {
        llfprintk("Bank0: CH%d CE%d, Bank1: CH%d CE%d\r\n", gubBank0CH, gubBank0CE, gubBank1CH, gubBank1CE);
        return ERR_CONFIG_MAPPING;
    }
#else
    if(((gubBank0CH << 4 | gubBank0CE) != _REG08(BANK_IMAPPING_TABLE_ADDR)) ||
            ((gubBank1CH << 4 | gubBank1CE) != _REG08(BANK_IMAPPING_TABLE_ADDR + 1)))
    {
        llfprintk("Bank0/1 mapping is NOT corresponding to config mapping\r\n");
        llfprintk("Bank0: CH%d CE%d, Bank1: CH%d CE%d\r\n", gubBank0CH, gubBank0CE, gubBank1CH, gubBank1CE);
        return ERR_CONFIG_MAPPING;
    }
#endif

    SetParserSequencerTable(PARSER_TABLE_VA_ADDR, SEQUENCER_TABLE_VA_ADDR);
    SetParserSequencerIndex(PARSER_INDEX_ADDR, SEQUENCER_INDEX_ADDR, AUTO_INSERT_INDEX_ADDR);

    HlcParserIndexInit();
    llfFcInitReg();
#ifndef RL6531_VB
    gubLdpcCodeRate = _REG08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LDPC_CODE_RATE);
    guwLdpcParityLen = _REG16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_LDPC_PARITY_LEN);
#endif
#if defined(RL6447_VA)||defined(RL6531_VB)
    gubECC_CFG = gubECC_CFG0;
#else
    gubECC_CFG = gubLdpcCodeRate;//should keep same as config value
#endif
    Change_ldpc(gubECC_CFG);
    llfprintk("ECC cfg:%d, coderate:%x, ecclength:%d\r\n", gubECC_CFG, gubLdpcCodeRate,
              guwLdpcParityLen);

    //Check if LDPC code rate is OK
#ifdef IS_8K_PAGE
    if(NandPara.uwFullPageSize < NandPara.ubSectorNumPerPage * 512)
#else
    if(NandPara.uwFullPageSize < NandPara.ubSectorNumPerPage * 512
            || NandPara.ubSectorNumPerPage != 32)
#endif
    {
        return ERR_CONFIG_LDPC_CODERATE;
    }
    else
    {
        // gub_Total_len_per_2K already be setted in bl_FcInitColumnAddr
#ifdef IS_8K_PAGE
        if(NandPara.uwFullPageSize < (gub_Total_len_per_2K << 2))
#else
        if(NandPara.uwFullPageSize < (gub_Total_len_per_2K << 3))
#endif
        {
            return ERR_CONFIG_LDPC_CODERATE;
        }
    }

#ifdef AUTO_DETECT_DIE
    llfAutoDetectDie();
    if (ret != ERR_OK)
    {
        llfPrintNandPara();
        return ret;
    }
#endif
#ifdef BLK_REMAP_PRO
#ifdef RDT_REMAP
    if(_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
#endif
    {
        ret = llfBlkRemapInit();
        if (ret != ERR_OK)
        {
            return ret;
        }
    }
#endif

    //llfResetNand();

    ret = llfinitConfigBE(pVendorCmdResponse);
    if (ret != ERR_OK)
    {
        llfPrintNandPara();
        return ret;
    }
#if defined(RL6577_VA)||defined(RTS5771_VA)
#if defined(KEEP_RDT_RESULT)
    gubExtendRdtResultBlk = RDT_RESULT_BLK_EXTEND_NUM;
    if(llfCheckOriginalRDTResultExist() != ERR_OK)
    {
        gubExtendRdtResultBlk = 0;
    }
#else
    gubExtendRdtResultBlk = 0;
#endif
    printk("[KEEP RDT RESULT] check extend RDT blk num : %d\r\n", gubExtendRdtResultBlk);
#endif

#if defined(RL6577_VA)
    U8 disablereadpara;
    disablereadpara = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_DISABLE_RAED_PARA);
    if(FLASH_VENDOR(gulFlashVendorNum) != IS_SANDISK && !disablereadpara)
    {
        NandReadAllPara();
    }
#ifdef VID_CHECK
    if (FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON && !disablereadpara)
    {
        U32 BeOption, VIDresult;
        BeOption = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_BE_OPTION1);
        VIDresult = NandReadVID();
        if(VIDresult & VID_NO_SNAPREAD)
        {
            BeOption = (0xb << 28) | ((BeOption & 0xfffffff) | VIDresult);
        }
        _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_BE_OPTION1) = BeOption;
        printk("BeOption %x\r\n", _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_BE_OPTION1));
    }
#endif
#ifdef UID_CHECK
    if(disablereadpara == 0 && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YG2T)
    {
        U32 UIDret;
        UIDret = NandReadUID();
        if(UIDret == UID_FAIL)
        {
            printk("[ERR] Read UID fail.\r\n");
        }
        else
        {
#ifdef YX2T_STATUS_CHECK
            U32 bank;
            U8 failflag = 0;
            U8 FifthByte;
            for(bank = 0; bank < NandPara.ubBankNumPerLun; bank++)
            {
                FifthByte = _REG08(TEMP_BUF_ADDR + 4 + bank * 32);
                if(FifthByte == 0x36 || FifthByte == 0x37)
                {
                    printk("Bank %d 5th byte of UID is 0x%d and will be going to check status.\r\n", bank,
                           FifthByte - 18);
                    U32 StateCheckResult = FCYX2TBugStateCheck(ONFI_DDR2_TOGGLE, bank, 0);
                    if(StateCheckResult == YX2T_CHECKSTATUS_HAVE_TO_RETRIM)
                    {
                        printk("[WARN] Bank %d have to retrim.\r\n", bank);
                        failflag |= 0x1;
                        AddErrorMessage(bank, 0, ERR_CHECK_STATUS_RETRIM);
                    }
                    else if(StateCheckResult == YX2T_CHECKSTATUS_NOT_TO_RETRIM)
                    {
                        printk("Bank %d check status pass.\r\n", bank);
                    }
                    else
                    {
                        printk("[ERR] Bank %d check status fail.\r\n", bank);
                        failflag |= 0x2;
                        AddErrorMessage(bank, 0, ERR_CHECK_STATUS_FAIL);
                    }
                }
            }
            if(failflag & 0x2)
            {
                return ERR_CHECK_STATUS_FAIL;
            }
            else if(failflag & 0x1)
            {
                return ERR_CHECK_STATUS_RETRIM;
            }
#endif
        }
    }
#endif
    U32 sys_mapping;
    U8 ExternalTempExistFlag;

    sys_mapping = _MEM32(CONFIG_BASE_VA_ADDR + CONFIG_FE_OPTION1);
    if((sys_mapping >> 0x1C) == 0xF)
    {
        ExternalTempExistFlag = (sys_mapping >> 0x04) & 1;
    }
    else
    {
        ExternalTempExistFlag = 0;
    }
    if(ExternalTempExistFlag)
    {
        I2C_STATUS i2c_status1 = 0xFFFF;
        U8 result = 0x0;
        U32 SMBUS;
        U32 val;

        val = READ_REG_32(I2C_REG_ENABLE_STATUS);
        while (val & I2C_EN)
        {
            WRITE_REG_32(I2C_REG_ENABLE, 0);
            val = READ_REG_32(I2C_REG_ENABLE_STATUS);
        }

        SMBUS = _REG32(SMBUS_PAD_MODE);
        _REG32(SMBUS_PAD_MODE) = I2C_MODE;
        i2c_status1 = i2c_init_ENE(0x48);
        if (i2c_status1 != I2C_OK)
        {
            printk("I2C thermal init timeout.\r\n");
            return ERR_AUTO_DETECT;
        }
        i2c_status1 = i2c_transfer_byte_rd(0x00, &result);
        _REG32(SMBUS_PAD_MODE) = SMBUS;
        if (i2c_status1 != I2C_OK)
        {
            printk("I2C thermal get temp timeout.\r\n");
            return ERR_AUTO_DETECT;
        }
        printk("[EXT]temp1:%d\r\n", result);
    }
#endif

#if defined(FTL_N38A) || defined(FTL_N38B)
    llfInitErrorMessage();
    if(ERR_OK != FCSetFeatureAndCheck(ONFI_DDR2_TOGGLE, 0x91, 0x206,
                                      GETFEATURE_BY_BANK_UC, GETFEATURE_BY_BANK_PHY))
    {
        return ERR_SET_FEATURE;
    }
#endif

#if defined(FTL_Q5171A)
    llfInitErrorMessage();
    if(ERR_OK != FCSetFeatureAndCheck(ONFI_DDR2_TOGGLE, 0x8B, 0x0,
                                      GETFEATURE_BY_BANK_UC, GETFEATURE_BY_BANK_PHY))
    {
        return ERR_SET_FEATURE;
    }
#endif

#if defined (FTL_H3DQV5) && (defined(RL6577_VA)||defined(RTS5771_VA))
    llfInitErrorMessage();
    ret = llfHynixBuildRRT();
    if(ret != ERR_OK)
    {
        return ERR_RRT_REV;
    }
#endif
#ifdef SLC_RRT
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DTV7) || defined(FTL_SSV2) || defined(FTL_SSV4) || defined(FTL_SSV5) || defined(FTL_SSV6) || defined(FTL_SSV7) || defined(FTL_H3DQV5)
    RRTBaseAddr = llfInitMemLayout();
#if defined(FTL_SSV5) || defined(FTL_SSV6) || defined(FTL_SSV7)
    InitTLCRetrySetting(RRTBaseAddr);
#elif defined(FTL_SSV2) || defined(FTL_SSV4)
    InitSLCRetrySetting(RRTBaseAddr);
#endif
#if defined (FTL_H3DTV3) || defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DTV7) || defined(FTL_H3DQV5)
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        ret = llfHynixBuildRRT(bank, RRTBaseAddr);
        if(ret == ERR_OK)
        {
            break;
        }
        else if (bank == NandPara.ubBankNum - 1)
        {
            memset((void *)RRTBaseAddr, 0, sizeof(struct _HYNIX_RETRY_SLC));
        }
    }
    if (ret != ERR_OK)
    {
        printk("Get manual table!\r\n");
        InitSLCRetrySetting(RRTBaseAddr);
        ret = ERR_OK;
    }
#endif
#endif
#endif

#ifdef NEW_MUL_WR_CACHE
    U8 ubIFType, ubClkMode;
    ubIFType = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
    ubClkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);
    gubCalibrateConfig  = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_AUTO_CALIBRATE_EN);

    // 1.force auto-k
    // 2.inherit auto-k + RDT  3.inherit auto-k + clear all
#ifdef AUTO_K_ENABLE
    if((gubCalibrateConfig == 1) || (gubCalibrateConfig == 3) ||
            ((gubCalibrateConfig == 2) && (gubLLFMode == LLF_FORCE_FORMAT)))
    {
        if(gubCalibrateConfig == 2)
        {
            gubCalibrateConfig = 1;  // reset to 1
        }
        llfAPBECalibrateTxAuto(ubIFType, ubClkMode);
        if(pVendorCmdResponse->err_msg_num != 0)
        {
            return pVendorCmdResponse->res_err_code;
        }
        llfAPBECalibrateRxAuto(ubIFType, ubClkMode);
        if(pVendorCmdResponse->err_msg_num != 0)
        {
            return pVendorCmdResponse->res_err_code;
        }
    }
    else
#endif
    {
        // inherit driving
        if(gubCalibrateConfig == 2)
        {
            gubCalibrateConfig = 1;  // reset to 1
            ret = llfInheritDriving(ubIFType);
            if(ret != ERR_OK)
            {
                // TODO:: need to reset FC, or calibrate may fail
                // UART REDUCE: [ERR] Inherit driving fail, use default
                llfprintk("LLF inherit driving fail\r\n");
            }
            llfInitErrorMessage();
        }
        llfAPBECalibrate(ubIFType, ubClkMode);
        if(pVendorCmdResponse->err_msg_num != 0)
        {
            return pVendorCmdResponse->res_err_code;
        }
    }

    ret = repeated_WriteReadCache(MUL_WR_CACHE_COUNT);
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "Multi write and read cache error!!!\r\n");
    }
#endif

#ifdef SPECIAL_SYSTEM_BLOCK
    memset((void*)SP_DBT_ADDR, 0x0, SP_DBT_SIZE);
    cache_area_dwbinval(SP_DBT_ADDR, SP_DBT_SIZE);
    cache_dummy_update_read();
#endif

    return ret;
}

void llfmpInit(void)
{
#if defined(RL6643_VA) || defined(RL6577_VA) || defined(RL6447_VA) ||defined(RTS5771_VA)
    //for Dump LLF Log
    gubLLFMemMsgFlag = 0;
    gulLLFMemMsgFreeRoom = LLFMEMMSGBUF_TOTALSIZE;
    gubCalibrateConfig = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_AUTO_CALIBRATE_EN);
    llfprintk("AutoK 0x%x, Log Addr 0x%x, Size 0x%x\r\n",
              gubCalibrateConfig, LLFMEMMSGBUF_BASE_ADDR, LLFMEMMSGBUF_TOTALSIZE);
#endif

    U32 ret;
    PVENDOR_CMD_RESPONSE pVendorCmdResponse;
#ifndef NOFE_LLF_FLOW
    entry_t entry = 0;
    U32 lock_flag;
#endif

    // Point the Vendor command to cmd address
    pVendorCmdResponse = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;// SRAM used for vendor cmd response
    if(pVendorCmdResponse->res_state == VENDOR_CMD_START)
    {
        // LLF is running
        pVendorCmdResponse->res_state = VENDOR_CMD_EXECUTE;
        pVendorCmdResponse->res_progress = 0;
        pVendorCmdResponse->res_err_code = ERR_OK;
    }

    ret = llf_init();

    // update cmd state to no command
    pVendorCmdResponse->res_progress = 100;
    pVendorCmdResponse->res_state = VENDOR_CMD_IDLE;
    pVendorCmdResponse->res_err_code = ret;

    // show the return value.
    PrintDone();

    // send msg to FE.
#ifndef NOFE_LLF_FLOW
    entry = pVendorCmdResponse->entry;
    gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
    spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
    SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
    spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
#endif
}

void llfAPReadNAND()
{
    PVENDOR_CMD_RESPONSE pLLF_response_info;
    entry_t entry = 0;
    U32 lock_flag;
    U32 ulMode, cmp, ret = ERR_OK;
    U8 ch, ce, value, reg;
    U8 ubBankno;
    U16 uwBlockno, uwPageno;

    //get Bank_no
    ce = (_REG32(LLF_READ_NAND_CHCE) & 0xF);
    ch = (_REG32(LLF_READ_NAND_CHCE) >> 4 ) & 0xF;
    reg = ch * 2 + ce / 4;
#if defined(RL6531_VB)
    value = FC_TOP_REG(FR_FC_CE_MAP0 + reg * 4);
#elif defined(RL6577_VA)||defined(RL6447_VA)||defined(RL6643_FPGA)||defined(RL6643_VA) || defined(RTS5771_FPGA) || defined(RTS5771_VA)
    value = FC_TOP_REG(FR_PARSER_TO_LOGIC_CMDBUF_MAP0 + reg * 4);
#endif
    ubBankno = (value >> (8 * (ce % 4))) & (0xff);
    //get Block_no & Page_no
    uwBlockno = (_REG32(LLF_READ_NAND_BLK_PAGE) >> 16) & 0xffff;
    uwPageno = _REG32(LLF_READ_NAND_BLK_PAGE) & 0xffff;

    //response info
    pLLF_response_info = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;// SRAM used for responding to host
    pLLF_response_info->res_data_addr = TEMP_BUF_ADDR;
    pLLF_response_info->res_progress = 0;
    pLLF_response_info->err_msg_num = 0;

    //cache invalid
    cache_area_dinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    gul_FW_TAG = llfBETagSetting(TAG_READ, ubBankno);
    llfDbgPrintk(ALWAYS_MSG, "llfAPReadNAND:%d_%d_%d.\r\n", ubBankno, uwBlockno, uwPageno);
    llfFCCmdRead_DRAM(ulMode, ubBankno, 0, uwBlockno, uwPageno, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                      TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if (ret == ERR_FIO_TIMEOUT)
    {
        llfDbgPrintk(ALWAYS_MSG, "ret:0x%x.\r\n", ret);
    }
    else
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "cmp:0x%x\r\n", cmp);
        }
        llfDbgPrintk(ALWAYS_MSG, "BlkId:0x%x\r\n", _REG32(TEMP_HBUF_ADDR));
    }

    //send msg to FE
    pLLF_response_info->res_progress = 100;
    pLLF_response_info->res_state = VENDOR_CMD_IDLE;
    pLLF_response_info->res_err_code = 0;

    entry = pLLF_response_info->entry;
    gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
    spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
    SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
    spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
}

void llfAPReadRDTNandLog()
{
    PVENDOR_CMD_RESPONSE pLLF_response_info;
    entry_t entry = 0;
    U32 ulMode, ulCmp, ulRet, ulFlag, ulECCThv;
    U8 ubReadTimes, ubFailSeq, ufFinishThisBlock, ufReadDone, ubMaxReadTimes;

    ulRet = ERR_OK;
    ulFlag = 0;
    ubReadTimes = ubFailSeq = 0;
    ufFinishThisBlock = ufReadDone = FALSE;

    // Data buffer size 48k, need to be less than 12
    ubMaxReadTimes = 11;

    //response info
    pLLF_response_info = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;// SRAM used for responding to host
    pLLF_response_info->res_data_addr = TEMP_BUF_ADDR;
    pLLF_response_info->res_progress = 0;
    pLLF_response_info->err_msg_num = 0;

    cache_area_dinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_area_dinval(TEMP_BUF_ADDR, TEMP_BUF_SIZE);

    cache_dummy_update_read();

    _MEM32(TEMP_BUF_ADDR) = READ_RDT_NANDLOG_STOP;

    ulECCThv = FR_G_CFG_REG32(FR_ECC_THV);
    FR_G_CFG_REG32_W(FR_ECC_THV, 0x50);

    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    while(ufReadDone == FALSE)
    {
        gul_FW_TAG = llfBETagSetting(TAG_READ, gsRDTLog.ubBank);
        llfFCCmdRead_DRAM(ulMode, gsRDTLog.ubBank, 0, gsRDTLog.ubBlock, gsRDTLog.uwPage,
                          (TEMP_BUF_PHY_ADDR + RDT_NAND_LOG_4K * ubReadTimes), 0x1000,
                          TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE / 4);

        ulRet = FCCompletionPolling(&ulCmp, (gul_FW_TAG));
        if (ulRet != ERR_OK)
        {
            llfDbgPrintk(ALWAYS_MSG, "[LOG][wsRDT] ERR bank %d block %d page %d ret %x\r\n",
                         gsRDTLog.ubBank, gsRDTLog.ubBlock, gsRDTLog.uwPage, ulRet);
            ubFailSeq++;
            if(ubFailSeq > 30)
            {
                ufFinishThisBlock = TRUE;
            }
        }
        else if((ulCmp & BE_COMPLETION_ERROR_MASK) != 0)
        {
            if((ulCmp & ALL_FF) && (ulCmp & CRC_ERR))
            {
                llfprintk("[LOG][wsRDT] Empty bank %d block %d page %d\r\n",
                          gsRDTLog.ubBank, gsRDTLog.ubBlock, gsRDTLog.uwPage);
                ufFinishThisBlock = TRUE;
            }
            else
            {
                llfDbgPrintk(ALWAYS_MSG, "[LOG][wsRDT] ERR bank %d block %d page %d cmp %x\r\n",
                             gsRDTLog.ubBank, gsRDTLog.ubBlock, gsRDTLog.uwPage, ulCmp);

                ubFailSeq++;
                if(ubFailSeq > 30)
                {
                    ufFinishThisBlock = TRUE;
                }
            }
        }
        else
        {
            if(_MEM32(TEMP_HBUF_ADDR) == RDTNANDLOG_BLK_ID)// Read OK
            {
                ubReadTimes++;
                _MEM32(TEMP_BUF_ADDR + (RDT_NAND_LOG_4K * ubReadTimes)) = READ_RDT_NANDLOG_CONT;
                if(gsRDTLog.ufFoundBlock == FALSE)
                {
                    gsRDTLog.ufFoundBlock = TRUE;
                    gsRDTLog.ubGetBlock++;
                    guwCaclSSGroupBegin = _MEM16(TEMP_HBUF_ADDR + 0x0008);
                    guwCaclSSGroupEnd = _MEM16(TEMP_HBUF_ADDR + 0x000a);
                    llfDbgPrintk(ALWAYS_MSG, "[LOG][wsRDT] Bank %d Block %d SSBegin %d SSEnd %d flag %x\r\n",
                                 gsRDTLog.ubBank, gsRDTLog.ubBlock, guwCaclSSGroupBegin, guwCaclSSGroupEnd, ulFlag);
                }
            }
            else
            {
                if(gsRDTLog.uwPage == 0)
                {
                    llfDbgPrintk(ALWAYS_MSG, "[LOG][wsRDT] Unexpected header: bank %d block %d\r\n",
                                 gsRDTLog.ubBank, gsRDTLog.ubBlock);
                }
                ufFinishThisBlock = TRUE;
            }
        }

        if(ufFinishThisBlock == TRUE)
        {
            ubFailSeq = 0;
            gsRDTLog.uwPage = NandPara.uwSLCPageNumPerBlock - 1; // Skip this block
            ufFinishThisBlock = FALSE;
        }

        gsRDTLog.uwPage++;
        if(gsRDTLog.uwPage == NandPara.uwSLCPageNumPerBlock)
        {
            gsRDTLog.uwPage = 0;
            gsRDTLog.ubBlock++;
            gsRDTLog.ufFoundBlock = FALSE;

            if((gsRDTLog.ubBlock >= (guwCaclSSGroupEnd * NandPara.ubPlaneNumPerLun))
                    && (gsRDTLog.ubGetBlock < SNAPSHOT_BS_NUM_PER_DIE))
            {
                ufReadDone = TRUE;
                _MEM32(TEMP_BUF_ADDR + (RDT_NAND_LOG_4K * ubReadTimes)) = READ_RDT_NANDLOG_STOP;
            }
            else if(gsRDTLog.ubGetBlock == SNAPSHOT_BS_NUM_PER_DIE)
            {
                gsRDTLog.ubBlock = (guwCaclSSGroupBegin * NandPara.ubPlaneNumPerLun);
                gsRDTLog.ubGetBlock = 0;
                gsRDTLog.ubBank++;
                if(gsRDTLog.ubBank == NandPara.ubBankNum)
                {
                    ufReadDone = TRUE;
                    _MEM32(TEMP_BUF_ADDR + (RDT_NAND_LOG_4K * ubReadTimes)) = READ_RDT_NANDLOG_STOP;
                }
            }
        }
        if (ubReadTimes == ubMaxReadTimes)
        {
            ufReadDone = TRUE;
        }
    }

    FR_G_CFG_REG32_W(FR_ECC_THV, ulECCThv);

    //send msg to FE
    pLLF_response_info->res_progress = 100;
    pLLF_response_info->res_state = VENDOR_CMD_IDLE;
    pLLF_response_info->res_err_code = 0;

    entry = pLLF_response_info->entry;
    gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
    spin_lock_irqsave(&g_be2fe_admin_lock, &ulFlag);
    SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
    spin_unlock_irqrestore(&g_be2fe_admin_lock, &ulFlag);
}

void llfMpSelfTest(void)
{
#if 0
    U32 test_addr = DRAM_START;
    U32 test_len = 64 * 1024;
    U32 retaddr = 0, retval = ERR_INIT;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    entry_t entry;
    U32 lock_flag;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

    retval = MPDdrTest((U32*)test_addr, test_len, (U32*)&retaddr);
    if(retval != ERR_OK)
    {
        pResponseInfo->res_state = VENDOR_CMD_IDLE;
        pResponseInfo->res_progress = 100;
        pResponseInfo->res_err_code = ERR_DRAM_TEST;
        pResponseInfo->err_msg_num = 1;
        WRITE_REG_32(LLF_RES_ERRMSG_START_VA_ADDR, retaddr);
    }
    else
    {
        pResponseInfo->res_state = VENDOR_CMD_IDLE;
        pResponseInfo->res_progress = 100;
        pResponseInfo->res_err_code = ERR_OK;
        pResponseInfo->err_msg_num = 0;

        pResponseInfo->err_msg_num = 0;
        WRITE_REG_32(LLF_RES_ERRMSG_START_VA_ADDR, 0);
    }

    entry = pResponseInfo->entry;
    gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
    spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
    SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
    spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
#endif
}

void llfCheckThenErase(U8 ubIFType, U8 ubClkMode)
{
    PVENDOR_CMD_RESPONSE pResponseInfo;
    struct _LLF_UNI_INFO *pLLFUniInfo;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (struct _LLF_UNI_INFO *)LLF_UNI_INFO_ADDR;

    if(gubLLFSubStep.ubLlfStep == STEP_LLF_INHERIT)
    {
        // we check defect here, so we do NOT need to check it later
#if 0
        CheckDefectMark();
#else
        while(pResponseInfo->res_state != VENDOR_CMD_IDLE)
        {
            llfAPBuildDBT();
        }
#endif

        // erase all blocks besides marked defect
        llfAPEraseAll(ERASE_ALL_WITH_DBT);

        // if we check defect in advance, we can directly write SBlock and DBT, no need to check again
        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pLLFUniInfo->ulWorkFunc = WRITE_CONFIG_FUNC;
        gubLLFSubStep.ubLlfStep = STEP_LLF_EXECUTE;
    }
    else
    {
        llfAPExecuteLLF(ubIFType, ubClkMode);
    }
}

U32 llfEraseThenCheck(U8 ubIFType, U8 ubClkMode)
{
    PVENDOR_CMD_RESPONSE pResponseInfo;
    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    U8 i;
    U32 time_tmp;
    U8 ubErsMode = ERASE_ALL_WITH_DBT;

#if defined(RTS5771_FPGA) || defined(RTS5771_VA) || defined(RL6577_FPGA) || defined(RL6577_VA)
    time_tmp = GET_TICK;
#else
#if defined(RL6643_VA)
    gulTicks = (READ_REG_32(FAST_REG_CNT0) ? READ_REG_32(FAST_REG_CNT0) : READ_REG_32(FAST_REG_CNT0));
#endif
    time_tmp = gulTicks;
#endif

#ifdef BLK_REMAP_PRO
    if (gubBlkRemapProFlag == 1)
    {
        ubErsMode = ERASE_ALL;
    }
#endif

#ifdef SPECIAL_SYSTEM_BLOCK
    ubErsMode = ERASE_ALL;
#endif

    // erase all blocks besides marked defect
    if((pResponseInfo->res_state == VENDOR_CMD_ERASE)
            || (pResponseInfo->res_state == VENDOR_CMD_ERASE_CT))
    {
        if(!gfSelfTestFlag)
        {
            if(pResponseInfo->res_state == VENDOR_CMD_ERASE_CT)
                pResponseInfo->res_state = VENDOR_CMD_EXECUTE;

            while((pResponseInfo->res_state == VENDOR_CMD_ERASE)
                    || (pResponseInfo->res_state == VENDOR_CMD_EXECUTE))
            {
                gubErasetimeout = 0;
                llfAPEraseAll(ubErsMode);
                llfBECheckStatus();
                if( gubErasetimeout == 1 )
                {
                    printk("[FAIL]Erase polling timeout %d\r\n", gufLastEraseFlag);
                    for(i = 0; i < CH_NUM_MAX * CE_NUM_MAX; i++)
                    {
                        if(gubEraseCmpNum[i] != 0)
                        {
                            llfAddErrorMessage(i, 0, ERR_FIO_TIMEOUT);
                            printk("bank:%d->%d\r\n", i, gubEraseCmpNum[i]);
                        }
                    }
                    return ERR_FIO_TIMEOUT;
                }
#if defined(RTS5771_FPGA) || defined(RTS5771_VA) || defined(RL6577_FPGA) || defined(RL6577_VA)
                if(GET_TICK - time_tmp > 1000)
                {
                    time_tmp = GET_TICK;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE_CT;
                    return ERR_OK;
                }
#else
#if defined(RL6643_VA)
                gulTicks = (READ_REG_32(FAST_REG_CNT0) ? READ_REG_32(FAST_REG_CNT0) : READ_REG_32(FAST_REG_CNT0));
#endif
                if(gulTicks - time_tmp > 1000)
                {
                    time_tmp = gulTicks;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE_CT;
                    return ERR_OK;
                }
#endif
            }
        }
        return ERR_OK;
    }

    if((pResponseInfo->res_state == VENDOR_CMD_BUILD_DBT)
            || (pResponseInfo->res_state == VENDOR_CMD_EXECUTE))
    {
        if(!gfSelfTestFlag)
        {
            llfAPBuildDBT();
        }
        return ERR_OK;
    }

    if((pResponseInfo->res_state == VENDOR_CMD_WRITE_FW) ||
            (pResponseInfo->res_state == VENDOR_CMD_WRITE_FW_CT2))
    {
        llfAPExecuteLLF(ubIFType, ubClkMode);
    }
    else
    {
        ASSERT(LLF_MSG, 0);
    }
    return ERR_OK;
}

U32 llfCheckFlashSignature()
{
    U32 isErr = ERR_OK;

    switch(_MEM32(BE_VERSION_TAG))//need to check
    {
    case SIG_B0KB:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B0K)
        {
            isErr = 0x01;
        }
        break;
    case SIG_BICS:
    case SIG_SBIC:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_8T22_SDR) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT23_SDR_TOGGLE_64GB) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_9T23_SDR_TOGGLE) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XW23))
        {
            isErr = 0x02;
        }
        break;
    case SIG_B16A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B16)
        {
            isErr = 0x03;
        }
        break;
    case SIG_B17A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B17)
        {
            isErr = 0x04;
        }
        break;
    case SIG_B27A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B27A)
        {
            isErr = 0x05;
        }
        break;
    case SIG_B27B:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B27B)
        {
            isErr = 0x06;
        }
        break;
    case SIG_B37R:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B37R)
        {
            isErr = 0x07;
        }
        break;
    case SIG_N18A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N18A)
        {
            isErr = 0x08;
        }
        break;
    case SIG_N28A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N28)
        {
            isErr = 0x09;
        }
        break;
    case SIG_IN28:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N28)
        {
            isErr = 0x0A;
        }
        break;
    case SIG_SSV4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV4
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV4_64G)
        {
            isErr = 0x0B;
        }
        break;
    case SIG_SSV5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV5
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV5_64G)
        {
            isErr = 0x0C;
        }
        break;
    case SIG_SSV6:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV6
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV6_1v8
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV6_512Gb
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV6_512Gb_1v8)
        {
            isErr = 0x0D;
        }
        break;
    case SIG_H3V5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV5)
        {
            isErr = 0x0E;
        }
        break;
    case SIG_HQV5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DQV5)
        {
            isErr = 0x0F;
        }
        break;
    case SIG_H3V6:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV6)
                && (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV6_1Tb))
        {
            isErr = 0x10;
        }
        break;
    case SIG_YG2T:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YG2T) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YG2T_CS2))
        {
            isErr = 0x11;
        }
        break;
    case SIG_YX2T:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YX2T)
        {
            isErr = 0x12;
        }
        break;
    case SIG_TBI4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24_64G)
        {
            isErr = 0x13;
        }
        break;
    case SIG_SBI4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT24_64G
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS4P5_256Gb
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS4P5_512Gb)
        {
            isErr = 0x14;
        }
        break;
    case SIG_TBI5:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT25_TOGGLE_64GB)
                && (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XT25_TOGGLE_128GB))
        {
            isErr = 0x15;
        }
        break;
    case SIG_B47R:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B47R)
        {
            isErr = 0x16;
        }
        break;
    case SIG_N38A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N38A)
        {
            isErr = 0x17;
        }
        break;
    case SIG_SBI5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS5_512Gb
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS5_1024Gb
                && FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS5_1024Gb_ODT)
        {
            isErr = 0x18;
        }
        break;
    case SIG_TBQ4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_XF24)
        {
            isErr = 0x19;
        }
        break;
    case SIG_H3V7:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV7_512Gb) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV7_1Tb))
        {
            isErr = 0x1A;
        }
        break;
    case SIG_YX2Q:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YX2Q)
        {
            isErr = 0x1B;
        }
        break;
    case SIG_SBQ4:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS4_QLC)
        {
            isErr = 0x1C;
        }
        break;
    case SIG_H3V4:
        if((FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV4) &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV4_512Gb))
        {
            isErr = 0x1D;
        }
        break;
    case SIG_N48R:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N48R)
        {
            isErr = 0x1E;
        }
        break;
    case SIG_SBQ5:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_BICS5_QLC)
        {
            isErr = 0x1F;
        }
        break;
    case SIG_H3V3:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_H3DTV3)
        {
            isErr = 0x20;
        }
        break;
    case SIG_YX3T:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YX3T_WYS &&
                (FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YX3T_WDS))
        {
            isErr = 0x21;
        }
        break;
    case SIG_SSV7:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV7_512Gb)
        {
            isErr = 0x22;
        }
        break;
    case SIG_SSV2:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_SSV2_128G)
        {
            isErr = 0x23;
        }
        break;

    case SIG_B58R:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_B58R)
        {
            isErr = 0x24;
        }
        break;
    case SIG_YX3D:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_YX3T_WDS)
        {
            isErr = 0x24;
        }
        break;
    case SIG_N38B:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_N38B)
        {
            isErr = 0x27;
        }
        break;
    case SIG_Q57A:
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) != IS_Q5171A)
        {
            isErr = 0x28;
        }
        break;
    default:
    {
        isErr = 0xFF;
    }
    }
    return isErr;
}

void llfAll()
{
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PLLF_UNI_INFO pLLFUniInfo;
    U8 ubIFType, ubClkMode;
#ifdef NEW_MUL_WR_CACHE
    entry_t entry;
    U32 lock_flag;
    PVENDOR_CMD pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
#endif
    U32 ret = ERR_OK;
    U8  bank_no;
#ifdef SBLK_EXPAND
    U8 banknum;
#endif

#ifdef LLF_AUTO_RMA
    _REG32(TX_PART1_DONE_ADDR) = PRINT_IDLE;
    _REG32(TX_PART2_DONE_ADDR) = PRINT_IDLE;
#endif

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
    pResponseInfo->err_msg_num = 0;

    ubIFType = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
    ubClkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);
    gubCalibrateConfig  = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_AUTO_CALIBRATE_EN);
#ifdef SLC_RRT
    gubmaxslcretry = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_AUTO_RETRY_CNT);
    gubmaxslcretry = llfCompareReadRetrycnt();
#endif

#ifdef RL6643_VA
    gubFcClkOffset = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLK_DOWN_OFFSET);
#endif
    hlc_size_write_mp_cache_DRAM = 18;
    hlc_size_read_mp_cache_DRAM = 18;
    hlc_size_write_6p_cache_DRAM = 30;
    hlc_size_read_6p_cache_DRAM = 33;
    _REG08(SBLK_ADDR + SBLK_OFFSET_VTH_ENABLE) = 0;
    gubEraseNum = 0;

    if (_MEM32(FW_RDT_TAG) == SIGNATURE_RDT)
    {
        gubLLFMode = LLF_FORCE_FORMAT;
        gubRdtImg = 1;
    }
//    else if (_MEM32(FW_RDT_TAG) == SIGNATURE_WSRDT)
//    {
//        gubLLFMode = LLF_FORCE_FORMAT;
//        gubRdtImg = 1;
//    }

    if(gubLLFALLStep == STEP_CALIBRATE)
    {
        if (_MEM32(FW_RDT_TAG) != SIGNATURE_RDT)
        {
            ret = llfCheckFlashSignature();
            if(ret != ERR_OK)
            {
                llfEndResponce(ERR_FLASH_VENDOR);
                return;
            }
        }

        if(!gfSelfTestFlag)
        {
#ifndef NEW_MUL_WR_CACHE

#ifdef AUTO_K_ENABLE
#if defined(RL6577_VA) || defined(RL6447_VA) || defined(RL6643_VA) || defined(RL6531_VB) ||defined(RTS5771_VA)
            if(gubCalibrateConfig)
            {
                llfprintk("AUTO-CALIBRATION START, %d\r\n", pLLFUniInfo->ubData0);
#ifdef RL6643_VA
                if(pLLFUniInfo->ubData0 == AUTO_CALIBRATE_VREF_START)
                {
                    if(((gubCalibrateConfig >> 2) & 0x1))
                    {
                        Vref_10M_calibrate();
                    }
                    else
                    {
                        pLLFUniInfo->ubData0 = AUTO_CALIBRATE_TX_START;
                    }
                }
#endif
                if(pLLFUniInfo->ubData0 == AUTO_CALIBRATE_TX_START)
                {
                    llfAPBECalibrateTxAuto(ubIFType, ubClkMode);
                }
                else if(pLLFUniInfo->ubData0 == AUTO_CALIBRATE_RX_START)
                {
                    llfAPBECalibrateRxAuto(ubIFType, ubClkMode);
                }
                else
                {
                    llfprintk("[ERR] Invalid calibrate flow\r\n");
                    llfEndResponce(ERR_UNKNOWN_VENDOR_CMD);
                }
            }
            else
#endif
#endif
            {
                if(gubLLFSubStep.ubKStep == STEP_CALIBRATE_INIT)
                {
                    llfprintk("AUTO-CALIBRATION DISABLE\r\n");
                }
                llfAPBECalibrate(ubIFType, ubClkMode);
            }

#else

            entry = pResponseInfo->entry;
            pResponseInfo->res_state = VENDOR_CMD_IDLE;
            pResponseInfo->res_err_code = ERR_OK;
            pResponseInfo->res_progress = 100;
            if(pVendorCmd->subcmd == BE_LLF_ALL)
            {
                pResponseInfo->err_msg_num = 0;
                pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
                pResponseInfo->res_err_code = ERR_OK;
                pResponseInfo->res_progress = 30;
                if(gubLLFMode <= LLF_FORCE_INHERIT || (gubLLFMode == LLF_DEFECT_BAD_BLOCK))
                {
                    gubLLFALLStep = STEP_LOAD_RDT_START;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                }
                else if((gubLLFMode == LLF_FORCE_FORMAT))
                {
                    gubLLFALLStep = STEP_FORMAT_INIT;
                    pResponseInfo->res_state = VENDOR_CMD_ERASE;
                }
                else
                {
                    gubLLFALLStep = STEP_FORMAT_INIT;
                    pResponseInfo->res_state = VENDOR_CMD_BUILD_DBT;
                }
                _MEM08(LLF_RES_ERRMSG_START_VA_ADDR) = 0;
            }
            else if(pVendorCmd->subcmd == BE_LLF_CALIBRATE)
            {
                SetupJumpPoint(SSD_FEPRODUCER, (U32)jtDummy);
                PrintDone();
                gpHostAdminCmd[entry].message_type = MSG_BE_RESP;
                spin_lock_irqsave(&g_be2fe_admin_lock, &lock_flag);
                SllAddToTail(&BE2FEAdminCmdListCtrl, entry);
                spin_unlock_irqrestore(&g_be2fe_admin_lock, &lock_flag);
            }
#endif
        }
        else
        {
            pResponseInfo->res_err_code = ERR_OK;
        }

        if(pResponseInfo->res_err_code != ERR_OK)
        {
            llfEndResponce(pResponseInfo->res_err_code);
            return;
        }
        return;
    }

    // flow to download RDT code
    if ((_MEM32(FW_RDT_TAG) == SIGNATURE_RDT) && (ubRDTLLFNormalEn == 0))
    {
        gubRdtImg = 1;
#if defined RDT_REMAP || !defined BLK_REMAP_PRO
        if(gubLLFSubStep.ubLlfStep == STEP_LLF_INHERIT)
        {
            llfDbgPrintk(ALWAYS_MSG, "[llfDBG]download RDT code mode, erase blk0\r\n");
            gubLLFSubStep.ubLlfStep = STEP_LLF_ERASE;
        }
        llfTodoRDT();
        return;
#endif
    }

    if(gubLLFALLStep <= STEP_INHERIT_DONE)
    {
        if ((gubLLFMode == LLF_INHERIT_OR_FORMAT || gubLLFMode == LLF_FORCE_INHERIT)
                || gubLLFMode == LLF_DEFECT_BAD_BLOCK)
        {
            if(gubLLFSubStep.ubLlfStep == STEP_LLF_INHERIT)
            {
                llfDbgPrintk(ALWAYS_MSG, "InHerit mode %d\r\n", gubLLFMode);
                if(gubLLFALLStep == STEP_LOAD_RDT_START)
                {
                    pLLFUniInfo->ulData5 = 0;
                    pLLFUniInfo->uwData4 = 0;
                    if(llfInherit() != ERR_OK && (gubLLFMode == LLF_FORCE_INHERIT
                                                  || gubLLFMode == LLF_DEFECT_BAD_BLOCK))
                    {
                        llfEndResponce(ERR_READ_RDT);
                        return;
                    }
                    gubLLFALLStep = STEP_INHERIT_DONE;
                    gubLLFSubStep.ubLlfStep = STEP_LLF_ERASE;
#ifdef BLK_REMAP_PRO
#ifdef EXTEND_STATIC_DBT
                    if(gubNeedRebuildRemap == 0)
#endif
                    {
                        ret = LlfCheckRemappingTable();
                        if (ret != ERR_OK)
                        {
                            llfEndResponce(ERR_BLOCK_MAPPING);
                            return;
                        }
                    }
#endif
                    return;
                }
            }
            else
            {
                if(gubLLFSubStep.ubLlfStep == STEP_LLF_ERASE)
                {
                    llfDbgPrintk(ALWAYS_MSG, "InHerit Done From : %d ([2]RDT [3]Whole system)\r\n", gfDBTInitDone);
                }
                // we already have DBT, so we do NOT need to check defect again
                if (gfDBTInitDone != LLF_DBT_NONE)
                {
                    ASSERT(LLF_MSG, (gfDBTInitDone == LLF_DBT_RDT) || (gfDBTInitDone == LLF_DBT_SYSTEM));
                    pResponseInfo->res_state = VENDOR_CMD_WRITE_FW;
                    llfAPExecuteLLF(ubIFType, ubClkMode);
                    return;
                }
                gubLLFALLStep = STEP_FORMAT_INIT;
                ASSERT(LLF_MSG, pResponseInfo->res_state == VENDOR_CMD_ERASE);
            }
        }
    }

    // FLOW 2 - Force format (First LLF)
    if(gubLLFALLStep == STEP_FORMAT_INIT)
    {
        llfDbgPrintk(ALWAYS_MSG, "First LLF Mode: %d\r\n", gubLLFMode);
#ifdef SBLK_EXPAND
        llfSblkInit();
#endif
#ifdef COM_BANKING
        U8 block_no;
        gulFailedImageBitMap = 0xFFFFFFFF;
        // Clear DBT  Set default defect table of super block as zero
        memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
        cache_area_dwb(DBT_ADDR, DBT_SIZE);
        cache_dummy_update_read();
        memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
        cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
        cache_dummy_update_read();
#ifdef SBLK_EXPAND
        if(NandPara.ubBankNum > SYS_BANK_NUM)
            banknum = SYS_BANK_NUM;
        else
            banknum = NandPara.ubBankNum;

        for(bank_no = 0; bank_no < banknum; bank_no++)
#else
        for(bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
#endif
        {
            for(block_no = 0; block_no < SYS_BLK; block_no++)
            {
#ifndef SBLK_EXPAND
                if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block_no))
#else
                if(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no + gubSblkBankStart, block_no + gubSblkStart))
#endif
                {
                    continue;
                }
                gulFailedImageBitMap &= ~(1 << (block_no + bank_no * 4));
            }
        }

        llfDbgPrintk(ALWAYS_MSG, "gulFailedImageBitMap:0x%x from First LLF Mode\r\n", gulFailedImageBitMap);
#endif
        //init new BB info
        for(bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO + bank_no * SBLK_OFFSET_RDT_PER_BANK_SIZE) =
                0; //PFailNum
            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO + bank_no * SBLK_OFFSET_RDT_PER_BANK_SIZE +
                   SHORT_BYTE_SIZE) = 0; //EFailNum
            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_PER_BANK_FAIL_IFO + bank_no * SBLK_OFFSET_RDT_PER_BANK_SIZE + 2 *
                   SHORT_BYTE_SIZE) = 0; //RFailNum
            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_FACTORY_BAD + bank_no * SHORT_BYTE_SIZE) = 0; //FFailNum
            _MEM16(SBLK_ADDR + SBLK_OFFSET_RDT_BLOCK_BAD + bank_no * SHORT_BYTE_SIZE) = 0;
        }

#ifdef AVG_ERASE_COUNT_TEST
        //search && inherit data BS && L2p BS Erase Count
        //llfprintk("clear all to call llfSearchInheritBSEraseCount\r\n");
        ret = llfSearchInheritBSEraseCount();
        //ASSERT(ALWAYS_MSG, ret == ERR_OK);
        if(ret != ERR_OK) // no dynamic sblock, no static block ,no inherit frome RDT result, set 0
        {
            gulDataBsAvgEraseCnt = 0;
            gulL2PBsAvgEraseCnt = 0;
        }
#endif

        gfDBTInitDone = LLF_DBT_INIT;  // initialize DBT done, we can do erase now
        pLLFUniInfo->uwData4 = 0;
        pLLFUniInfo->ulData5 = 0;
        gufLastEraseFlag = 0;
        gubLLFALLStep = STEP_FORMAT_START;
    }

#ifdef KEEP_ORIG_DBT
    if(!gubSetOrigDBT && pResponseInfo->res_state == VENDOR_CMD_ERASE)
    {
        ret = llfLoadOriginalDBT(ORIG_DBT_PHY_ADDR);
        if(ret != ERR_OK)
        {
            // UART REDUCE: [Original DBT] DBT not found! Create DBT in llf
            llfprintk("Load original dbt fail\r\n");
            memset((void*)ORIG_DBT_ADDR, 0x0, ORIG_DBT_SIZE);
            _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_RESET_TO_ORIG_DBT) = 0;
            gubSetOrigDBT = TRUE;
        }
    }
#endif

    // check defect in advance, then erase
    if (gubLLFMode == LLF_FORMAT_ERASE_NONDEFECT)
    {
        llfCheckThenErase(ubIFType, ubClkMode);
        return;
    }
    else
    {
        ret = llfEraseThenCheck(ubIFType, ubClkMode);
        if( ret == ERR_FIO_TIMEOUT )
        {
            llfEndResponce(ERR_FIO_TIMEOUT);
        }
    }

#ifdef KEEP_ORIG_DBT
    if(gubSetOrigDBT == TRUE)
    {
        memcpy((void*)(ORIG_DBT_ADDR), (void*)(DBT_ADDR), ORIG_DBT_SIZE);
        cache_area_dwb(ORIG_DBT_ADDR, ORIG_DBT_SIZE);
        cache_dummy_update_read();
    }
#endif
}

void BE_llf_handle_UTask()
{
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PVENDOR_CMD pVendorCmd;
    U8 ubIFType, ubClkMode;
#ifdef AUTO_K_ENABLE
    PLLF_UNI_INFO pLLFUniInfo;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;
#endif

    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);
    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;

    ubIFType = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
    ubClkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);
    gubCalibrateConfig  = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_AUTO_CALIBRATE_EN);
    //printk("%d %d\r\n", gubCalibrateConfig, pResponseInfo->res_state);

    if(pResponseInfo->res_state != VENDOR_CMD_IDLE)
    {
        if(pVendorCmd->cmd == BE_VEN_READ)
        {
        }
        else if (pVendorCmd->cmd == BE_VEN_WRITE)
        {
        }
        else if (pVendorCmd->cmd == BE_VEN_NONDAT)
        {
            if (pVendorCmd->subcmd == BE_LLF_INIT)
            {
                ASSERT_LLF(pVendorCmd->arg0 == LLF_MP_TAG);
                ASSERT_LLF(pVendorCmd->arg1 == LLF_MP_TAG);
                llfmpInit();
            }
            else if(pVendorCmd->subcmd == BE_LLF_CALIBRATE)//delete
            {
#ifdef AUTO_K_ENABLE
                if(gubCalibrateConfig)
                {
                    llfprintk("AUTO-CALIBRATION START, %d\r\n", pLLFUniInfo->ubData0);
#if defined(RTS5771_FPGA)

#else
                    llfAPBECalibrateTxAuto(ubIFType, ubClkMode);
#endif
                    if(pResponseInfo->err_msg_num != 0)
                    {
                        llfEndResponce(pResponseInfo->res_err_code);
                    }
                    else
                    {
#if defined(RTS5771_FPGA)

#else
                        llfAPBECalibrateRxAuto(ubIFType, ubClkMode);
#endif
                    }
                }
                else
#endif
                {
                    llfAPBECalibrate(ubIFType, ubClkMode);
                }
            }
            else if(pVendorCmd->subcmd == BE_LLF_ODBT)
            {
                llfAPBuildDBT();
            }
            else if(pVendorCmd->subcmd == BE_LLF_EXECUTE)
            {
                if(pVendorCmd->arg0 == 0xaa && pVendorCmd->arg1 == 0x55)
                {
                    llfAPReadNAND();
                }
                else if(pVendorCmd->arg0 == 0xaa && pVendorCmd->arg1 == 0x33)
                {
                    llfAPReadRDTNandLog();
                }
                else
                {
#if	defined(RTS5771_FPGA) && defined(NOFE_LLF_FLOW)
                    llfAPExecuteLLF(ubIFType, ubClkMode);
#else
                    llfAPExecuteLLF(pVendorCmd->arg0, pVendorCmd->arg1);
#endif
                }
            }
            else if (pVendorCmd->subcmd == BE_LLF_ERASENAND)
            {
                llfAPEraseAll(pVendorCmd->arg0);
                llfBECheckStatus();
            }
            else if (pVendorCmd->subcmd == BE_LLF_ALL)
            {
#ifdef REPLACE_IMAGE
                if((gulReplaceImageNum & 0xF) == TAG_FOR_REPLACE_IMAGE)
                {
                    llfprintk("[RI] RINum = 0x%x\r\n", gulReplaceImageNum);
                    llfReplaceImage();
                }
                else
#endif
                {
                    llfAll();
                }
            }
            else if(pVendorCmd->subcmd == BE_SELFTEST)
            {
#ifdef NAND_TEST_EN
                if(pVendorCmd->arg0 == NAND_TEST_TAG && pVendorCmd->arg1 == NAND_TEST_TAG)
                {
                    NandTestHandle();
                }
                else
#endif
                {
                    llfMpSelfTest();
                }
            }
            else
            {
                pResponseInfo->res_state = VENDOR_CMD_IDLE;
                pResponseInfo->res_progress = 100;
                pResponseInfo->res_err_code = ERR_UNKNOWN;
            }
        }
    }

    if(pResponseInfo->res_state != VENDOR_CMD_IDLE)
    {
        schedulerScheduleUTaskCore0(&taskTable_BE_Cpu, e_BE_llf_handle_taskid);
    }
#if (defined(RL6577_VA) && defined(FOR_WEIKE)) || defined(RL6643_VA)
    else
    {
        if(pResponseInfo->res_err_code != 0)
        {
            // frequency == 256 ms
            if(((gulTicks >> 7) & 0x1) == 0)
            {
                GPIO_SetLow(GPIO8 | GPIO9);
            }
            else
            {
                GPIO_SetHigh(GPIO8 | GPIO9);
            }
            schedulerScheduleUTaskCore0(&taskTable_BE_Cpu, e_BE_llf_handle_taskid);
        }
        else
        {
            GPIO_SetLow(GPIO8 | GPIO9);
        }
    }
#endif
}

void llfTodoRDT()
{
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PLLF_UNI_INFO pLLFUniInfo;
    U8 ubIFType, ubClkMode;
    U32 ret;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

    ubIFType = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
    ubClkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);

    if(gubLLFSubStep.ubLlfStep == STEP_LLF_ERASE)
    {
        // Clear DBT
        memset((void*)DBT_ADDR, 0x0, DBT_SIZE);
        cache_area_dwb(DBT_ADDR, DBT_SIZE);
        cache_dummy_update_read();
        memset((void*)SYS_BLK_DBT_ADDR, 0, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
        cache_area_dwbinval(SYS_BLK_DBT_ADDR, SYS_BLK_DBT_BYTE_SIZE_PER_BLOCK * SYSTEM_BLOCK_MAX_NUM);
        cache_dummy_update_read();

        ret = llfAPEraseAllSblk(NandPara.ubBankNum);
        if(ret != ERR_OK)
        {
            llfEndResponce(ret);
            return;
        }

        pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
        pLLFUniInfo->ulWorkFunc = WRITE_CONFIG_FUNC;
        gubLLFSubStep.ubLlfStep = STEP_LLF_EXECUTE;
    }
    else
    {
        llfAPExecuteLLF(ubIFType, ubClkMode);
    }
}

U32 repeated_WriteReadCache(U16 cycle)
{
    llfDbgPrintk(ALWAYS_MSG, "Multi Write read cache test, please wait...\r\n");
    U16 i = 0;
    U16 j = 0;
    U32 ret = ERR_OK;
    U16 retry = _MEM16(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_CHECK_CACHE_RETRY);
    if (retry == 0xFFFF)
    {
        return ret;
    }
    gubMultiWRCacheBankIndex = 0;
    gulMultiWRCacheBlockIndex = 0;
    gulMultiWRPageIndex = NandPara.uwSLCPageNumPerBlock;
    for(i = 0; i < BANK_NUM_MAX; i++)
        gulMultiWRBlockIndex[i] = SYSTEM_BLOCK_MAX_NUM;
    if(retry == 0)
        retry = 1;

#if defined(RL6577_VA)||defined(RTS5771_VA)
    FR_G_CFG_REG32_W(FR_BMU_EN, 1); //Disable transfer done cmp of write cmd
#endif
#if defined(FTL_SANDISK_BICS3) || defined(FTL_SANDISK_BICS4)|| defined(FTL_SANDISK_BICS5)|| defined(FTL_SANDISK_BICS4_QLC)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1,
                     0); //do not error check in WriteCache aviod residual flash error bit
#endif
    FR_G_CFG_REG32_W(FR_ECC_THV, gubCacheErrbitLimit);
    llfDbgPrintk(ALWAYS_MSG, "Multi WR Cache Retry: %d, ECC: %d\r\n", retry, gubCacheErrbitLimit);
    llfInitErrorMessage();
    for(j = 0; j < retry; j++)
    {
        //llfDbgPrintk(ALWAYS_MSG, "Multi WR Cache Retry %d\r\n", j);
        for(i = 0; i < cycle * gubRealLunNum; i++)
        {
            ret = WriteReadFlashCache_for_repeate();
            if(ret != ERR_OK)
                break;
        }
        if(ret != ERR_OK)
            break;
    }
#if defined(FTL_SANDISK_BICS3) || defined(FTL_SANDISK_BICS4)|| defined(FTL_SANDISK_BICS5)|| defined(FTL_SANDISK_BICS4_QLC)
    FR_G_CFG_REG32_W(FR_PAR_ERROR_CHECK_1, 0x0100); //reset error_check_1 register
#endif
    FR_G_CFG_REG32_W(FR_ECC_THV, 0x50);
    return ret;
}

U32 WriteReadFlashCache_for_repeate()
{
    U32 data_pAddr, data_pAddr_read, tag;
    U32 head_pAddr, head_pAddr_read;
    U8 read_no, ubBankNum, bank_no = 0;
    U8 count;
    U32 ulMode, cmp;
    U32 i;
    U32 ret = ERR_OK;
    U32 normal_cmp_num, cmp_mask;
    U32 err_info0, err_info1;
    U32 cmp_count = 0;
    U32 time_count = 0;
    U8 lun_no;
    U32 block_no;
    U8 cmp_tag[CH_NUM_MAX * CMD_FIFO_NUM_PER_CH];
    U8 cmp_tag_write[CH_NUM_MAX * CMD_FIFO_NUM_PER_CH];
    U32 cmp_save_index = 0;
    U32 cmp_save[128];
#if defined(RL6577_FPGA) ||defined(RL6577_VA) ||defined(RL6643_FPGA) || defined(RL6643_VA) ||defined(RTS5771_VA)
    gubCmpSel = 0;
    cmp_mask = 0x7ff;
#else
    cmp_mask = 0xffff;
#endif

    for(i = 0; i < CH_NUM_MAX * CMD_FIFO_NUM_PER_CH; i++)
    {
        cmp_tag[i] = 0;
        cmp_tag_write[i] = 0;
    }
    for(i = 0; i < 128; i++)
        cmp_save[i] = 0;

    cache_area_dwbinval(MUL_HBUF_ADDR, MUL_HBUF_SIZE);
    cache_area_dwbinval(MUL_BUF_ADDR, MUL_BUF_SIZE);
    cache_dummy_update_read();

    //set header data
    for (i = 0; i < (DRAM_HEAD_SIZE  * NandPara.ubPlaneNumPerLun); i += 4)
    {
        _REG32(MUL_HBUF_ADDR + i) = ((i << 24) | (i << 16) | (i << 8) | i);
    }

    // DWB that cache line
    cache_area_dwbinval(MUL_HBUF_ADDR, (DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun));
    cache_dummy_update_read();

    // Program DRAM for Data DMA
    llfProgramSeqData(MUL_BUF_ADDR, (NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun));

    // DWB that cache line
    cache_area_dwbinval(MUL_BUF_ADDR, (NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun));
    cache_dummy_update_read();

    ubBankNum = UnbalancedGetBankNum();
    bank_no = gubMultiWRCacheBankIndex;
    block_no = gulMultiWRCacheBlockIndex;
    read_no = 0;
    normal_cmp_num = 0;
    data_pAddr_read = MUL_BUF_PHY_ADDR;
    while(data_pAddr_read < (MUL_BUF_PHY_ADDR + MUL_BUF_SIZE - 2 * NandPara.ubPlaneNumPerLun *
                             NandPara.ubSectorNumPerPage * 512))
    {
        //---------write----------
        head_pAddr = MUL_HBUF_PHY_ADDR;
        data_pAddr = MUL_BUF_PHY_ADDR;

        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
        while (FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE) >=
                (hlc_size_write_6p_cache_DRAM + hlc_size_read_6p_cache_DRAM))
        {
            //llfprintk("Multi WRCache: No: %d Bank: %d, block: %d, addr: 0x%x\r\n", read_no, bank_no, block_no, data_pAddr_read);
            lun_no = bank_no / NandPara.ubBankNumPerLun;
            gul_FW_TAG = llfBETagSetting(TAG_WRITE_CACHE, bank_no);
            if(NandPara.ubPlaneNumPerLun == 1)
            {
                FCWriteCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr,
                                 NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                 head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
            }
            else if(NandPara.ubPlaneNumPerLun == 2)
            {
                FCMultiWriteCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr,
                                      NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                      head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
            }
            else if(NandPara.ubPlaneNumPerLun == 4)
            {
                FCQuadWriteCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr,
                                     NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                     head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
            }
#ifdef SIX_PLANE
            else if(NandPara.ubPlaneNumPerLun == 3 || NandPara.ubPlaneNumPerLun == 6)
            {
                FCHexaWriteCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr,
                                     NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                     head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
            }
#endif
            else
            {
                AddErrorMessage(bank_no, 0, ERR_MULTI_WR_CACHE_TEST);
                return ERR_MULTI_WR_CACHE_TEST;
            }
            cmp_tag[bank_no] += 0x01;
#if defined(RL6577_VA)||defined(RTS5771_VA)
            cmp_tag_write[bank_no] += 1;
            cmp_count += 1;
#else
            cmp_tag_write[bank_no] += 2;
            cmp_count += 2;
#endif
            normal_cmp_num++;

            //---------read----------
            head_pAddr_read = head_pAddr + (DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun * (read_no + 1));
            data_pAddr_read = data_pAddr + NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun *
                              (read_no + 1);

            gul_FW_TAG = llfBETagSetting(TAG_READ_CACHE, bank_no);
            if(NandPara.ubPlaneNumPerLun == 1)
            {
                FCReadCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr_read,
                                NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun); // read data only
            }
            else if(NandPara.ubPlaneNumPerLun == 2)
            {
                FCMultiReadCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr_read,
                                     NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                     head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun); // read data only
            }
            else if(NandPara.ubPlaneNumPerLun == 4)
            {
                FCQuadReadCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr_read,
                                    NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                    head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun); // read data only
            }
#ifdef SIX_PLANE
            else if(NandPara.ubPlaneNumPerLun == 3 || NandPara.ubPlaneNumPerLun == 6)
            {

                FCHexaReadCacheDRAM(ulMode, bank_no, lun_no, block_no, data_pAddr_read,
                                    NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                    head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun); // read data only
            }
#endif
            else
            {
                AddErrorMessage(bank_no, 0, ERR_MULTI_WR_CACHE_TEST);
                return ERR_MULTI_WR_CACHE_TEST;
            }
            cmp_tag[bank_no] += 0x10;
            cmp_count++;
            normal_cmp_num++;

            read_no++;
            //llfprintk("read_no = %d\r\n", read_no);
            if(data_pAddr_read >= (MUL_BUF_PHY_ADDR + MUL_BUF_SIZE - 2 * (NandPara.ubSectorNumPerPage * 512 *
                                   NandPara.ubPlaneNumPerLun)))
            {
                //llfprintk("pAddr_read =%x\r\n", data_pAddr_read);
                bank_no++;
                if(bank_no == ubBankNum)
                {
                    bank_no = 0;
                    if(gubIsSerialMultiLUN && (NandPara.ubLunNumPerCE == 1)) // extend
                    {
                        block_no += guwRealBlockNum;
                        if(block_no == gubRealLunNum * guwRealBlockNum)
                            block_no = 0;
                    }
                }
                break;
            }

            bank_no++;
            if(bank_no == ubBankNum)
            {
                bank_no = 0;
                if(gubIsSerialMultiLUN && (NandPara.ubLunNumPerCE == 1)) // extend
                {
                    block_no += guwRealBlockNum;
                    if(block_no == gubRealLunNum * guwRealBlockNum)
                        block_no = 0;
                }
            }
        }
        if((FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE) < (hlc_size_write_6p_cache_DRAM +
                hlc_size_read_6p_cache_DRAM)))
        {
            //llfprintk("Command fifo full bank%d: %d\r\n", bank_no, FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE));
            break;
        }
    }
    gubMultiWRCacheBankIndex = bank_no;
    gulMultiWRCacheBlockIndex = block_no;
    count = read_no * 2;
    //llfprintk("count = %d, cmp_count: %d, normal_cmp_num: %d\r\n", count, cmp_count, normal_cmp_num);
#ifdef IS_8K_PAGE
    FcBusyWait1ms(5);
#endif
    while(1)
    {
        if(normal_cmp_num != 0)
        {
            while((FC_TOP_REG(FR_CMPQ_BC) & cmp_mask) != 0)
            {
                cmp = FcPollCompletion(&err_info0, &err_info1);
                cmp_save[cmp_save_index++] = cmp;
                tag = cmp >> 16;
                if(((cmp >> 15) & 0x01) == 1)//write has two completion, transfer done
                {
                    if((cmp_tag_write[tag & 0xf] & 0x01) == 0x01 || cmp_tag_write[tag & 0xf] == 0)
                    {
                        llfprintk("Invalid cmp: 0x%x, count write: %d\r\n", cmp, cmp_tag_write[tag & 0xf]);
                    }
                    cmp_count--;
                    cmp_tag_write[tag & 0xf]--;
                    continue;
                }
                else
                {
                    //llfprintk("cmp = %x\r\n", cmp);
                }
                count--;
                cmp_count--;

                if((cmp & CMP_ERR) != 0)
                {
                    llfprintk("cmp = %x\r\n", cmp);
                    tag = cmp >> 16;
                    if((tag  >> 8) == TAG_READ_CACHE)
                    {
                        llfprintk("err_bankis:0x%x  ,FW tag: 0x%x \r\n", (tag & 0xf), TAG_READ_CACHE);
                    }
                    else if((tag  >> 8) == TAG_WRITE_CACHE)
                    {
                        llfprintk("err_bankis:0x%x  ,FW tag: 0x%x \r\n", (tag & 0xf), TAG_WRITE_CACHE);
                    }
                    AddErrorMessage((tag & 0xf), 0, ERR_MULTI_WR_CACHE_TEST);
                    ret =  ERR_MULTI_WR_CACHE_TEST;
                }
                else
                {
                    tag = cmp >> 16;
                    if((tag >> 8) == TAG_READ_CACHE)
                    {
                        cmp_tag[tag & 0xf] -= 0x10;
                    }
                    else if((tag >> 8) == TAG_WRITE_CACHE)
                    {
                        cmp_tag[tag & 0xf] -= 0x01;
                        cmp_tag_write[tag & 0xf]--;
                    }
                }
                if(count == 0)
                {
                    if((FC_TOP_REG(FR_CMPQ_BC) & cmp_mask) != 0)
                    {
                        llfprintk("Multi WRCache CMP polling fatal error\r\n");
                        llfprintk("FR_CMPQ_BC: 0x%x\r\n", FC_TOP_REG(FR_CMPQ_BC));
                        ret = ERR_MULTI_WR_CACHE_TEST;
                        break;
                    }
                    for(bank_no = 0; bank_no < NandPara.ubBankNum ; bank_no++)
                    {
                        if(FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE) != FC_CMDQ_SIZE / 4)
                        {
                            llfprintk("Multi WRCache CMP polling fatal error\r\n");
                            llfprintk("Bank: %d, FR_CMDQ_ROOM: %d\r\n", bank_no,
                                      FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE));
                            AddErrorMessage(bank_no, 0, ERR_MULTI_WR_CACHE_TEST);
                            ret = ERR_MULTI_WR_CACHE_TEST;
                        }
                    }
                    break;
                }
            }
            if(count == 0)
            {
                break;
            }
            FcBusyWait10us(1);
            time_count++;
            if(time_count > 1000)
            {
                llfprintk("Next bank: %d\r\n", gubMultiWRCacheBankIndex);
                llfprintk("count = %d, cmp_count: %d, normal_cmp_num: %d\r\n", count, cmp_count, normal_cmp_num);
                for(i = 0; i < CH_NUM_MAX * CMD_FIFO_NUM_PER_CH; i++)
                {
                    llfprintk("bank: %d, cmp_tag[%d] = 0x%x, cmp_tag_write[%d]: %d\r\n", i, i, cmp_tag[i], i,
                              cmp_tag_write[i]);
                    if(cmp_tag[i] != 0)
                    {
                        AddErrorMessage(i, 0, ERR_MULTI_WR_CACHE_TEST);
                    }
                }
                for(i = 0; i < cmp_save_index; i++)
                {
                    llfprintk("cmp%d: 0x%x\r\n", i, cmp_save[i]);
                }
                ret = ERR_MULTI_WR_CACHE_TEST;
                break;
            }
        }
        else
        {
            llfprintk("MultiWRCache CMP polling fatal error\r\n");
            ret = ERR_MULTI_WR_CACHE_TEST;
            break;
        }
    }
    //llfprintk("count = %d, cmp_count: %d, normal_cmp_num: %d\r\n", count, cmp_count, normal_cmp_num);

    if(ret != ERR_OK)
    {
        return ret;
    }

    // Compare header and data
    for(i = 1; i <= read_no; i++)
    {
        ret |= llfCompareHead(MUL_HBUF_ADDR,
                              (MUL_HBUF_ADDR + DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun * i),
                              DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
        ret |= llfCompareData(MUL_BUF_ADDR,
                              (MUL_BUF_ADDR + NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun * i),
                              (NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun));
        if (ret != ERR_OK)
        {
            llfprintk("MultiWRCache ret = %x\r\n", ret);
            return ERR_MULTI_WR_CACHE_TEST;
        }
    }

    return ERR_OK;
}

U32 WriteReadFlash_for_repeate()
{
    U32 data_pAddr, data_pAddr_read, tag;
    U32 head_pAddr, head_pAddr_read;
    U8 read_no, bank_no = 0;
    U8 count;
    U32 ulMode, cmp;
    U32 i;
    U32 ret = ERR_OK;
    U32 normal_cmp_num, cmp_mask;
    U32 err_info0, err_info1;
    U32 cmp_count = 0;
    U32 time_count = 0;
    U8 ubBankNum, lun_no = 0;
    U8 cmp_tag[CH_NUM_MAX * CMD_FIFO_NUM_PER_CH];
    U8 cmp_tag_write[CH_NUM_MAX * CMD_FIFO_NUM_PER_CH];
    U32 cmp_save_index = 0;
    U32 cmp_save[128];
#if defined(RL6577_FPGA) ||defined(RL6577_VA) ||defined(RL6643_FPGA) || defined(RL6643_VA) ||defined(RTS5771_VA)
    gubCmpSel = 0;
    cmp_mask = 0x7ff;
#else
    cmp_mask = 0xffff;
#endif

    for(i = 0; i < CH_NUM_MAX * CMD_FIFO_NUM_PER_CH; i++)
    {
        cmp_tag[i] = 0;
        cmp_tag_write[i] = 0;
    }
    for(i = 0; i < 128; i++)
        cmp_save[i] = 0;

    //set header data
    for (i = 0; i < (DRAM_HEAD_SIZE  * NandPara.ubPlaneNumPerLun); i++)
        _REG32(MUL_HBUF_ADDR + (i * 4)) = ((i << 24) | (i << 16) | (i << 8) | i);

    // DWB that cache line
    cache_area_dwbinval(MUL_HBUF_ADDR, (HEADER_MAX_LEN * NandPara.ubPlaneNumPerLun));
    cache_dummy_update_read();

    // Program DRAM for Data DMA
    llfProgramSeqData(MUL_BUF_ADDR, (NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun));

    // DWB that cache line
    cache_area_dwbinval(MUL_BUF_ADDR, (NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun));
    cache_dummy_update_read();

    ubBankNum = UnbalancedGetBankNum();
    bank_no = gubMultiWRCacheBankIndex;
    read_no = 0;
    normal_cmp_num = 0;
    data_pAddr_read = MUL_BUF_PHY_ADDR;
    while(data_pAddr_read < (MUL_BUF_PHY_ADDR + MUL_BUF_SIZE - NandPara.ubPlaneNumPerLun *
                             NandPara.ubSectorNumPerPage * 512))
    {
        //---------write----------
        head_pAddr = MUL_HBUF_PHY_ADDR;
        data_pAddr = MUL_BUF_PHY_ADDR;

        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
        while (FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE) >=
                (hlc_size_write_6p_cache_DRAM + hlc_size_read_6p_cache_DRAM))
        {
            if(gulMultiWRPageIndex >= NandPara.uwSLCPageNumPerBlock)
            {
                gulMultiWRPageIndex = 0;
                for(i = 0; i < ubBankNum; i++)
                {
                    gulMultiWRBlockIndex[i] += NandPara.ubPlaneNumPerLun;
                    llfprintk("Bank%d next block to write: %d\r\n", i, gulMultiWRBlockIndex[i]);
                    while(llfIsMpBlockBad(DBT_ADDR, i, gulMultiWRBlockIndex[i] / NandPara.ubPlaneNumPerLun))
                    {
                        gulMultiWRBlockIndex[i] += NandPara.ubPlaneNumPerLun;
                        llfprintk("Bank%d next block to write: %d\r\n", i, gulMultiWRBlockIndex[i]);
                        if(gulMultiWRBlockIndex[i] >= NandPara.uwBlockNumPerLun)
                        {
                            llfprintk("Bank%d block invalid\r\n", i);
                            break;
                        }
                    }
                }
            }
            //llfprintk("Multi WR: No: %d Bank: %d, block: %d, page: %d, addr: 0x%x\r\n",
            //        read_no, bank_no, gulMultiWRBlockIndex[bank_no], gulMultiWRPageIndex, data_pAddr_read);
            lun_no = bank_no / NandPara.ubBankNumPerLun;
            gul_FW_TAG = llfBETagSetting(TAG_WRITE_CACHE, bank_no);
            if(NandPara.ubPlaneNumPerLun == 1)
            {
                FCWriteCacheDRAM(ulMode, bank_no, lun_no, gulMultiWRBlockIndex[bank_no],
                                 data_pAddr, NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                 head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
            }
            else if(NandPara.ubPlaneNumPerLun == 2)
            {
                FCMultiWriteDRAM(ulMode, bank_no, lun_no, gulMultiWRBlockIndex[bank_no], gulMultiWRPageIndex,
                                 data_pAddr, NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                 head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun, 1);
            }
            else if(NandPara.ubPlaneNumPerLun == 4)
            {
                FCQuadWriteDRAM(ulMode, bank_no, lun_no, gulMultiWRBlockIndex[bank_no], gulMultiWRPageIndex,
                                data_pAddr, NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                head_pAddr, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun, 1);
            }
            else
            {
                AddErrorMessage(bank_no, 0, ERR_MULTI_WR_CACHE_TEST);
                return ERR_MULTI_WR_CACHE_TEST;
            }
            cmp_tag[bank_no] += 0x01;
            cmp_tag_write[bank_no] += 1;
            cmp_count += 1;
            normal_cmp_num++;

            //---------read----------
            head_pAddr_read = head_pAddr + (DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun * (read_no + 1));
            data_pAddr_read = data_pAddr + NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun *
                              (read_no + 1);

            gul_FW_TAG = llfBETagSetting(TAG_READ_CACHE, bank_no);
            if(NandPara.ubPlaneNumPerLun == 1)
            {
                FCSingleReadDRAM(ulMode, bank_no, lun_no, gulMultiWRBlockIndex[bank_no], gulMultiWRPageIndex,
                                 data_pAddr_read, NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                 head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun, 1);
            }
            else if(NandPara.ubPlaneNumPerLun == 2)
            {
                FCMultiReadDRAM(ulMode, bank_no, lun_no, gulMultiWRBlockIndex[bank_no], gulMultiWRPageIndex,
                                data_pAddr_read, NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                                head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun, 1);
            }
            else if(NandPara.ubPlaneNumPerLun == 4)
            {
                FCQuadReadDRAM(ulMode, bank_no, lun_no, gulMultiWRBlockIndex[bank_no], gulMultiWRPageIndex,
                               data_pAddr_read, NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun,
                               head_pAddr_read, DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun, 1);
            }
            else
            {
                AddErrorMessage(bank_no, 0, ERR_MULTI_WR_CACHE_TEST);
                return ERR_MULTI_WR_CACHE_TEST;
            }
            cmp_tag[bank_no] += 0x10;
            cmp_count++;
            normal_cmp_num++;

            read_no++;
            //llfprintk("read_no = %d\r\n", read_no);
            if(data_pAddr_read >= (MUL_BUF_PHY_ADDR + MUL_BUF_SIZE - (NandPara.ubSectorNumPerPage * 512 *
                                   NandPara.ubPlaneNumPerLun)))
            {
                //llfprintk("pAddr_read =%x\r\n", data_pAddr_read);
                bank_no++;
                if(bank_no == ubBankNum)
                {
                    gulMultiWRPageIndex++;
                    bank_no = 0;
                }
                break;
            }

            bank_no++;
            if(bank_no == ubBankNum)
            {
                gulMultiWRPageIndex++;
                bank_no = 0;
            }
        }
        if((FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE) < (hlc_size_write_mp_cache_DRAM +
                hlc_size_read_mp_cache_DRAM)))
        {
            //llfprintk("Command fifo full bank%d: %d\r\n", bank_no, FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE));
            break;
        }
    }
    gubMultiWRCacheBankIndex = bank_no;
    count = read_no * 2;
    //llfprintk("count = %d, cmp_count: %d, normal_cmp_num: %d\r\n", count, cmp_count, normal_cmp_num);

    while(1)
    {
        if(normal_cmp_num != 0)
        {
            while((FC_TOP_REG(FR_CMPQ_BC) & cmp_mask) != 0)
            {
                cmp = FcPollCompletion(&err_info0, &err_info1);
                cmp_save[cmp_save_index++] = cmp;
                tag = cmp >> 16;
                if(((cmp >> 15) & 0x01) == 1)//write has two completion, transfer done
                {
                    if((cmp_tag_write[tag & 0xf] & 0x01) == 0x01 || cmp_tag_write[tag & 0xf] == 0)
                    {
                        llfprintk("Invalid cmp: 0x%x, count write: %d\r\n", cmp, cmp_tag_write[tag & 0xf]);
                    }
                    cmp_count--;
                    cmp_tag_write[tag & 0xf]--;
                    continue;
                }
                else
                {
                    //llfprintk("cmp = %x\r\n", cmp);
                }
                count--;
                cmp_count--;

                if((cmp & CMP_ERR) != 0)
                {
                    llfprintk("cmp = %x\r\n", cmp);
                    tag = cmp >> 16;
                    if((tag  >> 8) == TAG_READ_CACHE)
                    {
                        llfprintk("err_bankis:0x%x  ,FW tag: 0x%x \r\n", (tag & 0xf), TAG_READ_CACHE);
                    }
                    else if((tag  >> 8) == TAG_WRITE_CACHE)
                    {
                        llfprintk("err_bankis:0x%x  ,FW tag: 0x%x \r\n", (tag & 0xf), TAG_WRITE_CACHE);
                    }
                    AddErrorMessage((tag & 0xf), 0, ERR_MULTI_WR_CACHE_TEST);
                    ret =  ERR_MULTI_WR_CACHE_TEST;
                }
                else
                {
                    tag = cmp >> 16;
                    if((tag >> 8) == TAG_READ_CACHE)
                    {
                        cmp_tag[tag & 0xf] -= 0x10;
                    }
                    else if((tag >> 8) == TAG_WRITE_CACHE)
                    {
                        cmp_tag[tag & 0xf] -= 0x01;
                        cmp_tag_write[tag & 0xf]--;
                    }
                }
                if(count == 0)
                {
                    if((FC_TOP_REG(FR_CMPQ_BC) & cmp_mask) != 0)
                    {
                        llfprintk("Multi WRCache CMP polling fatal error\r\n");
                        llfprintk("FR_CMPQ_BC: 0x%x\r\n", FC_TOP_REG(FR_CMPQ_BC));
                        ret = ERR_MULTI_WR_CACHE_TEST;
                        break;
                    }
                    for(bank_no = 0; bank_no < NandPara.ubBankNum ; bank_no++)
                    {
                        if(FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE) != FC_CMDQ_SIZE / 4)
                        {
                            llfprintk("Multi WRCache CMP polling fatal error\r\n");
                            llfprintk("Bank: %d, FR_CMDQ_ROOM: %d\r\n", bank_no,
                                      FC_CMD_BUF_REG(FR_CMDQ_ROOM_CE0 + bank_no * FR_CMDQ_CE_SIZE));
                            AddErrorMessage(bank_no, 0, ERR_MULTI_WR_CACHE_TEST);
                            ret = ERR_MULTI_WR_CACHE_TEST;
                        }
                    }
                    break;
                }
            }
            if(count == 0)
            {
                break;
            }
            FcBusyWait10us(1);
            time_count++;
            if(time_count > 1000)
            {
                llfprintk("Next bank: %d\r\n", gubMultiWRCacheBankIndex);
                llfprintk("count = %d, cmp_count: %d, normal_cmp_num: %d\r\n", count, cmp_count, normal_cmp_num);
                for(i = 0; i < CH_NUM_MAX * CMD_FIFO_NUM_PER_CH; i++)
                {
                    llfprintk("bank: %d, cmp_tag[%d] = 0x%x, cmp_tag_write[%d]: %d\r\n", i, i, cmp_tag[i], i,
                              cmp_tag_write[i]);
                    if(cmp_tag[i] != 0)
                    {
                        AddErrorMessage(i, 0, ERR_MULTI_WR_CACHE_TEST);
                    }
                }
                for(i = 0; i < cmp_save_index; i++)
                {
                    llfprintk("cmp%d: 0x%x\r\n", i, cmp_save[i]);
                }
                ret = ERR_MULTI_WR_CACHE_TEST;
                break;
            }
        }
        else
        {
            llfprintk("MultiWRCache CMP polling fatal error\r\n");
            ret = ERR_MULTI_WR_CACHE_TEST;
            break;
        }
    }
    //llfprintk("count = %d, cmp_count: %d, normal_cmp_num: %d\r\n", count, cmp_count, normal_cmp_num);

    if(ret != ERR_OK)
    {
        return ret;
    }

    // Compare header and data
    for(i = 1; i <= read_no; i++)
    {
        ret |= llfCompareHead(MUL_HBUF_ADDR,
                              (MUL_HBUF_ADDR + DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun * i),
                              DRAM_HEAD_SIZE * NandPara.ubPlaneNumPerLun);
        ret |= llfCompareData(MUL_BUF_ADDR,
                              (MUL_BUF_ADDR + NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun * i),
                              (NandPara.ubSectorNumPerPage * 512 * NandPara.ubPlaneNumPerLun));
        if (ret != ERR_OK)
        {
            llfprintk("MultiWRCache ret = %x\r\n", ret);
            return ERR_MULTI_WR_CACHE_TEST;
        }
    }

    return ERR_OK;
}

U32 llfEraseOneBlk(U8 bank_no, U16 block_no)
{
    U8 lun_no;
    U16 uwPage = 0;
    U32 ret = ERR_OK, ulMode, cmp;

    llfDbgPrintk(ALWAYS_MSG, "Erase Bank %d Block %d\r\n", bank_no, block_no);
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    lun_no = bank_no / NandPara.ubBankNumPerLun;
    gul_FW_TAG = llfBETagSetting(TAG_ERASE, bank_no);
#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A)
    for(uwPage = 0; uwPage < (INTELQ_PAGE_PER_SECTION * INTELQ_SECTION_PER_BLOCK);
            uwPage += INTELQ_PAGE_PER_SECTION)
#endif
    {
//#if defined(FTL_N38B) //temp ignore
//        if(uwPage == 0)
//        {
//            gul_FW_TAG = llfBETagSetting(RDTTAG_SP_ERASE, bank_no);
//            FCPresetModeSingle(ulMode, bank_no, lun_no, block_no, 0, BS_TLC_MODE);
//            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
//            if (ret != ERR_OK)
//            {
//                printk("LLF Preset block for N38B fail ret %x cmp %x bank %d block %d\r\n", ret, cmp, bank_no, block_no);
//            }
//        }
//#endif
        FCSingleErase(ulMode, bank_no, lun_no, block_no, uwPage, 0);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret != ERR_OK)
        {
            llfDbgPrintk(ALWAYS_MSG, "Erase err %d\r\n", bank_no);
            return ERR_ERASE_BLK0;
        }
        else if((cmp & 0x7fff) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "Blk erase fail cmp%x bank%d\r\n", cmp, bank_no);
            return ERR_ERASE_BLK0;
        }
    }
    return ERR_OK;
}

U32 llfEraseSblk()
{
    U8 bank_no, block_no, lun_no;
    U16 uwPage = 0, uwSblkStart;
    U32 ret = ERR_OK, ulMode, cmp;
#ifdef SBLK_EXPAND
    uwSblkStart = gubSblkStart;
#else
    uwSblkStart = 0;
#endif

    llfDbgPrintk(ALWAYS_MSG, "Erase FW Block\r\n");
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    U32 banknum = NandPara.ubBankNum;
#if defined(RL6577_VA) ||defined(RTS5771_VA)
    if(banknum > 8 && gubLLFMode == LLF_ONLY_UPDATE_FW)
    {
        banknum = 8;
        printk("Replace FW not to erase dynamic sblk.\r\n");
    }
#endif
    for (bank_no = 0; bank_no < banknum; bank_no++)
    {
        lun_no = bank_no / NandPara.ubBankNumPerLun;
        for(block_no = uwSblkStart; block_no < uwSblkStart + SYS_BLK; block_no ++)
        {
#ifdef SBLK_EXPAND
            if(!(llfIsGoodSblk(bank_no, block_no)))
            {
                continue;
            }
#else
            if(!((gulSysblk >> (bank_no * 4)) & (1 << block_no)))//skip bad block
            {
                continue;
            }
#endif

            gul_FW_TAG = llfBETagSetting(TAG_ERASE, bank_no);
#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A)
            for(uwPage = 0; uwPage < (INTELQ_PAGE_PER_SECTION * INTELQ_SECTION_PER_BLOCK);
                    uwPage += INTELQ_PAGE_PER_SECTION)
#endif
            {
                FCSingleErase(ulMode, bank_no, lun_no, block_no, uwPage, 0);
                ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                if(ret != ERR_OK)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Erase err %d\r\n", bank_no);
                    return ERR_ERASE_BLK0;
                }
                else if((cmp & 0x7fff) != 0)
                {
#ifdef ERASEALL_TWICE
                    if((gubFWFeatureSetting & 0x20) != 0)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "Blk 1st erase fail cmp%x bank%d\r\n", cmp, bank_no);
                        FCSingleErase(ulMode, bank_no, lun_no, block_no, uwPage, 0);
                        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
                        if(ret != ERR_OK)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "Erase err %d\r\n", bank_no);
                            return ERR_ERASE_BLK0;
                        }
                        else if((cmp & 0x7fff) != 0)
                        {
                            llfDbgPrintk(ALWAYS_MSG, "Blk 2nd erase fail cmp%x bank%d\r\n", cmp, bank_no);
                            return ERR_ERASE_BLK0;
                        }
                    }
                    else
#endif
                    {
                        llfDbgPrintk(ALWAYS_MSG, "Blk erase fail cmp%x bank%d\r\n", cmp, bank_no);
                        return ERR_ERASE_BLK0;
                    }
                }
            }
        }
    }
    return ERR_OK;
}

U32 TSBToggleResetFlash()
{
    U8 bank_no, ubBankNum, err_num = 0;
    U32 cmp;
    U32 ret = ERR_OK;

    ubBankNum = UnbalancedGetBankNum();
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA &&
            (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_9T23_SDR_TOGGLE ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XF24 ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_64GB ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT25_TOGGLE_128GB))
    {
        for(bank_no = 0; bank_no < ubBankNum; bank_no++)
        {
            FCReset(ONFI_DDR2_TOGGLE, bank_no);
            FCCompletionPolling(&cmp, (gul_FW_TAG));
            FcBusyWait1ms(1);
            //diff and DDR2 will keep
            gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, bank_no);
            FCStatusPolling(ONFI_DDR2_TOGGLE, bank_no);
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "FC reset flash timeout: %d\r\n", bank_no);
                llfAddErrorMessage(bank_no, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "FC reset flash error: %d  cmp %x\r\n", bank_no, cmp);
                llfAddErrorMessage(bank_no, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }

#if defined(RL6577_VA)||defined(RL6447_VA)||defined(RTS5771_VA)
            SetWarmUpOdtDrvDiff(gulNandODTDiffVrefValue, gubNandDriv,
                                GETFEATURE_BY_BANK_UC,  GETFEATURE_BY_BANK_PHY);
            FcGetDiffFeature(FR_CONFIG_CH(FR_FC_MODE, gubStartCH), gulNandODTDiffVrefValue);
#endif

            gul_FW_TAG = bank_no;
            FCGetfeature(ONFI_DDR2_TOGGLE, bank_no, 0x80, GET_FEATURE_PHY_ADDR,
                         0x10);//diff and DDR2 will keep, 0 means ddr2 mode
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "DDR2 Get Fail,bank %d,%x\r\n", bank_no, cmp);
                llfAddErrorMessage(bank_no, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }
            else
            {
                if( (_MEM32(GET_FEATURE_VA_ADDR) & 0xff) != 0 )
                {
                    llfprintk("DDR2 mode Get value fail 0 -> %x\r\n", _MEM32(GET_FEATURE_VA_ADDR));
                    llfAddErrorMessage(bank_no, 0, ERR_SET_FEATURE);
                    err_num++;
                    continue;
                }
            }
            llfprintk("bank %d get feature DDR2 OK\r\n", bank_no);
        }
        if(err_num > 0)
        {
            return ERR_SET_FEATURE;
        }
    }

    return ret;
}

U32 SandiskResetFlash()
{
    U8 ubBankNo, ubBankNum;
    U8 ubIFType, ubClkMode;
    U32 cmp;
    U32 ulRegValue = 0;
    U32 ulMode = ONFI_SDR;
    U32 ret = ERR_OK;
    U16 FcCycleNum[17] = {10, 20, 28, 33, 40, 50, 66, 67, 83, 100, 133, 166, 200, 266, 333, 400, 533};

    ubBankNum = UnbalancedGetBankNum();
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK &&
            (FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_9T23_SDR_TOGGLE ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XF24 ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
             FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC))
    {
        llfDbgPrintk(ALWAYS_MSG, "Sandisk reset flash\r\n");
        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            //llfDbgPrintk(ALWAYS_MSG, "Bank%d FC get flash mode\r\n", ubBankNo);
            gul_FW_TAG = llfBETagSetting(TAG_GETFEATURE, ubBankNo);
            FCGetfeature(ulMode, ubBankNo, 0x80, GET_FEATURE_PHY_ADDR, 0x10);
            FcBusyWait1ms(1);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "Bank%d error cmp %x\r\n", ubBankNo, cmp);
            }
            else
            {
                //llfDbgPrintk(ALWAYS_MSG, "Bank%d  %x \r\n", ubBankNo, _MEM32(GET_FEATURE_VA_ADDR));
                if((_MEM32(GET_FEATURE_VA_ADDR) & 0xFF) != 0)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Bank%d Flash mode error %x \r\n", ubBankNo, _MEM32(GET_FEATURE_VA_ADDR));
                }
            }
        }

        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            //llfDbgPrintk(ALWAYS_MSG, "Bank%d FC disable diff\r\n", ubBankNo);
            gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, ubBankNo);
            FCSetfeature(ulMode, ubBankNo, 0x02, 0x00);
            FcBusyWait1ms(1);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "Bank%d FC disable diff timeout\r\n", ubBankNo);
            }
            if((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR)
            {
                llfDbgPrintk(ALWAYS_MSG, "Bank%d FC disable diff error cmp %x\r\n", ubBankNo, cmp);
            }
        }

        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            //llfDbgPrintk(ALWAYS_MSG, "Bank%d FC force flash to sdr\r\n", ubBankNo);
            gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, ubBankNo);
            if(gubNandDefaultMode  == 0)
            {
                FCSetfeature(ulMode, ubBankNo, 0x80, 1);
            }
            else
            {
                FCSetfeature(ulMode, ubBankNo, 0x80, 0);
            }
            FcBusyWait1ms(1);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "Bank%d Flash reset command Use SDR cmp %x\r\n", ubBankNo, cmp);
            }
        }
        FcBusyWait1ms(10);

        if(gubNandDefaultMode == 0)
        {
            ChangeFCClk(ONFI_SDR, FC_PLL_CLK_10M);
            FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_SDR);
        }
        else
        {
            ChangeFCClk(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M);
            FR_G_CFG_REG32_W(FR_FC_MODE, ONFI_DDR2_TOGGLE);
        }
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
        FR_G_CFG_REG32_W(FR_PAR_CFG, (0x5 << 8));
#else
        FR_G_CFG_REG32_W(FR_PAR_CFG, (0xa << 8));
#endif
        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

#if defined(RL6577_VA)||defined(RL6447_VA)||defined(RTS5771_VA)
        if(gubNandDefaultMode == 0)
        {
            onfi4_change_setting(ONFI_SDR, FC_PLL_CLK_10M, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG, 0);
        }
        else
        {
            onfi4_change_setting(ONFI_DDR2_TOGGLE, FC_PLL_CLK_10M, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG, 0);
        }
#elif defined(RL6643_VA)
        onfi4_ocd_odt_setting(FC_OCD_DRIVE, FC_OCD_DRIVE, FC_ODT_CFG, FC_ODT_CFG);
        fc_diff_setting(ulMode, FC_PLL_CLK_10M, 0);
#elif defined(RL6531_VB)
        Fc_ocd_odt_setting(FC_OCD_DRIVE, FC_ODT_CFG);
        fc_diff_setting(ulMode, FC_PLL_CLK_10M, 0);
#endif
        llfLoadDefaultTiming();
        FcBusyWait1ms(10);

        // reset all CHs and CEs one by one here,if lun num>1,reset all luns in ce
        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
        for(ubBankNo = 0; ubBankNo < ubBankNum; ubBankNo++)
        {
            //llfDbgPrintk(ALWAYS_MSG, "Bank%d FC reset flash\r\n", ubBankNo);
            gul_FW_TAG = llfBETagSetting(TAG_RESET, ubBankNo);
            FCReset(ulMode, ubBankNo);
            FcBusyWait1ms(5);
            ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "Bank%d FC reset flash error ret %x\r\n", ubBankNo, ret);
                //AddErrorMessage(ubBankNo, 0, ERR_FIO_TIMEOUT);
            }
            else
            {
                gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, ubBankNo);
                FCStatusPolling(ulMode, ubBankNo);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if(ret != ERR_OK)
                {
                    llfDbgPrintk(ALWAYS_MSG, "Bank%d FC poll status error ret %x\r\n", ubBankNo, ret);
                    //AddErrorMessage(ubBankNo, 0, ERR_FIO_TIMEOUT);
                }
                else
                {
                    if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "Bank%d FC poll status error cmp %x\r\n", cmp);
                        //AddErrorMessage(ubBankNo, 0, ERR_FC_CMP);
                    }
                }
            }
        }

        ubIFType = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
        ubClkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);
        ret = FCInterfaceChange(ubIFType, FC_PLL_CLK_10M);
        if(ret != ERR_OK)
        {
            llfDbgPrintk(ALWAYS_MSG, "FC interface change fail\r\n");
            return ret;
        }

        FCGetNandODTDiffVrefValue();
        ret = SetWarmUpOdtDrvDiff(gulNandODTDiffVrefValue, gubNandDriv,
                                  GETFEATURE_BY_BANK_UC,  GETFEATURE_BY_BANK_PHY);
        if(ret != ERR_OK)
        {
            llfDbgPrintk(ALWAYS_MSG, "Set odt drive diff fail\r\n");
            return ret;
        }

        ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
        llfLoadTimingFromConfig();
        ChangeFCClk(ulMode, ubClkMode);
#if defined(RTS5771_VA) || defined(RTS5771_FPGA)
        ulRegValue = ((FR_CONFIG_CH(FR_PAR_CFG,
                                    gubStartCH) & 0xFFFC00FF) | ((FcCycleNum[ubClkMode] >> 1) << 8));
#else
        ulRegValue = ((FR_CONFIG_CH(FR_PAR_CFG, gubStartCH) & 0xFFFC00FF) | (FcCycleNum[ubClkMode] << 8));
#endif
        FR_G_CFG_REG32_W(FR_PAR_CFG, ulRegValue);

#if defined(RL6577_VA)||defined(RL6447_VA)||defined(RTS5771_VA)
        onfi4_change_setting(ulMode, ubClkMode, gulFc_ocd, gulFc_dqs_odt, gulFc_dq_re_odt,
                             gubFCDiffEnable);
        llfLoadSblkPerCh(SBLK_ADDR, 0);
#elif defined(RL6643_VA)
        onfi4_ocd_odt_setting(gulFc_ocd, gulFc_ocd, gulFc_dq_re_odt, gulFc_dqs_odt);
        fc_diff_setting(ulMode, ubClkMode, gubFCDiffEnable);
#elif defined(RL6531_VB)
        SetOcdOdtFromSblk();
        fc_diff_setting(ulMode, ubClkMode, gubFCDiffEnable);
#endif
    }

    return ret;
}

U32 llfReadOffsetSetting(U32 addr)
{
    U8 bank, ubBankNum;
    U8 err_num = 0;
    U32 cmp, ret;
    U32 mode;

    mode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    ubBankNum = UnbalancedGetBankNum();

    if(_REG08(addr + SBLK_OFFSET_VTH_ENABLE) == 0x4f)
    {
        for(bank = 0; bank < ubBankNum; bank++)
        {
            gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank);
            FCSetfeature(mode, bank, 0xAD, _REG08(addr + SBLK_OFFSET_VTH_BANK0 + bank));// Enable offset read
            llfDbgPrintk(ALWAYS_MSG, "AD %x\r\n", _REG08(addr + SBLK_OFFSET_VTH_BANK0 + bank));
            FcBusyWait10us(50);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "Set readoffset ERR bank %d,%x,%d,%s\r\n", bank, cmp, __LINE__, __FILE__);
                llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }

            FCGetfeature(mode, bank, 0xAD, GET_FEATURE_PHY_ADDR, 0x10);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "Get readoffset bank %d,%x,%d,%s\r\n", bank, cmp, __LINE__, __FILE__);
                llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }
            else
            {
                if((_MEM32(GET_FEATURE_VA_ADDR) & 0xFF) != _REG08(addr + SBLK_OFFSET_VTH_BANK0 + bank))
                {
                    llfDbgPrintk(ALWAYS_MSG, "Set readoffset Fail%x \r\n", _MEM32(GET_FEATURE_VA_ADDR));
                    llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                    err_num++;
                    continue;
                }
            }
        }

        for(bank = 0; bank < ubBankNum; bank++)
        {
            gul_FW_TAG = llfBETagSetting(TAG_SETFEATURE, bank);
            FCSetfeature(mode, bank, 0xAF, _REG08(addr + SBLK_OFFSET_VTH_BANK0 + bank));// Enable offset read
            llfDbgPrintk(ALWAYS_MSG, "AF %x\r\n", _REG08(addr + SBLK_OFFSET_VTH_BANK0 + bank));
            FcBusyWait10us(50);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "Set readoffset ERR bank %d,%x,%d,%s\r\n", bank, cmp, __LINE__, __FILE__);
                llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }

            FCGetfeature(mode, bank, 0xAF, GET_FEATURE_PHY_ADDR, 0x10);
            ret = FCCompletionPolling(&cmp, gul_FW_TAG);
            if(ret != ERR_OK || ((cmp & FC_CMP_ECC_MASK) != FC_COMPLETION_NO_ERR))
            {
                llfDbgPrintk(ALWAYS_MSG, "Get readoffset bank %d,%x,%d,%s\r\n", bank, cmp, __LINE__, __FILE__);
                llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                err_num++;
                continue;
            }
            else
            {
                if((_MEM32(GET_FEATURE_VA_ADDR) & 0xFF) != _REG08(addr + SBLK_OFFSET_VTH_BANK0 + bank))
                {
                    llfDbgPrintk(ALWAYS_MSG, "Set readoffset Fail%x \r\n", _MEM32(GET_FEATURE_VA_ADDR));
                    llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                    err_num++;
                    continue;
                }
            }
        }
        if(err_num > 0)
        {
            return ERR_SET_FEATURE;
        }
    }
    return ERR_OK;
}

U32 llfChkDefectBlkForSandisk(U8 bank_no, U16 blk_no)
{
    U32 ulMode;
    U32 cmp;
    U32 ret;
    U8 lun_no;

    lun_no = bank_no / NandPara.ubBankNumPerLun;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

#if defined(RL6643_VA)
    lun_no = 0;
    if(gubIsSerialMultiLUN)
    {
        while(blk_no >= guwRealBlockNum)
        {
            blk_no -= guwRealBlockNum;
            lun_no++;
        }
    }
#endif

    gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank_no);
    llfFCCmdSlcWrite_Nondata(ulMode, bank_no, lun_no, blk_no, 0);
    ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
    if(ret == ERR_OK)
    {
        if((cmp & BE_COMPLETION_ERROR_MASK) == FLASH_ERR)
        {
            return 0;//bad
        }
        else
        {
            return 1;
        }
    }
    else
    {
        llfDbgPrintk(ALWAYS_MSG, "Check Factory Defect Assert %d, %s\r\n", __LINE__, __FILE__);
        ASSERT(ALWAYS_MSG, 0);
        return 0;
    }
}

U32 HynixTo12V()
{
    U32 ret = ERR_OK;
    U8 ubBankNum, bank, fcMode;

    fcMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    ubBankNum = UnbalancedGetBankNum();

    for(bank = 0; bank < ubBankNum; bank++)
    {
        gul_FW_TAG = bank;
        FCSetfeature(fcMode, bank, 0x01, (0x00010000 | 0x1 << 6));
    }
#if defined(RL6577_FPGA)||defined(RL6577_VA)||defined(RL6643_FPGA)||defined(RL6643_VA)||defined(RTS5771_VA)
    ret = llfPollingAllBankCMPwithErrMsg("Set 1.2V Poll cmp", ERR_SET_FEATURE);
#else
    ret = PollingAllBankCMP("Set 1.2V Poll cmp");
#endif
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "Set Feature Fail\r\n");
#if defined(RL6577_FPGA)||defined(RL6577_VA)||defined(RL6643_FPGA)||defined(RL6643_VA)||defined(RTS5771_VA)
        return ret;
#else
        for (bank = 0; bank < ubBankNum; bank++)
        {
            llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
        }
        return ERR_SET_FEATURE;
#endif
    }

    for (bank = 0; bank < ubBankNum; bank++)
    {
        gul_FW_TAG = bank;
        FCGetfeature(fcMode, bank, 0x01, GETFEATURE_BY_BANK_PHY + bank * 16, 0x10);
    }
#if defined(RL6577_FPGA)||defined(RL6577_VA)||defined(RL6643_FPGA)||defined(RL6643_VA)||defined(RTS5771_VA)
    ret = llfPollingAllBankCMPwithErrMsg("Get 1.2V Poll cmp", ERR_SET_FEATURE);
    //TODO: TKU change back to ERR_GET_FEATURE
#else
    ret = PollingAllBankCMP("Get 1.2V Poll cmp");
#endif
    if(ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "Get Feature Fail\r\n");
#if defined(RL6577_FPGA)||defined(RL6577_VA)||defined(RL6643_FPGA)||defined(RL6643_VA)||defined(RTS5771_VA)
        return ret;
#else
        for (bank = 0; bank < ubBankNum; bank++)
        {
            llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
        }
        return ERR_SET_FEATURE;
#endif
    }
    else
    {
        for (bank = 0; bank < ubBankNum; bank++)
        {
            // llfprintk("get value = %x\r\n", _MEM32(GETFEATURE_BY_BANK_UC + bank * 16));

            if(_MEM32(GETFEATURE_BY_BANK_UC + bank * 16) != (0x00010000 | 0x1 << 6))
            {
                ret = ERR_SET_FEATURE;
                llfAddErrorMessage(bank, 0, ERR_SET_FEATURE);
                llfprintk("Get Feature Value Fail %x -> %x\r\n", (0x00010000 | 0x1 << 6),
                          _MEM32(GETFEATURE_BY_BANK_UC + bank * 16));
            }
        }
    }
    return ret;
}
#if defined(RL6577_VA)

void i2c_clr_int()
{
#ifndef DV_SIM //not define
    READ_REG_32(I2C_REG_CLR_INTR);
    READ_REG_32(I2C_REG_CLR_RX_UNDER);
    READ_REG_32(I2C_REG_CLR_RX_OVER);
    READ_REG_32(I2C_REG_CLR_TX_OVER);
    READ_REG_32(I2C_REG_CLR_RD_REQ);
    READ_REG_32(I2C_REG_CLR_TX_ABRT);
    READ_REG_32(I2C_REG_CLR_RX_DONE);
    READ_REG_32(I2C_REG_CLR_ACTIVITY);
    READ_REG_32(I2C_REG_CLR_STOP_DET);
    READ_REG_32(I2C_REG_CLR_START_DET);
    READ_REG_32(I2C_REG_CLR_GEN_CALL);
#endif
}
U32 i2c_init_ENE(U8 i2c_addr)
{
    U32 val;
    I2C_REGISTERS gregs;
    U32 localTick;
    if ((READ_REG_32(I2C_REG_TAR) & 0xff) == i2c_addr)
    {
        return I2C_OK;
    }
    else
    {

        val = READ_REG_32(I2C_REG_ENABLE_STATUS);
        localTick = 0;
        while (val & I2C_EN)
        {
            WRITE_REG_32(I2C_REG_ENABLE, 0);
            val = READ_REG_32(I2C_REG_ENABLE_STATUS);
            if(localTick > 5000)
            {
                //printk("[EXT]i2c_init_ENE timeout.\r\n");
                return I2C_TIMER_OUT;
            }
            localTick++;
        }
        // DBGPRINTK_ROM(SATA_MSG,"HCNT:0x%x\r\n",READ_REG_32(I2C_REG_FS_SCL_HCNT));
        // DBGPRINTK_ROM(SATA_MSG,"LCNT:0x%x\r\n",READ_REG_32(I2C_REG_FS_SCL_LCNT));

        WRITE_REG_32(I2C_REG_FS_SCL_HCNT, 37);
        WRITE_REG_32(I2C_REG_FS_SCL_LCNT, 79);

        gregs.CON.BITS.MST_MODE = 1;
        //gregs.CON.BITS.SPEED = 2;
        gregs.CON.BITS.SPEED = 1;
        gregs.CON.BITS.ADDR_10BIT_SLV = 0;
        gregs.CON.BITS.ADDR_10BIT_MST = 0;	//addr_7bit
        gregs.CON.BITS.IC_RESTART_EN = 0;
        gregs.CON.BITS.IC_SLV_DISABLE = 1;
        WRITE_REG_32(I2C_REG_CON, gregs.CON.AsU32);

        gregs.TAR.BITS.TAR = i2c_addr;
        gregs.TAR.BITS.GC_OR_START = 0;
        gregs.TAR.BITS.SPECIAL = 0;
        gregs.TAR.BITS.ADDR_10BIT_MST = 0;
        WRITE_REG_32(I2C_REG_TAR, gregs.TAR.AsU32);

        WRITE_REG_32(I2C_REG_RX_TL, 0);
        WRITE_REG_32(I2C_REG_TX_TL, 0X1f);

        WRITE_REG_32(I2C_REG_INTR_MASK, 0x0);
        localTick = 0;
        do
        {
            WRITE_REG_32(I2C_REG_ENABLE, 0x1);
            if(localTick > 5000)
            {
                //printk("[EXT]i2c_init_ENE timeout.\r\n");
                return I2C_TIMER_OUT;
            }
            localTick++;
        }
        while (!(READ_REG_32(I2C_REG_ENABLE_STATUS) & I2C_EN));
        i2c_clr_int();
    }
    return I2C_OK;
}
I2C_STATUS i2c_transfer_byte_rd(U32 addr, U8 * p_result)
{
#ifndef DV_SIM //not define
    U32 data_cmd[2];
    I2C_REGISTERS gregs;
    U32 i;
    U32 rd, rd1;
    U8 val = 0;
    U8 tx_buffer_depth;	//rx_buffer_depth;
    U32 localTick;
    gregs.COMP_PARAM_1.AsU32 = READ_REG_32(I2C_REG_COMP_PARAM_1);
    tx_buffer_depth = gregs.COMP_PARAM_1.BITS.TX_BUFFER_DEPTH + 1;	// + 1;
    //rx_buffer_depth = gregs.COMP_PARAM_1.BITS.RX_BUFFER_DEPTH + 1;// + 1;

    gregs.DATA_CMD.BITS.DAT = (U8) (addr & 0xff);
    gregs.DATA_CMD.BITS.CMD = 0;
    gregs.DATA_CMD.BITS.STOP = 0;
    gregs.DATA_CMD.BITS.RESTART = 0;
    data_cmd[0] = gregs.DATA_CMD.AsU32;

    gregs.DATA_CMD.BITS.DAT = 0;
    gregs.DATA_CMD.BITS.CMD = 1;
    gregs.DATA_CMD.BITS.STOP = 1;
    gregs.DATA_CMD.BITS.RESTART = 0;
    data_cmd[1] = gregs.DATA_CMD.AsU32;

    for (i = 0; i < 2; i++)
    {
        localTick = 0;
        while (1)
        {
            rd = READ_REG_32(I2C_REG_TXFLR);
            if (rd < tx_buffer_depth)
            {
                break;
            }
            else if (localTick > 5000)
            {
                //printk("[EXT]I2CTimeOut1\r\n");
                return I2C_TIMER_OUT;
            }
            localTick ++;
        }
        WRITE_REG_32(I2C_REG_DATA_CMD, data_cmd[i]);
    }

    localTick = 0;
    while (1)
    {
        rd1 = READ_REG_32(I2C_REG_RXFLR);
        if (rd1 > 0)
        {
            break;
        }
        else if (localTick > 5000)
        {
            //printk("[EXT]I2CTimeOut2\r\n");
            return I2C_TIMER_OUT;
        }
        localTick ++;
    }
    val |= ((READ_REG_32(I2C_REG_DATA_CMD)) & 0xff);
    *p_result = val;

    return I2C_OK;
#else
    return 0;
#endif
}
#endif

#ifdef KEEP_ORIG_DBT
U32 llfLoadOriginalDBT(U32 ulAddr)
{
    U8 ubBank, ubReadPage;
    U16 uwBlock, uwPage, uwBasePage;
    U32 ulMode, ulRet, ulCmp, ulCmpHeader;
    ulRet = ERR_OK;
    uwBasePage = ORIG_DBT_PAGENO;
    ulCmpHeader = ODBT_BLK_ID;
    for(ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
    {
#ifndef SBLK_EXPAND
        for(uwBlock = 0; uwBlock < SYS_BLK; uwBlock++)
#else
        for(uwBlock = gubSblkStart; uwBlock < gubSblkStart + SYS_BLK; uwBlock++)
#endif
        {
            for (uwPage = 0, ubReadPage = 0; uwPage < ORIG_DBT_PAGENUM; uwPage++)
            {
                ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
                gul_FW_TAG = llfBETagSetting(TAG_READ, ubBank);
                llfFCCmdRead_DRAM(ulMode, ubBank, 0, uwBlock, uwPage + uwBasePage,
                                  ulAddr + (DRAM_DATA_SIZE * uwPage),
                                  DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);

                ulRet = FCCompletionPolling(&ulCmp, gul_FW_TAG);
                if(ulRet != ERR_OK)
                {
                    //llfAddErrorMessage(ubBank, 0, ERR_FIO_TIMEOUT);
                    ulRet = ERR_FIO_TIMEOUT;
                    break;
                }
                else
                {
                    if(((ulCmp & BE_COMPLETION_ERROR_MASK) == 0) && (_REG32(TEMP_HBUF_ADDR) == ulCmpHeader))
                    {
                        // UART REDUCE: Orignal DBT read OK, %d_%d_%d
                        llfprintk("Get original dbt %d %d %d\r\n", ubBank, uwBlock, uwPage + uwBasePage);
                        ubReadPage++;
                    }
                    else
                    {
                        // UART REDUCE: Orignal DBT fail, %d_%d_%d %x
                        llfprintk("Orig dbt fail %d %d %d %x\r\n", ubBank, uwBlock, uwPage + uwBasePage,
                                  _REG32(TEMP_HBUF_ADDR));
                        ulRet = ERR_ECC;
                        break;
                    }
                }
                if(ubReadPage == ORIG_DBT_PAGENUM)
                {
                    // UART REDUCE: Orignal DBT Read %d page Data OK!
                    llfprintk("All original dbt ready %d\r\n", ORIG_DBT_PAGENUM);
                    return ERR_OK;
                }
            }
        }
    }
    return ulRet;
}
#endif

#ifdef AVG_ERASE_COUNT_TEST
U32 llfParseRDTResultForClearAll(U8 bank_no, U32 resultAddr)
{
    U32 ret;

    cache_area_dinval(resultAddr, NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);
    cache_dummy_update_read();

    gulDataBsAvgEraseCnt = _REG32(resultAddr + RDT_RESULT_DATA_BS_EC);
    gulL2PBsAvgEraseCnt = _REG32(resultAddr + RDT_RESULT_L2P_BS_EC);
    llfprintk("Get DataBSAvgEC = %x, L2PBSAvgEC = %x(in Parser RDT_result mode)\r\n",
              gulDataBsAvgEraseCnt, gulL2PBsAvgEraseCnt);

    ret = ERR_OK;
    return ret;
}

U32 llfLoadRDTResultForClearAll()
{
    U8 bank_no, passNum = 0, existNum = 0;

    Change_ldpc(0);
    for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
    {
        if (llfGetRDTBankResult(false, bank_no, true) == ERR_OK)
        {
            if (_REG32(TEMP_BUF_ADDR) == ERR_OK || _REG32(TEMP_BUF_ADDR) == 0xFFFFFFFF)
            {
                existNum++;
            }
            if(llfParseRDTResultForClearAll(bank_no, TEMP_BUF_ADDR) == ERR_OK)
            {
                passNum++;
            }
        }
    }

    Change_ldpc(gubECC_CFG); //ECC mode change back; //[RL6531]

    if((existNum != 0) && (passNum != 0))
    {
        return ERR_OK;
    }
    else
    {
        return ERR_READ_RDT;
    }
}

U32 llfSearchInheritBSEraseCount()
{
    U8 bank_no, StaticSblkTag = 0, bank;
    U8 aes_bypass, ubBSInfoPageOffset;
    U16 block, MpBlockNumPerLun;
    U32 ret;
    U32 DynamicSblkAddr = TEMP_BUF_ADDR + DRAM_DATA_SIZE, BSRangeStart;

    FcInitColumnAddr();
    // case 1: RDT data exist
    if (llfLoadRDTResultForClearAll() == ERR_OK)
    {
        return ERR_OK;
    }

    // case 2: get static SBlock to find dynamic sblk range
    llfDbgPrintk(ALWAYS_MSG, "LLF inherit [WS Info] Start...\r\n");
    Change_ldpc(0);
    for(bank_no = NandPara.ubBankNum - 1; bank_no < NandPara.ubBankNum; bank_no--)
    {
#ifndef SBLK_EXPAND
        for(block = 0; block < SYS_BLK; block++)//need to check
#else
        for(block = gubSblkStart; block < gubSblkStart + SYS_BLK; block++)
#endif
        {
            ret = llfReadSBlock(TEMP_BUF_ADDR, TEMP_HBUF_ADDR, bank_no, block, 0);
            llfDbgPrintk(ALWAYS_MSG, "Read Static SBLK ret %x \r\n", ret);
            if (ret == ERR_OK)
            {
                StaticSblkTag = STATIC_SBLK_INFO;
                break;
            }
        }
        if(StaticSblkTag == STATIC_SBLK_INFO)
            break;
    }

    aes_bypass = FcCmdBypass.bits.aes_bypass;
    FcCmdBypass.bits.aes_bypass = 0;
    Change_ldpc(gubECC_CFG); //ECC mode change back; //[RL6531]

    // Not find static sblk, try to read dynamic sblk to find dynamic range
    if(bank_no >= NandPara.ubBankNum)
    {
        for(block = 0; block < SYSTEM_BLOCK_MAX_NUM; block++)
        {
            for(bank = 0; bank < NandPara.ubBankNum; bank++)
            {
#ifndef SBLK_EXPAND
                if((block < SYS_BLK) && (bank < SYS_BANK_NUM))
#else
                if((block < gubSblkStart) || ((bank < gubSblkBankStart) && (block < gubSblkStart + SYS_BLK)))
#endif
                    continue;
                else
                {
                    if(llfReadSBlock(TEMP_BUF_ADDR, TEMP_HBUF_ADDR, bank, block, 0) == ERR_OK)
                    {
                        llfDbgPrintk(ALWAYS_MSG, "No static SBlk and read Dsblk bank %d Block %d\r\n", bank, block);
                        StaticSblkTag = DYNAMIC_SBLK_INFO;
                        break;
                    }
                }
            }
            if(StaticSblkTag == DYNAMIC_SBLK_INFO)
                break;
        }
        if (block >= SYSTEM_BLOCK_MAX_NUM)
        {
            FcCmdBypass.bits.aes_bypass = aes_bypass;
            llfDbgPrintk(ALWAYS_MSG, "[WARN] Read first dynamic SBlock failed\r\n");
            return ERR_READ_SBLK;
        }
    }

    llfprintk("StaticSblkTag is %d   (StaticSblkTag[1] DYNAMIC_SBLK_INFO[2])\r\n", StaticSblkTag);

    // case 3: get dynamic SBlock actually
    if (llfSearchDynamicSBlock(TEMP_BUF_ADDR, DynamicSblkAddr, StaticSblkTag) != ERR_OK)
    {
        FcCmdBypass.bits.aes_bypass = aes_bypass;
        gulDataBsAvgEraseCnt = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_DATA_BS_AVG_EC);
        gulL2PBsAvgEraseCnt = _REG32(TEMP_BUF_ADDR + SBLK_OFFSET_L2P_BS_AVG_EC);
        llfprintk("Get DataBSAvgEC = %x, L2PBSAvgEC = %x from [Static sblock](in LLF mode)\r\n",
                  gulDataBsAvgEraseCnt, gulL2PBsAvgEraseCnt);

        return ERR_OK;
    }
    else // get dynamic SBlock is ok
    {
        gulDataBsAvgEraseCnt = _REG32(DynamicSblkAddr + SBLK_OFFSET_DATA_BS_AVG_EC);
        gulL2PBsAvgEraseCnt = _REG32(DynamicSblkAddr + SBLK_OFFSET_L2P_BS_AVG_EC);
        guwL2PGroupBegin = _REG16(DynamicSblkAddr + SBLK_OFFSET_SNAP_MP_BLOCK_END_INDEX);
        guwL2PGroupEnd = _REG16(DynamicSblkAddr + SBLK_OFFSET_L2P_MP_BLOCK_END_INDEX);
#ifdef BLK_REMAP_PRO
        guwRealMPBlkNum = _REG16(DynamicSblkAddr + SBLK_OFFSET_REAL_BLOCK_NUM);
#endif
        llfprintk("Get DataBSAvgEC = %x, L2PBSAvgEC = %x from [Dynamic Sblock](in LLF mode)\r\n",
                  gulDataBsAvgEraseCnt, gulL2PBsAvgEraseCnt);
        //llfprintk("h_wang Get guwL2PGroupBegin = %d, guwL2PGroupEnd = %d\r\n", guwL2PGroupBegin, guwL2PGroupEnd);
    }

    BSRangeStart = _REG32(DynamicSblkAddr + SBLK_OFFSET_SNAP_BS_START);
    ubBSInfoPageOffset = _REG16(DynamicSblkAddr + SBLK_OFFSET_WS_BS_INFO_OFFSET) >> 8;
    llfprintk("BSRangeStart %x PageOffset %d\r\n", BSRangeStart, ubBSInfoPageOffset);

    ret = llfReadBlockInfo(DynamicSblkAddr, BSRangeStart, ubBSInfoPageOffset);
    FcCmdBypass.bits.aes_bypass = aes_bypass;
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Read BlockInfo failed\r\n");
        return ret;
    }

    MpBlockNumPerLun = _REG16(DynamicSblkAddr + SBLK_OFFSET_MP_BLOCK_NUM);
    ret = llfParseBlockInfoToEC(DynamicSblkAddr, MpBlockNumPerLun);
    if (ret != ERR_OK)
    {
        llfDbgPrintk(ALWAYS_MSG, "[ERR] Parse BlockInfo to EC failed\r\n");
        return ret;
    }

    return ERR_OK;
}

U32 llfParseBlockInfoToEC(U32 blockInfo_addr, U16 MpBlockNumPerLun)
{
    U32 dataBSEC = 0, avgECDataBS, tempEC, fullTag;
    U32 tempForDataBS, tempForL2pBS, maxEraseCount = 0xF4240; // max EraseCount is 1,000,000
    U16 BSIndex, blockInfoEraseCountIndex = 3;
    volatile U16 *blockStripeBaseAddr;
    U16 key = 5 * 4;
    BOOL skipDataBS = FALSE, skipL2PBS = FALSE;
#ifdef L2P_ROLLING
    U16 L2PBSNum = guwL2PGroupEnd - guwL2PGroupBegin;
    U16 dataBSNum = NandPara.uwMpBlockNumPerLun - guwL2PGroupEnd;
#else
    U32 l2pBsEC = 0, avgECL2pBS;
#endif

    fullTag = (1 << 20) - 1; // fullTag is max = 000fffff

    if(guwL2PGroupBegin >= guwL2PGroupEnd)
    {
        return ERR_READ_BLOCKINFO;
    }

    if(CheckInheritEC() != ERR_OK)
    {
        return ERR_OK;
    }

    tempForDataBS = gulDataBsAvgEraseCnt & fullTag;
    tempForL2pBS = gulL2PBsAvgEraseCnt & fullTag;

    if(tempForDataBS >= maxEraseCount) // overflow, then hold on
    {
        llfprintk("DataBsAvgEraseCnt is record in MAX value!!!\r\n");
        skipDataBS = TRUE;
    }
    if(tempForL2pBS >= maxEraseCount)
    {
        llfprintk("L2PBsAvgEraseCnt is record in MAX value!!!\r\n");
        skipL2PBS = TRUE;
    }

#ifdef L2P_ROLLING
    if(skipL2PBS == FALSE)
    {
        gulL2PBsAvgEraseCnt = 0;
    }
    if(skipDataBS == FALSE)
    {
        gulDataBsAvgEraseCnt = 0;
        //calculate all BS avg EC
        tempForDataBS = (tempForL2pBS * L2PBSNum + tempForDataBS * dataBSNum) / (L2PBSNum + dataBSNum);
        for(BSIndex = guwL2PGroupBegin; BSIndex < NandPara.uwMpBlockNumPerLun; BSIndex++)
        {
            if(BSIndex < MpBlockNumPerLun)
            {
                blockStripeBaseAddr = (volatile U16 *)(blockInfo_addr + BSIndex * BS_INFO_SIZE);
                //llfprintk("h_wang Data: ptr = 0x%x,ptr + 3= 0x%x, *ptr = %d\r\n", blockStripeBaseAddr, blockStripeBaseAddr + blockInfoEraseCountIndex, *(blockStripeBaseAddr + blockInfoEraseCountIndex));
                tempEC = *(blockStripeBaseAddr + blockInfoEraseCountIndex);
                dataBSEC += tempEC;
            }
            else
            {
                break;
            }
        }
        avgECDataBS = dataBSEC / (BSIndex - guwL2PGroupBegin);
        //llfprintk("h_wang %d = %d/%d\r\n", avgECDataBS, dataBSEC, BSIndex - guwL2PGroupEnd);
        tempForDataBS += avgECDataBS;
        gulDataBsAvgEraseCnt |= tempForDataBS;
        gulDataBsAvgEraseCnt |= tempForDataBS << key;
    }
    llfprintk("Calculate EC: DataBSAvgEC = %x, L2PBSAvgEC = %x (L2P_ROLLING)\r\n", gulDataBsAvgEraseCnt,
              gulL2PBsAvgEraseCnt);
#else
    if(skipL2PBS == FALSE)
    {
        gulL2PBsAvgEraseCnt = 0;
        //L2p BS Erase Count
        for(BSIndex = guwL2PGroupBegin; BSIndex < guwL2PGroupEnd; BSIndex++)
        {
            blockStripeBaseAddr = (volatile U16 *)(blockInfo_addr + BSIndex * BS_INFO_SIZE);
            //llfprintk("h_wang L2P: ptr = 0x%x,ptr + 3= 0x%x, *ptr = %d\r\n", blockStripeBaseAddr, blockStripeBaseAddr + blockInfoEraseCountIndex, *(blockStripeBaseAddr + blockInfoEraseCountIndex));
            tempEC = *(blockStripeBaseAddr + blockInfoEraseCountIndex);
            l2pBsEC += tempEC;
        }
        avgECL2pBS = l2pBsEC / (BSIndex - guwL2PGroupBegin);
        //llfprintk("h_wang %d = %d/%d\r\n", avgECL2pBS, l2pBsEC, BSIndex - guwL2PGroupBegin);
        tempForL2pBS += avgECL2pBS;
        gulL2PBsAvgEraseCnt |= tempForL2pBS;
        gulL2PBsAvgEraseCnt |= tempForL2pBS << key;
    }
    if(skipDataBS == FALSE)
    {
        gulDataBsAvgEraseCnt = 0;
        //Data BS Erase Count
        for(BSIndex = guwL2PGroupEnd; BSIndex < NandPara.uwMpBlockNumPerLun; BSIndex++)
        {
            if(BSIndex < MpBlockNumPerLun)
            {
                blockStripeBaseAddr = (volatile U16 *)(blockInfo_addr + BSIndex * BS_INFO_SIZE);
                //llfprintk("h_wang Data: ptr = 0x%x,ptr + 3= 0x%x, *ptr = %d\r\n", blockStripeBaseAddr, blockStripeBaseAddr + blockInfoEraseCountIndex, *(blockStripeBaseAddr + blockInfoEraseCountIndex));
                tempEC = *(blockStripeBaseAddr + blockInfoEraseCountIndex);
                dataBSEC += tempEC;
            }
            else
            {
                break;
            }
        }
        avgECDataBS = dataBSEC / (BSIndex - guwL2PGroupEnd);
        //llfprintk("h_wang %d = %d/%d\r\n", avgECDataBS, dataBSEC, BSIndex - guwL2PGroupEnd);
        tempForDataBS += avgECDataBS;
        gulDataBsAvgEraseCnt |= tempForDataBS;
        gulDataBsAvgEraseCnt |= tempForDataBS << key;
    }

    llfprintk("Calculate EC: DataBSAvgEC = %x, L2PBSAvgEC = %x\r\n", gulDataBsAvgEraseCnt,
              gulL2PBsAvgEraseCnt);
#endif

    return ERR_OK;
}

BOOL CheckInheritEC()
{
    U16 checkECDataBS, checkECL2pBS, key = 5 * 4;
    checkECDataBS = gulDataBsAvgEraseCnt >> key;
    checkECL2pBS = gulL2PBsAvgEraseCnt >> key;
    if((checkECDataBS != (gulDataBsAvgEraseCnt & 0xFFF))
            || (checkECL2pBS != (gulL2PBsAvgEraseCnt & 0xFFF))
            || (gulDataBsAvgEraseCnt == 0xFFFFFFFF) || (gulL2PBsAvgEraseCnt == 0xFFFFFFFF))
    {
        gulDataBsAvgEraseCnt = 0;
        gulL2PBsAvgEraseCnt = 0;
        llfprintk("[WARN] InheritEC value is illegal clear them all!\r\n");
        return ERR_UNKNOWN;
    }
    return ERR_OK;
}

#endif

#ifdef BLK_REMAP_PRO
U32 LlfInitReMapTable(U32 Address)
{
    U8 bank_num;
    U16	Table_Lenth;
    U16 PerBankTableSize;
    U8 PlaneFlag;

    Table_Lenth = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_REMAP_TABLE_PER_BANK);
    if (Table_Lenth > 256)
    {
        llfprintk("[ERR] Over Max table size\r\n");
        return ERR_BLOCK_MAPPING;
    }
    PerBankTableSize = Table_Lenth * 2 * NandPara.ubPlaneNumPerLun;
    guwReMapTableSizePerBank = PerBankTableSize;
    llfprintk("Table lenth from conifg is %d, PerBankTableSize is %d\r\n", Table_Lenth,
              PerBankTableSize);

    // reset remap table
    //llfprintk("Set Remap table,addr is 0x%x ,bank num is %d \r\n", Address, NandPara.ubBankNum);
    for(bank_num = 0; bank_num < NandPara.ubBankNum ; bank_num ++)
    {
        for(PlaneFlag = 0; PlaneFlag < NandPara.ubPlaneNumPerLun; PlaneFlag ++)
        {
            guwRemapFlag[bank_num][PlaneFlag] = 0;
        }

        for(PlaneFlag = 0; PlaneFlag < NandPara.ubPlaneNumPerLun; PlaneFlag ++)
        {
            guwRemapTempBlock[bank_num][PlaneFlag] = 0xFFFF;
        }

        guwMpDefectBlock[bank_num] = 0xFFFF;

        memset((void *) (Address + bank_num * PerBankTableSize), 0, PerBankTableSize);
    }

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    //RL6817 max 64 banks, use U64 bitmap
    if ((BLK_REMAP_TEMP_ADDR + Table_Lenth * 8) > (BLK_REMAP_TABLE_ADDR + BLK_REMAP_TABLE_SIZE))
    {
        printk("[ERR] BLK_REMAP_TABLE_SIZE not enough\r\n");
        return ERR_FULL;
    }
    else
#endif
    {
        return ERR_OK;
    }
}

void LlfCreatReMappingTable(U8 bank)
{
    U8 i;
    U16	Table_Lenth;
    U16 TableSizePerPlane;

    TableSizePerPlane = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun;
    Table_Lenth = _MEM16(CONFIG_BASE_VA_ADDR + CONFIG_REMAP_TABLE_PER_BANK);

    for(i = 0; i < NandPara.ubPlaneNumPerLun; i++)
    {

        if (guwRemapFlag[bank][i] < Table_Lenth)
        {
            if (guwRemapTempBlock[bank][i] != 0xFFFF)
            {
                _REG16(BLK_REMAP_TABLE_ADDR + guwReMapTableSizePerBank * bank + TableSizePerPlane * i +
                       guwRemapFlag[bank][i] * 2)
                    = guwRemapTempBlock[bank][i];
                llfprintk("Push bank%d plane:%d flag:%d block:%d \r\n", bank, i, guwRemapFlag[bank][i],
                          guwRemapTempBlock[bank][i]);
                guwRemapFlag[bank][i] ++ ;
            }
        }

        guwRemapTempBlock[bank][i] = 0xFFFF;
    }

}

U16 LlfGetRealMaxBsNum()
{
    U8  i, bank;
    U16 MinReMapMpBlock;
    U16 tempBSvale;
    U16 ReMapMpBlockNum[BANK_NUM_MAX];

    ReMapMpBlockNum[0] = 0;

    //get each bank good Mpblock num
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        MinReMapMpBlock = guwRemapFlag[bank][0];
        for(i = 1; i < NandPara.ubPlaneNumPerLun; i++)
        {
            if (MinReMapMpBlock > guwRemapFlag[bank][i])
                MinReMapMpBlock = guwRemapFlag[bank][i];
        }
        ReMapMpBlockNum[bank] = MinReMapMpBlock;
        guwMpDefectBlock[bank] = MinReMapMpBlock;
        llfprintk("ReMapMpBlock[%d] = %d\r\n", bank, ReMapMpBlockNum[bank]);
    }

    // get Max BS num
    tempBSvale = ReMapMpBlockNum[0];
    for(bank = 0; bank < NandPara.ubBankNum; bank++)
    {
        if(tempBSvale < ReMapMpBlockNum[bank])
            tempBSvale = ReMapMpBlockNum[bank];
    }
    guwRealMPBlkNum = tempBSvale + NandPara.uwMpBlockNumPerLun;
    //llfprintk("tempBSvale = %d\r\n", tempBSvale);
    llfprintk("=REMAP= guwRealMPBlkNum = %d \r\n", guwRealMPBlkNum);

    return ERR_OK;

}

void LlfGetGoodMPBlock()
{
    U16 mpblock_no;
    U8 bank_no;
    U16 MpDefectPerBS;
    U8 PlaneNum;

    //llfprintk("LlfGetGoodMPBlock\r\n");
    // From L2P Begin Scan
    for (mpblock_no = guwCaclSSGroupEnd; mpblock_no < NandPara.uwMpBlockNumPerLun; mpblock_no++)
    {
        MpDefectPerBS = 0;
        for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
        {
            if (llfIsMpBlockBad(DBT_ADDR, bank_no, mpblock_no))
            {
                MpDefectPerBS++;
            }
        }

        if (MpDefectPerBS > BAD_BS_TH)
        {
            for (bank_no = 0; bank_no < NandPara.ubBankNum; bank_no++)
            {
                if (!llfIsMpBlockBad(DBT_ADDR, bank_no, mpblock_no))
                {
                    for(PlaneNum = 0; PlaneNum < NandPara.ubPlaneNumPerLun; PlaneNum++)
                    {
                        guwRemapTempBlock[bank_no][PlaneNum] = mpblock_no * NandPara.ubPlaneNumPerLun + PlaneNum;

                    }
                    llfMarkUserMPBlkDefect(DBT_ADDR, bank_no, mpblock_no);
                    llfprintk("bank %d mpblock %d mark bad, and push into remap table\r\n", bank_no, mpblock_no);
                    LlfCreatReMappingTable(bank_no);
                }
            }
        }

    }

}


void LlfGetGoodSystemBlock()
{
    U16 block_no;
    U8  bank_num;
    U16 block_begin;
    U16 MpNum;
    U8 i;

    block_begin = guwCaclSSGroupEnd * NandPara.ubPlaneNumPerLun;

    //llfprintk("LlfGetGoodSystemBlock\r\n");
    for(bank_num = 0; bank_num < NandPara.ubBankNum ; bank_num ++)
    {
        guwMpDefectBlock[bank_num] = 0xFFFF;
        for (block_no = block_begin; block_no < SYSTEM_BLOCK_MAX_NUM; block_no++)
        {
            MpNum = block_no / NandPara.ubPlaneNumPerLun;
            if (llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_num, block_no))
            {
                if(guwMpDefectBlock[bank_num] == MpNum)
                {
                    //second
                    for (i = 0;  i < NandPara.ubPlaneNumPerLun ; i ++)
                    {
                        if ((MpNum * NandPara.ubPlaneNumPerLun) + i >= block_no)
                        {
                            if((MpNum * NandPara.ubPlaneNumPerLun) + i != block_no)
                                guwRemapTempBlock[bank_num][i] = (MpNum * NandPara.ubPlaneNumPerLun) + i;
                            else
                                guwRemapTempBlock[bank_num][i] = 0xFFFF;
                        }
                    }
                }
                else
                {
                    //push tempblock into table
                    if (guwMpDefectBlock[bank_num] != 0xFFFF)
                        LlfCreatReMappingTable(bank_num);

                    guwMpDefectBlock[bank_num] = MpNum;

                    //First
                    for (i = 0;  i < NandPara.ubPlaneNumPerLun ; i ++)
                    {
                        if((MpNum * NandPara.ubPlaneNumPerLun) + i != block_no)
                            guwRemapTempBlock[bank_num][i] = (MpNum * NandPara.ubPlaneNumPerLun) + i;
                        else
                            guwRemapTempBlock[bank_num][i] = 0xFFFF;
                    }

                }
                if (block_no == SYSTEM_BLOCK_MAX_NUM - 1)
                {
                    LlfCreatReMappingTable(bank_num);
                }
            }
            if ( (block_no == SYSTEM_BLOCK_MAX_NUM - 1 ) && !llfIsBlockBad(DBT_ADDR, bank_num, block_no))
            {
                LlfCreatReMappingTable(bank_num);
            }
        }
    }

}

void LlfUpdateDBT()
{
    U16 mp_num;
    U8  bank_num;

    //llfprintk("Update DBT info\r\n");

    for(mp_num = 0; mp_num < guwRealMPBlkNum - NandPara.uwMpBlockNumPerLun; mp_num ++)
    {
        for(bank_num = 0; bank_num < NandPara.ubBankNum ; bank_num ++)
        {
            if(mp_num >= guwMpDefectBlock[bank_num])
                llfSetDBT(bank_num, (mp_num + NandPara.uwMpBlockNumPerLun)*NandPara.ubPlaneNumPerLun, DBT_ADDR);
        }
    }

}
U32 LlfCheckRemappingTable()
{
    U32 i, j, k, z, m;
    U16 uwMinReMapMpBlock;
    U16 uwCurCheckSPBlk, uwRemainSPBlk;
    U16 uwReMapSizePerPlane;
    U16 uwMpBlkIdx;
    U16 uwcount = 0;
    U16 uwReMapMpFlag[BANK_NUM_MAX][NandPara.ubPlaneNumPerLun];
    U16 uwReMapMpBlock[NandPara.ubPlaneNumPerLun];
    U16 uwReMapMpBlockNum[BANK_NUM_MAX];
#ifdef NEW_BLK_REMAP
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    U64 uwCurMPBlkInBitMapTable;
#else
    U32 uwCurMPBlkInBitMapTable;
#endif
    U32	uwCurMPBlkInIdxTable;
#endif
    uwReMapSizePerPlane = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun ;

    llfprintk("Start checking Remapping table ...\r\n");

    //check index repeat or not each Bank and plane
    for (i = 0; i < NandPara.ubBankNum; i += 1)
    {
        for (j = 0; j < NandPara.ubPlaneNumPerLun; j += 1)
        {
            uwReMapMpFlag[i][j] = 0;
            for (k = 0; k < uwReMapSizePerPlane ; k += 2)
            {
                uwCurCheckSPBlk = _MEM16(BLK_REMAP_TABLE_ADDR + k + (j * uwReMapSizePerPlane) +
                                         (i * guwReMapTableSizePerBank));
                //if equal 0 means didn't remapping, so don't need to check
                if (uwCurCheckSPBlk == 0)
                {
                    ;
                }
                else
                {
                    uwReMapMpFlag[i][j] = uwReMapMpFlag[i][j] + 1;
                    //make sure use same plane's block to remapping
                    if ((uwCurCheckSPBlk % NandPara.ubPlaneNumPerLun) != j)
                    {
                        llfprintk("Take wrong plane blk to remaping, Bank = %d, Plane = %d, SPBlk = %d\r\n", i, j,
                                  uwCurCheckSPBlk);
                        AddErrorMessage(i, 0, ERR_BLOCK_MAPPING);
                        return ERR_BLOCK_MAPPING;
                    }
                    for (z = k + 2; z < uwReMapSizePerPlane ; z += 2)
                    {
                        uwRemainSPBlk = _MEM16(BLK_REMAP_TABLE_ADDR + z + (j * uwReMapSizePerPlane) +
                                               (i * guwReMapTableSizePerBank));
                        if( uwCurCheckSPBlk == uwRemainSPBlk)
                        {
                            llfprintk("Find duplicate index in same plane, Bank = %d, Plane = %d, SPBlk = %d\r\n", i, j,
                                      uwCurCheckSPBlk);
                            AddErrorMessage(i, 0, ERR_BLOCK_MAPPING);
                            return ERR_BLOCK_MAPPING;
                        }
                    }
                    uwMpBlkIdx = uwCurCheckSPBlk / NandPara.ubPlaneNumPerLun;
                    //check DBT
                    if (llfIsMpBlockBad(DBT_ADDR, i, uwMpBlkIdx))
                    {
                        ;//Remapping Block in DBT
                    }
                    else//if DBT did not mark bad, need to check is more than half of the block broken
                    {
                        for (m = 0; m < NandPara.ubBankNum; m += 1)
                        {
                            if(llfIsMpBlockBad(DBT_ADDR, m, uwMpBlkIdx))
                            {
                                uwcount++;
                            }

                        }
                        if ((uwcount > (NandPara.ubBankNum >> 1)) && (uwcount != NandPara.ubBankNum))
                        {
                            ;
                        }
                        else
                        {
#ifdef NEW_BLK_REMAP
#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
                            uwCurMPBlkInBitMapTable = _MEM64(BLK_REMAP_BITMAP_TABLE_ADDR +  (uwMpBlkIdx * 8));
#else
                            uwCurMPBlkInBitMapTable = _MEM16(BLK_REMAP_BITMAP_TABLE_ADDR +  (uwMpBlkIdx * 2));
#endif
                            uwCurMPBlkInIdxTable = _MEM08(BLK_REMAP_BS_INDEX_TABLE_ADDR + uwMpBlkIdx);
                            //NEW_BLK_REMAP need to check BLK_REMAP_BITMAP_TABLE and BLK_REMAP_BS_INDEX_TABLE
                            if (uwCurMPBlkInBitMapTable != 0 && uwCurMPBlkInIdxTable != 0xff)
                            {
                                ;
                            }
                            else
#endif
                            {
                                llfprintk("Can't find Remap block in DBT table, and no more than half of the blocks are broken, Bank = %d, Plane = %d, MPblk = %d",
                                          i, j, uwMpBlkIdx);
                                AddErrorMessage(i, 0, ERR_BLOCK_MAPPING);
                                return ERR_BLOCK_MAPPING;
                            }
                        }
                    }
                }
            }
        }
    }

    for(i = 0; i < NandPara.ubBankNum; i += 1)
    {
        uwMinReMapMpBlock = uwReMapMpFlag[i][0];
        for(j = 1; j < NandPara.ubPlaneNumPerLun; j++)
        {
            if (uwMinReMapMpBlock > uwReMapMpFlag[i][j])
            {
                uwMinReMapMpBlock = uwReMapMpFlag[i][j];
            }
        }
        uwReMapMpBlockNum[i] = uwMinReMapMpBlock;
    }

    for (m = 0; m < NandPara.ubBankNum; m += 1)
    {
        for(z = 0; z < uwReMapMpBlockNum[m] * 2; z += 2)
        {
            for (j = 0; j < NandPara.ubPlaneNumPerLun; j += 1)
            {
                uwCurCheckSPBlk = _MEM16(BLK_REMAP_TABLE_ADDR + z + (j * uwReMapSizePerPlane) +
                                         (m * guwReMapTableSizePerBank));
                uwReMapMpBlock[j] =  uwCurCheckSPBlk / guwRealBlockNum;
            }
            for (k = 0; k < (NandPara.ubPlaneNumPerLun - 1); k += 1)
            {
                if (uwReMapMpBlock[k] != uwReMapMpBlock[k + 1])
                {
                    llfprintk("Use different lun to remapping a MPBlk Bank = %d\r\n", m);
                    AddErrorMessage(m, 0, ERR_BLOCK_MAPPING);
                    return ERR_BLOCK_MAPPING;
                }
            }
        }
    }

    llfprintk("Each remapping MPBlk is composed of the same lun\r\n");

    llfprintk("Checking Remapping table successful...\r\n");
    return ERR_OK;
}

U32 LlfInheritRemappingTable(U32 addr)
{
    U8  bank_num, ubSblkStart = 0, ubSblkEnd = SYS_BLK;
    U16	block_num;
    U16 page_num;
    U32 ulMode, ret, cmp, i;
    U8  flag;
    U16 base_page = 0;
    U32 header_id;
    U8 lun_num = 0;

    ret = ERR_READ_BLOCKREMAPPING;
#if defined(KEEP_RDT_RESULT) || defined(MST_MERGE)
    if((gubLLFMode == LLF_DEFECT_BAD_BLOCK))
    {
#ifdef SBLK_EXPAND
        ubSblkStart = gubSblkStart + SYS_BLK;
        ubSblkEnd = gubSblkStart + SYS_BLK + EXTEND_RDT_BLK;
#else
        ubSblkStart = SYS_BLK;
        ubSblkEnd = SYS_BLK + EXTEND_RDT_BLK;
#endif
    }
    else
#endif
    {
#ifdef SBLK_EXPAND
        ubSblkStart = gubSblkStart;
        ubSblkEnd = gubSblkStart + SYS_BLK + EXTEND_RDT_BLK;
#endif
    }

    if(gfDBTInitDone == LLF_DBT_NONE ||  gfDBTInitDone == LLF_DBT_RDT)
    {
        base_page = RDT_RESULT_PAGE_NUM;
//      header_id = REMAPTABLE_BLK_ID;
        header_id = RDT_BLK_ID;//For Tem by LeiWan
    }
    else
    {
        base_page = BLK_REMAP_PRO_PAGENO;
        header_id = CODE_BLK_ID;
    }

    llfprintk("Base_page of remap table is %d \r\n", base_page);
    cache_area_dinval(TEMP_HBUF_ADDR, WORD_BYTE_SIZE << _5BIT_SHIFT);
    cache_dummy_update_read();
    ASSERT_LLF(DRAM_DATA_SIZE == (NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT));
    for (bank_num = 0; bank_num < NandPara.ubBankNum; bank_num++)
    {
        for(block_num = ubSblkStart; block_num < ubSblkEnd; block_num++)
        {
            flag = 0 ;
            for (page_num = 0; page_num <  BLK_REMAP_PRO_PAGENUM; page_num++) //4 page data
            {
                cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage << SECTOR_BYTE_SHIFT);
                cache_dummy_update_read();

                ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
                gul_FW_TAG = llfBETagSetting(TAG_READ, bank_num);
                lun_num =  bank_num / NandPara.ubBankNumPerLun;
                llfFCCmdRead_DRAM(ulMode, bank_num, lun_num, block_num, page_num + base_page,
                                  addr + (DRAM_DATA_SIZE * page_num), DRAM_DATA_SIZE,
                                  TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);

                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                if (ret != ERR_OK)
                {
                    AddErrorMessage(bank_num, 0, ERR_READ_RDT);
                    ret = ERR_READ_RDT;
                    break;
                }
#ifdef SLC_RRT
                ret = llfReadRetry(bank_num, block_num, page_num + base_page, &cmp,
                                   addr + (DRAM_DATA_SIZE * page_num), TEMP_HBUF_PHY_ADDR);
                if (ret != ERR_OK)
                {
                    ret = ERR_READ_RDT;
                    break;
                }
#endif

                else
                {
                    if(((cmp & BE_COMPLETION_ERROR_MASK) == 0) && (_REG32(TEMP_HBUF_ADDR) == header_id))
                    {
                        llfprintk("read OK,block = %d, page = %d\r\n", block_num, page_num + base_page);
                        flag++;
                    }
                    else
                    {
                        printk("Block remap read page %d cmp %x\r\n", page_num + base_page, cmp);
                        dma_memset(addr + (DRAM_DATA_SIZE * page_num), 0, DRAM_DATA_SIZE);
                        ret = ERR_READ_BLOCKREMAPPING;
                        break;
                    }
                }

                if (flag == BLK_REMAP_PRO_PAGENUM)
                {
                    llfprintk("Block Remapping Read 4 page Data OK!\r\n");
                    //LlfParseBlockRemapTable(addr);

#if 1
                    addr = BLK_REMAP_TABLE_ADDR;
                    for(i = 0; i < guwReMapTableSizePerBank * NandPara.ubBankNum; i += 32)
                    {
                        llfprintk("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                                  _MEM16(addr + i),
                                  _MEM16(addr + i + 2),
                                  _MEM16(addr + i + 4),
                                  _MEM16(addr + i + 6),
                                  _MEM16(addr + i + 8),
                                  _MEM16(addr + i + 10),
                                  _MEM16(addr + i + 12),
                                  _MEM16(addr + i + 14),
                                  _MEM16(addr + i + 16),
                                  _MEM16(addr + i + 18),
                                  _MEM16(addr + i + 20),
                                  _MEM16(addr + i + 22),
                                  _MEM16(addr + i + 24),
                                  _MEM16(addr + i + 26),
                                  _MEM16(addr + i + 28),
                                  _MEM16(addr + i + 30));
                    }
#endif

                    return ERR_OK;
                }

            }
        }
    }

    return ret;

}

#if 0
void LlfParseBlockRemapTable(U32 addr)
{
    U16 i;
    U16 per_plane_lenth;
    U8 bank_num;
    U8 plane_num;
    U16 mp_num;

    //output table
    for(i = 0; i < guwReMapTableSizePerBank * NandPara.ubBankNum; i += 32)
    {
        llfprintk("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                  _MEM16(addr + i),
                  _MEM16(addr + i + 2),
                  _MEM16(addr + i + 4),
                  _MEM16(addr + i + 6),
                  _MEM16(addr + i + 8),
                  _MEM16(addr + i + 10),
                  _MEM16(addr + i + 12),
                  _MEM16(addr + i + 14),
                  _MEM16(addr + i + 16),
                  _MEM16(addr + i + 18),
                  _MEM16(addr + i + 20),
                  _MEM16(addr + i + 22),
                  _MEM16(addr + i + 24),
                  _MEM16(addr + i + 26),
                  _MEM16(addr + i + 28),
                  _MEM16(addr + i + 30));
    }

    //parse table
    per_plane_lenth = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun / 2;
    llfprintk("guwReMapTableSizePerBank is %d ,per_plane_lenth is %d\r\n", guwReMapTableSizePerBank,
              per_plane_lenth);

    for(bank_num = 0 ; bank_num < NandPara.ubBankNum; bank_num ++)
    {
        for(plane_num = 0; plane_num < NandPara.ubPlaneNumPerLun; plane_num ++)
        {
            for(mp_num = 0; mp_num < per_plane_lenth; mp_num ++)
                if(_REG16(addr + guwReMapTableSizePerBank * bank_num + per_plane_lenth * 2 * plane_num + mp_num * 2)
                        != 0)
                    guwRemapFlag[bank_num][plane_num]++;
        }

    }

    LlfGetRealMaxBsNum();

}
#endif

U32 LlfOutPutRemapTable()
{
    U32 addr, ret;
    U32 i;

    addr = BLK_REMAP_TABLE_ADDR;
    llfprintk("LlfOutPutRemapTable()\r\n");
    llfprintk("guwReMapTableSizePerBank %d NandPara.ubBankNum %d \r\n", guwReMapTableSizePerBank,
              NandPara.ubBankNum);

    for(i = 0; i < guwReMapTableSizePerBank * NandPara.ubBankNum; i += 32)
    {
        llfprintk("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                  _MEM16(addr + i),
                  _MEM16(addr + i + 2),
                  _MEM16(addr + i + 4),
                  _MEM16(addr + i + 6),
                  _MEM16(addr + i + 8),
                  _MEM16(addr + i + 10),
                  _MEM16(addr + i + 12),
                  _MEM16(addr + i + 14),
                  _MEM16(addr + i + 16),
                  _MEM16(addr + i + 18),
                  _MEM16(addr + i + 20),
                  _MEM16(addr + i + 22),
                  _MEM16(addr + i + 24),
                  _MEM16(addr + i + 26),
                  _MEM16(addr + i + 28),
                  _MEM16(addr + i + 30));
    }

#ifdef NEW_BLK_REMAP

    addr = BLK_REMAP_BITMAP_TABLE_ADDR;

    llfprintk("BLK_REMAP_BITMAP_TABLE_ADDR\r\n");

#if defined(RTS5771_FPGA) || defined(RTS5771_VA)
    for(i = 0; i < NandPara.uwMpBlockNumPerLun * 8; i += 64)
    {
        llfprintk("%x_%x\t%x_%x\t%x_%x\t%x_%x\t%x_%x\t%x_%x\t%x_%x\t%x_%x\r\n",
                  _MEM32(addr + i),
                  _MEM32(addr + i + 4),
                  _MEM32(addr + i + 8),
                  _MEM32(addr + i + 12),
                  _MEM32(addr + i + 16),
                  _MEM32(addr + i + 20),
                  _MEM32(addr + i + 24),
                  _MEM32(addr + i + 28),
                  _MEM32(addr + i + 32),
                  _MEM32(addr + i + 36),
                  _MEM32(addr + i + 40),
                  _MEM32(addr + i + 44),
                  _MEM32(addr + i + 48),
                  _MEM32(addr + i + 52),
                  _MEM32(addr + i + 56),
                  _MEM32(addr + i + 60));
    }

#else
    for(i = 0; i < NandPara.uwMpBlockNumPerLun * 2; i += 32)
    {
        llfprintk("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                  _MEM16(addr + i),
                  _MEM16(addr + i + 2),
                  _MEM16(addr + i + 4),
                  _MEM16(addr + i + 6),
                  _MEM16(addr + i + 8),
                  _MEM16(addr + i + 10),
                  _MEM16(addr + i + 12),
                  _MEM16(addr + i + 14),
                  _MEM16(addr + i + 16),
                  _MEM16(addr + i + 18),
                  _MEM16(addr + i + 20),
                  _MEM16(addr + i + 22),
                  _MEM16(addr + i + 24),
                  _MEM16(addr + i + 26),
                  _MEM16(addr + i + 28),
                  _MEM16(addr + i + 30));

    }
#endif

    addr = BLK_REMAP_BS_INDEX_TABLE_ADDR;

    llfprintk("BLK_REMAP_BS_INDEX_TABLE_ADDR\r\n");
    for(i = 0; i < NandPara.uwMpBlockNumPerLun; i += 16)
    {
        llfprintk("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",
                  _MEM08(addr + i),
                  _MEM08(addr + i + 1),
                  _MEM08(addr + i + 2),
                  _MEM08(addr + i + 3),
                  _MEM08(addr + i + 4),
                  _MEM08(addr + i + 5),
                  _MEM08(addr + i + 6),
                  _MEM08(addr + i + 7),
                  _MEM08(addr + i + 8),
                  _MEM08(addr + i + 9),
                  _MEM08(addr + i + 10),
                  _MEM08(addr + i + 11),
                  _MEM08(addr + i + 12),
                  _MEM08(addr + i + 13),
                  _MEM08(addr + i + 14),
                  _MEM08(addr + i + 15));

    }
#endif

    ret = LlfCheckRemappingTable();
    if (ret != ERR_OK)
    {
        return ERR_BLOCK_MAPPING;
    }
    gubBlkIheritRemapFlag = 1;
    return ERR_OK;

}

U32 LlfUpdateRemapBlock()
{
    U8	plane_no, bank_no;
    U16 reamp_table_index;
    U16 block_no;
    U32 defect_flg;
    U8  ubtype;
    U32 ret = ERR_OK;

    //llfprintk("LlfUpdateRemapBlock,check bad block ....\r\n");
    if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC)
            && FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK )
    {
        ubtype = 1;
    }
    else
    {
        ubtype = 0;
    }

    //add for remap table data is ok
    for(bank_no = 0 ; bank_no < NandPara.ubBankNum; bank_no ++)
    {
        for(plane_no = 0; plane_no < NandPara.ubPlaneNumPerLun; plane_no++)
        {
            llfprintk("guwRemapFlag[%d][%d] = %d\r\n", bank_no, plane_no, guwRemapFlag[bank_no][plane_no]);
            for(reamp_table_index = 0; reamp_table_index < guwRemapFlag[bank_no][plane_no];
                    reamp_table_index ++)
            {
                block_no = _MEM16(BLK_REMAP_TABLE_ADDR +
                                  (guwReMapTableSizePerBank * bank_no) +
                                  ((guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun) * plane_no) +
                                  ((reamp_table_index) * 2));

                //scan all block in Remap Table
                if(ubtype == 0)
                {
                    defect_flg = llfChkDefectBlk(bank_no, block_no);//return 0 is bad
                }
                else
                {
                    defect_flg = llfChkDefectBlkForSandisk(bank_no, block_no);
                }
                if(defect_flg == ERR_ECC || defect_flg == ERR_FIO_TIMEOUT)
                {
                    llfprintk("defect read fail\r\n");
                    AddErrorMessage(bank_no, 0, ERR_ALLOCATE_DBT);
                    ret = ERR_ALLOCATE_DBT;
                    return ret;
                }

                // block defect
                if(!defect_flg)
                {
                    //check DBT
                    if(block_no < SYSTEM_BLOCK_MAX_NUM + gubBlockNoBank[bank_no])
                    {
                        if(!(llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block_no)))
                        {
                            llfMarkSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block_no);
                        }
                    }
                    else
                    {
                        if(!(llfIsBlockBad(DBT_ADDR, bank_no, block_no)))
                        {
                            llfSetDBT(bank_no, block_no, DBT_ADDR);
                        }
                    }
                    llfprintk("In Remap table bank %d block %d is defect,REMOVE THIS BLOCK >>>>>\r\n", bank_no,
                              block_no);
                    _MEM16(BLK_REMAP_TABLE_ADDR +
                           (guwReMapTableSizePerBank * bank_no) +
                           ((guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun) * plane_no) +
                           ((reamp_table_index) * 2)) = _MEM16(BLK_REMAP_TABLE_ADDR +
                                                        (guwReMapTableSizePerBank * bank_no) +
                                                        ((guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun) * plane_no) +
                                                        ((guwRemapFlag[bank_no][plane_no] - 1) * 2));
                    _MEM16(BLK_REMAP_TABLE_ADDR +
                           (guwReMapTableSizePerBank * bank_no) +
                           ((guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun) * plane_no) +
                           ((guwRemapFlag[bank_no][plane_no] - 1) * 2)) = 0;

                    guwRemapFlag[bank_no][plane_no] -- ;
                    reamp_table_index --;
                }

            }
        }


    }

    return ret;
}

#if 0
void LlfGetGoodMPNumInTable()
{
    U16 per_plane_lenth;
    U8 bank_num;
    U16 remap_mp_num[NandPara.ubBankNum];

    per_plane_lenth = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun / 2;
    llfprintk("guwReMapTableSizePerBank is %d ,per_plane_lenth is %d\r\n", guwReMapTableSizePerBank,
              per_plane_lenth);

    for(bank_num = 0; bank_num < NandPara.ubBankNum ; bank_num ++)
    {
        remap_mp_num[bank_num] = LlfSearchMpGood(bank_num, per_plane_lenth);
    }
}

U16 LlfSearchMpGood(U8 bank_no, U16 per_plane_lenth)
{
    U8	plane_no = 0;
    U16 reamp_table_index;
    U16 block_no;
    U8  ubtype;
    U32 ret = ERR_OK;
    U8  sp_exist_num = 0;
    U8  mp_good_num = 0;

    //add for remap table data is ok
    for(reamp_table_index = 0; reamp_table_index < per_plane_lenth; reamp_table_index ++)
    {
        block_no = _MEM16(BLK_REMAP_TABLE_BASE_ADDR +
                          (guwReMapTableSizePerBank * bank_no) +
                          ((guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun) * 0) +
                          ((reamp_table_index) * 2));
        if (block_no == 0) // end of this plane
        {
            llfprintk("[Remap]bank %d plane0 remapIndex %d,search MpNum %d ,end...\r\n", bank_no,
                      reamp_table_index, mp_good_num);
            return mp_good_num;
        }
        else
        {
            for(plane_no = 1; plane_no < NandPara.ubPlaneNumPerLun; plane_no++)
            {
                ret = LlfSearchSPNoInRemapTable(bank_no, block_no, plane_no);
                if(ret == ERR_OK)
                {
                    sp_exist_num ++;
                }
                if(sp_exist_num = NandPara.ubPlaneNumPerLun - 1)
                {
                    mp_good_num ++;
                    llfMarkUserMPBlkDefect(DBT_ADDR, bank_no, block_no / NandPara.ubPlaneNumPerLun);
                }
            }
        }
    }

}

U32 LlfSearchSPNoInRemapTable(U8 bank_no, U16 block_no, U8 plane_no)
{
    U16 per_plane_lenth;
    U16 reamp_table_index;
    U16 target_block;
    U16 remap_block;

    target_block = block_no + plane_no;
    per_plane_lenth = guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun / 2;
    for(reamp_table_index = 0; reamp_table_index < per_plane_lenth; reamp_table_index ++)
    {
        remap_block = _MEM16(BLK_REMAP_TABLE_BASE_ADDR +
                             (guwReMapTableSizePerBank * bank_no) +
                             ((guwReMapTableSizePerBank / NandPara.ubPlaneNumPerLun) * plane_no) +
                             ((reamp_table_index) * 2));
        if (remap_block == target_block)
        {
            llfprintk("P0_block %d P%d_block %d, search ok !!!\r\n", plane_no, remap_block);
            return ERR_OK;
        }
        else if (remap_block == 0 )
        {
            llfprintk("P0_block %d P%d_block %d, search ok !!!\r\n", plane_no, remap_block);
            return ERR_BLOCK_MAPPING;
        }

    }
}

#endif
#endif

#ifdef REPLACE_IMAGE
U32 llfEraseForReplaceImage(U8 bank_no, U8 block_no)
{
    LLF_UNI_INFO *pLLFUniInfo;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PVENDOR_CMD pVendorCmd;
    U32 ret = ERR_OK;
    U32 ulMode;
    U32 cmp;
    U16 uwPage = 0;;
    U8 lun_no;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (struct _LLF_UNI_INFO *)LLF_UNI_INFO_ADDR;
    pVendorCmd = (PVENDOR_CMD)(LLF_CMD_BUF_VA_ADDR);


    // initialize erase arguments
    pResponseInfo->res_progress = 0;
    pLLFUniInfo->ulWorkFunc = ERASE_ALL_FUNC;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    lun_no = bank_no / NandPara.ubBankNumPerLun;

    gul_FW_TAG = llfBETagSetting(TAG_ERASE, bank_no);
#ifdef REPLACE_IMAGE_DEBUG
    llfprintk("[DEBUG] Begin to Erase bank_block(%d_%d)\r\n", bank_no, block_no);
#endif

#if defined(FTL_N38A) || defined(FTL_N38B) || defined(FTL_Q5171A)
    for(uwPage = 0; uwPage < (INTELQ_PAGE_PER_SECTION * INTELQ_SECTION_PER_BLOCK);
            uwPage += INTELQ_PAGE_PER_SECTION)
#endif
    {
        FCSingleErase(ulMode, bank_no, lun_no, block_no, uwPage, 0);
        ret = FCCompletionPolling(&cmp, gul_FW_TAG);
        if(ret != ERR_OK)
        {
            llfDbgPrintk(ALWAYS_MSG, "Err%d\r\n", bank_no);
        }
        //whether need to check
        if((cmp & 0x7fff) != 0)
        {
            llfDbgPrintk(ALWAYS_MSG, "bank%d block%d erase fail cmp %x \r\n", bank_no, block_no, cmp);
        }
    }

    if (pVendorCmd->subcmd != BE_LLF_ALL)  // pVendorCmd->subcmd == BE_LLF_ERASENAND
    {
        llfEndResponceNonPrint(ERR_OK);
        return ERR_OK;
    }

    // erase end
    pResponseInfo->res_progress = 80;
    pResponseInfo->res_err_code = ERR_OK;
    pLLFUniInfo->ulWorkFunc = NONE_FUNC;
    PrintDone();

    return ret;
}

void llfReplaceImage()
{
    U8 bank, block, ubIFType, ubClkMode;
    PVENDOR_CMD_RESPONSE pResponseInfo;
    PLLF_UNI_INFO pLLFUniInfo;
    U32 ret;

    pResponseInfo = (PVENDOR_CMD_RESPONSE)LLF_RES_BUF_VA_ADDR;
    pLLFUniInfo = (PLLF_UNI_INFO)LLF_UNI_INFO_ADDR;

    bank = (gulReplaceImageNum >> RI_BANK_MASK) & 0xF;
    block = (gulReplaceImageNum >> RI_BLOCK_MASK) & 0xF;

    ubIFType = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_INTERFACE_OFFSET);
    ubClkMode = _MEM08(CONFIG_BASE_VA_ADDR + CONFIG_CLOCK_OFFSET);

    // erase maximum all banks block0
    if(bank >= MIN(NandPara.ubBankNum, SYS_BANK_NUM) || block >= SYS_BLK)
    {
        llfprintk("[ERR] ReplaceImage input bank_block(%d_%d) is invaild!\r\n", bank, block);
    }

    ret = llfEraseForReplaceImage(bank, block);
    if(ret != ERR_OK)
    {
        llfEndResponce(ret);
        return;
    }

    pResponseInfo->res_state = VENDOR_CMD_EXECUTE;
    pLLFUniInfo->ulWorkFunc = WRITE_CONFIG_FUNC;
    llfAPExecuteLLF(ubIFType, ubClkMode);
}

U32 llfWriteCodeBlockForReplaceImage(U8 fw_type, U8 bank_no, U8 block_no)
{
    U32 ulMode;
    U8 i;
    U8 lun_no;
    U16 page;
    U16 fw_page_count;
    U32 ulCBlkBid;
    U32 cmp;
    U32 addr, paddr;
    U32 ret = ERR_OK;
    U8 BadSblk_flag;

    //get flash interface type here
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);

    //initial code head data
    for(i = 0; i < DRAM_HEAD_SIZE / 4; i++)
    {
        _REG32(TEMP_HBUF_ADDR + i * 4) = CODE_BLK_ID;
    }
    cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
    cache_dummy_update_read();

    // initial bank, block, page
    page  = 0;

    // checking page number is acceptable
    if(fw_type == FW_IN_DRAM)
    {
        fw_page_count = (MAX_DRAM_CODE_SIZE / (NandPara.ubSectorNumPerPage * 512));
    }
    else
    {
        fw_page_count = (FW_CODE_SIZE
                         + (NandPara.ubSectorNumPerPage * 512) - 1) / (NandPara.ubSectorNumPerPage * 512);
    }
    llfDbgPrintk(ALWAYS_MSG, "fw_page_count = %d\r\n", fw_page_count);
    if(fw_page_count > (NandPara.uwSLCPageNumPerBlock - 1))
    {
        llfDbgPrintk(ALWAYS_MSG, "[RI] fw_page_count = %d\r\n", fw_page_count);
        return ERR_CODE_SIZE;
    }

    // start to write FW code.
    Change_ldpc(gubECC_CFG);

    lun_no = bank_no / NandPara.ubBankNumPerLun;

    llfprintk("[RI] pSblkNo[%d][%d]\r\n", bank_no, block_no);

    BadSblk_flag = 0;
    // Write, page=1 (the page0 is sblock)
    for(page = 1; page < NandPara.uwSLCPageNumPerBlock; page++)
    {
        _REG32(TEMP_HBUF_ADDR + 4) = page;
        cache_area_dwbinval(TEMP_HBUF_ADDR, HEADER_MAX_LEN);
        cache_dummy_update_read();
        if(( page >= TABLE_PAGENO ) && ( page < TABLE_PAGENO + TABLE_PAGENUM ))
        {
            addr = TABLE_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO);
            paddr = TABLE_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO);
        }
        else if( (page >= DBT_PAGENO) && (page < DBT_PAGENO + DBT_PAGENUM))
        {
            addr = DBT_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - DBT_PAGENO);
            paddr = DBT_PHY_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - DBT_PAGENO);
        }
        else if(page <= fw_page_count)
        {
            addr = FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * (page - 1) * 512);
            paddr = FW_CODE_PHY_ADDR + (NandPara.ubSectorNumPerPage * 512) * (page - 1);
        }
        else
        {
            addr = FW_CODE_ADDR;
            paddr = FW_CODE_PHY_ADDR;
        }

        cache_area_dwbinval(addr, NandPara.ubSectorNumPerPage * 512);
        cache_dummy_update_read();

        gul_FW_TAG = llfBETagSetting(TAG_WRITE, bank_no);
        llfFCCmdWrite_DRAM(ulMode, bank_no, lun_no, block_no, page, paddr,
                           DRAM_DATA_SIZE, TEMP_HBUF_PHY_ADDR, DRAM_HEAD_SIZE);
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret == ERR_OK)
        {
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "write code blk error: %d_0x%x\r\n", bank_no, cmp);
                BadSblk_flag = 1;
                break;
            }
        }
        else
        {
            llfprintk("Write code completion timeout bank%d, block%d\r\n", bank_no, block_no);
            ASSERT_LLF(0);
        }
    }

    if(BadSblk_flag)
    {
        return ERR_WRITE_CODE_TIMEOUT;
    }
    // Read back, only compare we need.
    for(page = 0; page < fw_page_count; page++)
    {
        // Read back
        cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
        cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);

        gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);
        llfFCCmdRead_DRAM(ulMode, bank_no, lun_no, block_no, (page + 1), TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                          TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret == ERR_OK)
        {
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "read code blk error: %d_%d_0x%x\r\n", bank_no, 0, cmp);
                BadSblk_flag = 1;
                break;
            }

            ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
            if(ulCBlkBid != CODE_BLK_ID)
            {
                BadSblk_flag = 1;
                break;
            }
            ret = llfCompareData(FW_CODE_ADDR + (NandPara.ubSectorNumPerPage * page * 512),
                                 TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "code blk data compare error: %d_%d\r\n", bank_no, block_no);
                BadSblk_flag = 1;
                break;
            }
        }
        else
        {
            llfprintk("read code cmp timeout bank%d block%d\r\n", bank_no, block_no);
            ASSERT_LLF(0);
        }

    }
    if(BadSblk_flag)
    {
        return ERR_READ_CODE;
    }
    llfDbgPrintk(ALWAYS_MSG, "[RI] bank%d block%d read FW OK!\r\n", bank_no, block_no);

    for(page = TABLE_PAGENO; page < TABLE_PAGENO + TABLE_PAGENUM; page++ )
    {
        // Read back
        cache_area_dinval((TEMP_HBUF_ADDR + HEADER_MAX_LEN), HEADER_MAX_LEN);
        cache_dummy_update_read();
        cache_area_dinval(TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
        cache_dummy_update_read();

        gul_FW_TAG = llfBETagSetting(TAG_READ, bank_no);
        llfFCCmdRead_DRAM(ulMode, bank_no, lun_no, block_no, page, TEMP_BUF_PHY_ADDR, DRAM_DATA_SIZE,
                          TEMP_HBUF_PHY_ADDR + HEADER_MAX_LEN, DRAM_HEAD_SIZE);
        ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
        if(ret == ERR_OK)
        {
            if((cmp & BE_COMPLETION_ERROR_MASK) != 0)
            {
                llfDbgPrintk(ALWAYS_MSG, "read FC TABLE error: %d_%d_%d\r\n", bank_no, block_no, page);
                BadSblk_flag = 1;
                break;
            }

            ulCBlkBid = _REG32(TEMP_HBUF_ADDR + HEADER_MAX_LEN);
            if(ulCBlkBid != CODE_BLK_ID)
            {
                llfDbgPrintk(ALWAYS_MSG, "FC table blk id compare error: %d_%d\r\n", bank_no, block_no);
                BadSblk_flag = 1;
                break;
            }
            ret = llfCompareData(TABLE_ADDR + NandPara.ubSectorNumPerPage * 512 * (page - TABLE_PAGENO),
                                 TEMP_BUF_ADDR, NandPara.ubSectorNumPerPage * 512);
            if(ret != ERR_OK)
            {
                llfDbgPrintk(ALWAYS_MSG, "FC table blk data compare error: %d_%d\r\n", bank_no, block_no);
                BadSblk_flag = 1;
                break;
            }
            //page ++;    //sync problem to llfWriteCodeBlock
        }
        else
        {
            llfprintk("read FC table cmp timeout bank%d, block%d\r\n", bank_no, block_no);
            ASSERT_LLF(0);
        }
    }
    if( BadSblk_flag )
    {
        return ERR_READ_SBLK;
    }
    llfDbgPrintk(ALWAYS_MSG, "[RI] bank%d block%d read FC table ok!\r\n", bank_no, block_no);

    return ret;
}

U32 llfllfWriteSysGroupForReplaceImage(U8 bank_no, U8 block_no)
{
    U32 ret;
    U32 addr;

    // write FW mpblock
    if(READ_REG_32(CPU_MODE_REG)&CPU_EA_MODE)
        addr = SPI_DL_TAG_ADDR_WHEN_EA;
    else
        addr = SPI_DL_TAG_VA_ADDR;
#ifdef REPLACE_IMAGE_DEBUG
    llfprintk("[RI] [DEBUG] addr = %x, READ_REG_32(addr) = %x\r\n", addr, READ_REG_32(addr));
#endif

    if(READ_REG_32(addr) == IMEM1_JUMP_TAG)
    {
        ret = llfWriteCodeBlockForReplaceImage(FW_IN_DRAM, bank_no, block_no);
    }
    else
    {
        if(gubMpModeSelect)
        {
            ret = llfWriteCodeBlockForReplaceImage(FW_IN_DRAM, bank_no, block_no);
        }
        else
        {
            ret = llfWriteCodeBlockForReplaceImage(FW_IN_IMEM1, bank_no, block_no);
        }
    }

    if (ret != ERR_OK)
        return ret;

    return ERR_OK;

}

U32 llfWriteConfigForReplaceImage(U8 ubIfType, U8 ubClkMode)
{
    U32 ret = ERR_OK;
    U8 bank_no, block_no, isSandisk = 0;

    bank_no = (gulReplaceImageNum >> RI_BANK_MASK) & 0xF;
    block_no = (gulReplaceImageNum >> RI_BLOCK_MASK) & 0xF;
#ifdef REPLACE_IMAGE_DEBUG
    llfprintk("[RI] [DEBUG] _MEM32(FW_RDT_TAG) = 0x%x, gfSelfTestFlag = %d\r\n", _MEM32(FW_RDT_TAG),
              gfSelfTestFlag);
#endif


    //check bad bank block
    if((FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24 ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT24_64G ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_256Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4P5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_512Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_1024Gb_ODT ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_QLC ||
            FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS5_QLC)
            && (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK ))
    {
        isSandisk = 1;
    }

    if(isSandisk == 1)
    {
        if (llfChkDefectBlkForSandisk(bank_no, block_no) == 1 &&
                (llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block_no) == 0))
        {
            // Get a good block for S block
            llfprintk("[RI] bank%d block%d is good sblock\r\n", bank_no, block_no);
        }
        else
        {
            llfprintk("[FATAL][ERR] bank%d block%d is bad sblock\r\n", bank_no, block_no);
            llfAddErrorMessage(bank_no, 0, ERR_BAD_SBLOCK);
            return ERR_BAD_SBLOCK;
        }
    }
    else
    {
        if (llfChkDefectBlk(bank_no, block_no) == 1 &&
                (llfIsSystemBlkDefect(SYS_BLK_DBT_ADDR, bank_no, block_no) == 0))
        {
            // Get a good block for S block
            llfprintk("[RI] bank%d block%d is good sblock\r\n", bank_no, block_no);
        }
        else
        {
            llfprintk("[FATAL][ERR] bank%d block%d is bad sblock\r\n", bank_no, block_no);
            llfAddErrorMessage(bank_no, 0, ERR_BAD_SBLOCK);
            return ERR_BAD_SBLOCK;
        }
    }

    //write Code Table and DBT
    ret = llfllfWriteSysGroupForReplaceImage(bank_no, block_no);
    if(ret != ERR_OK)
    {
        return ret;
    }

    return ERR_OK;

}

#endif

void llfLoadDefaultFromGPIO()
{
    U32 reg;
    reg = READ_REG_32(U_MODE_FLAG);

    //Default all GPIO pull up, so default state is SDR & 5address cycle & 1.8V
    gubFcAddressCycleNum = (reg & POL_ADDR_CYCLE_MASK) ? 5 : 6;
    vdd_1v8_en = (reg & NAND_IO_USING_V18) ? 1 : 0;
    llfprintk("vdd_1v8_en = %d\r\n", vdd_1v8_en);
}

#ifdef AUTO_DETECT_DIE
U32 llfAutoDetectDie()
{
    U8 ubLun, ubBank, ubCh, ubCe, ubNoLunBank = 0;
    U8 ubLunExitNum = 0;
    U32 ulmode, cmp, ret;

    if(FLASH_INTERFACE(gulFlashVendorNum) == (FLASH_IF_SUP_TOGGLE | FLASH_IF_SUP_DDR2))
    {
        FC_TOP_REG(FR_FC_MODE) = 2;
        //FC_TOP_REG(FR_INTERFACE_SEL) = 1;
    }
    else if(FLASH_INTERFACE(gulFlashVendorNum) == (FLASH_IF_SUP_TOGGLE | FLASH_IF_SUP_DDR2 |
            FLASH_IF_SUP_SDR))
    {
        FC_TOP_REG(FR_FC_MODE) = 0;
        //FC_TOP_REG(FR_INTERFACE_SEL) = 1;
    }
    else
    {
        FC_TOP_REG(FR_FC_MODE) = 0;
        //FC_TOP_REG(FR_INTERFACE_SEL) = 0;
    }

    ulmode = FC_TOP_REG(FR_FC_MODE);

    for(ubCh = 0 ; ubCh < CH_NUM_MAX; ubCh++)
    {
        gulCHDieMap[ubCh][0] = 0;
        gulCHDieMap[ubCh][1] = 0;
    }

    // reset all ce & polling
    for(ubLun = 0; ubLun < 8; ubLun++)
    {
        for(ubCh = 0 ; ubCh < CH_NUM_MAX; ubCh++)
        {
            for(ubCe = 0; ubCe < CE_NUM_MAX; ubCe++)
            {
                ubBank = _MEM08(CONFIG_BASE_VA_ADDR + SBLK_OFFSET_FC_DIE_MAPPING_CE + ubCh * 8 + ubCe);

                //reset
                gul_FW_TAG = llfBETagSetting(TAG_RESET, ubBank);
                FCReset(ulmode, ubBank);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));
                FcBusyWait1ms(5);

                //polling satus
                gul_FW_TAG = llfBETagSetting(TAG_POLLING_STATUS, ubBank);

                FCStatusPollingLun(ulmode, ubBank, ubLun);
                ret = FCCompletionPolling(&cmp, (gul_FW_TAG));

                if(ret == ERR_OK)
                {
                    printk("[ADD] detect ch %d ce %d lun %d\r\n", ubCh, ubCe, ubLun);
                    if(ubCe < 4)
                    {
                        gulCHDieMap[ubCh][0] = gulCHDieMap[ubCh][0] | (1 << (ubCe * 8 + ubLun));
                    }
                    else
                    {
                        gulCHDieMap[ubCh][1] = gulCHDieMap[ubCh][1] | (1 << ((ubCe - 4) * 8 + ubLun));
                    }
                }
                else
                {
                    printk("[ADD] ch %d ce %d lun %d PollingLun timeout, cmp %x\r\n", ubCh, ubCe, ubLun, cmp);
                    llfResetAndRecoverFC10M(ulmode, FC_PLL_CLK_10M);
                }

            }
        }
        ubLunExitNum++;
        if(ubLunExitNum == gubRealLunNum * NandPara.ubCENumPerCh)
        {
            printk("[ADD] TotalLunNumPerCH %d\r\n", ubLunExitNum);
            break;
        }
    }

    for(ubCh = 0 ; ubCh < CH_NUM_MAX; ubCh ++)
    {
        printk("[ADD] CHDieMap[CH%d] = %x_%x\r\n", ubCh, gulCHDieMap[ubCh][1], gulCHDieMap[ubCh][0]);
        if (gulCHDieMap[ubCh][0] == 0 && gulCHDieMap[ubCh][1] == 0)
        {
            ubNoLunBank++;
        }
    }

    if(ubNoLunBank == CH_NUM_MAX)
    {
        for (ubBank = 0; ubBank < NandPara.ubBankNum; ubBank++)
        {
            llfAddErrorMessage(ubBank, 0, ERR_FIO_TIMEOUT);
        }
        return ERR_FIO_TIMEOUT;
    }

    CreateBankConfigByDieMap();
    llfResetAndRecoverFC10M(ulmode, FC_PLL_CLK_10M);
    ConvertBankToChCe(CONFIG_BASE_VA_ADDR, BANK_IMAPPING_TABLE_ADDR);
    return ERR_OK;
}
#endif
#ifdef SLC_RRT
U32 llfReadRetry(U8 bank, U16 block, U16 page, U32 *cmp, U32 ulAddr, U32 ulHeadAddr)
{
    U16 DblkRetryNum;
    U8 lun_no;
    U32 ret = ERR_OK, ulMode, ulcmp = *cmp;
    ulMode = FR_CONFIG_CH(FR_FC_MODE, gubStartCH);
    lun_no = bank / (NandPara.ubChNum * NandPara.ubCENumPerCh);
    if(gubmaxslcretry == 0)
    {
        return ret;
    }
    else
    {
        if(ulcmp & (ECC_UNCORRECT_ERR | CRC_ERR | UNC_ERR | OVER_ECC_THRESHOLD_ERR ))
        {
            //printk("normalread  %d_%d_%d cmp=0x%x\r\n", bank, block, page, ulcmp);
            for(DblkRetryNum = 1; DblkRetryNum <= gubmaxslcretry; DblkRetryNum++)
            {
                FCReadRetryDRAM(ulMode, bank, lun_no, block, page, DblkRetryNum,
                                ulAddr, DRAM_DATA_SIZE, ulHeadAddr, DRAM_HEAD_SIZE, 1);
                ret = FCCompletionPolling(&ulcmp, (gul_FW_TAG));
                if(ret != ERR_OK)
                {
                    ret = ERR_READ_SBLK_TIMEOUT;
                    printk("read Retry fail %d_%d_%d ret=0x%x !\r\n", bank, block, page, ret);
                    continue;
                }
                if((ulcmp & BE_COMPLETION_ERROR_MASK) != 0)
                {
                    if(DblkRetryNum < gubmaxslcretry)
                    {
                        // printk("read Retry %d_%d_%d cmp=0x%x ! DblkRetryNum:%d\r\n", bank, block, page, ulcmp, DblkRetryNum);
                        continue;
                    }
                    else
                    {
                        // printk("read Retry fail %d_%d_%d cmp=0x%x ! DblkRetryNum:%d\r\n", bank, block, page, ulcmp, DblkRetryNum);
                        *cmp = ulcmp;
                        return ERR_READ_SBLK;
                    }
                }
                else
                {
                    printk("read Retry OK %d_%d_%d cmp=0x%x ! DblkRetryNum:%d\r\n", bank, block, page, ulcmp,
                           DblkRetryNum);
                    *cmp = ulcmp;
                    return ERR_OK;
                }
            }
            return ret;
        }
        else
        {
            return ret;
        }
    }
}

U32 llfCompareReadRetrycnt()
{
    U8 slcreadretrycnt = 0;
    if(FLASH_VENDOR(gulFlashVendorNum) == IS_HYNIX)
    {
#if defined(FTL_H3DTV4) || defined(FTL_H3DTV5) || defined(FTL_H3DTV6) || defined(FTL_H3DTV7) || defined(FTL_H3DQV5)
        slcreadretrycnt = 54;
#endif
    }

    if((FLASH_VENDOR(gulFlashVendorNum) == IS_MICRON) || (FLASH_VENDOR(gulFlashVendorNum) == IS_INTEL))
    {
#if defined(FTL_L85A)
        slcreadretrycnt = 8;
#elif defined(FTL_B47R) || defined(FTL_N48R)
        slcreadretrycnt = 9;
#elif defined(FTL_N18A) || defined(FTL_N28A) || defined(FTL_N38A) || defined(FTL_N38B)
        slcreadretrycnt = 17;
#elif defined(FTL_B16A) || defined(FTL_B17A) || defined(FTL_B27A) || defined(FTL_B27B) || defined(FTL_B36R) || defined(FTL_B37R)
        slcreadretrycnt = 19;
#endif
    }

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_SAMSUNG)
    {
#if defined(FTL_SSV2) || defined(FTL_SSV4) || defined(FTL_SSV5) || defined(FTL_SSV6)
        slcreadretrycnt = 33;
#endif
    }

    if (FLASH_VENDOR(gulFlashVendorNum) == IS_SANDISK)
    {
#if defined(FTL_SANDISK_BICS4_QLC)
        slcreadretrycnt = 32;
#elif defined(FTL_SANDISK_BICS3)
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_8T22_SDR)
        {
            if(guwRealBlockNum == 2956)
            {
                slcreadretrycnt = 27;
            }
            else //(guwRealBlockNum == 1516)
            {
                slcreadretrycnt = 28;
            }
        }
        else //(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_XT23_SDR_TOGGLE_64GB)
        {
            slcreadretrycnt = 22;
        }
#elif defined(FTL_SANDISK_BICS4)
        if(FLASH_SERIAL_NUM(gulFlashVendorNum) == IS_BICS4_32GB)
        {
            slcreadretrycnt = 23;
        }
        else
        {
            slcreadretrycnt = 18;
        }
#elif defined(FTL_SANDISK_BICS5) || defined(FTL_SANDISK_BICS5_QLC)
        slcreadretrycnt = 26;
#endif
    }

    if (FLASH_VENDOR(gulFlashVendorNum) == IS_TOSHIBA)
    {
#if defined(FTL_TSB_BICS4_QLC)
        slcreadretrycnt = 7;
#elif defined(FTL_TSB_BICS3)
        slcreadretrycnt = 41;
#elif defined(FTL_TSB_BICS4)
        slcreadretrycnt = 9;
#elif defined(FTL_TSB_BICS5)
        slcreadretrycnt = 7;
#endif
    }

    if(FLASH_VENDOR(gulFlashVendorNum) == IS_YMTC)
    {
#if defined(FTL_YX2T)
        slcreadretrycnt = 8;
#elif defined(FTL_YX2Q)
        slcreadretrycnt = 10;
#elif defined(FTL_YG2T)
        slcreadretrycnt = 14;
#elif defined(FTL_YX3T) || defined(FTL_YX3T_WDS)
        slcreadretrycnt = 6; //Brian need fix
#endif
    }

    if(gubmaxslcretry > slcreadretrycnt)
    {
        return slcreadretrycnt;
    }

    return gubmaxslcretry;

}
#endif

//creating a new bracn is quick AND simp;
