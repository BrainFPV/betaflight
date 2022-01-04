/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_VIDEO Code for OSD video generator
 * @brief Output video (black & white pixels) over SPI
 * @{
 *
 * @file       pios_video.c
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2013-2015
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010-2014.
 * @brief      OSD gen module, handles OSD draw. Parts from CL-OSD and SUPEROSD projects
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
//#include "system.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/light_led.h"
#include "drivers/time.h"

#include "drivers/rcc.h"

#include "brainfpv/brainfpv_osd.h"
#include "brainfpv/auto_sync_threshold.h"

#if defined(INCLUDE_VIDEO_QUADSPI)

#if defined(STM32F446xx)
#include <stm32f4xx_qspi.h>
#else
#if defined(STM32H750xx)
#include "stm32h7xx_hal_qspi.h"
#include "stm32h7xx_hal_mdma.h"

QSPI_HandleTypeDef hqspi;
MDMA_HandleTypeDef hmdma;
#else
#error "MCU not supported"
#endif
#endif

#ifdef VIDEO_DEBUG_PIN
static IO_t debugPin;
#endif
static IO_t hsync_io;
static IO_t vsync_io;

#if defined(STM32H750xx)
static void Error_Handler(void) { while (1) { } }
#endif

#if !defined(VIDEO_QUADSPI_Y_OFFSET)
#define VIDEO_QUADSPI_Y_OFFSET 0
#endif /* !defined(VIDEO_QUADSPI_Y_OFFSET) */

#if defined(VIDEO_QSPI_IO2_PIN) && defined(VIDEO_QSPI_IO3_PIN)
#define VIDEO_QSPI_USE_4_LINES
#endif

#include "ch.h"
#include "video.h"

// How many frames until we redraw
#define VSYNC_REDRAW_CNT 2

// Minimum micro seconds between VSYNCS
#define MIN_DELTA_VSYNC 10000
#define MAX_SPURIOUS_VSYNCS 10

extiCallbackRec_t vsyncIntCallbackRec;
extiCallbackRec_t hsyncIntCallbackRec;

binary_semaphore_t onScreenDisplaySemaphore;

#define GRPAHICS_RIGHT_NTSC 351
#define GRPAHICS_RIGHT_PAL  359

static const struct video_type_boundary video_type_boundary_ntsc = {
    .graphics_right  = GRPAHICS_RIGHT_NTSC, // must be: graphics_width_real - 1
    .graphics_bottom = 239,                 // must be: graphics_hight_real - 1
};

static const struct video_type_boundary video_type_boundary_pal = {
    .graphics_right  = GRPAHICS_RIGHT_PAL, // must be: graphics_width_real - 1
    .graphics_bottom = 265,                // must be: graphics_hight_real - 1
};

#define NTSC_BYTES (GRPAHICS_RIGHT_NTSC / (8 / VIDEO_BITS_PER_PIXEL) + 1)
#define PAL_BYTES (GRPAHICS_RIGHT_PAL / (8 / VIDEO_BITS_PER_PIXEL) + 1)

static const struct video_type_cfg video_type_cfg_ntsc = {
    .graphics_hight_real   = 240,   // Real visible lines
    .graphics_column_start = 260,   // First visible OSD column (after Hsync)
    .graphics_line_start   = 20,    // First visible OSD line
    .dma_buffer_length     = NTSC_BYTES + NTSC_BYTES % 4, // DMA buffer length in bytes (has to be multiple of 4)
};

static const struct video_type_cfg video_type_cfg_pal = {
    .graphics_hight_real   = 266,   // Real visible lines
    .graphics_column_start = 420,   // First visible OSD column (after Hsync)
    .graphics_line_start   = 28,    // First visible OSD line
    .dma_buffer_length     = PAL_BYTES + PAL_BYTES % 4, // DMA buffer length in bytes (has to be multiple of 4)
};


uint8_t buffer0[BUFFER_HEIGHT * BUFFER_WIDTH] __attribute__ ((section(".video_ram"), aligned(4)));
uint8_t buffer1[BUFFER_HEIGHT * BUFFER_WIDTH] __attribute__ ((section(".video_ram"), aligned(4)));

// Pointers to each of these buffers.
uint8_t *draw_buffer;
uint8_t *disp_buffer;


const struct video_type_boundary *video_type_boundary_act = &video_type_boundary_pal;

// Private variables
static bool video_initialized = false;
static uint8_t spurious_vsync_cnt = 0;
static int16_t active_line = 0;
static uint32_t buffer_offset;
static int8_t y_offset = 0;
static uint16_t num_video_lines = 0;
static int8_t video_type_tmp = VIDEO_TYPE_PAL;
static int8_t video_type_act = VIDEO_TYPE_NONE;
static const struct video_type_cfg *video_type_cfg_act = &video_type_cfg_pal;

uint8_t black_pal = 30;
uint8_t white_pal = 110;
uint8_t black_ntsc = 10;
uint8_t white_ntsc = 110;

#if defined(USE_BRAINFPV_AUTO_SYNC_THRESHOLD)
bool useAutoSyncThreshold = false;
#endif

// Private functions
static void swap_buffers();

// Re-enable the video if it has been disabled
void video_qspi_enable(void)
{
    // re-enable interrupts
    spurious_vsync_cnt = 0;
    EXTIEnable(vsync_io, true);
    EXTIEnable(hsync_io, true);
}

/**
 * @brief Vsync interrupt service routine
 */
FAST_CODE void Vsync_ISR(extiCallbackRec_t *cb)
{
    (void)cb;
    static uint32_t t_last = 0;
    static uint16_t Vsync_update = 0;

    uint32_t t_now;

    CH_IRQ_PROLOGUE();

    t_now = microsISR();

    if (t_now - t_last < MIN_DELTA_VSYNC) {
        spurious_vsync_cnt += 1;
        if (spurious_vsync_cnt >= MAX_SPURIOUS_VSYNCS) {
            // spurious detections: disable interrupts
            EXTIEnable(vsync_io, false);
            EXTIEnable(hsync_io, false);
        }
        return;
    }
    else {
        spurious_vsync_cnt = 0;
    }
    t_last = t_now;


    // discard spurious vsync pulses (due to improper grounding), so we don't overload the CPU
    if (active_line > 0 && active_line < video_type_cfg_ntsc.graphics_hight_real - 10) {
        return false;
    }

    // Update the number of video lines
    num_video_lines = active_line + video_type_cfg_act->graphics_line_start + y_offset;

    // check video type
    if (num_video_lines > VIDEO_TYPE_PAL_ROWS) {
        video_type_tmp = VIDEO_TYPE_PAL;
    }

    // if video type has changed set new active values
    if (video_type_act != video_type_tmp) {
        video_type_act = video_type_tmp;
        if (video_type_act == VIDEO_TYPE_NTSC) {
            video_type_boundary_act = &video_type_boundary_ntsc;
            video_type_cfg_act = &video_type_cfg_ntsc;
            //dev_cfg->set_bw_levels(black_ntsc, white_ntsc);
        } else {
            video_type_boundary_act = &video_type_boundary_pal;
            video_type_cfg_act = &video_type_cfg_pal;
            //dev_cfg->set_bw_levels(black_pal, white_pal);
        }
    }

    video_type_tmp = VIDEO_TYPE_NTSC;

    // Every VSYNC_REDRAW_CNT field: swap buffers and trigger redraw
    if (++Vsync_update >= VSYNC_REDRAW_CNT) {
        Vsync_update = 0;
        swap_buffers();
        chSysLockFromISR();
        chBSemSignalI(&onScreenDisplaySemaphore);
        chSysUnlockFromISR();
    }

    // Get ready for the first line. We will start outputting data at line zero.
    active_line = 0 - (video_type_cfg_act->graphics_line_start + y_offset);
    buffer_offset = 0;
}

FAST_CODE void Hsync_ISR(extiCallbackRec_t *cb)
{
    (void)cb;
#ifdef VIDEO_DEBUG_PIN
    IOHi(debugPin);
#endif
    active_line++;
#if defined(STM32F446xx)
    EXTI->PR = 0x04;
#endif


    if ((active_line >= 0) && (active_line < video_type_cfg_act->graphics_hight_real)) {
        // Check if QUADSPI is busy
        if (QUADSPI->SR & 0x20)
            return;

#if defined(STM32F446xx)
        // Disable DMA
        DMA2_Stream7->CR &= ~(uint32_t)DMA_SxCR_EN;

        // Clear the DMA interrupt flags
        DMA2->HIFCR  |= DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_FEIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_DMEIF7;

        // Load new line
        DMA2_Stream7->M0AR = (uint32_t)&disp_buffer[buffer_offset];

        // Set length
        DMA2_Stream7->NDTR = (uint16_t)video_type_cfg_act->dma_buffer_length;
        QUADSPI->DLR = (uint32_t)video_type_cfg_act->dma_buffer_length - 1;

        // Enable DMA
        uint32_t cr = DMA2_Stream7->CR;
        DMA2_Stream7->CR = cr | (uint32_t)DMA_SxCR_EN;
#endif /* defined(STM32F446xx) */

#if defined(STM32H750xx)
        // Disable
        __HAL_MDMA_DISABLE(&hmdma);

        // Clear interrupt flags
        __HAL_MDMA_CLEAR_FLAG(&hmdma, MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_FLAG_BFTC | MDMA_FLAG_BT | MDMA_FLAG_BRT);

        // Load new line
        hmdma.Instance->CSAR = (uint32_t)&disp_buffer[buffer_offset];

        // Set length
        hmdma.Instance->CBNDTR = (uint32_t)video_type_cfg_act->dma_buffer_length;
        QUADSPI->DLR = (uint32_t)video_type_cfg_act->dma_buffer_length - 1;

#if !defined(DEBUG)
        // Write-back cache
        uint32_t start_addr = (uint32_t)&disp_buffer[buffer_offset];
        uint32_t end_addr = start_addr + (uint32_t)video_type_cfg_act->dma_buffer_length;
        start_addr &= ~0x1F; // 32-byte alignment
        SCB_CleanDCache_by_Addr((uint32_t *)start_addr, end_addr - start_addr + 1);
#endif

        // Enable DMA
        __HAL_MDMA_ENABLE(&hmdma);

        // Trigger transfer
        //hmdma.Instance->CCR |=  MDMA_CCR_SWRQ;
#endif /* defined(STM32H750xx) */

        buffer_offset += BUFFER_WIDTH;
    }
#ifdef VIDEO_DEBUG_PIN
    IOLo(debugPin);
#endif
}

/**
 * swap_buffers: Swaps the two buffers. Contents in the display
 * buffer is seen on the output and the display buffer becomes
 * the new draw buffer.
 */
FAST_CODE static void swap_buffers()
{
    // While we could use XOR swap this is more reliable and
    // dependable and it's only called a few times per second.
    // Many compilers should optimize these to EXCH instructions.
    uint8_t *tmp;

    SWAP_BUFFS(tmp, disp_buffer, draw_buffer);
}

/**
 * Init
 */
void Video_Init()
{
    chBSemObjectInit(&onScreenDisplaySemaphore, FALSE);

    /* Configure and clear buffers */
    draw_buffer = buffer0;
    disp_buffer = buffer1;

    /* Map pins to QUADSPI */
    IOInit(IOGetByTag(IO_TAG(VIDEO_QSPI_CLOCK_PIN)),  OWNER_OSD, 0);
    IOInit(IOGetByTag(IO_TAG(VIDEO_QSPI_IO0_PIN)), OWNER_OSD, 0);
    IOInit(IOGetByTag(IO_TAG(VIDEO_QSPI_IO1_PIN)), OWNER_OSD, 0);

    IOConfigGPIOAF(IOGetByTag(IO_TAG(VIDEO_QSPI_CLOCK_PIN)), IOCFG_AF_PP, GPIO_AF9_QUADSPI);
    IOConfigGPIOAF(IOGetByTag(IO_TAG(VIDEO_QSPI_IO0_PIN)), IOCFG_AF_PP, GPIO_AF9_QUADSPI);
    IOConfigGPIOAF(IOGetByTag(IO_TAG(VIDEO_QSPI_IO1_PIN)), IOCFG_AF_PP, GPIO_AF9_QUADSPI);

#if defined(VIDEO_QSPI_USE_4_LINES)
    IOInit(IOGetByTag(IO_TAG(VIDEO_QSPI_IO2_PIN)), OWNER_OSD, 0);
    IOInit(IOGetByTag(IO_TAG(VIDEO_QSPI_IO3_PIN)), OWNER_OSD, 0);
    IOConfigGPIOAF(IOGetByTag(IO_TAG(VIDEO_QSPI_IO2_PIN)), IOCFG_AF_PP, GPIO_AF9_QUADSPI);
    IOConfigGPIOAF(IOGetByTag(IO_TAG(VIDEO_QSPI_IO3_PIN)), IOCFG_AF_PP, GPIO_AF9_QUADSPI);
#endif


#if defined(STM32F446xx)
    /* Enable QUADSPI clock */
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_QSPI, ENABLE);

    /* Configure QUADSPI */
    QSPI_InitTypeDef qspi_init = {
        .QSPI_SShift     = QSPI_SShift_NoShift,
        .QSPI_Prescaler  = 12,  // 180MHz / 12 = 15MHz
        .QSPI_CKMode     = QSPI_CKMode_Mode0,
        .QSPI_CSHTime    = QSPI_CSHTime_1Cycle,
        .QSPI_FSize      = 0x1F,
        .QSPI_FSelect    = QSPI_FSelect_1,
        .QSPI_DFlash     = QSPI_DFlash_Disable};

    QSPI_Init(&qspi_init);

    QSPI_ComConfig_InitTypeDef qspi_com_config;
    QSPI_ComConfig_StructInit(&qspi_com_config);

    qspi_com_config.QSPI_ComConfig_FMode       = QSPI_ComConfig_FMode_Indirect_Write;
    qspi_com_config.QSPI_ComConfig_DDRMode     = QSPI_ComConfig_DDRMode_Disable;
    qspi_com_config.QSPI_ComConfig_DHHC        = QSPI_ComConfig_DHHC_Disable;
    qspi_com_config.QSPI_ComConfig_SIOOMode    = QSPI_ComConfig_SIOOMode_Disable;
#if defined(VIDEO_QSPI_USE_4_LINES)
    qspi_com_config.QSPI_ComConfig_DMode       = QSPI_ComConfig_DMode_4Line;
#else
    qspi_com_config.QSPI_ComConfig_DMode       = QSPI_ComConfig_DMode_2Line;
#endif
    qspi_com_config.QSPI_ComConfig_DummyCycles = 0;
    qspi_com_config.QSPI_ComConfig_ABMode      = QSPI_ComConfig_ABMode_NoAlternateByte;
    qspi_com_config.QSPI_ComConfig_ADMode      = QSPI_ComConfig_ADMode_NoAddress;
    qspi_com_config.QSPI_ComConfig_IMode       = QSPI_ComConfig_IMode_NoInstruction;
    QSPI_ComConfig_Init(&qspi_com_config);

    QSPI_SetFIFOThreshold(3);

    /* Configure DMA */
    DMA_InitTypeDef dma_cfg = {
        .DMA_Channel            = DMA_Channel_3,
        .DMA_PeripheralBaseAddr = (uint32_t)&(QUADSPI->DR),
        .DMA_DIR                = DMA_DIR_MemoryToPeripheral,
        .DMA_BufferSize         = 400,
        .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
        .DMA_MemoryInc          = DMA_MemoryInc_Enable,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
        .DMA_MemoryDataSize     = DMA_MemoryDataSize_Word,
        .DMA_Mode               = DMA_Mode_Normal,
        .DMA_Priority           = DMA_Priority_VeryHigh,
        .DMA_FIFOMode           = DMA_FIFOMode_Enable,
        .DMA_FIFOThreshold      = DMA_FIFOThreshold_Full,
        .DMA_MemoryBurst        = DMA_MemoryBurst_INC4,
        .DMA_PeripheralBurst    = DMA_PeripheralBurst_Single};
    DMA_Init(DMA2_Stream7, &dma_cfg);

    /* Enable TC interrupt */
    QSPI_ITConfig(QSPI_IT_TC, ENABLE);
    QSPI_ITConfig(QSPI_IT_FT, ENABLE);

    /* Enable DMA */
    QSPI_DMACmd(ENABLE);

    // Enable the QUADSPI
    QSPI_Cmd(ENABLE);
#endif /* defined(STM32F446xx) */

#if defined(STM32H750xx)
    __HAL_RCC_QSPI_CLK_ENABLE();
    __HAL_RCC_QSPI_FORCE_RESET();
    __HAL_RCC_QSPI_RELEASE_RESET();

    hqspi.Instance = QUADSPI;

    hqspi.Init.ClockPrescaler = 15; // 240MHz / 16 = 15MHz
    hqspi.Init.FifoThreshold = 16;
    hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
    hqspi.Init.FlashSize = 0x1F;
    hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
    hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
    hqspi.Init.FlashID = QSPI_FLASH_ID_1;
    hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;

    if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
        Error_Handler();
    }

    // MDMA
    __HAL_RCC_MDMA_CLK_ENABLE();

    hmdma.Instance = MDMA_Channel0;
    hmdma.Init.Request = MDMA_REQUEST_QUADSPI_FIFO_TH;
    hmdma.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    hmdma.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
    hmdma.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma.Init.SourceInc = MDMA_SRC_INC_WORD;
    hmdma.Init.DestinationInc = MDMA_DEST_INC_DISABLE;
    hmdma.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
    hmdma.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
    hmdma.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma.Init.BufferTransferLength = 16;
    hmdma.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma.Init.DestBurst = MDMA_SOURCE_BURST_SINGLE;
    hmdma.Init.SourceBlockAddressOffset = 0;
    hmdma.Init.DestBlockAddressOffset = 0;

    if (HAL_MDMA_Init(&hmdma) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&hqspi, hmdma, hmdma);

    QSPI_CommandTypeDef cmd;
    cmd.InstructionMode   = QSPI_INSTRUCTION_NONE;
    cmd.AddressMode       = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
#if defined(VIDEO_QSPI_USE_4_LINES)
    cmd.DataMode          = QSPI_DATA_4_LINES;
#else
    cmd.DataMode          = QSPI_DATA_2_LINES;
#endif
    cmd.DummyCycles       = 0;
    cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
    cmd.NbData            = 132;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK) {
        Error_Handler();
    }

    // Set DMA dest address
    hmdma.Instance->CDAR = (uint32_t)&(QUADSPI->DR);

    /* Configure QSPI: CCR register with functional mode as indirect write */
    //MODIFY_REG(hqspi.Instance->CCR, QUADSPI_CCR_FMODE, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

    // Enable interrupts
    //__HAL_QSPI_ENABLE_IT(&hqspi, QSPI_IT_TE | QSPI_IT_TC);
    // Test
    //if (HAL_QSPI_Transmit_DMA(&hqspi, &buffer0[0]) != HAL_OK) {
    //    Error_Handler();
    //}
#endif /* defined(STM32H750xx) */

#ifdef VIDEO_DEBUG_PIN
    debugPin = IOGetByTag(IO_TAG(VIDEO_DEBUG_PIN));
    IOConfigGPIO(debugPin, IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN));
#endif

    // VSYNC interrupt
    vsync_io = IOGetByTag(IO_TAG(VIDEO_VSYNC));
    IOInit(vsync_io, OWNER_OSD, 0);
    EXTIHandlerInit(&vsyncIntCallbackRec, Vsync_ISR);
    EXTIConfig(vsync_io, &vsyncIntCallbackRec, NVIC_BUILD_PRIORITY(3, 1), IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_FALLING);

    // HSYNC interrupt
    hsync_io = IOGetByTag(IO_TAG(VIDEO_HSYNC));
    IOInit(hsync_io, OWNER_OSD, 0);
    EXTIHandlerInit(&hsyncIntCallbackRec, Hsync_ISR);
    EXTIConfig(hsync_io, &hsyncIntCallbackRec, NVIC_BUILD_PRIORITY(1, 1), IOCFG_IN_FLOATING, BETAFLIGHT_EXTI_TRIGGER_FALLING);

    // Enable interrupts
    EXTIEnable(vsync_io, true);
    EXTIEnable(hsync_io, true);

#if defined(USE_BRAINFPV_AUTO_SYNC_THRESHOLD)
   if (bfOsdConfig()->sync_threshold_mode == SYNC_THRESHOLD_AUTO) {
       if (autoSyncThresholdInit() == 0) {
           useAutoSyncThreshold = true;
       }
   }
#endif

    video_initialized = true;
}

bool VideoIsInitialized(void)
{
    return video_initialized;
}

/**
 *
 */
uint16_t Video_GetLines(void)
{
    return num_video_lines;
}

/**
 *
 */
uint16_t Video_GetType(void)
{
    return video_type_act;
}

#endif /* INCLUDE_VIDEO_QUADSPI */
