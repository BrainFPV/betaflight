/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/resource.h"
#include "drivers/dma.h"

#include "drivers/adc_impl.h"
#include "brainfpv/auto_sync_threshold.h"

#ifdef USE_BRAINFPV_AUTO_SYNC_THRESHOLD

#define SYNC_TH_BUFFER_LEN 1024

#define SYNC_TIP_TO_TH_DIFFERENCE_MV 200

#define SYNC_TIP_VOLTAGE_MIN_MV 50
#define SYNC_TIP_VOLTAGE_MAX_MV 400

#define SYNC_TIP_VOLTAGE_MIN_CNT ((255 * SYNC_TIP_VOLTAGE_MIN_MV) / ADC_VOLTAGE_REFERENCE_MV)
#define SYNC_TIP_VOLTAGE_MAX_CNT ((255 * SYNC_TIP_VOLTAGE_MAX_MV) / ADC_VOLTAGE_REFERENCE_MV)

#define SYNC_TIP_HIST_LEN (SYNC_TIP_VOLTAGE_MAX_CNT - SYNC_TIP_VOLTAGE_MIN_CNT + 2)

DMA_RAM uint8_t syncThresholdBuffer[SYNC_TH_BUFFER_LEN] __attribute__((aligned(32)));

static uint16_t histogramBuffer[SYNC_TIP_HIST_LEN];

static adcDevice_t thAdcDevice;
static IO_t adcPin;


int autoSyncThresholdInit(void)
{
    ADCDevice dev;

    memset(syncThresholdBuffer, 0, SYNC_TH_BUFFER_LEN);
    memset(histogramBuffer, 0, SYNC_TIP_HIST_LEN * sizeof(uint16_t));

    dev = adcDeviceByInstance(AUTO_SYNC_THRESHOLD_ADC_INSTANCE);
    memcpy(&thAdcDevice, &adcHardware[dev], sizeof(thAdcDevice));

    adcPin = IOGetByTag(IO_TAG(AUTO_SYNC_THRESHOLD_ADC_PIN));
    IOInit(adcPin, OWNER_OSD, 0);
    IOConfigGPIO(adcPin, IO_CONFIG(GPIO_MODE_ANALOG, 0, GPIO_NOPULL));

    // Initialize ADC
    RCC_ClockCmd(thAdcDevice.rccADC, ENABLE);

    ADC_HandleTypeDef *hadc = &thAdcDevice.ADCHandle;

    hadc->Instance = thAdcDevice.ADCx;

    hadc->Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV1; // 64 MHz HSI clock
    hadc->Init.Resolution               = ADC_RESOLUTION_8B;
    hadc->Init.ScanConvMode             = ENABLE;
    hadc->Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait         = DISABLE;
    hadc->Init.ContinuousConvMode       = ENABLE;
    hadc->Init.NbrOfConversion          = 1;
    hadc->Init.DiscontinuousConvMode    = DISABLE;
    hadc->Init.NbrOfDiscConversion      = 1;
    hadc->Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;

    hadc->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    hadc->Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.OversamplingMode         = DISABLE;

    if (HAL_ADC_Init(hadc) != HAL_OK) {
        return -1;
    }

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        return -2;
    }

    ADC_ChannelConfTypeDef sConfig;

    sConfig.Channel      = AUTO_SYNC_THRESHOLD_ADC_CHANNEL;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5; // 64 MHz / (64.5 + 7.5) = 888.8 kSMPS
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&thAdcDevice.ADCHandle, &sConfig) != HAL_OK) {
        return -3;
    }

    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByPeripheral(DMA_PERIPH_ADC, dev, 15);
    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaSpec->ref);

    if (!dmaSpec || !dmaAllocate(dmaIdentifier, OWNER_ADC, RESOURCE_INDEX(dev))) {
        return -4;
    }

    thAdcDevice.DmaHandle.Instance                 = dmaSpec->ref;
    thAdcDevice.DmaHandle.Init.Request             = dmaSpec->channel;

    thAdcDevice.DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    thAdcDevice.DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
    thAdcDevice.DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
    thAdcDevice.DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    thAdcDevice.DmaHandle.Init.MemDataAlignment    = DMA_PDATAALIGN_BYTE;
    thAdcDevice.DmaHandle.Init.Mode                = DMA_CIRCULAR;
    thAdcDevice.DmaHandle.Init.Priority            = DMA_PRIORITY_LOW;

    dmaEnable(dmaIdentifier);

    HAL_DMA_DeInit(&thAdcDevice.DmaHandle);
    HAL_DMA_Init(&thAdcDevice.DmaHandle);

    // Associate the DMA handle
    __HAL_LINKDMA(&thAdcDevice.ADCHandle, DMA_Handle, thAdcDevice.DmaHandle);

    if (HAL_ADC_Start_DMA(&thAdcDevice.ADCHandle, (uint32_t*)&syncThresholdBuffer[0], SYNC_TH_BUFFER_LEN) != HAL_OK) {
        return -5;
    }

    return 0;
}

uint16_t autoSyncThresholdGet(void)
{
    uint16_t syncThMv;

    // Build histogram
    //memset(histogramBuffer, 0, SYNC_TIP_HIST_LEN * sizeof(uint16_t));

    for (int i = 0; i < SYNC_TH_BUFFER_LEN; i++) {
        uint8_t val = syncThresholdBuffer[i];
        if ((val >= SYNC_TIP_VOLTAGE_MIN_CNT) && (val <= SYNC_TIP_VOLTAGE_MAX_CNT)) {
            histogramBuffer[val - SYNC_TIP_VOLTAGE_MIN_CNT] += 1;
        }
    }

    // Find the center of mass and scale
    uint32_t histCntMul = 0;
    uint32_t histCnt = 0;
    for (int i = 0; i < SYNC_TIP_HIST_LEN; i++) {
        histCnt += histogramBuffer[i];
        histCntMul += i * histogramBuffer[i];

        histogramBuffer[i] = histogramBuffer[i] >> 2;
    }

    float histCntr = SYNC_TIP_VOLTAGE_MIN_CNT + (float)histCntMul / histCnt;

    // Convert to mV
    syncThMv = (histCntr * ADC_VOLTAGE_REFERENCE_MV) / 255;

    // Add offset
    syncThMv += SYNC_TIP_TO_TH_DIFFERENCE_MV;

    return syncThMv;
}

#endif
