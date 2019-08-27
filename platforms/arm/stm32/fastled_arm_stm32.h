#ifndef __INC_FASTLED_ARM_SAM_H
#define __INC_FASTLED_ARM_SAM_H

// Include the sam headers
#include "fastpin_arm_stm32.h"
#ifdef FASTLED_STM32_DMA
    #include "clockless_spi_stm32.h"
#else
    #include "clockless_arm_stm32.h"
#endif

#endif
