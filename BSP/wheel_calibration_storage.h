#ifndef __WHEEL_CALIBRATION_STORAGE_H
#define __WHEEL_CALIBRATION_STORAGE_H

#include "wheel_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STM32F407VET6 has 512 KiB Flash ending at 0x08080000.  The application is
   linked to 0x08000000..0x08040000; sectors 6 and 7 are reserved exclusively
   for calibration records. */
#define WHEEL_CALIBRATION_STORAGE_SLOT0_ADDRESS 0x08040000UL
#define WHEEL_CALIBRATION_STORAGE_SLOT1_ADDRESS 0x08060000UL
#define WHEEL_CALIBRATION_STORAGE_SLOT_COUNT 2U

uint8_t Wheel_Calibration_Storage_Load(CalibrationData_t *out);
uint8_t Wheel_Calibration_Storage_Save(const CalibrationData_t *data);

#ifdef WHEEL_CALIBRATION_STORAGE_HOST_TEST
void Wheel_Calibration_Storage_ResetForTest(void);
void Wheel_Calibration_Storage_CorruptSlotForTest(uint8_t slot);
void Wheel_Calibration_Storage_DropCommitForTest(uint8_t slot);
#endif

#ifdef __cplusplus
}
#endif

#endif
