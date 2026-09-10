#include "wheel_calibration_storage.h"

#include <stddef.h>

#define WHEEL_CALIBRATION_STORAGE_MAGIC 0x43414C31UL
#define WHEEL_CALIBRATION_STORAGE_FORMAT_VERSION 1U
#define WHEEL_CALIBRATION_STORAGE_PAYLOAD_SIZE 28U
#define WHEEL_CALIBRATION_STORAGE_COMMIT_MARKER 0xC0A17ED1UL
#define WHEEL_CALIBRATION_STORAGE_RECORD_WORDS 12U

static uint32_t Wheel_Calibration_Storage_CrcUpdate(uint32_t crc, uint8_t value)
{
    uint32_t bit;

    crc ^= value;
    for (bit = 0U; bit < 8U; ++bit) {
        if ((crc & 1UL) != 0UL) {
            crc = (crc >> 1) ^ 0xEDB88320UL;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

static uint32_t Wheel_Calibration_Storage_CrcWords(const uint32_t *words)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t word_index;
    uint32_t byte_index;

    for (word_index = 1U; word_index <= 9U; ++word_index) {
        for (byte_index = 0U; byte_index < 4U; ++byte_index) {
            crc = Wheel_Calibration_Storage_CrcUpdate(
                crc,
                (uint8_t)((words[word_index] >> (byte_index * 8U)) & 0xFFUL));
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

static void Wheel_Calibration_Storage_Encode(const CalibrationData_t *data,
                                             uint32_t generation,
                                             uint32_t *words)
{
    words[0] = WHEEL_CALIBRATION_STORAGE_MAGIC;
    words[1] = (uint32_t)WHEEL_CALIBRATION_STORAGE_FORMAT_VERSION |
               ((uint32_t)WHEEL_CALIBRATION_STORAGE_PAYLOAD_SIZE << 16);
    words[2] = generation;
    words[3] = (uint32_t)data->valid |
               ((uint32_t)(uint8_t)data->left_encoder_polarity << 8) |
               ((uint32_t)(uint8_t)data->right_encoder_polarity << 16);
    words[4] = (uint32_t)data->left_min_start_pwm_permille |
               ((uint32_t)data->right_min_start_pwm_permille << 16);
    words[5] = (uint32_t)data->version |
               ((uint32_t)data->reserved << 16);
    {
        const WheelCalibrationParameters_t *parameters = Wheel_Calibration_GetParameters();
        words[6] = parameters->encoder_ppr;
        words[7] = parameters->quadrature_factor;
        words[8] = parameters->gear_ratio_x1000;
        words[9] = parameters->wheel_radius_mm_x1000;
    }
    words[10] = Wheel_Calibration_Storage_CrcWords(words);
    words[11] = WHEEL_CALIBRATION_STORAGE_COMMIT_MARKER;
}

static uint8_t Wheel_Calibration_Storage_Decode(const uint32_t *words,
                                                CalibrationData_t *data,
                                                uint32_t *generation)
{
    if ((words == 0) || (data == 0) || (generation == 0)) {
        return 0U;
    }
    if ((words[0] != WHEEL_CALIBRATION_STORAGE_MAGIC) ||
        ((words[1] & 0xFFFFUL) != WHEEL_CALIBRATION_STORAGE_FORMAT_VERSION) ||
        ((words[1] >> 16) != WHEEL_CALIBRATION_STORAGE_PAYLOAD_SIZE) ||
        (words[10] != Wheel_Calibration_Storage_CrcWords(words)) ||
        (words[11] != WHEEL_CALIBRATION_STORAGE_COMMIT_MARKER)) {
        return 0U;
    }

    data->valid = (uint8_t)(words[3] & 0xFFUL);
    data->left_encoder_polarity = (int8_t)((words[3] >> 8) & 0xFFUL);
    data->right_encoder_polarity = (int8_t)((words[3] >> 16) & 0xFFUL);
    data->left_min_start_pwm_permille = (uint16_t)(words[4] & 0xFFFFUL);
    data->right_min_start_pwm_permille = (uint16_t)((words[4] >> 16) & 0xFFFFUL);
    data->version = (uint16_t)(words[5] & 0xFFFFUL);
    data->reserved = (uint16_t)((words[5] >> 16) & 0xFFFFUL);
    Wheel_Calibration_SetParameters(&(WheelCalibrationParameters_t){
        words[6], words[7], words[8], words[9]});
    *generation = words[2];
    return Wheel_Calibration_IsDataValid(data);
}

static uint8_t Wheel_Calibration_Storage_IsNewer(uint32_t candidate,
                                                 uint32_t reference)
{
    int32_t distance = (int32_t)(candidate - reference);
    return (distance > 0) && (distance != INT32_MIN);
}

#ifdef WHEEL_CALIBRATION_STORAGE_HOST_TEST

static uint32_t g_host_slots[WHEEL_CALIBRATION_STORAGE_SLOT_COUNT]
                             [WHEEL_CALIBRATION_STORAGE_RECORD_WORDS];

static const uint32_t *Wheel_Calibration_Storage_Slot(uint8_t slot)
{
    return g_host_slots[slot];
}

static uint32_t *Wheel_Calibration_Storage_WritableSlot(uint8_t slot)
{
    return g_host_slots[slot];
}

void Wheel_Calibration_Storage_ResetForTest(void)
{
    uint32_t slot;
    uint32_t word;

    for (slot = 0U; slot < WHEEL_CALIBRATION_STORAGE_SLOT_COUNT; ++slot) {
        for (word = 0U; word < WHEEL_CALIBRATION_STORAGE_RECORD_WORDS; ++word) {
            g_host_slots[slot][word] = 0xFFFFFFFFUL;
        }
    }
}

void Wheel_Calibration_Storage_CorruptSlotForTest(uint8_t slot)
{
    if (slot < WHEEL_CALIBRATION_STORAGE_SLOT_COUNT) {
        g_host_slots[slot][10] ^= 1UL;
    }
}

void Wheel_Calibration_Storage_DropCommitForTest(uint8_t slot)
{
    if (slot < WHEEL_CALIBRATION_STORAGE_SLOT_COUNT) {
        g_host_slots[slot][11] = 0xFFFFFFFFUL;
    }
}

#else

#include "main.h"

static const uint32_t *Wheel_Calibration_Storage_Slot(uint8_t slot)
{
    const uint32_t addresses[WHEEL_CALIBRATION_STORAGE_SLOT_COUNT] = {
        WHEEL_CALIBRATION_STORAGE_SLOT0_ADDRESS,
        WHEEL_CALIBRATION_STORAGE_SLOT1_ADDRESS
    };
    return (const uint32_t *)addresses[slot];
}

static uint8_t Wheel_Calibration_Storage_ProgramSlot(uint8_t slot,
                                                     const uint32_t *words)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    uint32_t address;
    uint32_t word;
    HAL_StatusTypeDef status;
    const uint32_t addresses[WHEEL_CALIBRATION_STORAGE_SLOT_COUNT] = {
        WHEEL_CALIBRATION_STORAGE_SLOT0_ADDRESS,
        WHEEL_CALIBRATION_STORAGE_SLOT1_ADDRESS
    };
    const uint32_t sectors[WHEEL_CALIBRATION_STORAGE_SLOT_COUNT] = {
        FLASH_SECTOR_6,
        FLASH_SECTOR_7
    };

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = sectors[slot];
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (status == HAL_OK) {
        address = addresses[slot];
        for (word = 0U; word < 11U; ++word) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       address + (word * sizeof(uint32_t)),
                                       words[word]);
            if (status != HAL_OK) {
                break;
            }
        }
        if (status == HAL_OK) {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                       address + (11U * sizeof(uint32_t)),
                                       words[11]);
        }
    }
    (void)HAL_FLASH_Lock();
    /* HAL_FLASHEx_Erase() reports a successful erase with 0xFFFFFFFF in
       SectorError; zero is a sector number and is therefore the first
       possible failure value, not the success value. */
    return (status == HAL_OK) && (sector_error == 0xFFFFFFFFUL);
}

#endif

static uint8_t Wheel_Calibration_Storage_FindLatest(uint8_t *slot_out,
                                                    uint32_t *generation_out,
                                                    CalibrationData_t *data_out)
{
    uint8_t found = 0U;
    uint8_t slot;
    CalibrationData_t candidate;
    uint32_t generation;

    for (slot = 0U; slot < WHEEL_CALIBRATION_STORAGE_SLOT_COUNT; ++slot) {
        if (Wheel_Calibration_Storage_Decode(
                Wheel_Calibration_Storage_Slot(slot), &candidate, &generation) == 0U) {
            continue;
        }
        if ((found == 0U) || Wheel_Calibration_Storage_IsNewer(generation, *generation_out)) {
            found = 1U;
            *slot_out = slot;
            *generation_out = generation;
            *data_out = candidate;
        }
    }
    return found;
}

uint8_t Wheel_Calibration_Storage_Load(CalibrationData_t *out)
{
    uint8_t latest_slot = 0U;
    uint32_t latest_generation = 0U;
    CalibrationData_t latest_data;

    if (out == 0) {
        return 0U;
    }
    if (Wheel_Calibration_Storage_FindLatest(&latest_slot,
                                             &latest_generation,
                                             &latest_data) == 0U) {
        return 0U;
    }
    *out = latest_data;
    return 1U;
}

uint8_t Wheel_Calibration_Storage_Save(const CalibrationData_t *data)
{
    uint8_t latest_slot = 0U;
    uint8_t target_slot = 0U;
    uint32_t latest_generation = 0U;
    uint32_t words[WHEEL_CALIBRATION_STORAGE_RECORD_WORDS];
    CalibrationData_t latest_data;

    if (Wheel_Calibration_IsDataValid(data) == 0U) {
        return 0U;
    }
    if (Wheel_Calibration_Storage_FindLatest(&latest_slot,
                                             &latest_generation,
                                             &latest_data) != 0U) {
        target_slot = (uint8_t)(latest_slot ^ 1U);
    }
    Wheel_Calibration_Storage_Encode(data,
                                     (latest_generation == 0U) ? 1U
                                                               : latest_generation + 1U,
                                     words);

#ifdef WHEEL_CALIBRATION_STORAGE_HOST_TEST
    {
        uint32_t *target = Wheel_Calibration_Storage_WritableSlot(target_slot);
        uint32_t word;
        for (word = 0U; word < WHEEL_CALIBRATION_STORAGE_RECORD_WORDS; ++word) {
            target[word] = words[word];
        }
    }
    return 1U;
#else
    return Wheel_Calibration_Storage_ProgramSlot(target_slot, words);
#endif
}
