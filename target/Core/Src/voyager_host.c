#include "voyager.h"
#include "stm32f7xx_hal.h"
#include "udp.h"
#include <string.h>

#define APP_FLASH_START_ADDRESS 0x08040000
#define APP_FLASH_START_SECTOR FLASH_SECTOR_5
#define APP_FLASH_END_ADDRESS 0x081FFFFF
#define APP_FLASH_END_SECTOR FLASH_SECTOR_11

#define APP_FLASH_RESET_VECTOR_OFFSET 0x4 // Value found in linker script that points to the reset vector

#define BOOTLOADER_FLASH_START_ADDRESS 0x08000000
#define BOOTLOADER_FLASH_END_ADDRESS 0x0801FFFF

#define APP_INFO_NVM_ADDRESS 0x08020000
#define APP_INFO_NVM_SECTOR FLASH_SECTOR_4

typedef struct {
    uint32_t app_size;
    uint32_t app_crc;
} application_info_flash_t;

static application_info_flash_t read_application_info(void);
static HAL_StatusTypeDef write_application_info(application_info_flash_t const * const info);

extern struct udp_pcb* upcb_udp_server;
extern ip_addr_t server_ip_addr;
extern u16_t server_port;
extern bool server_connected;

voyager_error_E voyager_bootloader_send_to_host(void const *const data, size_t len)
{
    voyager_error_E error = VOYAGER_ERROR_NONE;
    // Send the data to the host using UDP
    // Create a new pbuf
    do {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p == NULL)
        {
            error = VOYAGER_ERROR_GENERIC_ERROR;
            break;
        }

        // Copy the data into the pbuf
        memcpy(p->payload, data, len);

        // Connect to the host
        udp_connect(upcb_udp_server, &server_ip_addr, server_port);

        // Send the pbuf to the host
        udp_send(upcb_udp_server, p);

        // Disconnect from the host
        udp_disconnect(upcb_udp_server);

        // Free the pbuf
        pbuf_free(p);
    } while (false);

    return error;
}

voyager_error_E voyager_bootloader_nvm_write(const voyager_nvm_key_E key, voyager_bootloader_nvm_data_t const *const data)
{
    voyager_error_E error = VOYAGER_ERROR_NONE;
    switch (key)
    {
        case VOYAGER_NVM_KEY_APP_CRC: {
                // read the application info from the flash
                application_info_flash_t application_info = read_application_info();
                // update the application info
                application_info.app_crc = data->app_crc;
                // write the application info to the flash
                HAL_StatusTypeDef hal_status = write_application_info(&application_info);
                if (hal_status != HAL_OK)
                {
                    error = VOYAGER_ERROR_GENERIC_ERROR;
                }
            }
            break;
        case VOYAGER_NVM_KEY_APP_SIZE: {
                // read the application info from the flash
                application_info_flash_t application_info = read_application_info();
                // update the application info
                application_info.app_size = data->app_size;
                // write the application info to the flash
                HAL_StatusTypeDef hal_status = write_application_info(&application_info);
                if (hal_status != HAL_OK)
                {
                    error = VOYAGER_ERROR_GENERIC_ERROR;
                }
            }
            break;
        case VOYAGER_NVM_KEY_APP_START_ADDRESS:
        case VOYAGER_NVM_KEY_APP_END_ADDRESS:
        case VOYAGER_NVM_KEY_APP_RESET_VECTOR_ADDRESS:
        case VOYAGER_NVM_KEY_VERIFY_FLASH_BEFORE_JUMPING:
        default:
            // The following keys above are read only in this context
            error = VOYAGER_ERROR_INVALID_ARGUMENT;
            break;
    }

    return error;
}

voyager_error_E voyager_bootloader_nvm_read(const voyager_nvm_key_E key, voyager_bootloader_nvm_data_t *const data)
{
    voyager_error_E error = VOYAGER_ERROR_NONE;
    switch (key)
    {
        case VOYAGER_NVM_KEY_APP_CRC:
            data->app_crc = read_application_info().app_crc;
            break;
        case VOYAGER_NVM_KEY_APP_SIZE:
            data->app_size = read_application_info().app_size;
            break;
        case VOYAGER_NVM_KEY_APP_START_ADDRESS:
            data->app_start_address = APP_FLASH_START_ADDRESS;
            break;
        case VOYAGER_NVM_KEY_APP_END_ADDRESS:
            data->app_end_address = APP_FLASH_END_ADDRESS;
            break;
        case VOYAGER_NVM_KEY_APP_RESET_VECTOR_ADDRESS:
            data->app_reset_vector_address = APP_FLASH_START_ADDRESS + APP_FLASH_RESET_VECTOR_OFFSET;
            break;
        case VOYAGER_NVM_KEY_VERIFY_FLASH_BEFORE_JUMPING:
            data->verify_flash_before_jumping = true;
            break;
        default:
            error = VOYAGER_ERROR_INVALID_ARGUMENT;
            break;
    }

    return error;
}

// This static function is used to read the application info from the flash
// as I am using a spare flash sector to store the application info
static application_info_flash_t read_application_info(void)
{
    application_info_flash_t application_info = {0};

    // Read the data from the flash using memcpy
    memcpy(&application_info, (void *)APP_INFO_NVM_ADDRESS, sizeof(application_info_flash_t));
    return application_info;
}

// This static function is used to write the application info to the emulated EEPROM sector
static HAL_StatusTypeDef write_application_info(application_info_flash_t const * const info)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    do {
        // Ensure the flash is not busy
        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

        // Setup the flash erase parameters
        FLASH_EraseInitTypeDef erase_type = {0};
        erase_type.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase_type.Sector = APP_INFO_NVM_SECTOR;
        erase_type.NbSectors = 1;
        erase_type.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        // Unlock the flash
        hal_status = HAL_FLASH_Unlock();
        if (hal_status != HAL_OK)
        {
            break;
        }

        // Erase the flash
        uint32_t sector_error = 0;
        hal_status = HAL_FLASHEx_Erase(&erase_type, &sector_error);
        if (hal_status != HAL_OK)
        {
            break;
        }

        // Write the data to the flash
        for (size_t i = 0; i < sizeof(application_info_flash_t); i += 4)
        {
            // We can only write 32 bit words to the flash at a time
            hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_INFO_NVM_ADDRESS + i, ((uint32_t *)info)[i / 4]);
            if (hal_status != HAL_OK)
            {
                break;
            }
        }

        // Lock the flash
        hal_status = HAL_FLASH_Lock();
        if (hal_status != HAL_OK)
        {
            break;
        }

    } while (false);

    return hal_status;
}

voyager_error_E voyager_bootloader_hal_erase_flash(const voyager_bootloader_addr_size_t start_address,
                                                   const voyager_bootloader_addr_size_t end_address)
{
    voyager_error_E error = VOYAGER_ERROR_NONE;
    // Erase the flash from start_address to end_address
    do {
        // Ensure the flash is not busy
        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

        // Setup the flash erase parameters
        FLASH_EraseInitTypeDef erase_type = {0};
        erase_type.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase_type.Sector = APP_FLASH_START_SECTOR;
        erase_type.NbSectors = APP_FLASH_END_SECTOR - APP_FLASH_START_SECTOR + 1;
        erase_type.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        // Unlock the flash
        HAL_StatusTypeDef hal_status = HAL_FLASH_Unlock();
        if (hal_status != HAL_OK)
        {
            error = VOYAGER_ERROR_GENERIC_ERROR;
            break;
        }

        // Erase the flash
        uint32_t sector_error = 0;
        hal_status = HAL_FLASHEx_Erase(&erase_type, &sector_error);
        if (hal_status != HAL_OK)
        {
            error = VOYAGER_ERROR_GENERIC_ERROR;
            break;
        }

        // Lock the flash
        hal_status = HAL_FLASH_Lock();
        if (hal_status != HAL_OK)
        {
            error = VOYAGER_ERROR_GENERIC_ERROR;
            break;
        }

    } while (false);

    return error;
}

voyager_error_E voyager_bootloader_hal_write_flash(const voyager_bootloader_addr_size_t address, void const *const data,
                                                   size_t const length)
{
    voyager_error_E error = VOYAGER_ERROR_NONE;
    do {
        // Ensure the flash is not busy
        FLASH_WaitForLastOperation(HAL_MAX_DELAY);

        // Unlock the flash
        HAL_StatusTypeDef hal_status = HAL_FLASH_Unlock();
        if (hal_status != HAL_OK)
        {
            error = VOYAGER_ERROR_GENERIC_ERROR;
            break;
        }

        // Program the flash using BYTE
        for (size_t i = 0; i < length; i += 1)
        {
            // We can only write 32 bit words to the flash at a time
            hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, ((uint8_t *)data)[i]);
            if (hal_status != HAL_OK)
            {
                error = VOYAGER_ERROR_GENERIC_ERROR;
                break;
            }
        }

        // Lock the flash
        hal_status = HAL_FLASH_Lock();
        if (hal_status != HAL_OK)
        {
            error = VOYAGER_ERROR_GENERIC_ERROR;
            break;
        }

    } while (false);

    return error;
}
