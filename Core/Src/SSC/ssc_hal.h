#pragma once

/**
 * plz read "Application Note ET9300.pdf"
 * This file has similar function as el9800hw.h, but designed to be used with different stack
 * @see el9800hw.h
 */

#include "Src/ecat_def.h"
#include "Src/esc.h"

#define IS_SSC_LOWER_5P10 ((SSC_VERSION_MAJOR == 5 && SSC_VERSION_MINOR <= 10) || SSC_VERSION_MAJOR < 5)

/**
 * Section III-ET1100 Hardware Description ch6.3.4
 */
#define ESC_RD 0x02       /**< \brief Indicates a read access to ESC or EEPROM*/
#define ESC_RD_WAIT 0x03  /**< \brief Read with following wait state bytes*/
#define ESC_WR 0x04       /**< \brief Indicates a write access to ESC or EEPROM.*/
#define ESC_ADDR_EXT 0x06 /**< \brief Address Extension (3 address/command bytes) */

#define WAIT_STATE_BYTE 0xFF /**< \brief Wait state byte for read access */

#if INTERRUPTS_SUPPORTED == 1

/**
 * @brief
 * Enable external interrupts
 * @note
 * ch5.1
 * If interrupts are used also two macros shall be defined “ENABLE_ESC_INT” and
 * “DISABLE_ESC_INT”. These shall enable/disable all four interrupt sources.
 */
void
ENABLE_ESC_INT();
/**
 * @brief
 * Disable external interrupts
 * @note
 *
 */
void
DISABLE_ESC_INT();
#endif

// ch5.2.1 Generic
/**
 * @return 0 if initialization is successful, otherwise >0
 * @note
 * Initializes the host controller, process data interface (PDI) and allocates
 * resources which are required for hardware access.
 */
UINT8
HW_Init(void);
/**
 * @brief
 * This function shall be implemented if hardware resources need to be release when the sample application stops
 * @note
 * Release allocated resources.
 */
void
HW_Release(void);

/**
 * @brief
 * This function gets the current content of ALEvent register
 * @note Get the first two bytes of the AL Event register (0x220-0x221).
 * @return first two Bytes of ALEvent register (0x220)
 * If you don't expect SSC `SWAPWORD` really swap uint16_t (usual case),
 * return uint16 value should has bit0 is AL Control event --> byte[0] is 0x0220 on little endian machine
 */
UINT16
HW_GetALEventRegister(void);
/**
 * @brief
 * The SPI PDI requires an extra ESC read access functions from interrupts service routines. The behaviour is equal to
 * "HW_GetALEventRegister()"
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * HW_GetALEventRegister.
 * Get the first two bytes of the AL Event register (0x220-0x221).
 * @return first two Bytes of ALEvent register (0x220)
 */
UINT16
HW_GetALEventRegister_Isr(void);

#if AL_EVENT_ENABLED == 1 && IS_SSC_LOWER_5P10
/**
 * @deprecated
 * @note
 * Performs a logical AND with the AL Event Mask register (0x0204 : 0x0205).
 * This function is only required if “AL_EVENT_ENABLED” is set.
 * NOTE: This function is only required for SSC 5.10 or older.
 */
void
HW_ResetALEventMask(UINT16 intMask);
/**
 * @deprecated
 * @note
 * Performs a logical OR with the AL Event Mask register (0x0204 : 0x0205).
 * This function is only required if “AL_EVENT_ENABLED” is set.
 * NOTE: This function is only required for SSC 5.10 or older.
 */
void
HW_SetALEventMask(UINT16 intMask);
#endif

/**
 * @note
 * Updates the EtherCAT Run and Error LEDs (or EtherCAT Status LED).
 */
void
HW_SetLed(UINT8 RunLed, UINT8 ErrLed);

#if BOOTSTRAPMODE_SUPPORTED == 1
/**
 * @note
 * Resets the hardware. This function is only required if
 * “BOOTSTRAPMODE_SUPPORTED” is set.
 */
void
HW_RestartTarget(void);
#endif

#if IS_SSC_LOWER_5P10
/**
 * @deprecated
 * @note
 * Disables selected SyncManager channel. Sets bit 0 of the corresponding 0x807
 * register.
 * NOTE: This function is only required for SSC 5.10 or older.
 */
void
HW_DisableSyncManChannel(UINT8 channel);
/**
 * @deprecated
 * @note
 * Enables selected SyncManager channel. Resets bit 0 of the corresponding
 * 0x807 register.
 * NOTE: This function is only required for SSC 5.10 or older.
 */
void
HW_EnableSyncManChannel(UINT8 channel);
/**
 * @deprecated
 * @return
 * Pointer to the SyncManager channel description. The SyncManager description
 * structure size is always 8 Byte, the content of “TSYNCMAN” differs depending
 * on the supported ESC access.
 * @note
 * Gets the content of the SyncManager register from the stated channel. Reads 8
 * Bytes starting at 0x800 + 8*channel.
 * NOTE: This function is only required for SSC 5.10 or older.
 */
TSYNCMAN*
HW_GetSyncMan(UINT8 channel);
#endif

#if ECAT_TIMER_INT == 0

// default will use same timer ISR to to increment internal timer

#define ECAT_TIMER_INC_P_MS 1

/**
 * @brief
 * Access to the hardware timer
 * @note
 * Reads the current register value of the hardware timer. If no hardware timer is
 * available the function shall return the counter value of a multimedia timer. The
 * timer ticks value (increments / ms) is defined in “ECAT_TIMER_INC_P_MS”.
 * This function is required if no timer interrupt is supported (“ECAT_TIMER_INT”
 * = 0) and to calculate the bus cycle time.
 */
UINT32
HW_GetTimer(void);

/**
 * @brief
 * Clear the hardware timer
 * @note
 * Clears the hardware timer value.
 */
void
HW_ClearTimer(void);
#endif

#if ESC_EEPROM_EMULATION == 1
/**
 * @note
 * This function is called if an EEPROM reload request is triggered by the master.
 * Only required if EEPROM Emulation is supported and the function pointer
 * “pAPPL_EEPROM_Reload” is not set.
 * In case that the full eeprom emulation is configured (register 0x502, bit6 is 1)
 * the reload function is not called and does not to be implemented.
 */
UINT16
HW_EepromReload(void);
#endif

// ch5.2.2 Read Access
/**
 * @note Reads from the EtherCAT Slave Controller. This function is used to access
 * ESC registers and the DPRAM area.
 */
void
HW_EscRead(MEM_ADDR* pData, UINT16 Address, UINT16 Len);
/**
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscRead”.
 * Reads from the EtherCAT Slave Controller. This function is used to access
 * ESC registers and the DPRAM area.
 */
void
HW_EscReadIsr(MEM_ADDR* pData, UINT16 Address, UINT16 Len);
/**
 * @brief
 * 32Bit ESC read access
 * @note
 * Reads two words from the specified address of the EtherCAT Slave Controller.
 * In case that no specific read DWORD marco is used the default EscRead
 * function may be used:
 * “HW_EscRead(((MEM_ADDR *)&(DWordValue)),((UINT16)(Address)),4)”
 */
#define HW_EscReadDWord(DWordValue, Address) HW_EscRead(((MEM_ADDR*)&(DWordValue)), ((UINT16)(Address)), 4)
/**
 * @brief
 * Interrupt specific 32Bit ESC read access
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscReadWord”.
 * Reads two words from the specified address of the EtherCAT Slave Controller.
 */
#define HW_EscReadDWordIsr(DWordValue, Address) HW_EscReadIsr(((MEM_ADDR*)&(DWordValue)), ((UINT16)(Address)), 4)
/**
 * @brief
 * 16Bit ESC read access
 * @note
 * Reads one word from the specified address of the EtherCAT Slave Controller.
 * Only required if “ESC_32BIT_ACCESS” is not set.
 * In case that no specific read WORD marco is used the default EscRead
 * function may be used:
 * “HW_EscRead(((MEM_ADDR *)&(WordValue)),((UINT16)(Address)),2)”
 */
#define HW_EscReadWord(WordValue, Address) HW_EscRead(((MEM_ADDR*)&(WordValue)), ((UINT16)(Address)), 2)
/**
 * @brief
 * Interrupt specific 16Bit ESC read access
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscReadWord”.
 * Reads one word from the specified address of the EtherCAT Slave Controller.
 * Only required if “ESC_32_BIT_ACCESS” is not set.
 */
#define HW_EscReadWordIsr(WordValue, Address) HW_EscReadIsr(((MEM_ADDR*)&(WordValue)), ((UINT16)(Address)), 2)

#if ESC_16BIT_ACCESS == 0 && ESC_32BIT_ACCESS == 0
/**
 * @note
 * Reads one byte from the EtherCAT Slave Controller.
 * Only required if “ESC_16BIT_ACCESS” and “ESC_32BIT_ACCESS” are not
 * set.
 */
#define HW_EscReadByte(ByteValue, Address) HW_EscRead(((MEM_ADDR*)&(ByteValue)), ((UINT16)(Address)), 1)
/**
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscReadByte”.
 * Reads one byte from the EtherCAT Slave Controller.
 * Only required if “ESC_16BIT_ACCESS” and “ESC_32BIT_ACCESS” are not
 * set.
 */
#define HW_EscReadByteIsr(ByteValue, Address) HW_EscReadIsr(((MEM_ADDR*)&(ByteValue)), ((UINT16)(Address)), 1)
#endif

/**
 * @brief
 * The mailbox data is stored in the local uC memory therefore the default read function is used.
 * @note
 * Reads data from the ESC and copies to slave mailbox memory. If the local
 * mailbox memory is also located in the application memory this function is equal
 * to “HW_EscRead”.
 */
#define HW_EscReadMbxMem(pData, Address, Len) HW_EscRead(((MEM_ADDR*)(pData)), ((UINT16)(Address)), (Len))

// ch5.2.3 Write Access
/**
 * @note
 * Writes from the EtherCAT Slave Controller. This function is used to access\
 * ESC registers and the DPRAM area.
 */
void
HW_EscWrite(MEM_ADDR* pData, UINT16 Address, UINT16 Len);
/**
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscWrite”.
 * Writes from the EtherCAT Slave Controller. This function is used to access ESC
 * registers and the DPRAM area.
 */
void
HW_EscWriteIsr(MEM_ADDR* pData, UINT16 Address, UINT16 Len);
/**
 * @brief
 * 32Bit ESC write access
 * @note
 * Writes one word to the EtherCAT Slave Controller.
 */
#define HW_EscWriteDWord(DWordValue, Address) HW_EscWrite(((MEM_ADDR*)&(DWordValue)), ((UINT16)(Address)), 4)
/**
 * @brief
 * Interrupt specific 32Bit ESC write access
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscWriteWord”.
 * Writes two words to the EtherCAT Slave Controller.
 */
#define HW_EscWriteDWordIsr(DWordValue, Address) HW_EscWriteIsr(((MEM_ADDR*)&(DWordValue)), ((UINT16)(Address)), 4)

#if ESC_32BIT_ACCESS == 0
/**
 * @brief
 * 16Bit ESC write access
 * @note
 * Writes one word to the EtherCAT Slave Controller. Only required if
 * “ESC_32BIT_ACCESS” is not set.
 */
#define HW_EscWriteWord(WordValue, Address) HW_EscWrite(((MEM_ADDR*)&(WordValue)), ((UINT16)(Address)), 2)
/**
 * @brief
 * Interrupt specific 16Bit ESC write access
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscWriteWord”.
 * Writes one word to the EtherCAT Slave Controller. Only required if
 * “ESC_32BIT_ACCESS” is not set.
 */
#define HW_EscWriteWordIsr(WordValue, Address) HW_EscWriteIsr(((MEM_ADDR*)&(WordValue)), ((UINT16)(Address)), 2)
#endif

#if ESC_16BIT_ACCESS == 0 && ESC_32BIT_ACCESS == 0
/**
 * @note
 * Writes one byte to the EtherCAT Slave Controller.
 * Only defined if “ESC_16BIT_ACCESS” and “ESC_32BIT_ACCESS” are
 * disabled.
 */
#define HW_EscWriteByte(ByteValue, Address) HW_EscWrite(((MEM_ADDR*)&(ByteValue)), ((UINT16)(Address)), 1)

/**
 * @note
 * This function should be implemented if a special function for ESC access from
 * interrupt service routines is required; otherwise this function is defined as
 * “HW_EscWriteByte”.
 * Writes one byte to the EtherCAT Slave Controller.
 * Only defined if “ESC_16BIT_ACCESS” and “ESC_32BIT_ACCESS” are
 * disabled.
 */
#define HW_EscWriteByteIsr(ByteValue, Address) HW_EscWriteIsr(((MEM_ADDR*)&(ByteValue)), ((UINT16)(Address)), 1)
#endif

/**
 * @brief
 * The mailbox data is stored in the local uC memory therefore the default write function is used.
 * @note
 * Writes data from the slave mailbox memory to ESC memory. If the local
 * mailbox memory is also located in the application memory this function is equal
 * to “HW_EscWrite”.
 */
#define HW_EscWriteMbxMem(pData, Address, Len) HW_EscWrite(((MEM_ADDR*)(pData)), ((UINT16)(Address)), (Len))
