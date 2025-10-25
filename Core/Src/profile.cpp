#include "task.hpp"
extern "C"
{
#include <objdef.h>
}


// Define a key function out-of-line to ensure the vtable for Profile is emitted
Profile::~Profile() = default;

// Provide default no-op implementations for base virtuals
void
Profile::pre_process()
{
}
void
Profile::post_process()
{
}
void
Profile::process()
{
}

/**
 * @todo consider device profile offset e.g. 0x6800
 */
Slave401Profile::Slave401Profile(DigitalInputFn DI_hal, DigitalOutputFn DO_hal)
  : m_ReadDigitalInput8Bit0x6000(*reinterpret_cast<TOBJ6000*>(OBJ_GetObjectHandle(0x6000)->pVarPtr))
  , m_PolarityDigitalInput8Bit0x6002(*reinterpret_cast<TOBJ6002*>(OBJ_GetObjectHandle(0x6002)->pVarPtr))
  , m_WriteDigitalOutput8Bit0x6200(*reinterpret_cast<TOBJ6200*>(OBJ_GetObjectHandle(0x6200)->pVarPtr))
  , m_ChangePolarityDigitalOutput8Bit0x6202(*reinterpret_cast<TOBJ6202*>(OBJ_GetObjectHandle(0x6202)->pVarPtr))
  , m_DI_hal(std::move(DI_hal))
  , m_DO_hal(std::move(DO_hal))
{
}
Slave401Profile::~Slave401Profile() = default;

void
Slave401Profile::pre_process()
{
  // app logic –> control calc–> output –> sensor
}

void
Slave401Profile::post_process()
{
  // output
  for (UINT8 i = 0; i < m_WriteDigitalOutput8Bit0x6200.u16SubIndex0; i++) {
    uint8_t output_value = m_WriteDigitalOutput8Bit0x6200.aEntries[i];
    uint8_t output_group_id = i;
    m_DO_hal(output_group_id, output_value);
  }

  // sensor
  for (UINT8 i = 0; i < m_ReadDigitalInput8Bit0x6000.u16SubIndex0; i++) {
    uint8_t input_group_id = i;
    uint8_t input_value = m_DI_hal(input_group_id);
    m_ReadDigitalInput8Bit0x6000.aEntries[i] = input_value;
  }
}

void
Slave401Profile::process()
{
  // TODO: filter, debounce, etc.
}
