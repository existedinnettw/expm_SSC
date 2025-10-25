#pragma once

#include "task.h"
#include <cstdint>
#include <functional>

#ifdef __cplusplus
extern "C"
{
// will include SSC-DeviceObjects.h
#include <SSC-Device.h>
}
#endif

/**
 * \brief Base class for CANopen application-specific device profiles
 */
class Profile
{
public:
  // Ensure a key function is defined out-of-line to emit the vtable
  virtual ~Profile();
  virtual void pre_process();
  virtual void post_process();
  virtual void process();
};

// Lightweight aliases to simplify callback type usage
using DigitalInputFn = std::function<uint8_t(uint8_t)>;
using DigitalOutputFn = std::function<void(uint8_t, uint8_t)>;

class Slave401Profile : public Profile
{
public:
  Slave401Profile(DigitalInputFn DI_hal, DigitalOutputFn DO_hal);
  ~Slave401Profile();
  void pre_process() override;
  void post_process() override;
  void process() override;

protected:
  TOBJ6000& m_ReadDigitalInput8Bit0x6000;
  TOBJ6002& m_PolarityDigitalInput8Bit0x6002;
  TOBJ6200& m_WriteDigitalOutput8Bit0x6200;
  TOBJ6202& m_ChangePolarityDigitalOutput8Bit0x6202;

  DigitalInputFn m_DI_hal;
  DigitalOutputFn m_DO_hal;
};