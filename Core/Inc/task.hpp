#pragma once

#include "task.h"

#ifdef __cplusplus
extern "C"
{
// will include SSC-DeviceObjects.h
#include <SSC-Device.h>
}
#endif

class Profile
{
public:
  // Ensure a key function is defined out-of-line to emit the vtable
  virtual ~Profile();
  virtual void pre_process();
  virtual void post_process();
  virtual void process();
};

class Slave401Profile : public Profile
{
public:
  void pre_process() override;
  void post_process() override;
  void process() override;

protected:
  // bind to `TOBJ6000 ReadDigitalInput8Bit0x6000`
  TOBJ6000* m_pObj6000;
  TOBJ6002* m_pObj6002;
  TOBJ6200* m_pObj6200;
  TOBJ6202* m_pObj6202;
};