#include "task.hpp"
#include "main.h"
#include <memory>
#include <vector>

std::vector<std::unique_ptr<Profile>> profiles;


void
task_init()
{
  profiles.push_back(std::make_unique<Slave401Profile>(
    [](uint8_t input_group_id) {
      // value from MCU
      // PA0 is key0
      (void)input_group_id;
      return static_cast<uint8_t>(0);
    },
    [](uint8_t output_group_id, uint8_t output_value) {
      // write to MCU
      switch (output_group_id) {
        case 1: {
          // PA6 is led0
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (output_value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        } break;

        default:
          break;
      }
      return;
    }));
}

void
task_loop()
{
  for (auto& profile : profiles) {
    profile->pre_process();
  }
  for (auto& profile : profiles) {
    profile->process();
  }
  for (auto& profile : profiles) {
    profile->post_process();
  }
}
