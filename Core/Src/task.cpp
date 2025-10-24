#include "task.hpp"
#include <memory>
#include <vector>

std::vector<std::unique_ptr<Profile>> profiles;

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

void
task_init()
{
  profiles.push_back(std::make_unique<Slave401Profile>());
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

void
Slave401Profile::pre_process()
{
  // TODO
}

void
Slave401Profile::post_process()
{
  // TODO
}

void
Slave401Profile::process()
{
  // TODO
}