#pragma once

#include "task.h"


class Profile{
public:
    // Ensure a key function is defined out-of-line to emit the vtable
    virtual ~Profile();
    virtual void pre_process();
    virtual void post_process();
    virtual void process();
};

class Slave401Profile : public Profile{
public:
    void pre_process() override;
    void post_process() override;
    void process() override;
};