#pragma once

struct SSC_appl_singleton{
    void (*APPL_Application)(void);
};

extern struct SSC_appl_singleton SSC_appl_singleton_instance;