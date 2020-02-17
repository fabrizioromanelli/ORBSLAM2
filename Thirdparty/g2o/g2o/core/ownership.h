#ifndef G2O_OWNERSHIP_H
#define G2O_OWNERSHIP_H

#include <g2o/config.h>

#include <iostream>

namespace g2o
{
    template<typename T>
    void release(T* obj)
    {
        (void)obj;
#if G2O_DELETE_IMPLICITLY_OWNED_OBJECTS
        if (obj != nullptr)
        {
            delete obj;
            obj = nullptr;
        }
#endif
    }
}

#endif