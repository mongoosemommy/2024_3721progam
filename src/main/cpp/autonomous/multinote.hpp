#pragma once

#include "../subsystems/subsystem.hpp"

namespace autonomous
{
    class MultiNote
    {
    private:
        double st;  // start time
    public:
        MultiNote(/* args */);
        void run();
    };
} // namespace autonomous
