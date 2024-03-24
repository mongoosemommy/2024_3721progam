
#pragma once

#include "../subsystems/subsystem.hpp"
namespace autonomous
{
    class SendIt
    {
    private:
        double st;  // start time
    public:
        SendIt(/* args */);
        void run();
    };
} // namespace autonomous
