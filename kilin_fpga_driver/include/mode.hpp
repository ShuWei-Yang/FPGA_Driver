#ifndef __MODE_H
#define __MODE_H

#pragma once
enum class Mode{
    REST = 0,
    SET_ZERO = 1,
    HALL_CALIBRATE = 2,
    MOTOR = 3,
    CONFIG = 4,
    CONTROL = 5
};

enum class Behavior{
    SET_THETA = 0,
    TCP_SLAVE = 1,
    SET_SERVO = 2,
    CUSTOM_1  = 3,
    CUSTOM_2  = 4,
    CUSTOM_3  = 5
};


#endif

// ok with RSBL