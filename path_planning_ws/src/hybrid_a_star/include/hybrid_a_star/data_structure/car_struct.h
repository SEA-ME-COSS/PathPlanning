#pragma once

struct Car {
    // PID Config
    double kp = 1.0;
    double ki = 1.0;
    double kd = 1.0;

    // Vehicle Config
    double RF = 3.3;    // [cm] distance from rear to vehicle front end of vehicle
    double RB = 0.8;    // [cm] distance from rear to vehicle back end of vehicle
    double W = 23.5;     // [cm] width of vehicle
    double H = 29.6;
    double WD = 0.7*W;  // [cm] distance between left-right wheels
    double WB = 2.5;    // [cm] Wheel base
    double TR = 0.44;   // [cm] Tyre radius
    double TW = 0.7;    // [cm] Tyre width

    double MAX_STEER = 0.65;    // [rad] Maximum steering angle
    double MAX_SPEED = 10.0;    // [cm/s] Maximum speed
};