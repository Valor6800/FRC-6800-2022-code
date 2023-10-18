#pragma once

namespace valor {

/**
 * @brief Container to hold PID and feed forward values for the motor controller
 */
struct PIDF
{
    /// Proportion control of the feedback term
    double P = 0.1;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    /// Feedforward term
    double F = 0.000244;
    /// Max velocity: revolutions per 1s
    double velocity = 1500;
    /// Max acceleration: revolutions per 1s^2
    double acceleration = 15000;
    /// Minimum error threshold
    double error = 0.5;

    double aFF = 0;
    double aFFTarget = 90;

    // Sets the s-curve value for the motion profile. This helps control the jerk of the mechanimsm and reduces the oscillation, but makes it slower
    // Values 0-8. 0 is a trapezoidal motion profile, 1-8 adds a curve 
    int sCurveStrength = 0;
};
}
