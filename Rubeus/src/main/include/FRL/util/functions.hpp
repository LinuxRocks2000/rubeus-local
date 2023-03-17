#pragma once

bool withinDeadband(double num, double dead, double reference = 0) {
    num -= reference; // So reference becomes 0
        
    return (num < dead) && 
        (num > -dead);
}

/**
    * Calculate error between a setpoint and current position *based on the fact that there are always 2 ways to reach any given point on a circle*.
    @param set The setpoint
    @param cur The current position
    */