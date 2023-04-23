#pragma once

class Timer {
    unsigned long long current;
public:
    void start() {
        current = 0;
    }
    void elapse() {
        current ++;
    }
    unsigned long long get() {
        return current;
    }
};