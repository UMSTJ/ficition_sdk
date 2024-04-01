//
// Created by anaple on 24-4-1.
//

#include "battery.h"
#include <iostream>

BatteryMonitor::BatteryMonitor(double maxVoltage, double minVoltage)
        : maxVoltage_(maxVoltage), minVoltage_(minVoltage) {
}

bool BatteryMonitor::updateVoltage(double voltage) {
    if (voltages_.size() >= 100) {
        voltages_.pop_front();
    }
    voltages_.push_back(voltage);

    double averageVoltage = calculateAverage();

    // 检查是否在充电，即电压是否上升
    bool isCharging = !lastAverageVoltage_ || averageVoltage > lastAverageVoltage_;
    lastAverageVoltage_ = averageVoltage;

    return isCharging;
}

double BatteryMonitor::calculateBatteryPercentage() {
    if (voltages_.empty()) return 0.0;
    double currentVoltage = voltages_.back();
    return (currentVoltage - minVoltage_) / (maxVoltage_ - minVoltage_) * 100;
}

double BatteryMonitor::calculateAverage() {
    double sum = 0.0;
    for (double voltage : voltages_) {
        sum += voltage;
    }
    return sum / voltages_.size();
}
