// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import static edu.wpi.first.units.Units.Celsius;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;

/** Add your docs here. */
public class SensorIOGrapple implements SensorIO {
    private final LaserCan laserCan;
    private final double threshold;

    public SensorIOGrapple(int id, RangingMode rangingMode, RegionOfInterest regionOfInterest, double threshold) {
        laserCan = new LaserCan(id);
        this.threshold = threshold;
        try {
            laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_33MS);
            laserCan.setRangingMode(rangingMode);
            laserCan.setRegionOfInterest(regionOfInterest);
        } catch (ConfigurationFailedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void updateData(SensorDataAutoLogged data) {
        Measurement measurement = laserCan.getMeasurement();
        data.connected = (measurement != null);
        data.raw = measurement.distance_mm;
        data.detected = (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) && (measurement.distance_mm < threshold);
        data.temperature = Celsius.of(20);
    }
}
