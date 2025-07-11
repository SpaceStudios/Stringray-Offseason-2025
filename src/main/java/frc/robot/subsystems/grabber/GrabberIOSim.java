// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.grabber;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.sensors.SensorIO;
import frc.robot.util.MotorData;

/** Add your docs here. */
public class GrabberIOSim implements GrabberIO {
    private final SensorIO sensor;

    public GrabberIOSim(SensorIO sensor) {
        this.sensor = sensor;
    }

    @Override
    public void setVolts(Voltage volts) {
        if (volts.baseUnitMagnitude() > 0) {
            sensor.simOverride(false);
        } else if (volts.baseUnitMagnitude() < 0) {
            sensor.simOverride(true);            
        }
    }

    @Override
    public boolean hasAlgae() {
        return sensor.detected();
    }

    @Override
    public void updateInputs(MotorData data) {
        
    }
}
