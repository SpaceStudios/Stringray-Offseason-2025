// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorData;

/** Add your docs here. */
public interface OuttakeIO {
    public default void updateData(MotorData data) {}
    public default void setVolts(Voltage volts) {}
}
