// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorDataAutoLogged;

/** Add your docs here. */
public interface IntakeIO {
  public default void updateData(MotorDataAutoLogged data) {}

  public default void setVoltage(Voltage volts) {}
}
