// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.grabber;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorData;

/** Add your docs here. */
public interface GrabberIO {
  public default void setVolts(Voltage volts) {}

  public default void setOutput(double output) {}

  public default void updateInputs(MotorData data) {}
}
