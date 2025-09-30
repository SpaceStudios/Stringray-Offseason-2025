// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {
  @AutoLog
  public class ClimbData {
    public boolean connected;

    public double angle;

    public double temperature;
    public double statorCurrent;
    public double supplyCurrent;
    public double voltage;
  }

  public default void getData(ClimbDataAutoLogged data) {}

  public default void setAngle(double angle) {}
}
