// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {
  @AutoLog
  public class ClimbData {
    public boolean connected = false;

    public double angle = 0.0;
    public double angleSetpoint = 0.0;
    public double temperature = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public double voltage = 0.0;
  }

  public default void getData(ClimbDataAutoLogged data) {}

  public default void setAngle(double angle) {}
}
