// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface OuttakeIO {
  @AutoLog
  public class OuttakeData {
    public boolean connected = false;

    public double voltage = 0.0;
    public double temperature = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
  }

  public default void getData(OuttakeDataAutoLogged data) {}

  public default void setVoltage(double voltage) {}
}
