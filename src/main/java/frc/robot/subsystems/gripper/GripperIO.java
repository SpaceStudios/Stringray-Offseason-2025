// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface GripperIO {
  @AutoLog
  public class gripperData {
    public boolean detected = false;

    public boolean connected = false;
    public double voltage = 0.0;
    public double temperature = 0.0;
    public double supplyCurrent = 0.0;
    public double statorCurrent = 0.0;
  }

  public default void getData(gripperDataAutoLogged data) {}
  ;

  public default void setVoltage(double voltage) {}

  public default void setDetected(boolean detected) {}
}
