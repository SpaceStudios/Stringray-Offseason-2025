// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.proximity;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ProximityIO {
  @AutoLog
  public class ProximityData {
    public boolean connected;

    public double distance; // Distance in MM actually idk what it is in
    public boolean detected;

    public double temperature;
  }

  public default void getData(ProximityDataAutoLogged data) {}

  public default void setDetected(boolean detected) {}
}
