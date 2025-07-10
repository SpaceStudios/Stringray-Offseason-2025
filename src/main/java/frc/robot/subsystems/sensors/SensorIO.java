// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sensors;

import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SensorIO {
  @AutoLog
  public static class SensorData {
    public boolean connected = false;
    public double raw = 0.0;
    public boolean detected = false;
    public Temperature temperature;
  }

  public default void updateData(SensorDataAutoLogged data) {}
  public default boolean detected() {return false;}
}