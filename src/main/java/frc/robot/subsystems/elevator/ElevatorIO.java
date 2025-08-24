// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public class elevatorData {
    // Motor Data
    public boolean connected = false;
    public double motorTemperature = 0.0;
    public double motorPosition = 0.0;
    public double motorVelocity = 0.0;
    public double motorAcceleration = 0.0;
    public double motorVoltage = 0.0;
    public double motorOutput = 0.0;
    public double motorCurrent = 0.0;
    public double motorSupplyCurrent = 0.0;

    public boolean followerConnected = false;
    public double followerTemperature = 0.0;
    public double followerVoltage = 0.0;
    public double followerOutput = 0.0;
    public double followerCurrent = 0.0;
    public double followerSupplyCurrent = 0.0;

    // Elevator Data
    public double elevatorHeight = 0.0;
    public double elevatorSetpoint = 0.0;
    public double elevatorVelocity = 0.0;
  }

  public default void updateData(elevatorDataAutoLogged data) {}

  public default void setHeight(double height) {}

  public default void setVoltage(double voltage) {}

  public default void runVelocity(double joystick) {}

  public default double getHeight() {
    return 0.0;
  }
}
