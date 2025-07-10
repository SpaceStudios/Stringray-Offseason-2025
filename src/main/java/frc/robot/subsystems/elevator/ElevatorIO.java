// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.MotorDataAutoLogged;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
  @AutoLog
  public class ElevatorData {
    public Distance height = Meters.zero();
    public Distance setPoint = Meters.zero();
    public LinearVelocity velocity = MetersPerSecond.zero();
    public LinearAcceleration acceleration = MetersPerSecondPerSecond.zero();
    public boolean atSetpoint = false;
  }

  public default void setHeight(Distance height) {}

  public default Distance getHeight() {
    return Meters.zero();
  }

  public default void setVolts(Voltage volts) {}

  public default void resetEncoders() {}

  public default void stop() {
    setVolts(Volts.zero());
  }

  public default void updateData(
      ElevatorDataAutoLogged data,
      MotorDataAutoLogged leftMotorData,
      MotorDataAutoLogged rightMotorData) {}

  public default boolean atSetpoint() {
    return true;
  }
}
