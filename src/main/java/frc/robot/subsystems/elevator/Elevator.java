// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorDataAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final ElevatorIO io;

  private final ElevatorDataAutoLogged data = new ElevatorDataAutoLogged();
  private final MotorDataAutoLogged leftMotorData = new MotorDataAutoLogged();
  private final MotorDataAutoLogged rightMotorData = new MotorDataAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateData(data, leftMotorData, rightMotorData);
    Logger.processInputs("Elevator", data);
    Logger.processInputs("Elevator/Left Motor", leftMotorData);
    Logger.processInputs("Elevator/Right Motor", rightMotorData);
  }

  public void setTargetHeight(Distance height) {
    io.setHeight(height);
  }

  public Distance getHeight() {
    return io.getHeight();
  }

  public Distance getSetpoint() {
    return data.setPoint;
  }

  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public Command setHeight(Distance height) {
    return this.run(
        () -> {
          this.setTargetHeight(height);
        });
  }
}
