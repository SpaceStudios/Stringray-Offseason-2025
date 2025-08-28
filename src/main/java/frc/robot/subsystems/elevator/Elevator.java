// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldConstants.ReefConstants.coralTarget;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final elevatorDataAutoLogged data = new elevatorDataAutoLogged();
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateData(data);
    Logger.processInputs("Elevator", data);
  }

  public Command setElevatorHeight(double height) {
    return this.run(
        () -> {
          io.setHeight(height);
        });
  }

  public Command setElevatorHeight(DoubleSupplier height) {
    return this.run(
        () -> {
          io.setHeight(height.getAsDouble());
        });
  }

  public Command setElevatorHeight(coralTarget height) {
    return this.run(
        () -> {
          io.setHeight(height.height);
        });
  }

  public Command runVelocity(DoubleSupplier axis) {
    return this.run(
        () -> {
          io.runVelocity(axis.getAsDouble());
        });
  }

  public double getHeight() {
    return io.getHeight();
  }

  public double getSetpoint() {
    return data.elevatorSetpoint;
  }

  public boolean nearSetpoint() {
    return MathUtil.isNear(data.elevatorSetpoint, data.elevatorHeight, 0.025);
  }
}
