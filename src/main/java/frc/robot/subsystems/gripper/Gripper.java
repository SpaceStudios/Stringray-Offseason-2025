// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final gripperDataAutoLogged data = new gripperDataAutoLogged();
  /** Creates a new Gripper. */
  public Gripper(GripperIO io) {
    this.io = io;
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
            () -> {
              io.setVoltage(volts.getAsDouble());
            })
        .finallyDo(
            () -> {
              io.setVoltage(0.0);
            });
  }

  public Command setDetected(boolean detected) {
    return Commands.run(
        () -> {
          io.setDetected(detected);
        });
  }

  public boolean getDetected() {
    return data.detected;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    Logger.processInputs("Gripper", data);
  }
}
