// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */
  private final OuttakeIO io;

  private final OuttakeDataAutoLogged data = new OuttakeDataAutoLogged();

  public Outtake(OuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    Logger.processInputs("Outtake", data);
  }

  public Command setVoltage(double voltage) {
    return this.run(
        () -> {
          io.setVoltage(voltage);
        });
  }

  public Command setDetected(boolean detected) {
    return Commands.runOnce(
        () -> {
          io.setDetected(detected);
        });
  }

  public boolean getDetected() {
    return data.detected;
  }
}
