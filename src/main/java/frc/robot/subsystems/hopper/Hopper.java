// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.outtake.Outtake;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private HopperDataAutoLogged data = new HopperDataAutoLogged();
  /** Creates a new Hopper. */
  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    Logger.processInputs("Hopper", data);
  }

  public Command setVoltage(double volts, Outtake outtake) {
    return this.run(
            () -> {
              io.setVoltage(volts);
              outtake.setDetected(true);
            })
        .finallyDo(
            () -> {
              io.setVoltage(0.0);
            });
  }
}
