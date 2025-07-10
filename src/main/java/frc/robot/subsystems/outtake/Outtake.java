// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorDataAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */
  private final OuttakeIO io;

  private final MotorDataAutoLogged data = new MotorDataAutoLogged();

  public Outtake(OuttakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateData(data);
    Logger.processInputs("Outtake", data);
  }

  public Command setVolts(Voltage volts) {
    return this.run(
        () -> {
          io.setVolts(volts);
        });
  }

  public Command setVolts(double volts) {
    return this.run(
        () -> {
          io.setVolts(Volts.of(volts));
        });
  }

  // TODO add sensor check
  public boolean hasCoral() {
    return false;
  }
}
