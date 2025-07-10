// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.grabber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private final GrabberIO io;
  /** Creates a new Grabber. */
  public Grabber(GrabberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command setVoltage(Voltage voltage) {
    return this.run(() -> {io.setVolts(voltage);});
  }

  public Command setVoltage(double volts) {
    return this.run(() -> {io.setVolts(Volts.of(volts));});
  }

  //TODO add sensor check
  public boolean hasAlgae() {
    return false;
  }
}
