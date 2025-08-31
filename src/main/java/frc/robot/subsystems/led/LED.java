// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure.state;

public class LED extends SubsystemBase {
  private state currentState;
  /** Creates a new LED. */
  public LED() {}

  @Override
  public void periodic() {
    
  }

  public Command setState(state kState) {
    return this.run(() -> {
      this.currentState = kState;
    });
  }
}
