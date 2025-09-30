// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbDataAutoLogged data = new ClimbDataAutoLogged();
  private final ClimbIO io;
  /** Creates a new Climb. */
  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    Logger.processInputs("Climb", data);
  }

  public Command extend() {
    return setAngle(ClimbConstants.Setpoints.extended);
  }

  public Command retract() {
    return setAngle(ClimbConstants.Setpoints.score);
  }

  public Command setAngle(double angle) {
    return this.run(
        () -> {
          data.angleSetpoint = angle;
          io.setAngle(angle);
        });
  }
}
