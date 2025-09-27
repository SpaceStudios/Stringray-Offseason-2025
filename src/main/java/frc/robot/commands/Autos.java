// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure;
import frc.robot.Superstructure.state;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.util.FieldConstants.ReefConstants.coralTarget;

/** Some Preset Autos */
public class Autos {
  public static Command DoubleL4(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Hopper hopper,
      Superstructure superstructure) {
    return Commands.sequence(
            TrajectoryFollower.followTrajectory(TrajectoryFollower.loadTrajectory("aCtoG")),
            elevator.setElevatorHeight(coralTarget.L4.height).until(elevator::nearSetpoint),
            outtake.setVoltage(() -> (OuttakeConstants.L4)).until(() -> !(outtake.getDetected())),
            elevator.setElevatorHeight(0.0).until(elevator::nearSetpoint),
            Commands.parallel(
                    TrajectoryFollower.followTrajectory(TrajectoryFollower.loadTrajectory("GtoS")),
                    superstructure.setState(state.CORAL_INTAKE).withTimeout(0.15))
                .until(outtake::getDetected),
            TrajectoryFollower.followTrajectory(TrajectoryFollower.loadTrajectory("StoC")),
            elevator.setElevatorHeight(coralTarget.L4.height),
            outtake.setVoltage(() -> (OuttakeConstants.L4)).until(() -> !(outtake.getDetected())))
        .withTimeout(15);
  }

  public static Command testMultiPath() {
    return TrajectoryFollower.followTrajectory(TrajectoryFollower.loadTrajectory("aCtoG"))
        .andThen(TrajectoryFollower.followTrajectory(TrajectoryFollower.loadTrajectory("GtoS")));
  }
}
