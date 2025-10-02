// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefConstants.CoralTarget;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Some Preset Autos */
public class Autos {
  public static Command DoubleL4(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Hopper hopper,
      Superstructure superstructure) {
    return Commands.sequence(
        simInit(drive, outtake, true),
        AutoRoutines.runTrajectory("aCtoG"),
        scoreCoral(() -> (CoralTarget.L4), elevator, outtake),
        AutoRoutines.runTrajectory("GtoS"));
  }

  public static Command k4L4(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Hopper hopper,
      Superstructure superstructure) {
    return Commands.sequence(
        simInit(drive, outtake, true),
        AutoRoutines.runTrajectory("aCtoG"),
        scoreCoral(() -> (CoralTarget.L4), elevator, outtake),
        AutoRoutines.runTrajectory("GtoS"),
        intakeCoral(superstructure, outtake, hopper),
        AutoRoutines.runTrajectory("StoD"),
        scoreCoral(() -> (CoralTarget.L4), elevator, outtake),
        AutoRoutines.runTrajectory("DtoS"));
  }

  public static Command logCommand(Superstructure superstructure) {
    return Commands.run(
        () -> {
          Logger.recordOutput(
              "Autos/Coral Ready", superstructure.kCurrentState == State.CORAL_READY);
          Logger.recordOutput(
              "Autos/Coral Intake", superstructure.kCurrentState == State.CORAL_INTAKE);
        });
  }

  public static Command runTestAuto() {
    return Commands.sequence(
        AutoRoutines.runTrajectory("aCtoG"),
        AutoRoutines.runTrajectory("GtoS"),
        AutoRoutines.runTrajectory("StoD"),
        AutoRoutines.runTrajectory("DToS"));
  }

  // public static Command scoreCoral(
  //     Supplier<coralTarget> targetSupplier, Elevator elevator, Outtake outtake) {
  //   return Commands.sequence(
  //       // elevator
  //       //     .setElevatorHeight(() -> (targetSupplier.get().height))
  //       //     .until(elevator::nearSetpoint),
  //       // outtake
  //           .setVoltage(() -> (OuttakeConstants.voltageMap.get(targetSupplier.get().height)))
  //           .until(() -> !(outtake.getDetected()))));
  //       // elevator.setElevatorHeight(0.0).until(elevator::nearSetpoint));
  // }

  public static Command scoreCoral(
      Supplier<CoralTarget> targetSupplier, Elevator elevator, Outtake outtake) {
    return Commands.sequence(
        elevator.setTarget(() -> (targetSupplier.get().height)),
        elevator.setExtension(),
        Commands.waitUntil(elevator::atSetpoint),
        Commands.parallel(
            outtake
                .setVoltage(() -> (OuttakeConstants.voltageMap.get(elevator.getSetpoint())))
                .until(() -> !(outtake.getDetected())),
            Commands.waitSeconds(0.5)
                .andThen(() -> outtake.setDetected(false))
                .unless(Robot::isReal)),
        elevator.setTarget(() -> 0.0),
        elevator.setExtension());
  }

  public static Command intakeCoral(Superstructure superstructure, Outtake outtake, Hopper hopper) {
    // return superstructure
    //     .setState(State.CORAL_INTAKE)
    //     .andThen(
    //         Commands.run(
    //             () -> {
    //               Logger.recordOutput(
    //                   "Autos/Coral Ready", superstructure.kCurrentState == State.CORAL_READY);
    //               Logger.recordOutput(
    //                   "Autos/Coral Intake", superstructure.kCurrentState == State.CORAL_INTAKE);
    //             }));
    // return logCommand(superstructure);
    return Commands.parallel(
            hopper.setVoltage(OuttakeConstants.intake),
            outtake.setVoltage(() -> (OuttakeConstants.intake)),
            Commands.waitSeconds(0.5)
                .andThen(() -> outtake.setDetected(true))
                .unless(Robot::isReal))
        .until(outtake::getDetected);
  }

  public static Command simInit(Drive drive, Outtake outtake, boolean preload) {
    return Commands.run(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(
                      new Pose2d(7.25, FieldConstants.fieldWidth / 2, Rotation2d.k180deg)));
              if (Robot.isSimulation()) {
                outtake.setDetected(preload);
                System.out.println(preload ? "Coral Preloaded" : "Coral Was Not Preloaded");
              }
            })
        .until(() -> ((outtake.getDetected() == preload) || Robot.isReal()))
        .finallyDo(
            () -> {
              System.out.println("Determined Preload");
            });
  }
}
