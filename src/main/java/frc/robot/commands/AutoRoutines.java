// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.util.FieldConstants.ReefConstants;
import frc.robot.util.FieldConstants.ReefConstants.CoralTarget;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AutoRoutines {
  private static final PIDController xController = new PIDController(10.0, 0, 0);
  private static final PIDController yController = new PIDController(10.0, 0, 0);
  private static final PIDController rotController = new PIDController(7.5, 0, 0);

  static {
    rotController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static boolean done = false;
  private static Timer timer = new Timer();

  public static Supplier<Pose2d> poseGetter;
  public static Consumer<ChassisSpeeds> driveFunction;
  public static Drive driveSubsystem;
  public static AutoFactory autoFactory;

  public Command oneL4Coral(Drive drive, Outtake outtake, Hopper hopper, Elevator elevator) {
    return Commands.sequence(
        DriveCommands.autoAlign(drive, () -> ReefConstants.getBestBranch(drive::getPose, true)),
        elevator.setTarget(() -> CoralTarget.L4.height),
        elevator.setExtension(),
        Commands.waitUntil(elevator::atSetpoint),
        outtake.setVoltage(() -> OuttakeConstants.L4),
        Commands.waitUntil(() -> !outtake.getDetected()),
        elevator.setTarget(() -> 0.0),
        elevator.setExtension());
  }

  public static void setDataGetters(
      Supplier<Pose2d> kPoseGetter, Consumer<ChassisSpeeds> kDriveFunction, Drive kDriveSys) {
    poseGetter = kPoseGetter;
    driveFunction = kDriveFunction;
    driveSubsystem = kDriveSys;
  }

  public static Command followTrajectory(
      Trajectory<SwerveSample> traj,
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> driveSpeeds,
      Drive drive) {
    return drive
        .startRun(
            () -> {
              done = false;
              timer.reset();
              timer.start();
            },
            () -> {
              Logger.recordOutput("Autos/Trajectory", traj.getPoses());
              Optional<SwerveSample> sample = traj.sampleAt(timer.get(), isRedAlliance());
              Pose2d pose = poseSupplier.get();
              if (sample.isPresent()) {

              } else {
                done = true;
              }
            })
        .until(() -> (done));
  }

  private static boolean isRedAlliance() {
    return (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red))
        && Robot.isReal();
  }

  public static Optional<Trajectory<SwerveSample>> loadTrajectory(String path) {
    Optional<Trajectory<SwerveSample>> traj = Choreo.loadTrajectory(path + ".traj");
    System.out.println(traj.isEmpty());
    if (traj.isEmpty()) {
      DriverStation.reportError(
          "Tried to load "
              + path
              + " and failed \n Please double check to make sure the trajectory specified exists",
          done);
    }
    return traj;
  }

  public static Command runTrajectory(String path) {
    return autoFactory
        .trajectoryCmd(path + ".traj")
        .finallyDo(
            () -> {
              driveSubsystem.stop();
              xController.reset();
              yController.reset();
              rotController.reset();
            });
  }

  public Consumer<SwerveSample> driveController(Drive drive) {
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    return (sample) -> {
      final Pose2d pose = drive.getPose();
      Logger.recordOutput("Autos/Sample Pose", sample.getPose());
      Logger.recordOutput(
          "Autos/Speeds Field Relative", new ChassisSpeeds(sample.vx, sample.vy, sample.omega));
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              sample.vx + xController.calculate(pose.getX(), sample.getPose().getX()),
              sample.vy + yController.calculate(pose.getY(), sample.getPose().getY()),
              sample.omega
                  + rotController.calculate(
                      pose.getRotation().getRadians(), sample.getPose().getRotation().getRadians()),
              drive.getRotation());
      drive.runVelocity(speeds);
    };
  }

  public static final Map<String, Optional<Trajectory<SwerveSample>>> trajectoryMap =
      Map.of(
          "aCtoG", loadTrajectory("aCtoG"),
          "aBoG", loadTrajectory("aBoG"));
}
