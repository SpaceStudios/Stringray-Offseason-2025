// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class TrajectoryFollower {
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
                ChassisSpeeds speeds =
                    new ChassisSpeeds(
                        sample.get().vx
                            + xController.calculate(pose.getX(), sample.get().getPose().getX()),
                        sample.get().vy
                            + yController.calculate(pose.getY(), sample.get().getPose().getY()),
                        sample.get().omega
                            + rotController.calculate(
                                pose.getRotation().getRadians(), sample.get().heading));
                driveSpeeds.accept(
                    ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
              } else {
                done = true;
              }
            })
        .until(() -> (done));
  }

  public static Command followTrajectory(Optional<Trajectory<SwerveSample>> trajectory) {
    return driveSubsystem
        .startRun(
            () -> {
              done = false;
              timer.reset();
              timer.start();
            },
            () -> {
              if (trajectory.isPresent() && (driveSubsystem != null)) {
                Optional<SwerveSample> sample =
                    trajectory.get().sampleAt(timer.get(), isRedAlliance());
                if (sample.isPresent()) {
                  Pose2d pose = poseGetter.get();
                  Logger.recordOutput("Autos/Sample Pose", sample.get().getPose());
                  ChassisSpeeds desiredSpeeds =
                      sample
                          .get()
                          .getChassisSpeeds()
                          .plus(
                              new ChassisSpeeds(
                                  xController.calculate(pose.getX(), sample.get().getPose().getX()),
                                  yController.calculate(pose.getY(), sample.get().getPose().getY()),
                                  rotController.calculate(
                                      pose.getRotation().getRadians(),
                                      sample.get().getPose().getRotation().getRadians())));
                  driveFunction.accept(
                      ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, pose.getRotation()));
                } else {
                  done = true;
                }
              } else {
                done = true;
              }
            })
        .until(
            () ->
                (done
                    || AutoAlign.isNear(
                        poseGetter.get(), trajectory.get().getFinalPose(isRedAlliance()).get())))
        .finallyDo(
            () -> {
              xController.reset();
              yController.reset();
              rotController.reset();
              driveSubsystem.stop();
              timer.stop();
            });
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

  public static final Map<String, Optional<Trajectory<SwerveSample>>> trajectoryMap =
      Map.of(
          "aCtoG", loadTrajectory("aCtoG"),
          "aBoG", loadTrajectory("aBoG"));
}
