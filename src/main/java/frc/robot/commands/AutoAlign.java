// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

/** Add your docs here. */
public class AutoAlign {
  /** These are <b>constants</b> please change these to fit your robot */
  public static class AutoalignConstants {
    private static final LinearVelocity driveMaxVelocity = MetersPerSecond.of(1);
    private static final LinearAcceleration driveMaxAcceleration = MetersPerSecondPerSecond.of(1);
    private static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(2);
    private static final AngularAcceleration maxAngularAcceleration =
        RadiansPerSecondPerSecond.of(2);
  }

  private static Pose2d[] algaePoses = new Pose2d[] {};

  private static Pose2d[] coralPoses = new Pose2d[] {};

  private AutoAlign() {}

  public static Command autoalignToTarget(Drive drive, Pose2d targetPose) {
    ProfiledPIDController xController =
        new ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                AutoalignConstants.driveMaxVelocity.in(MetersPerSecond),
                AutoalignConstants.driveMaxAcceleration.in(MetersPerSecondPerSecond)));
    ProfiledPIDController yController =
        new ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                AutoalignConstants.driveMaxVelocity.in(MetersPerSecond),
                AutoalignConstants.driveMaxAcceleration.in(MetersPerSecondPerSecond)));
    ProfiledPIDController rotationController =
        new ProfiledPIDController(
            10.0,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                AutoalignConstants.maxAngularVelocity.in(RadiansPerSecond),
                AutoalignConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond)));
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    rotationController.setTolerance(0.001);
    return Commands.run(
        () -> {
          Pose2d currentPose = drive.getPose();
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xController.calculate(currentPose.getX(), targetPose.getX()),
                  yController.calculate(currentPose.getY(), targetPose.getY()),
                  rotationController.calculate(
                      currentPose.getRotation().getRadians(),
                      targetPose.getRotation().getRadians()),
                  drive.getRotation()));
        });
  }
}
