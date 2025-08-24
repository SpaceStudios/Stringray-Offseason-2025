// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoAlign {
  public static Command alignToPose(Supplier<Pose2d> poseSupplier, Drive drive) {
    ProfiledPIDController xController =
        new ProfiledPIDController(10.0, 0, 0, new Constraints(5.0, 10.0));
    ProfiledPIDController yController =
        new ProfiledPIDController(10.0, 0, 0, new Constraints(5.0, 10.0));
    ProfiledPIDController rotController =
        new ProfiledPIDController(5.0, 0, 0, new Constraints(5.0, 10.0));
    Pose2d targetPose = poseSupplier.get();
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    rotController.setGoal(targetPose.getRotation().getRadians());
    return Commands.run(
        () -> {
          if (targetPose != poseSupplier.get()) {
            xController.setGoal(poseSupplier.get().getX());
            yController.setGoal(poseSupplier.get().getY());
            rotController.setGoal(poseSupplier.get().getRotation().getRadians());
          }
          Pose2d currentPose = drive.getPose();
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xController.calculate(currentPose.getX()),
                  yController.calculate(currentPose.getY()),
                  rotController.calculate(currentPose.getRotation().getRadians()));
          drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        });
  }

  public static enum IntakeLocation {
    REEF,
    SOURCE
  }

  public static IntakeLocation getBestIntake(Drive drive) {
    Pose2d kPos = drive.getPose();
    Pose2d closestAlgae = kPos.nearest(List.of(FieldConstants.ReefConstants.algaeLocations));
    Pose2d closestCoral = kPos.nearest(List.of(FieldConstants.SourceConstants.sourcePoses));
    if (kPos.relativeTo(closestAlgae).getTranslation().getNorm()
        < kPos.relativeTo(closestCoral).getTranslation().getNorm()) {
      return IntakeLocation.REEF;
    }
    return IntakeLocation.SOURCE;
  }

  public static boolean isNear(Pose2d target, Pose2d actual) {
    return MathUtil.isNear(0, actual.relativeTo(target).getTranslation().getNorm(), 0.25);
  }

  public static Pose2d getBestSource(Pose2d pos) {
    return pos.nearest(List.of(FieldConstants.SourceConstants.sourcePoses));
  }
}
