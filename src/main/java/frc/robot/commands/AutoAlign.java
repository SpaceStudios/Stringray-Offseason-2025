// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
// import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class AutoAlign {

  private static List<Pose2d> waypoints =
      List.of(
          AllianceFlipUtil.apply(new Pose2d(3.22, 1.9, Rotation2d.fromRadians(1))),
          AllianceFlipUtil.apply(new Pose2d(7, 1.82, Rotation2d.fromRadians(2.3))),
          AllianceFlipUtil.apply(new Pose2d(7, 6.25, Rotation2d.fromDegrees(225))),
          AllianceFlipUtil.apply(new Pose2d(3.22, 6.25, Rotation2d.fromDegrees(-45))),
          AllianceFlipUtil.apply(new Pose2d(2.0105, 4, Rotation2d.kZero)));

  public static Pose2d[] waypointsLogged = waypoints.toArray(new Pose2d[waypoints.size()]);

  public static Command alignToPose(Supplier<Pose2d> poseSupplier, Drive drive) {
    ProfiledPIDController xController =
        new ProfiledPIDController(5.0, 0.01, 0.02, new TrapezoidProfile.Constraints(3.0, 4.0));
    ProfiledPIDController yController =
        new ProfiledPIDController(5.0, 0.01, 0.02, new TrapezoidProfile.Constraints(3.0, 4.0));
    ProfiledPIDController rotController =
        new ProfiledPIDController(100.0, 0, 0, new TrapezoidProfile.Constraints(10.0, 10.0));
    Pose2d targetPose = poseSupplier.get();
    xController.setGoal(targetPose.getX());
    yController.setGoal(targetPose.getY());
    rotController.setGoal(targetPose.getRotation().getRadians());
    return Commands.runOnce(
            () -> {
              Pose2d target = poseSupplier.get();
              xController.setGoal(target.getX());
              yController.setGoal(target.getY());
              rotController.setGoal(target.getRotation().getRadians());
            })
        .andThen(
            Commands.run(
                    () -> {
                      if (targetPose != poseSupplier.get()) {
                        xController.setGoal(poseSupplier.get().getX());
                        yController.setGoal(poseSupplier.get().getY());
                        rotController.setGoal(poseSupplier.get().getRotation().getRadians());
                      }
                      Pose2d currentPose = drive.getPose();
                      Pose2d relativePose = currentPose.relativeTo(poseSupplier.get());
                      ChassisSpeeds speeds =
                          new ChassisSpeeds(
                              Math.abs(xController.calculate(currentPose.getX()))
                                  * (relativePose.getX() / (-Math.abs(relativePose.getX()))),
                              Math.abs(yController.calculate(currentPose.getY()))
                                  * (relativePose.getY() / (-Math.abs(relativePose.getY()))),
                              rotController.calculate(currentPose.getRotation().getRadians()));
                      drive.runVelocity(
                          ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
                    })
                .finallyDo(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds());
                      xController.reset(drive.getPose().getX());
                      yController.reset(drive.getPose().getY());
                      rotController.reset(drive.getPose().getRotation().getRadians());
                    })
                .until(() -> (AutoAlign.isNear(poseSupplier.get(), drive.getPose()))));
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
    return MathUtil.isNear(0, actual.relativeTo(target).getTranslation().getNorm(), 0.025)
        && MathUtil.isNear(
            target.getRotation().getRadians(), actual.getRotation().getRadians(), 0.025);
  }

  public static boolean isNearWaypoint(Pose2d waypoint, Pose2d actual) {
    return MathUtil.isNear(0, actual.relativeTo(waypoint).getTranslation().getNorm(), 0.5)
        && MathUtil.isNear(
            waypoint.getRotation().getRadians(), actual.getRotation().getRadians(), 0.025);
  }

  private static final List<Pose2d> tagList = List.of(FieldConstants.ReefConstants.aprilTags);
  private static final List<Pose2d> sourceList =
      List.of(FieldConstants.SourceConstants.sourcePoses);
  private static final List<Pose2d> algaeList =
      List.of(FieldConstants.ReefConstants.algaeLocations);

  public static Pose2d getBestSource(Pose2d pose) {
    return pose.nearest(sourceList);
  }

  public static Pose2d getBestLeftBranch(Supplier<Pose2d> pose) {
    int index = tagList.indexOf(pose.get().nearest(tagList));
    if (index > 2) {
      return FieldConstants.ReefConstants.rightBranches[index];
    }
    return FieldConstants.ReefConstants.leftBranches[index];
  }

  public static Pose2d getBestRightBranch(Supplier<Pose2d> pose) {
    int index = tagList.indexOf(pose.get().nearest(tagList));
    if (index > 2) {
      return FieldConstants.ReefConstants.leftBranches[index];
    }
    return FieldConstants.ReefConstants.rightBranches[index];
  }

  public static Pose2d getBestAlgaePose(Supplier<Pose2d> pose) {
    return pose.get().nearest(algaeList);
  }

  public static Pose2d[] generatePath1Waypoint(Supplier<Pose2d> end, Supplier<Pose2d> start) {
    Pose2d[] path = new Pose2d[3];
    path[0] = start.get();
    path[1] = start.get().nearest(waypoints);
    path[2] = end.get();
    return path;
  }

  // private static final ArrayList<Pose2d> generatedWaypoints = new ArrayList<Pose2d>();

  // private static final double maxDistance = 3.5;

  // public static Pose2d[] generateMultiWaypointPath(Supplier<Pose2d> end, Supplier<Pose2d> start)
  // {
  //   if (generatedWaypoints.size() > 0) {
  //     generatedWaypoints.clear();
  //   }
  //   ArrayList<Pose2d> clonedWaypoints = new ArrayList<Pose2d>(waypoints);
  //   boolean finished = false;
  //   boolean foundNext = false;
  //   generatedWaypoints.add(start.get());
  //   while (!finished) {
  //     while (!foundNext) {
  //       Pose2d next = end.get().nearest(clonedWaypoints);
  //       if (getDistance(next, end.get()) <= getDistance(start.get(), end.get())
  //           && getDistance(next, end.get()) <= maxDistance) {
  //         clonedWaypoints.remove(next);
  //         generatedWaypoints.add(next);
  //         foundNext = true;
  //       }
  //       if (clonedWaypoints.size() == 0) {
  //         foundNext = true;
  //         finished = true;
  //       }
  //     }
  //     foundNext = false;
  //   }
  //   generatedWaypoints.add(end.get());
  //   return generatedWaypoints.toArray(new Pose2d[generatedWaypoints.size()]);
  // }

  public static Pose2d getBestWaypoint(
      Supplier<Pose2d> end, Supplier<Pose2d> start, List<Pose2d> waypoints) {
    boolean found = false;
    List<Pose2d> clonedList = List.of(waypointsLogged);
    while (!found) {
      Pose2d waypoint = start.get().nearest(clonedList);
      if (getDistance(waypoint, end.get()) < getDistance(start.get(), end.get())) {
        return waypoint;
      } else {
        clonedList.remove(waypoint);
      }
    }
    return new Pose2d();
  }

  private static double getDistance(Pose2d pose1, Pose2d pose2) {
    return pose1.relativeTo(pose2).getTranslation().getNorm();
  }
}
