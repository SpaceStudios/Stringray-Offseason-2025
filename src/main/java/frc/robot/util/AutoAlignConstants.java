// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants.ReefConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutoAlignConstants {
  public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static List<Pose2d> leftPersPose = new ArrayList<>();
  public static List<Pose2d> rightPersPose = new ArrayList<>();
  public static List<Pose2d> algaePose = new ArrayList<>();
  public static List<Pose2d> cagePoses = new ArrayList<>();
  public static HashMap<Integer, Pose2d> aprilPoses = new HashMap<>();
  public static List<Pose2d> nearestReefPoseLeft = new ArrayList<>();
  public static List<Pose2d> nearestReefPoseRight = new ArrayList<>();
  public static List<Pose2d> offsetList = new ArrayList<>();

  public static double AUTOALIGN_BACKUP_X = 18.0;
  public static double AUTOALIGN_BACKUP_Y = 10.0;
  public static Transform2d backwardsLeftTransform = new Transform2d(
      new Translation2d(
          Units.inchesToMeters(AUTOALIGN_BACKUP_X), Units.inchesToMeters(AUTOALIGN_BACKUP_Y)),
      Rotation2d.fromDegrees(-180));
  public static Transform2d backwardsRightTransform = new Transform2d(
      new Translation2d(
          Units.inchesToMeters(AUTOALIGN_BACKUP_X), -Units.inchesToMeters(AUTOALIGN_BACKUP_Y)),
      Rotation2d.fromDegrees(-180));

  public static void getAprilTagPoses() {

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    Logger.recordOutput("MathInfo/Alliance", alliance);

    // Defualt's to init with blue side so if it is red you need to change field
    // layout -- idk i
    // found this online and it works
    // FieldConstants.getLayout();

    aprilPoses.clear();
    offsetList.clear();
    nearestReefPoseLeft.clear();
    nearestReefPoseRight.clear();

    if (alliance == Alliance.Red) {

      for (int i = 6; i < 12; i++) {
        aprilPoses.put(i, FieldConstants.getLayout().getTagPose(i).get().toPose2d());
        offsetList.add(FieldConstants.getLayout().getTagPose(i).get().toPose2d());
      }

      for (int i = 6; i < 9; i++) {
        Pose2d pose = aprilPoses.get(i);
        nearestReefPoseLeft.add(pose.transformBy(backwardsRightTransform));
        nearestReefPoseRight.add(pose.transformBy(backwardsLeftTransform));
      }

      for (int i = 9; i < 12; i++) {
        Pose2d pose = aprilPoses.get(i);
        nearestReefPoseRight.add(pose.transformBy(backwardsRightTransform));
        nearestReefPoseLeft.add(pose.transformBy(backwardsLeftTransform));
      }
    }

    if (alliance == Alliance.Blue) {

      for (int i = 17; i < 23; i++) {
        aprilPoses.put(i, FieldConstants.getLayout().getTagPose(i).get().toPose2d());
        offsetList.add(FieldConstants.getLayout().getTagPose(i).get().toPose2d());
      }

      for (int i = 17; i < 20; i++) {
        Pose2d pose = aprilPoses.get(i);
        nearestReefPoseLeft.add(pose.transformBy(backwardsRightTransform));
        nearestReefPoseRight.add(pose.transformBy(backwardsLeftTransform));
      }

      for (int i = 20; i < 23; i++) {
        Pose2d pose = aprilPoses.get(i);
        nearestReefPoseRight.add(pose.transformBy(backwardsRightTransform));
        nearestReefPoseLeft.add(pose.transformBy(backwardsLeftTransform));
      }
    }
  }

  public static boolean inReefRange(Drive drive, double threshold) {
    Logger.recordOutput("StateTracker/ReefPose", ReefConstants.middleReef);
    Logger.recordOutput(
        "StateTracker/DistanceFromReef",
        ReefConstants.middleReef.getTranslation().getDistance(drive.getPose().getTranslation()));

    Logger.recordOutput(
        "StateTracker/isInRange",
        ReefConstants.middleReef.getTranslation().getDistance(drive.getPose().getTranslation()) <= threshold);

    return ReefConstants.middleReef.getTranslation().getDistance(drive.getPose().getTranslation()) <= threshold;
  }
}
