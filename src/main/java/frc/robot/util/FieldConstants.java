// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Field Constants for the First Robotics Competition 2025 Game Reefscape
 *
 * <pre>
 * _  _________________________  _
 *   /         | | | |         \
 *  |     /\   | | | |   /\     |
 *  |    |  |  | |=| |  |  |    |
 *  |     \/   | | | |   \/     |
 * _ \_________|_|_|_|_________/ _
 * </pre>
 */
public class FieldConstants {
  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final double fieldLength = fieldLayout.getFieldLength();
  public static final double fieldWidth = fieldLayout.getFieldWidth();
  public static final double widthBetweenPegs =
      0.328619; // Width Between Peg in meters ALWAYS go and check the field
  // BEFORE COMPETITION
  public static final double safeDistance = Units.inchesToMeters(17);

  public static AprilTagFieldLayout getLayout() {
    fieldLayout.setOrigin(
        DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
            ? OriginPosition.kRedAllianceWallRightSide
            : OriginPosition.kBlueAllianceWallRightSide);
    return fieldLayout;
  }

  public static class ReefConstants {
    public enum CoralTarget {
      L1(0.58),
      L2(0.79),
      L3(1.18),
      L4(1.78);

      public double height;

      private CoralTarget(double height) {
        this.height = height;
      }
    }

    public enum algaeTarget {
      L2(0.42),
      L3(0.81);

      public double height;

      private algaeTarget(double height) {
        this.height = height;
      }
    }

    public static Pose2d[] aprilTags =
        new Pose2d[] {
          fieldLayout.getTagPose(17).get().toPose2d(),
          fieldLayout.getTagPose(18).get().toPose2d(),
          fieldLayout.getTagPose(19).get().toPose2d(),
          fieldLayout.getTagPose(20).get().toPose2d(),
          fieldLayout.getTagPose(21).get().toPose2d(),
          fieldLayout.getTagPose(22).get().toPose2d()
        };
    public static Pose2d[] leftBranches =
        new Pose2d[] {
          aprilTags[0].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[1].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[2].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[3].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg))
        };
    public static Pose2d[] rightBranches =
        new Pose2d[] {
          aprilTags[0].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[1].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[2].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[3].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg))
        };

    private static List<Pose2d> tagList = List.of(aprilTags);
    private static List<Pose2d> leftBranchList = List.of(leftBranches);
    private static List<Pose2d> rightBranchList = List.of(rightBranches);

    public static Pose2d getBestBranch(Supplier<Pose2d> poseSupplier, boolean left) {
      flipConstants();
      Pose2d nearestTag = poseSupplier.get().nearest(tagList);
      if (nearestTag == aprilTags[3] || nearestTag == aprilTags[4] || nearestTag == aprilTags[5]) {
        left = !left;
      }

      Logger.recordOutput(
          "Field Constants/Nearest Left Branch", poseSupplier.get().nearest(leftBranchList));
      Logger.recordOutput(
          "Field Constants/Nearest Right Branch", poseSupplier.get().nearest(rightBranchList));
      if (left) {
        System.out.println(left);
        return poseSupplier.get().nearest(AutoAlignConstants.nearestReefPoseLeft);
      } else {
        System.out.println("false");
        return poseSupplier.get().nearest(AutoAlignConstants.nearestReefPoseRight);
      }
    }

    public static Pose2d[] algaeLocations =
        new Pose2d[] {
          aprilTags[0].transformBy(
              new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg)),
          aprilTags[1].transformBy(
              new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg)),
          aprilTags[2].transformBy(
              new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg)),
          aprilTags[3].transformBy(
              new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg))
        };

    public static final Pose2d middleReef = new Pose2d(4.47, 4.03, Rotation2d.k180deg);
  }

  public class SourceConstants {
    public static Pose2d[] sourceTags =
        new Pose2d[] {
          AllianceFlipUtil.apply(fieldLayout.getTagPose(12).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(13).get().toPose2d()),
        };
    public static Pose2d[] sourcePoses =
        new Pose2d[] {
          sourceTags[0].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero)),
          sourceTags[1].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero))
        };

    private static List<Pose2d> sourceList = List.of(sourcePoses);

    public static Pose2d getNearestSource(Supplier<Pose2d> poseSupplier) {
      return getNearest(poseSupplier, sourceList);
    }
  }

  public class BargeConstants {
    public static Pose2d[] bargeTags =
        new Pose2d[] {
          fieldLayout.getTagPose(14).get().toPose2d(), fieldLayout.getTagPose(4).get().toPose2d()
        };
    public static Pose2d[] bargePoses =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              bargeTags[0].transformBy(
                  new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg))),
          AllianceFlipUtil.apply(
              bargeTags[1].transformBy(
                  new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg)))
        };
    public static final Pose2d[] climbPoses =
        new Pose2d[] {
          AllianceFlipUtil.apply(
              bargeTags[0].transformBy(
                  new Transform2d(
                      0.25,
                      Units.inchesToMeters(-0.99995 - 0.25) - Units.inchesToMeters(42.937416),
                      Rotation2d.kZero))),
          AllianceFlipUtil.apply(
              bargeTags[0].transformBy(
                  new Transform2d(0.25, Units.inchesToMeters(-0.99995 - 0.25), Rotation2d.kZero))),
          AllianceFlipUtil.apply(
              bargeTags[0].transformBy(
                  new Transform2d(
                      0.25,
                      Units.inchesToMeters(-0.99995 - 0.25) + Units.inchesToMeters(42.937500),
                      Rotation2d.kZero)))
        };
    public static final Pose2d net = bargePoses[0];
    public static final double elevatorSetpoint = 1.78;

    public static boolean nearNet(Supplier<Pose2d> poseSupplier) {
      return inNetZone(getNearestNet(poseSupplier), poseSupplier.get());
    }

    public static boolean inNetZone(Pose2d netPose, Pose2d currentPose) {
      return MathUtil.isNear(netPose.getX(), currentPose.getX(), 0.25)
          && MathUtil.isNear(netPose.getY(), currentPose.getY(), FieldConstants.fieldWidth / 4)
          && MathUtil.isNear(
              netPose.getRotation().getRadians(),
              currentPose.getRotation().getRadians(),
              Math.PI / 4);
    }

    private static List<Pose2d> bargePoseList = List.of(bargePoses);

    public static Pose2d getNearestNet(Supplier<Pose2d> poseSupplier) {
      return poseSupplier.get().nearest(bargePoseList);
    }
  }

  public class ProcessorConstants {
    public static final double elevatorSetpoint = 0.0;
  }

  private static Pose2d endPose = new Pose2d(fieldLength, fieldWidth, Rotation2d.kZero);

  public static boolean inTolerance(
      Pose2d pose1, Pose2d pose2, double translationTolerance, double orientationTolerance) {
    return MathUtil.isNear(pose1.getX(), pose2.getX(), translationTolerance)
        && MathUtil.isNear(pose1.getY(), pose2.getY(), translationTolerance)
        && MathUtil.isNear(
            pose1.getRotation().getRadians(),
            pose2.getRotation().getRadians(),
            orientationTolerance);
  }

  public static Pose2d getNearest(Supplier<Pose2d> poseSupplier, List<Pose2d> poses) {
    return poseSupplier.get().nearest(poses);
  }

  private static Pose2d[] flipPoses(Pose2d[] poseList) {
    Pose2d[] kPoses = new Pose2d[poseList.length];
    for (int i = 0; i < poseList.length; i++) {
      kPoses[i] = AllianceFlipUtil.apply(poseList[i]);
    }
    return kPoses;
  }

  private static Pose2d[] flipPosesWithTransform(Pose2d[] kPoses, Transform2d transform) {
    Pose2d[] kPoseArray = new Pose2d[kPoses.length];
    for (int i = 0; i < kPoses.length; i++) {
      kPoseArray[i] = AllianceFlipUtil.apply(kPoses[i]).transformBy(transform);
    }
    return kPoseArray;
  }

  public static void flipConstants() {
    // Flip Reef
    // Flip Left Branches
    ReefConstants.leftBranches =
        flipPosesWithTransform(
            ReefConstants.aprilTags,
            new Transform2d(
                new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg));

    // Flip Right Branches
    ReefConstants.rightBranches =
        flipPosesWithTransform(
            ReefConstants.aprilTags,
            new Transform2d(
                new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg));

    // Update Lists
    ReefConstants.leftBranchList = List.of(ReefConstants.leftBranches);
    ReefConstants.rightBranchList = List.of(ReefConstants.rightBranches);

    // Flip Algae Locations.
    ReefConstants.algaeLocations =
        flipPosesWithTransform(
            ReefConstants.aprilTags,
            new Transform2d(new Translation2d(safeDistance, 0.0), Rotation2d.k180deg));

    // Update Tag List
    ReefConstants.tagList = List.of(flipPoses(ReefConstants.aprilTags));

    // Flip Barge
    BargeConstants.bargePoses =
        flipPosesWithTransform(
            BargeConstants.bargeTags, new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg));
    BargeConstants.bargePoseList = List.of(flipPoses(BargeConstants.bargePoses));

    // Flip Source
    SourceConstants.sourcePoses =
        flipPosesWithTransform(
            SourceConstants.sourceTags, new Transform2d(safeDistance, 0.0, Rotation2d.kZero));
    SourceConstants.sourceList = List.of(SourceConstants.sourcePoses);
  }

  public static void Log() {
    Logger.recordOutput("Field Constants/End Point", endPose);
    Logger.recordOutput("Field Constants/Reef/AprilTags", ReefConstants.aprilTags);
    Logger.recordOutput("Field Constants/Reef/Left Branches", ReefConstants.leftBranches);
    Logger.recordOutput("Field Constants/Reef/Right Branches", ReefConstants.rightBranches);
    Logger.recordOutput("Field Constants/Reef/Algae Poses", ReefConstants.algaeLocations);
    Logger.recordOutput("Field Constants/Source/Source Tags", SourceConstants.sourceTags);
    Logger.recordOutput("Field Constants/Source/Source Poses", SourceConstants.sourcePoses);
    Logger.recordOutput("Field Constants/Barge/Barge Tags", BargeConstants.bargeTags);
    Logger.recordOutput("Field Constants/Barge/Barge Poses", BargeConstants.bargePoses);
    Logger.recordOutput("Field Constants/Barge/Cage Poses", BargeConstants.climbPoses);
    Logger.recordOutput("Field Constants/Current Match Time", Timer.getMatchTime());
    Logger.recordOutput(
        "Field Constants/Current Alliance",
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : DriverStation.Alliance.Blue);
  }
}
