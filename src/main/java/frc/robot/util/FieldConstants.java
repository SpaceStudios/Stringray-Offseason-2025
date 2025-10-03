// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  public static class ReefConstants {
    public enum CoralTarget {
      L1(0.58),
      L2(0.79),
      L3(1.18),
      L4(1.73);

      public double height;

      private CoralTarget(double height) {
        this.height = height;
      }
    }

    public enum AlgaeTarget {
      L2(0.42),
      L3(0.81);

      public double height;

      private AlgaeTarget(double height) {
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
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / 2.0), Rotation2d.k180deg))
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
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[4].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg)),
          aprilTags[5].transformBy(
              new Transform2d(
                  new Translation2d(safeDistance, widthBetweenPegs / -2.0), Rotation2d.k180deg))
        };

    private static List<Pose2d> tagList = List.of(aprilTags);
    private static List<Pose2d> leftBranchList = List.of(leftBranches);
    private static List<Pose2d> rightBranchList = List.of(rightBranches);

    private static double L4Offset = 0.25;

    public static Pose2d getBestBranch(Supplier<Pose2d> poseSupplier, boolean left, boolean L4) {
      Pose2d pose = getNearestFlipped(poseSupplier, left ? leftBranchList : rightBranchList);
      return L4 ? pose.transformBy(new Transform2d(-L4Offset, 0.0, Rotation2d.kZero)) : pose;
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

    public static Pose2d getReef() {
      return AllianceFlipUtil.apply(middleReef);
    }

    public static boolean nearReef(Supplier<Pose2d> poseSupplier) {
      return getReef().getTranslation().getDistance(poseSupplier.get().getTranslation()) < 3.0;
    }
  }

  public class SourceConstants {
    public static Pose2d[] sourceTags =
        new Pose2d[] {
          fieldLayout.getTagPose(12).get().toPose2d(), fieldLayout.getTagPose(13).get().toPose2d(),
        };
    public static Pose2d[] sourcePoses =
        new Pose2d[] {
          sourceTags[0].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero)),
          sourceTags[1].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero))
        };

    private static List<Pose2d> sourceList = List.of(sourcePoses);

    public static Pose2d getNearestSource(Supplier<Pose2d> poseSupplier) {
      return getNearestFlipped(poseSupplier, sourceList);
    }
  }

  public class BargeConstants {
    public static Pose2d[] bargeTags =
        new Pose2d[] {
          fieldLayout.getTagPose(14).get().toPose2d(), fieldLayout.getTagPose(4).get().toPose2d()
        };
    public static Pose2d[] bargePoses =
        new Pose2d[] {
          bargeTags[0].transformBy(new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg)),
          bargeTags[1].transformBy(new Transform2d(safeDistance * 1.5, 0.0, Rotation2d.k180deg))
        };
    public static final Pose2d[] climbPoses =
        new Pose2d[] {
          bargeTags[0].transformBy(
              new Transform2d(
                  0.25,
                  Units.inchesToMeters(-0.99995 - 0.25) - Units.inchesToMeters(42.937416),
                  Rotation2d.kZero)),
          bargeTags[0].transformBy(
              new Transform2d(0.25, Units.inchesToMeters(-0.99995 - 0.25), Rotation2d.kZero)),
          bargeTags[0].transformBy(
              new Transform2d(
                  0.25,
                  Units.inchesToMeters(-0.99995 - 0.25) + Units.inchesToMeters(42.937500),
                  Rotation2d.kZero))
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
      return getNearestFlipped(poseSupplier, bargePoseList);
    }
  }

  public class ProcessorConstants {
    public static final double elevatorSetpoint = 0.0;
  }

  private static Pose2d endPose = new Pose2d(fieldLength, fieldWidth, Rotation2d.kZero);

  public static boolean inTolerance(
      Supplier<Pose2d> pose1,
      Supplier<Pose2d> pose2,
      double translationTolerance,
      double orientationTolerance) {
    return MathUtil.isNear(pose1.get().getX(), pose2.get().getX(), translationTolerance)
        && MathUtil.isNear(pose1.get().getY(), pose2.get().getY(), translationTolerance)
        && MathUtil.isNear(
            pose1.get().getRotation().getRadians(),
            pose2.get().getRotation().getRadians(),
            orientationTolerance);
  }

  public static Pose2d getNearestFlipped(Supplier<Pose2d> poseSupplier, List<Pose2d> poses) {
    return AllianceFlipUtil.apply(AllianceFlipUtil.apply(poseSupplier.get()).nearest(poses));
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
    Logger.recordOutput(
        "Field Constants/Reef/Middle", AllianceFlipUtil.apply(ReefConstants.middleReef));
    Logger.recordOutput("Field Constants/Barge/Cage Poses", BargeConstants.climbPoses);
    Logger.recordOutput("Field Constants/Current Match Time", Timer.getMatchTime());
    Logger.recordOutput(
        "Field Constants/Current Alliance",
        DriverStation.getAlliance().isPresent()
            ? DriverStation.getAlliance().get()
            : DriverStation.Alliance.Blue);
  }
}
