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

/** Add your docs here. */
public class FieldConstants {
  public static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public static final double fieldLength = fieldLayout.getFieldLength();
  public static final double fieldWidth = fieldLayout.getFieldWidth();
  public static final double widthBetweenPegs =
      0.328619; // Width Between Peg in meters ALWAYS go and check the field BEFORE COMPETITION
  public static final double safeDistance = Units.inchesToMeters(17);

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
      L2(0.79),
      L3(1.18);

      public double height;

      private algaeTarget(double height) {
        this.height = height;
      }
    }

    public static final Pose2d[] aprilTags =
        new Pose2d[] {
          AllianceFlipUtil.apply(fieldLayout.getTagPose(17).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(18).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(19).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(20).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(21).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(22).get().toPose2d())
        };
    public static final Pose2d[] leftBranches =
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
    public static final Pose2d[] rightBranches =
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
    public static final Pose2d[] algaeLocations =
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
  }

  public class SourceConstants {
    public static final Pose2d[] sourceTags =
        new Pose2d[] {
          AllianceFlipUtil.apply(fieldLayout.getTagPose(12).get().toPose2d()),
          AllianceFlipUtil.apply(fieldLayout.getTagPose(13).get().toPose2d()),
        };
    public static final Pose2d[] sourcePoses =
        new Pose2d[] {
          sourceTags[0].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero)),
          sourceTags[1].transformBy(new Transform2d(safeDistance, 0.0, Rotation2d.kZero))
        };
  }

  public class BargeConstants {
    public static final Pose2d[] bargeTags =
        new Pose2d[] {
          fieldLayout.getTagPose(14).get().toPose2d(), fieldLayout.getTagPose(4).get().toPose2d()
        };
    public static final Pose2d[] bargePoses =
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

    private static final List<Pose2d> bargePoseList = List.of(bargePoses);

    public static Pose2d getNearestNet(Supplier<Pose2d> poseSupplier) {
      return poseSupplier.get().nearest(bargePoseList);
    }
  }

  public class ProcessorConstants {
    public static final double elevatorSetpoint = 0.0;
  }

  private static Pose2d endPose = new Pose2d(fieldLength, fieldWidth, Rotation2d.kZero);

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
