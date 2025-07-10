// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import java.util.HashMap;
import java.util.List;

/** Add your docs here. */
public class ElevatorConstants {
  // Scoring Poses
  public static final Pose2d[] algaePoses = new Pose2d[] {};
  public static final Distance[] algaeSetpoints = new Distance[] {};
  private static HashMap<Pose2d, Distance> algaeMap;

  // Elevator Constants
  public static final double gearing = (5.0 / 1.0); // Elevator Gearing
  public static final Distance drumRadius = Inches.of(1.0); // The Elevator Lift Roller Radius
  public static final Mass carriageMass = Pounds.of(15);
  public static final Distance elevatorMaxheight = Meters.of(1.78);

  // PID Constants
  public static final double kP = 80.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double maxVelocity = 5.0;
  public static final double maxAcceleration = 10.0;
  public static final double kS = 0.0;
  public static final double kG = 0.0;

  public static Distance getNearestAlgaeHeight(Pose2d robotPose) {
    if (algaeMap == null) {
      algaeMap = new HashMap<Pose2d, Distance>();
      for (int i = 0; i < algaePoses.length; i++) {
        algaeMap.put(algaePoses[i], algaeSetpoints[i]);
      }
    }
    return algaeMap.get(robotPose.nearest(List.of(algaePoses)));
  }
}
