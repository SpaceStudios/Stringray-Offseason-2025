// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ascope;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

/** Creates a Countinous Elevator that works with AdvantageScope Components */
public class AScopeElevatorSim {
  private final Pose3d[] initialPoses;
  private final int[] elevatorComponents;
  private final Translation3d up = new Translation3d(0, 0, 1);
  private final AscopeModel displayBot;

  public AScopeElevatorSim(AscopeModel robot, int[] elevatorComponents) {
    this.elevatorComponents = elevatorComponents;
    initialPoses = new Pose3d[elevatorComponents.length];
    for (int i = 0; i < elevatorComponents.length; i++) {
      initialPoses[i] = robot.getComponentPose(elevatorComponents[i]);
    }
    displayBot = robot;
  }

  public void setHeight(Distance height) {
    double heightMeters = height.in(Meters);
    double step = heightMeters / elevatorComponents.length;
    double totalHeight = step;
    for (int i = 0; i < elevatorComponents.length; i++) {
      Pose3d targetPose =
          initialPoses[i].plus(
              new Transform3d(
                  up.rotateBy(initialPoses[i].getRotation()).times(totalHeight), Rotation3d.kZero));
      displayBot.setComponentPose(elevatorComponents[i], targetPose);
      totalHeight += step;
    }
  }
}
