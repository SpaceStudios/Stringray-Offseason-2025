// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ascope;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class AscopeModel {
  private Pose3d[] componentPoses;
  private static HashMap<String, AscopeModel> modelMap;
  private static int robotNum = 1;
  /**
   * @param components
   * @param name
   */
  public AscopeModel(int components, String name) {
    componentPoses = new Pose3d[components];
    for (int i = 0; i < components; i++) {
      componentPoses[i] = new Pose3d();
    }
    modelMap.put(name, this);
  }

  public AscopeModel(Pose3d[] initialPoses, String name) {
    componentPoses = initialPoses;
    modelMap.put(name, this);
  }

  public AscopeModel(int components) {
    this(components, "Robot_" + (robotNum));
    robotNum += 1;
  }

  public void setComponentPose(int component, Pose3d pose) {
    if (component < componentPoses.length) {
      componentPoses[component] = pose;
    } else {
      DriverStation.reportError(
          "Attempted to set Component Pose for a component that doesn't exist make sure to check component id",
          true);
    }
  }

  public void translate(int component, Transform3d translation) {
    if (component < componentPoses.length) {
      componentPoses[component] = componentPoses[component].plus(translation);
    } else {
      DriverStation.reportError(
          "Attempted to transform a Component Pose for a component that doesn't exist make sure to check components",
          true);
    }
  }

  public Pose3d getComponentPose(int component) {
    if (component < componentPoses.length) {
      return componentPoses[component];
    }
    DriverStation.reportError(
        "Attempted to get Component Pose for a component that doesn't exist make sure to check component id",
        true);
    return new Pose3d();
  }

  public void periodic() {
    Logger.recordOutput("Ascope/Model Components", componentPoses);
  }

  public static AscopeModel getRobot(String name) {
    if (modelMap.containsKey(name)) {
      return modelMap.get(name);
    }
    return null;
  }

  public int getComponentAmount() {
    return componentPoses.length;
  }
}
