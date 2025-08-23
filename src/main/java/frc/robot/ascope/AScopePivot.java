// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ascope;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class AScopePivot {
  private final Pose3d initialPose;
  private final int component;
  private final AscopeModel model;
  /**
   * Creates a pivot for display with Mechanical Advantage's Advantage Scope 3D model component
   * system.
   *
   * @param model the ascope model that has the pivot as a component
   * @param component
   */
  public AScopePivot(AscopeModel model, int component) {
    this.component = component;
    initialPose = model.getComponentPose(component);
    this.model = model;
  }

  /**
   * Set the rotation of the pivot
   *
   * @param rotation the rotation you want to input
   * @param relative true if relative to the initial pose, false is global
   */
  public void setRotation(Rotation3d rotation, boolean relative) {
    if (relative) {
      model.setComponentPose(component, initialPose.rotateBy(rotation));
    } else {
      model.setComponentPose(
          component,
          initialPose.rotateBy(initialPose.getRotation().unaryMinus()).rotateBy(rotation));
    }
  }

  public static enum rotationAxis {
    X,
    Y,
    Z
  }

  /**
   * Set the rotation of the pivot
   *
   * @param rotation the 2d rotation around the specificed axis you want.
   * @param relative true if relative to the initial pose, false is global
   */
  public void setRotation(Rotation2d rotation, rotationAxis axis, boolean relative) {
    Rotation3d setRotation =
        switch (axis) {
          case X -> new Rotation3d(rotation.getRadians(), 0.0, 0.0);
          case Y -> new Rotation3d(0.0, rotation.getRadians(), 0.0);
          case Z -> new Rotation3d(0.0, 0.0, rotation.getRadians());
        };
    this.setRotation(setRotation, relative);
  }
}
