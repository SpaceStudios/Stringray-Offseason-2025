// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.elevator.Elevator;

/** Add your docs here. */
public class Superstructure {
  public enum CoralTarget {
    L4(1.34),
    L3(0.0),
    L2(0.0),
    L1(0.0);

    public double height;

    private CoralTarget(double height) {
      this.height = height;
    }
  }

  // public static class ControllerLayout {
  //   public DoubleSupplier driveX;
  //   public DoubleSupplier driveY;
  //   public DoubleSupplier operatorY;
  //   public Trigger scoreRequest;
  //   public Trigger intakeRequest;
  //   public Trigger manualElevator;
  // }

  public Superstructure(Elevator elevator) {}

  public void periodic() {}
}
