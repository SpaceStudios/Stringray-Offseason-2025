// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import frc.robot.util.FieldConstants.ReefConstants.CoralTarget;
import java.util.Map;

/** Add your docs here. */
public class OuttakeConstants {
  public static final double intake = 5.0;
  public static final double L1 = 5.0;
  public static final double L2 = 5.0;
  public static final double L3 = 5.0;
  public static final double L4 = 5.0;

  public static final Map<Double, Double> voltageMap =
      Map.of(
          0.0,
          0.0,
          CoralTarget.L1.height,
          L1,
          CoralTarget.L2.height,
          L2,
          CoralTarget.L3.height,
          L3,
          CoralTarget.L4.height,
          L4);

  public static class MotorLimits {
    public static final int torque = 60;
    public static final int stator = 30;
    public static final int supply = 30;
    public static final int supplyLow = 30;

    public static final double rampPeriod = 0.5;
    public static final boolean inverted = false;
  }
}
