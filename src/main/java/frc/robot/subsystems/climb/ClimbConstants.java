// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

/** Add your docs here. */
public class ClimbConstants {
  public class PID {
    public static final double[] kP = new double[] {0.9, 0.9, 0.9, 0.9};
    public static final double[] kI = new double[] {0.9, 0.9, 0.9, 0.9};
    public static final double[] kD = new double[] {0.9, 0.9, 0.9, 0.9};
    public static final int slot = 0;
  }

  public class MotorLimits {
    public static final double gearing = (9.0 / 1.0) * (5.0 / 1.0) * (58.0 / 18.0) * (28.0 / 12.0);
    public static final int statorLimit = 120;
    public static final int supplyLimit = 120;
    public static final int supplyLow = 60;
    public static final double rampPeriod = 0.5;
    public static final boolean inverted = false;
  }

  public class Setpoints {
    public static final double extended = 90;
    public static final double score = 45;
  }
}
