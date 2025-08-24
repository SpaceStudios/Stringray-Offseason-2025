// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final double gearing = (5.0 / 1.0);
  public static final double drumRadius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
  // private final double drumAlternateRadius = Units.inchesToMeters(2.362/2.0);
  public static final double mass = Units.lbsToKilograms(25.1);
  public static final double maxHeight = 1.78;
  public static final double maxVelocity = 20;

  public static class PID {
    public static final double[] kP = new double[] {0.9, 0.9, 0.9};
    public static final double[] kI = new double[] {0.0, 0.0, 0.0};
    public static final double[] kD = new double[] {0.0, 0.0, 0.0};
    public static final double[] kG = new double[] {0.0, 0.0, 0.0};
    public static final double[] kS = new double[] {0.0, 0.0, 0.0};

    public static final double maxAcceleration = 10.0;
    public static final double maxVelocity = 3.5;

    public static final double kV = 3.122;
    public static final double kA = 0.1189;

    public static final double kMM_Expo_kV = 3.122;
    public static final double kMM_Expo_kA = 1.9;
  }

  public class MotorConstants {
    public static final int maxCurrent = 120;
    public static final int maxCurrentLow = 60;
    public static final double rampPeriod = 0.1;
  }
}
