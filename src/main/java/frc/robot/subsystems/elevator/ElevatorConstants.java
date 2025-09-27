// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.FieldConstants.ReefConstants.algaeTarget;
import frc.robot.util.FieldConstants.ReefConstants.coralTarget;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {

  // sim stuff
  public static final double elevatorMass =
      Pounds.of(30.0).in(Kilograms); // Returns The mass in kilograms
  public static final double maxHeight = Inches.of(69.6).in(Meters);
  public static final double minHeight = Inches.of(0).in(Meters);
  public static final boolean simGravity = false;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

  public enum ElevatorSetpoint {
    INTAKE(0.0), // 0.006 if its not at the right height
    L1(0.42),
    L1Flick(0.52),
    L2(0.79),
    L3(1.18),
    L4(1.77),
    A2(0.42),
    A3(0.81);

    public double height;

    private ElevatorSetpoint(double height) {
      this.height = height;
    }

    public static ElevatorSetpoint getSetpointFromCoralTarget(coralTarget target) {
      switch (target) {
        case L4:
          return ElevatorSetpoint.L4;
        case L3:
          return ElevatorSetpoint.L3;
        case L2:
          return ElevatorSetpoint.L2;
        case L1:
          return ElevatorSetpoint.L1;
        default:
          return ElevatorSetpoint.L1;
      }
    }

    public static ElevatorSetpoint getSetpointFromAlgaeTarget(algaeTarget target) {
      switch (target) {
        case L3:
          return ElevatorSetpoint.A3;
        case L2:
          return ElevatorSetpoint.A2;
        default:
          return ElevatorSetpoint.L1;
      }
    }
  }

  public static LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/PID/kP", 50.0);
  public static LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/PID/kI", 0.0);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/PID/kD", 0.0);

  public static LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/FF/kS", 0.7);
  public static LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/FF/kG", 0.45);
  public static LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/FF/kV", 3.12);
  public static LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/FF/kA", 0.1189);

  // Real Robot numbers
  public static double rampRate = 0.1;
  public static double maxVelocity = 3.5;
  public static double maxAcceleration = 10.0;
  public static int statorCurrent = 100;
  public static int supplyCurrent = 80;
  public static int supplyCurrentLow = 60;

  public static final double drumRadius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
  public static final double gearing = (5.0 / 1.0);
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;
  public static final double tolerance = 0.01;
}
