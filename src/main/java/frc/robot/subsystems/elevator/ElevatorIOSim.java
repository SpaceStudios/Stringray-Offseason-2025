// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.MotorDataAutoLogged;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;
  private final ProfiledPIDController pid;
  private final ElevatorFeedforward ff;

  private Voltage volts = Volts.zero();
  private Distance setPoint = Meters.zero();

  public ElevatorIOSim() {
    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            gearing,
            carriageMass.in(Kilograms),
            drumRadius.in(Meters),
            0.0,
            elevatorMaxheight.in(Meters),
            true,
            0.0);

    pid = new ProfiledPIDController(80.0, 0.0, 0.0, new Constraints(5.0, 10.0));
    ff =
        new ElevatorFeedforward(
            0.0,
            0.06,
            (DCMotor.getKrakenX60(1).KvRadPerSecPerVolt * drumRadius.in(Meters)) / gearing);
  }

  @Override
  public void updateData(
      ElevatorDataAutoLogged data,
      MotorDataAutoLogged leftMotorData,
      MotorDataAutoLogged rightMotorData) {
    sim.update(0.020);
    data.height = Meters.of(sim.getPositionMeters());
    data.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
    data.acceleration = MetersPerSecondPerSecond.of(0.0);
    data.setPoint = setPoint;

    leftMotorData.motorConnected = true;
    leftMotorData.motorCurrent = Amps.of(sim.getCurrentDrawAmps());
    leftMotorData.motorVoltage = volts;
  }

  @Override
  public void setVolts(Voltage volts) {
    this.volts = volts;
    sim.setInputVoltage(MathUtil.clamp(volts.in(Volts), -12.0, 12.0));
  }

  @Override
  public void setHeight(Distance height) {
    setPoint = height;
    setVolts(
        Volts.of(
            pid.calculate(sim.getPositionMeters(), height.in(Meters))
                + ff.calculate(pid.getSetpoint().velocity)));
  }
}
