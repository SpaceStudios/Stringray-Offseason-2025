// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {
  // private final AScopeElevatorSim elevator;
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          ElevatorConstants.gearing,
          ElevatorConstants.mass,
          ElevatorConstants.drumRadius,
          0,
          ElevatorConstants.maxHeight,
          true,
          0.0);
  private final ProfiledPIDController pid =
      new ProfiledPIDController(40.0, 0, 0.01, new Constraints(5.0, 10.0));
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(
          0.0,
          0.06 + 0.541,
          (DCMotor.getKrakenX60(1).KvRadPerSecPerVolt * ElevatorConstants.drumRadius)
              / ElevatorConstants.gearing);

  private double voltage = 0.0;

  public ElevatorIOSim() {
    // elevator = new AScopeElevatorSim(AscopeModel.getRobot("Stringray"), new int[] {0, 1, 2});
  }

  @Override
  public void updateData(final elevatorDataAutoLogged data) {
    setVoltage(pid.calculate(sim.getPositionMeters()) + ff.calculate(pid.getGoal().velocity));

    sim.update(0.020);
    // elevator.setHeight(Meters.of(sim.getPositionMeters()));
    data.connected = true;
    data.motorCurrent = sim.getCurrentDrawAmps();
    data.motorVoltage = voltage;

    data.elevatorHeight = getHeight();
    data.elevatorVelocity = sim.getVelocityMetersPerSecond();
    data.elevatorSetpoint = pid.getGoal().position;
  }

  @Override
  public void setHeight(final double height) {
    pid.setGoal(height);
  }

  @Override
  public double getHeight() {
    return sim.getPositionMeters();
  }

  @Override
  public void setVoltage(final double voltage) {
    this.voltage = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void runVelocity(final double joystick) {
    setHeight(getHeight() + (joystick * ElevatorConstants.maxVelocity * 0.020));
  }
}
