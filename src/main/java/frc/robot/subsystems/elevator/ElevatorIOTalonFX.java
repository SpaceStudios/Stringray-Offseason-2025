// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  public ElevatorIOTalonFX() {
    leftTalon = new TalonFX(21);
    rightTalon = new TalonFX(22);

    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));

    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot0.kP = ElevatorConstants.PID.kP[0];
    configuration.Slot0.kP = ElevatorConstants.PID.kI[0];
    configuration.Slot0.kP = ElevatorConstants.PID.kD[0];
    configuration.Slot0.kG = ElevatorConstants.PID.kG[0];
    configuration.Slot0.kS = ElevatorConstants.PID.kS[0];

    configuration.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot1.kP = ElevatorConstants.PID.kP[1];
    configuration.Slot1.kI = ElevatorConstants.PID.kI[1];
    configuration.Slot1.kD = ElevatorConstants.PID.kD[1];
    configuration.Slot1.kG = ElevatorConstants.PID.kG[1];
    configuration.Slot1.kS = ElevatorConstants.PID.kS[1];

    configuration.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot2.kP = ElevatorConstants.PID.kP[2];
    configuration.Slot2.kI = ElevatorConstants.PID.kI[2];
    configuration.Slot2.kD = ElevatorConstants.PID.kD[2];
    configuration.Slot2.kG = ElevatorConstants.PID.kG[2];
    configuration.Slot2.kS = ElevatorConstants.PID.kS[2];

    // Do not put any configuration below this
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(configuration, 0.25));
  }

  @Override
  public void updateData(elevatorDataAutoLogged data) {}

  @Override
  public void setHeight(double height) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void runVelocity(double joystick) {}

  @Override
  public double getHeight() {
    return 0.0;
  }
}
