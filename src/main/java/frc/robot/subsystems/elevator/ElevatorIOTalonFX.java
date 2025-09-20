// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {

  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final StatusSignal<Voltage> followerVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Temperature> followerTemperature;

  private final Debouncer leftConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer rightConnectedDebouncer = new Debouncer(0.5);

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage positionTorque =
      new MotionMagicVoltage(0.0).withEnableFOC(false);

  private double setpoint = 0.0;

  public ElevatorIOTalonFX() {
    leftTalon = new TalonFX(11);
    rightTalon = new TalonFX(10);

    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));

    final TalonFXConfiguration configuration = new TalonFXConfiguration();

    configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot0.kP = ElevatorConstants.PID.kP[0];
    configuration.Slot0.kP = ElevatorConstants.PID.kI[0];
    configuration.Slot0.kP = ElevatorConstants.PID.kD[0];
    configuration.Slot0.kG = ElevatorConstants.PID.kG[0];
    configuration.Slot0.kS = ElevatorConstants.PID.kS[0];
    configuration.Slot0.kV = ElevatorConstants.PID.kV;
    configuration.Slot0.kA = ElevatorConstants.PID.kA;

    configuration.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot1.kP = ElevatorConstants.PID.kP[1];
    configuration.Slot1.kI = ElevatorConstants.PID.kI[1];
    configuration.Slot1.kD = ElevatorConstants.PID.kD[1];
    configuration.Slot1.kG = ElevatorConstants.PID.kG[1];
    configuration.Slot1.kS = ElevatorConstants.PID.kS[1];
    configuration.Slot1.kV = ElevatorConstants.PID.kV;
    configuration.Slot1.kA = ElevatorConstants.PID.kA;

    configuration.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    configuration.Slot2.kP = ElevatorConstants.PID.kP[2];
    configuration.Slot2.kI = ElevatorConstants.PID.kI[2];
    configuration.Slot2.kD = ElevatorConstants.PID.kD[2];
    configuration.Slot2.kG = ElevatorConstants.PID.kG[2];
    configuration.Slot2.kS = ElevatorConstants.PID.kS[2];
    configuration.Slot2.kV = ElevatorConstants.PID.kV;
    configuration.Slot2.kA = ElevatorConstants.PID.kA;

    configuration.Feedback.SensorToMechanismRatio =
        ElevatorConstants.gearing / (ElevatorConstants.drumRadius * Math.PI * 2);

    configuration.TorqueCurrent.PeakForwardTorqueCurrent =
        ElevatorConstants.MotorConstants.maxCurrent;
    configuration.TorqueCurrent.PeakReverseTorqueCurrent =
        -ElevatorConstants.MotorConstants.maxCurrent;

    configuration.CurrentLimits.StatorCurrentLimit = ElevatorConstants.MotorConstants.maxCurrent;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;

    configuration.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.MotorConstants.maxCurrent;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

    configuration.CurrentLimits.SupplyCurrentLowerLimit =
        ElevatorConstants.MotorConstants.maxCurrentLow;
    configuration.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    configuration.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        ElevatorConstants.MotorConstants.rampPeriod;

    configuration.MotionMagic.MotionMagicAcceleration = ElevatorConstants.PID.maxAcceleration;
    configuration.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.PID.maxVelocity;

    configuration.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.PID.kMM_Expo_kA;
    configuration.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.PID.kMM_Expo_kV;

    // Do not put any configuration below this
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(configuration, 0.25));

    position = leftTalon.getPosition();
    velocity = leftTalon.getVelocity();
    voltage = leftTalon.getMotorVoltage();
    statorCurrent = leftTalon.getStatorCurrent();
    supplyCurrent = leftTalon.getSupplyCurrent();
    temperature = leftTalon.getDeviceTemp();

    followerVoltage = rightTalon.getMotorVoltage();
    followerStatorCurrent = rightTalon.getStatorCurrent();
    followerSupplyCurrent = rightTalon.getStatorCurrent();
    followerTemperature = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        velocity,
        voltage,
        statorCurrent,
        supplyCurrent,
        temperature,
        followerVoltage,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemperature);

    leftTalon.optimizeBusUtilization();
    rightTalon.optimizeBusUtilization();
  }

  @Override
  public void updateData(elevatorDataAutoLogged data) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        voltage,
        statorCurrent,
        supplyCurrent,
        temperature,
        followerVoltage,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemperature);

    data.elevatorHeight = position.getValueAsDouble();
    data.elevatorVelocity = velocity.getValueAsDouble();
    data.elevatorSetpoint = setpoint;

    data.connected =
        leftConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(voltage, statorCurrent, supplyCurrent, temperature));
    data.followerConnected =
        rightConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerVoltage,
                followerStatorCurrent,
                followerSupplyCurrent,
                followerTemperature));

    data.motorPosition = position.getValueAsDouble();
    data.motorVelocity = velocity.getValueAsDouble();
    data.motorVoltage = voltage.getValueAsDouble();
    data.motorOutput = leftTalon.get();
    data.motorCurrent = statorCurrent.getValueAsDouble();
    data.motorSupplyCurrent = supplyCurrent.getValueAsDouble();
    data.motorTemperature = temperature.getValueAsDouble();

    data.followerCurrent = followerStatorCurrent.getValueAsDouble();
    data.followerSupplyCurrent = followerSupplyCurrent.getValueAsDouble();
    data.followerOutput = rightTalon.get();
    data.followerTemperature = followerTemperature.getValueAsDouble();
    data.followerVoltage = followerVoltage.getValueAsDouble();
  }

  @Override
  public void setHeight(double height) {
    setpoint = height;
    leftTalon.setControl(positionTorque.withPosition(height));
  }

  @Override
  public void setVoltage(double voltage) {
    leftTalon.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void runVelocity(double joystick) {}

  @Override
  public void resetEncoders() {
    leftTalon.setPosition(0.0);
    rightTalon.setPosition(0.0);
  }

  @Override
  public double getHeight() {
    return position.getValueAsDouble();
  }
}
