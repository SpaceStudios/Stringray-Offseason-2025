// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.gripper.GripperConstants.MotorConstants;
import frc.robot.subsystems.proximity.ProximityDataAutoLogged;
import frc.robot.subsystems.proximity.ProximityIO;

/** Add your docs here. */
public class GripperIOTalonFX implements GripperIO {
  private final TalonFX talon = new TalonFX(40);
  private final ProximityIO sensor;
  private final ProximityDataAutoLogged proximityData = new ProximityDataAutoLogged();
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<AngularVelocity> velocity;

  private final VoltageOut voltageControl = new VoltageOut(0.0);

  public GripperIOTalonFX(ProximityIO sensor) {
    this.sensor = sensor;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = MotorConstants.statorLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = MotorConstants.supplyLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode =
        MotorConstants.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        MotorConstants.inverted
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    tryUntilOk(5, () -> (talon.getConfigurator().apply(config, 0.25)));

    voltage = talon.getMotorVoltage();
    temperature = talon.getDeviceTemp();
    statorCurrent = talon.getStatorCurrent();
    supplyCurrent = talon.getSupplyCurrent();
    velocity = talon.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, voltage, temperature, statorCurrent, supplyCurrent, velocity);

    talon.optimizeBusUtilization();
  }

  @Override
  public void getData(gripperDataAutoLogged data) {
    StatusCode status =
        BaseStatusSignal.refreshAll(voltage, temperature, statorCurrent, supplyCurrent, velocity);
    sensor.getData(proximityData);

    data.connected = status.isOK();
    data.detected = proximityData.detected;
    data.statorCurrent = statorCurrent.getValueAsDouble();
    data.supplyCurrent = supplyCurrent.getValueAsDouble();
    data.voltage = voltage.getValueAsDouble();
    data.temperature = temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageControl.withOutput(volts));
  }
}
