// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class OuttakeIOTalonFX implements OuttakeIO {
  private final TalonFX talon;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(false);

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public OuttakeIOTalonFX() {
    talon = new TalonFX(0); // TODO replace this with the actual outtake id;

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.TorqueCurrent.PeakForwardTorqueCurrent = OuttakeConstants.MotorLimits.torque;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -OuttakeConstants.MotorLimits.torque;

    config.CurrentLimits.StatorCurrentLimit = OuttakeConstants.MotorLimits.stator;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = OuttakeConstants.MotorLimits.supply;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLowerLimit = OuttakeConstants.MotorLimits.supplyLow;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.0;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = OuttakeConstants.MotorLimits.rampPeriod;

    // Getting Status Signals
    voltage = talon.getMotorVoltage();
    temperature = talon.getDeviceTemp();
    statorCurrent = talon.getStatorCurrent();
    supplyCurrent = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, voltage, temperature, statorCurrent, supplyCurrent);

    talon.optimizeBusUtilization();
  }

  @Override
  public void getData(OuttakeDataAutoLogged data) {
    data.connected = connectedDebouncer.calculate(talon.isConnected());

    data.voltage = voltage.getValueAsDouble();
    data.temperature = temperature.getValueAsDouble();
    data.statorCurrent = statorCurrent.getValueAsDouble();
    data.supplyCurrent = supplyCurrent.getValueAsDouble();
  }
}
