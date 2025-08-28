// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.proximity.ProximityDataAutoLogged;
import frc.robot.subsystems.proximity.ProximityIO;

/** Add your docs here. */
public class OuttakeIOTalonFX implements OuttakeIO {
  private final TalonFX talon;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temperature;

  private final VoltageOut voltageControl = new VoltageOut(0.0).withEnableFOC(false);

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private final ProximityIO sensor;
  private final ProximityDataAutoLogged sensorData = new ProximityDataAutoLogged();

  public OuttakeIOTalonFX(ProximityIO sensor) {
    talon = new TalonFX(0); // TODO replace this with the actual hopper id;

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        OuttakeConstants.MotorLimits.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = OuttakeConstants.MotorLimits.supply;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> (talon.getConfigurator().apply(config, 0.25)));

    // Getting Status Signals
    voltage = talon.getMotorVoltage();
    temperature = talon.getDeviceTemp();
    statorCurrent = talon.getStatorCurrent();
    supplyCurrent = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, voltage, temperature, statorCurrent, supplyCurrent);

    talon.optimizeBusUtilization();

    this.sensor = sensor;
  }

  @Override
  public void getData(OuttakeDataAutoLogged data) {
    sensor.getData(sensorData);
    data.connected = connectedDebouncer.calculate(talon.isConnected());

    BaseStatusSignal.refreshAll(voltage, temperature, statorCurrent, supplyCurrent);
    data.voltage = voltage.getValueAsDouble();
    data.temperature = temperature.getValueAsDouble();
    data.statorCurrent = statorCurrent.getValueAsDouble();
    data.supplyCurrent = supplyCurrent.getValueAsDouble();

    data.detected = sensorData.detected;
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(voltageControl.withOutput(voltage));
  }
}
