// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.outtake.OuttakeConstants;

/** Add your docs here. */
public class HopperIOTalonFX implements HopperIO {
  private final TalonFX talon;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Temperature> temperature;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  private final VoltageOut voltageControl = new VoltageOut(0.0);

  public HopperIOTalonFX() {
    talon = new TalonFX(30);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        HopperConstants.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = OuttakeConstants.MotorLimits.supply;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    tryUntilOk(5, () -> (talon.getConfigurator().apply(config, 0.25)));

    voltage = talon.getMotorVoltage();
    temperature = talon.getDeviceTemp();
    statorCurrent = talon.getStatorCurrent();
    supplyCurrent = talon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, voltage, temperature, statorCurrent, supplyCurrent);

    talon.optimizeBusUtilization();
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(voltageControl.withOutput(voltage));
  }
}
