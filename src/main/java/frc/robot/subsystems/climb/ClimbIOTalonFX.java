// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimbIOTalonFX {
    private final TalonFX talon = new TalonFX(50);

    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Angle> position;

    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    public ClimbIOTalonFX() {

        tryUntilOk(5, () -> (talon.getConfigurator().apply(talonConfig, 0.25)));

        temperature = talon.getDeviceTemp();
        voltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getStatorCurrent();
        position = talon.getPosition();
    }
}
