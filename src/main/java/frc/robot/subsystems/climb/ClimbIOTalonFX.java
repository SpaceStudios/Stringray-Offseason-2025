// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import javax.xml.datatype.DatatypeConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ClimbIOTalonFX implements ClimbIO {
    private final TalonFX talon = new TalonFX(50);

    private final PositionVoltage positionControl = new PositionVoltage(0.0);

    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Angle> position;

    private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    public ClimbIOTalonFX() {

        talonConfig.Slot0.kP = ClimbConstants.PID.kP[0];
        talonConfig.Slot0.kI = ClimbConstants.PID.kI[0];
        talonConfig.Slot0.kD = ClimbConstants.PID.kD[0];

        talonConfig.Slot1.kP = ClimbConstants.PID.kP[1];
        talonConfig.Slot1.kI = ClimbConstants.PID.kI[1];
        talonConfig.Slot1.kD = ClimbConstants.PID.kD[1];

        talonConfig.Slot2.kP = ClimbConstants.PID.kP[2];
        talonConfig.Slot2.kI = ClimbConstants.PID.kI[2];
        talonConfig.Slot2.kD = ClimbConstants.PID.kD[2];

        talonConfig.CurrentLimits.StatorCurrentLimit = ClimbConstants.MotorLimits.statorLimit;
        talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        talonConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.MotorLimits.supplyLimit;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        tryUntilOk(5, () -> (talon.getConfigurator().apply(talonConfig, 0.25)));

        temperature = talon.getDeviceTemp();
        voltage = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getStatorCurrent();
        position = talon.getPosition();

        tryUntilOk(5, () -> (BaseStatusSignal.setUpdateFrequencyForAll(50.0, temperature,voltage,supplyCurrent,statorCurrent,position)));
        tryUntilOk(5, () -> (talon.optimizeBusUtilization()));
    }

    @Override
    public void getData(ClimbDataAutoLogged data) {
        BaseStatusSignal.refreshAll(temperature,voltage,supplyCurrent,statorCurrent,position);

        data.connected = BaseStatusSignal.isAllGood(temperature,voltage,supplyCurrent,statorCurrent,position);
        data.angle = position.getValueAsDouble();
        data.statorCurrent = statorCurrent.getValueAsDouble();
        data.supplyCurrent = supplyCurrent.getValueAsDouble();
        data.voltage = voltage.getValueAsDouble();
        data.temperature = temperature.getValueAsDouble();
    }

    @Override
    public void setAngle(double angle) {
        talon.setControl(positionControl.withPosition(angle));
    }
}
