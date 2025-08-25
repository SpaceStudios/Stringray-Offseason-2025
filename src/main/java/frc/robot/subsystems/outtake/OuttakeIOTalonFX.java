// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class OuttakeIOTalonFX implements OuttakeIO {
    private final TalonFX talon;
    public OuttakeIOTalonFX() {
        talon = new TalonFX(0); // TODO replace this with the actual outtake id;
    }
}
