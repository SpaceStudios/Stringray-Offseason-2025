// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

/** Add your docs here. */
public class HopperIOSim implements HopperIO {
  private double voltage = 0.0;

  public HopperIOSim() {}

  @Override
  public void getData(HopperDataAutoLogged data) {
    data.connected = true;
    data.voltage = voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }
}
