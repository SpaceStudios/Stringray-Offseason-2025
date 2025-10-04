// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

/** Add your docs here. */
public class GripperIOSim implements GripperIO {
  private double voltage = 0.0;

  public GripperIOSim() {}

  @Override
  public void getData(GripperDataAutoLogged data) {
    data.connected = true;
    data.voltage = voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
    // if (voltage > 0 && gripperDetected.get()) {
    //   this.detected = false;
    // }
  }
}
