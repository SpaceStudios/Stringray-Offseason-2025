// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

/** Add your docs here. */
public class GripperIOSim implements GripperIO {
  private boolean detected = false;
  private double voltage = 0.0;

  private final Debouncer voltageOutDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  public GripperIOSim() {}

  @Override
  public void getData(gripperDataAutoLogged data) {
    data.detected = voltageOutDebouncer.calculate(detected);

    data.connected = true;
    data.voltage = voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
    if (voltage > 0 && detected) {
      detected = false;
    }
  }

  @Override
  public void setDetected(boolean detected) {
    this.detected = detected;
  }
}
