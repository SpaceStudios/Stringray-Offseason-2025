// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

/** Add your docs here. */
public class OuttakeIOSim implements OuttakeIO {
  private final Debouncer outDebouncer = new Debouncer(0.0, DebounceType.kFalling);
  private boolean detected = false;
  private double voltage = 0.0;

  public OuttakeIOSim() {}

  @Override
  public void getData(OuttakeDataAutoLogged data) {
    data.connected = true;
    data.detected = outDebouncer.calculate(detected);
    data.voltage = voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public void setDetected(boolean detected) {
    System.out.println("IO Detected: " + detected);
    this.detected = detected;
  }
}
