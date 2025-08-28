// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class OuttakeIOSim implements OuttakeIO {
  private final Debouncer outDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private double lastDetected;
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
    if (detected && voltage > 0 && Timer.getFPGATimestamp() - lastDetected > 1.0) {
      detected = false;
    }
  }

  @Override
  public void setDetected(boolean detected) {
    this.detected = detected;
    lastDetected = Timer.getFPGATimestamp();
  }
}
