// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** Add your docs here. */
public class OuttakeIOSim implements OuttakeIO {
  private final Debouncer outDebouncer = new Debouncer(0.0, DebounceType.kFalling);
  private double voltage = 0.0;

  private LoggedNetworkBoolean isDetected =
      new LoggedNetworkBoolean("/Tuning/CoralDetected", false);

  public OuttakeIOSim() {}

  @Override
  public void getData(OuttakeDataAutoLogged data) {
    data.connected = true;
    data.detected = isDetected.get();
    data.voltage = voltage;
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

  @Override
  public void setDetected(boolean detected) {
    isDetected.set(detected);
  }
}
