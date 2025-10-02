// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.proximity;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/** Add your docs here. */
public class ProximityIOSim implements ProximityIO {
  LoggedNetworkBoolean detected;

  public ProximityIOSim(String name) {
    detected = new LoggedNetworkBoolean("/Tuning/" + name, false);
  }

  @Override
  public void getData(ProximityDataAutoLogged data) {
    data.connected = true;

    data.distance = detected.get() ? 0 : 100;
    data.detected = detected.get();

    data.temperature = 0;
  }

  @Override
  public void setDetected(boolean detected) {
    this.detected.set(detected);
  }
}
