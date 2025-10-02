// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.proximity.ProximityDataAutoLogged;
import frc.robot.subsystems.proximity.ProximityIO;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {
  private final GripperIO io;
  private final ProximityIO proximityIO;
  private final GripperDataAutoLogged data = new GripperDataAutoLogged();
  private final ProximityDataAutoLogged proximityData = new ProximityDataAutoLogged();
  /** Creates a new Gripper. */
  public Gripper(GripperIO io, ProximityIO proximityIO) {
    this.io = io;
    this.proximityIO = proximityIO;
  }

  public Command setVoltage(DoubleSupplier volts) {
    return this.run(
            () -> {
              io.setVoltage(volts.getAsDouble());
            })
        .finallyDo(
            () -> {
              io.setVoltage(0.0);
            });
  }

  @AutoLogOutput(key = "Gripper/Detected")
  public boolean getDetected() {
    return proximityData.detected;
  }

  public void setDetected(boolean detected) {
    proximityIO.setDetected(detected);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    proximityIO.getData(proximityData);
    Logger.processInputs("Gripper", data);
  }
}
