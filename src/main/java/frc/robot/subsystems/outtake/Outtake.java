// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.proximity.ProximityDataAutoLogged;
import frc.robot.subsystems.proximity.ProximityIO;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */
  private final OuttakeIO io;

  private final ProximityIO proximityIO;

  private final OuttakeDataAutoLogged data = new OuttakeDataAutoLogged();
  private final ProximityDataAutoLogged proximityData = new ProximityDataAutoLogged();

  public Outtake(OuttakeIO io, ProximityIO proximityIO) {
    this.io = io;
    this.proximityIO = proximityIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    proximityIO.getData(proximityData);
    Logger.processInputs("Outtake", data);
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return this.run(
        () -> {
          io.setVoltage(voltage.getAsDouble());
        });
  }

  @AutoLogOutput(key = "Outtake/Detected")
  public boolean getDetected() {
    return proximityData.detected;
  }

  public void setDetected(boolean detected) {
    proximityIO.setDetected(detected);
  }
}
