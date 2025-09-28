// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Superstructure;
import frc.robot.Superstructure.State;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;

/** Some Preset Autos */
public class Autos {
  public static Command DoubleL4(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Hopper hopper,
      Superstructure superstructure) {
    return Commands.sequence(
        simInit(outtake, true),
        AutoRoutines.runTrajectory("aCtoG"),
        // scoreCoral(() -> (coralTarget.L4), elevator, outtake),
        AutoRoutines.runTrajectory("GtoS"));
  }
  // public static Command scoreCoral(
  //     Supplier<coralTarget> targetSupplier, Elevator elevator, Outtake outtake) {
  //   return Commands.sequence(
  //       // elevator
  //       //     .setElevatorHeight(() -> (targetSupplier.get().height))
  //       //     .until(elevator::nearSetpoint),
  //       // outtake
  //           .setVoltage(() -> (OuttakeConstants.voltageMap.get(targetSupplier.get().height)))
  //           .until(() -> !(outtake.getDetected()))));
  //       // elevator.setElevatorHeight(0.0).until(elevator::nearSetpoint));
  // }

  public static Command intakeCoral(Superstructure superstructure, Outtake outtake, Hopper hopper) {
    return Commands.parallel(
            superstructure.setState(State.CORAL_INTAKE),
            outtake.setVoltage(() -> (OuttakeConstants.intake)),
            hopper.setVoltage(OuttakeConstants.intake))
        .until(outtake::getDetected);
  }

  public static Command simInit(Outtake outtake, boolean preload) {
    return Commands.run(
            () -> {
              if (Robot.isSimulation()) {
                outtake.setDetectedFunction(preload);
                System.out.println(preload ? "Coral Preloaded" : "Coral Was Not Preloaded");
              }
            })
        .until(() -> ((outtake.getDetected() == preload) || Robot.isReal()))
        .finallyDo(
            () -> {
              System.out.println("Determined Preload");
            });
  }
}
