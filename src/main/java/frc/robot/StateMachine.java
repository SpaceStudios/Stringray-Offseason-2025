// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class StateMachine {
  public static enum robotState {
    IDLE,
    INTAKE_REVERSE,
    CLIMB_DEPLOY,
    CLIMB_START,
    PREFORMING_ACTION
  }

  public static enum gamepieceState {
    INTAKING,
    HELD,
    READY,
    IDLE
  }

  private final HashMap<robotState, Trigger> stateMap = new HashMap<robotState, Trigger>();
  private final HashMap<gamepieceState, Trigger> coralStateMap =
      new HashMap<gamepieceState, Trigger>();
  private final HashMap<gamepieceState, Trigger> algaeStateMap =
      new HashMap<gamepieceState, Trigger>();

  private robotState currentState;
  private gamepieceState kCoralState;
  private gamepieceState kAlgaeState;

  public StateMachine(
      DoubleSupplier driveX,
      DoubleSupplier driveY,
      Trigger scoreRequest,
      Trigger intakeRequest,
      Trigger targetL1, // Target L1 or Intake
      Trigger targetL2, // Target L2 or Processor
      Trigger targetL3, // Target L3 or Algae Intake Position
      Trigger targetL4, // Target L4 or Net
      Trigger up,
      Trigger down,
      Trigger left,
      Trigger right,
      Trigger reverseIntake,
      Elevator elevator,
      Drive drive) {
    for (robotState state : robotState.values()) {
      stateMap.put(state, new Trigger(() -> (currentState == state) && DriverStation.isEnabled()));
    }

    for (gamepieceState state : gamepieceState.values()) {
      coralStateMap.put(
          state, new Trigger(() -> (kCoralState == state) && DriverStation.isEnabled()));
      algaeStateMap.put(
          state, new Trigger(() -> (kAlgaeState == state) && DriverStation.isEnabled()));
    }

    stateMap.get(robotState.IDLE).onTrue(Commands.parallel(elevator.setHeight(Meters.zero())));
  }

  public Command forceState(robotState state) {
    return Commands.runOnce(
        () -> {
          this.currentState = state;
        });
  }
}
