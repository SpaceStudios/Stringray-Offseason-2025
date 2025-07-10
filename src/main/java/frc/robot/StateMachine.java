// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.elevator.ElevatorConstants.getNearestAlgaeHeight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Outtake;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

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

  @AutoLogOutput(key="State Machine/Score Request")
  private final Trigger scoreRequest;
  @AutoLogOutput(key="State Machine/Intake Request")
  private final Trigger intakeRequest;
  @AutoLogOutput(key="State Machine/Target L1")
  private final Trigger targetL1; // Target L1 or Intake
  @AutoLogOutput(key="State Machine/Target L2")
  private final Trigger targetL2; // Target L2 or Processor
  @AutoLogOutput(key="State Machine/Target L3")
  private final Trigger targetL3; // Target L3 or Algae Intake Position
  @AutoLogOutput(key="State Machine/Target L4")
  private final Trigger targetL4; // Target L4 or Net
  @AutoLogOutput(key="State Machine/Up")
  private final Trigger up;
  @AutoLogOutput(key="State Machine/Down")
  private final Trigger down;
  @AutoLogOutput(key="State Machine/Left")
  private final Trigger left;
  @AutoLogOutput(key="State Machine/Right")
  private final Trigger right;
  @AutoLogOutput(key="State Machine/Reverse Intake")
  private final Trigger reverseIntake;
  @AutoLogOutput(key="State Machine/Manual Elevator")
  private final Trigger manualElevator;

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
      Trigger manualElevator,
      Drive drive,
      Elevator elevator,
      Grabber grabber,
      Outtake outtake,
      Intake intake) {
    for (robotState state : robotState.values()) {
      stateMap.put(state, new Trigger(() -> (currentState == state) && DriverStation.isEnabled()));
    }

    for (gamepieceState state : gamepieceState.values()) {
      coralStateMap.put(
          state, new Trigger(() -> (kCoralState == state) && DriverStation.isEnabled()));
      algaeStateMap.put(
          state, new Trigger(() -> (kAlgaeState == state) && DriverStation.isEnabled()));
    }

    intakeRequest
        .and(algaeStateMap.get(gamepieceState.INTAKING))
        .and(() -> !(drive.nearSource()))
        .whileTrue(
            Commands.parallel(grabber.setVoltage(6), setAlgaeState(gamepieceState.INTAKING)));

    intakeRequest
        .and(coralStateMap.get(gamepieceState.IDLE))
        .and(drive::nearSource)
        .whileTrue(
            Commands.parallel(
                intake.setVolts(6), 
                outtake.setVolts(6), 
                setCoralState(gamepieceState.INTAKING)));

    intakeRequest
        .negate()
        .and(coralStateMap.get(gamepieceState.INTAKING))
        .and(outtake::hasCoral)
        .onTrue(setCoralState(gamepieceState.READY));

    intakeRequest
        .negate()
        .and(algaeStateMap.get(gamepieceState.INTAKING))
        .and(grabber::hasAlgae)
        .onTrue(setAlgaeState(gamepieceState.READY));

    targetL1
      .and(drive::nearSource)
      .and(coralStateMap.get(gamepieceState.IDLE))
      .and(manualElevator.negate())
      .onTrue(elevator.setHeight(ElevatorConstants.Intake));
    
    targetL1
      .and(() -> !(drive.nearSource()))
      .and(manualElevator.negate())
      .onTrue(elevator.setHeight(ElevatorConstants.L1));
    
    targetL2
      .and(coralStateMap.get(gamepieceState.READY))
      .and(() -> !(drive.nearProcessor()))
      .and(manualElevator.negate())
      .onTrue(
        elevator.setHeight(ElevatorConstants.L2));

    targetL3
      .and(coralStateMap.get(gamepieceState.IDLE).or(() -> !(outtake.hasCoral())))
      .and(algaeStateMap.get(gamepieceState.IDLE).or(() -> !(grabber.hasAlgae())))
      .and(manualElevator.negate())
      .onTrue(
        Commands.parallel(
          elevator.setHeight(getNearestAlgaeHeight(drive.getPose())),
          setAlgaeState(gamepieceState.INTAKING)));
    
    targetL3
      .and(coralStateMap.get(gamepieceState.READY))
      .and(manualElevator.negate())
      .onTrue(
        elevator.setHeight(ElevatorConstants.L3));
    
    targetL4
      .and(coralStateMap.get(gamepieceState.READY))
      .and(() -> !(drive.nearNet()))
      .and(manualElevator.negate())
      .onTrue(
        elevator.setHeight(ElevatorConstants.L4));
    
    scoreRequest
      .and(coralStateMap.get(gamepieceState.READY))
      .and(elevator::atSetpoint)
      .and(() -> (elevator.getSetpoint() == ElevatorConstants.L1) || (elevator.getSetpoint() == ElevatorConstants.L2) || (elevator.getSetpoint() == ElevatorConstants.L3) || (elevator.getSetpoint() == ElevatorConstants.L4))
      .onTrue(
        Commands.parallel(
          outtake.setVolts(6),
          setCoralState(gamepieceState.IDLE)
        ));
    
    scoreRequest
      .and(algaeStateMap.get(gamepieceState.READY))
      .and(elevator::atSetpoint)
      .and(drive::nearNet)
      .onTrue(
        Commands.parallel(
          grabber.setVoltage(6),
          setAlgaeState(gamepieceState.IDLE)
        ));
    
    scoreRequest
      .and(algaeStateMap.get(gamepieceState.READY))
      .and(elevator::atSetpoint)
      .and(drive::nearProcessor)
      .onTrue(
        Commands.parallel(
          grabber.setVoltage(6),
          setAlgaeState(gamepieceState.IDLE)
        ));

    stateMap
    .get(robotState.IDLE)
    .onTrue(Commands.parallel(elevator.setHeight(Meters.zero())));

    algaeStateMap
        .get(gamepieceState.IDLE)
        .and(grabber::hasAlgae)
        .onTrue(setAlgaeState(gamepieceState.INTAKING));

    algaeStateMap
        .get(gamepieceState.READY)
        .and(manualElevator.negate())
        .and(drive::nearNet)
        .and(targetL4)
        .onTrue(elevator.setHeight(ElevatorConstants.AN));
    
    algaeStateMap
      .get(gamepieceState.READY)
      .and(manualElevator.negate())
      .and(drive::nearProcessor)
      .and(targetL2)
      .onTrue(
        elevator.setHeight(ElevatorConstants.AP)
      );

    // Logging Triggers
    this.scoreRequest = scoreRequest;
    this.intakeRequest = intakeRequest;
    this.targetL1 = targetL1; // Target L1 or Intake
    this.targetL2 = targetL2; // Target L2 or Processor
    this.targetL3 = targetL3; // Target L3 or Algae Intake Position
    this.targetL4 = targetL4; // Target L4 or Net
    this.up = up;
    this.down = up;
    this.left = left;
    this.right = right;
    this.reverseIntake = reverseIntake;
    this.manualElevator = manualElevator;
  }

  public Command forceState(robotState state) {
    return Commands.runOnce(
        () -> {
          this.currentState = state;
        });
  }

  public Command setAlgaeState(gamepieceState state) {
    return Commands.runOnce(
        () -> {
          this.kAlgaeState = state;
        });
  }

  public Command setCoralState(gamepieceState state) {
    return Commands.runOnce(
        () -> {
          this.kAlgaeState = state;
        });
  }
}
