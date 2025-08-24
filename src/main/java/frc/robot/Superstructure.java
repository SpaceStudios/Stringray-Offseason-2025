// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.IntakeLocation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefConstants.coralTarget;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

/** Add your docs here. */
public class Superstructure {
  public static class ControllerLayout {
    public DoubleSupplier driveX;
    public DoubleSupplier driveY;
    public DoubleSupplier operatorY;
    public Trigger scoreRequest;
    public Trigger intakeRequest;
    public Trigger manualElevator;
    public Trigger L1;
    public Trigger L2;
    public Trigger L3;
    public Trigger L4;
    public Trigger climbRequest;
    public Trigger climbPull;
    public Trigger autoAlignLeft;
    public Trigger autoAlignRight;
    public Trigger cancelRequest;
  }

  private coralTarget kCoralTarget = coralTarget.L4;
  private state kCurrentState = state.IDLE;
  private boolean overrideLEDs = false;

  public enum state {
    IDLE,
    CORAL_INTAKE,
    CORAL_READY,
    CORAL_PRESCORE,
    ALGAE_INTAKE,
    ALGAE_READY,
    ALGAE_PRESCORE,
    CLIMB_READY,
    CLIMB_PULL,
    MANUAL_ELEVATOR
  }

  private final HashMap<state, Trigger> stateMap = new HashMap<state, Trigger>();
  private final ControllerLayout layout;

  LoggedMechanism2d mech;

  public Superstructure(Drive drive, Elevator elevator, Outtake outtake, ControllerLayout layout) {
    for (state kState : state.values()) {
      stateMap.put(kState, new Trigger(() -> (kCurrentState == kState)));
    }

    this.layout = layout;

    // Setting Up Controls
    layout.L1.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L1));

    layout.L2.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L2));

    layout.L3.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L3));

    layout.L4.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L4));

    layout
        .L1
        .and(layout.manualElevator)
        .onTrue(elevator.setElevatorHeight(coralTarget.L1.height).until(elevator::nearSetpoint));

    layout
        .L2
        .and(layout.manualElevator)
        .onTrue(elevator.setElevatorHeight(coralTarget.L2.height).until(elevator::nearSetpoint));

    layout
        .L3
        .and(layout.manualElevator)
        .onTrue(elevator.setElevatorHeight(coralTarget.L3.height).until(elevator::nearSetpoint));

    layout
        .L4
        .and(layout.manualElevator)
        .and(outtake::getDetected)
        .onTrue(elevator.setElevatorHeight(coralTarget.L4.height).until(elevator::nearSetpoint));

    layout
        .intakeRequest
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.SOURCE))
        .onTrue(
            Commands.parallel(
                setState(state.CORAL_INTAKE),
                elevator.setElevatorHeight(FieldConstants.SourceConstants.elevatorSetpoint)));

    layout
        .scoreRequest
        .and(outtake::getDetected)
        .onTrue(outtake.setVoltage(5).until(() -> (!outtake.getDetected())));

    // Sim State Triggers
    stateMap
        .get(state.CORAL_INTAKE)
        .and(() -> (AutoAlign.isNear(AutoAlign.getBestSource(drive.getPose()), drive.getPose())))
        .onTrue(outtake.setDetected(true));
  }

  public Command setCoralTarget(coralTarget target) {
    return Commands.runOnce(
        () -> {
          this.kCoralTarget = target;
        });
  }

  public Command setState(state newState) {
    return Commands.runOnce(
        () -> {
          System.out.println(newState.toString());
          this.kCurrentState = newState;
        });
  }

  // Logging
  public void periodic() {
    Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
    Logger.recordOutput("Superstructure/Layout/L2", layout.L2.getAsBoolean());
    Logger.recordOutput("Superstructure/Layout/L3", layout.L3.getAsBoolean());
    Logger.recordOutput("Superstructure/Layout/L4", layout.L4.getAsBoolean());
    Logger.recordOutput("Superstructure/Layout/Intake", layout.intakeRequest.getAsBoolean());
    Logger.recordOutput("Superstructure/Layout/Score", layout.scoreRequest.getAsBoolean());
    Logger.recordOutput(
        "Superstructure/Layout/Manual Elevator", layout.manualElevator.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/Cancel Request",
    // layout.cancelRequest.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
  }
}
