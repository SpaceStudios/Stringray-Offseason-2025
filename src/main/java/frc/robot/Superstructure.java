// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.IntakeLocation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefConstants.algaeTarget;
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
    public Trigger resetGyro;
    public Trigger revFunnel;
    public Trigger dejamCoral;
    public Trigger autoAlignCage;
    public Trigger setPrescoreCoral;
    public Trigger setPrescoreAlgae;
  }

  private coralTarget kCoralTarget = coralTarget.L4;
  private state kCurrentState = state.IDLE;
  private final Debouncer jamesWaitDebouncer = new Debouncer(0.5);
  private boolean shouldAutoRun = false;

  private Pose2d[] testPath =
      AutoAlign.generatePath1Waypoint(
          () -> (FieldConstants.ReefConstants.leftBranches[1]),
          () -> (FieldConstants.SourceConstants.sourcePoses[1]));

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

  public Superstructure(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Hopper hopper,
      Gripper gripper,
      ControllerLayout layout,
      RobotContainer container) {
    for (state kState : state.values()) {
      stateMap.put(
          kState, new Trigger(() -> (kCurrentState == kState) && DriverStation.isEnabled()));
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
        .and(() -> (!outtake.getDetected()))
        .and(stateMap.get(state.ALGAE_INTAKE).negate())
        .onTrue(
            Commands.parallel(
                setState(state.CORAL_INTAKE),
                elevator
                    .setElevatorHeight(FieldConstants.SourceConstants.elevatorSetpoint)
                    .until(elevator::nearSetpoint)));

    layout.revFunnel.whileTrue(hopper.setVoltage(-OuttakeConstants.intake));

    layout
        .scoreRequest
        .and(stateMap.get(state.CORAL_PRESCORE))
        .and(outtake::getDetected)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      this.shouldAutoRun = false;
                    }),
                elevator
                    .setElevatorHeight(() -> (kCoralTarget.height))
                    .until(elevator::nearSetpoint),
                Commands.parallel(
                    outtake
                        .setVoltage(() -> (OuttakeConstants.voltageMap.get(elevator.getSetpoint())))
                        .until(() -> !(outtake.getDetected())),
                    outtake.setDetected(false)),
                this.setState(state.IDLE)));

    layout
        .L1
        .and(stateMap.get(state.CORAL_PRESCORE))
        .onTrue(
            Commands.parallel(
                elevator
                    .setElevatorHeight(() -> (coralTarget.L1.height))
                    .until(elevator::nearSetpoint),
                this.setCoralTarget(coralTarget.L1)));

    layout
        .L2
        .and(stateMap.get(state.CORAL_PRESCORE))
        .onTrue(
            Commands.parallel(
                elevator
                    .setElevatorHeight(() -> (coralTarget.L2.height))
                    .until(elevator::nearSetpoint),
                this.setCoralTarget(coralTarget.L2)));

    layout
        .L3
        .and(stateMap.get(state.CORAL_PRESCORE))
        .onTrue(
            Commands.parallel(
                elevator
                    .setElevatorHeight(() -> (coralTarget.L3.height))
                    .until(elevator::nearSetpoint),
                this.setCoralTarget(coralTarget.L3)));

    layout
        .L4
        .and(stateMap.get(state.CORAL_PRESCORE))
        .onTrue(
            Commands.parallel(
                elevator
                    .setElevatorHeight(() -> (coralTarget.L4.height))
                    .until(elevator::nearSetpoint),
                this.setCoralTarget(coralTarget.L4)));

    // Auto Align
    layout
        .autoAlignLeft
        .or(layout.autoAlignRight)
        .and(stateMap.get(state.CORAL_READY).or(stateMap.get(state.CORAL_PRESCORE)))
        .whileTrue(
            AutoAlign.alignToPose(
                    () ->
                        (layout.autoAlignLeft.getAsBoolean()
                            ? AutoAlign.getBestLeftBranch(drive::getPose)
                            : AutoAlign.getBestRightBranch(drive::getPose)),
                    drive)
                .andThen(this.setState(state.CORAL_PRESCORE)));

    // Cancel
    layout.cancelRequest.onTrue(
        Commands.parallel(
            hopper.setVoltage(0.0),
            outtake.setVoltage(() -> (0.0)),
            elevator.setElevatorHeight(0.0),
            this.setState(state.IDLE)));

    // Coral State Triggers
    stateMap
        .get(state.CORAL_INTAKE)
        .onTrue(
            Commands.parallel(
                hopper.setVoltage(OuttakeConstants.intake).until(outtake::getDetected),
                outtake.setVoltage(() -> OuttakeConstants.intake).until(outtake::getDetected)));

    stateMap
        .get(state.CORAL_INTAKE)
        .and(layout.intakeRequest)
        .whileTrue(AutoAlign.alignToPose(() -> (AutoAlign.getBestSource(drive.getPose())), drive));

    stateMap
        .get(state.CORAL_INTAKE)
        .and(outtake::getDetected)
        .onTrue(
            Commands.parallel(
                this.setState(state.CORAL_READY), container.controllerRumble(5.0, 0.5)));

    stateMap
        .get(state.CORAL_READY)
        .and(
            () ->
                (AutoAlign.isNear(AutoAlign.getBestLeftBranch(drive::getPose), drive.getPose())
                    || AutoAlign.isNear(
                        AutoAlign.getBestRightBranch(drive::getPose), drive.getPose())))
        .onTrue(this.setState(state.CORAL_PRESCORE));

    stateMap
        .get(state.CORAL_PRESCORE)
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.shouldAutoRun = true;
                }));

    stateMap
        .get(state.CORAL_PRESCORE)
        .and(
            () -> (jamesWaitDebouncer.calculate(stateMap.get(state.CORAL_PRESCORE).getAsBoolean())))
        .onTrue(new PrintCommand("You are too slow"));

    layout
        .setPrescoreCoral
        .and(stateMap.get(state.CORAL_READY))
        .and(outtake::getDetected)
        .onTrue(this.setState(state.CORAL_PRESCORE));

    // Algae
    stateMap.get(state.ALGAE_INTAKE).onTrue(Commands.print("Hello"));
    stateMap.get(state.ALGAE_INTAKE).and(() -> true).onTrue(this.setState(state.ALGAE_READY));

    layout
        .L2
        .and(stateMap.get(state.CORAL_PRESCORE).negate())
        .and(() -> !(outtake.getDetected()))
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .onTrue(
            Commands.parallel(
                this.setState(state.ALGAE_INTAKE),
                elevator.setElevatorHeight(() -> (algaeTarget.L2.height))));

    layout
        .L3
        .and(stateMap.get(state.CORAL_PRESCORE).negate())
        .and(() -> !(outtake.getDetected()))
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .onTrue(
            Commands.parallel(
                this.setState(state.ALGAE_INTAKE),
                elevator.setElevatorHeight(() -> (algaeTarget.L3.height))));

    layout
        .intakeRequest
        .and(stateMap.get(state.ALGAE_INTAKE))
        .whileTrue(gripper.setVoltage(() -> (5.0)));

    // Idle State Triggers
    stateMap.get(state.IDLE).and(outtake::getDetected).onTrue(this.setState(state.CORAL_READY));

    // Sim State Triggers
    stateMap
        .get(state.CORAL_INTAKE)
        .and(
            () ->
                (AutoAlign.isNearWaypoint(
                    AutoAlign.getBestSource(drive.getPose()), drive.getPose())))
        .onTrue(outtake.setDetected(true));

    // Non State Stuff
    layout.resetGyro.onTrue(
        Commands.runOnce(
            () -> {
              drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
            }));
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
          if (kCurrentState != newState) {
            System.out.println(newState.toString());
          }
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

    Logger.recordOutput("Superstructure/State", kCurrentState);

    Logger.recordOutput("Auto Align/Waypoints", AutoAlign.waypointsLogged);
    Logger.recordOutput("Auto Align/Generated Path", testPath);
    // Logger.recordOutput("Superstructure/Layout/Cancel Request",
    // layout.cancelRequest.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
  }
}
