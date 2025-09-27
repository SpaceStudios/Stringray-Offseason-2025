// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlign.IntakeLocation;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOCandle;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.BargeConstants;
import frc.robot.util.FieldConstants.ReefConstants;
import frc.robot.util.FieldConstants.ReefConstants.coralTarget;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

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
    public Trigger autoAlignLeft;
    public Trigger autoAlignRight;
    public Trigger cancelRequest;
    public Trigger resetGyro;
    public Trigger revFunnel;
    public Trigger dejamCoral;
    public Trigger autoAlignCage;
    public Trigger setPrescoreCoral;
    public Trigger setPrescoreAlgae;
    public CommandXboxController driveController;
    public CommandXboxController operatorController;
  }

  private coralTarget kCoralTarget = coralTarget.L4;
  private state kCurrentState = state.IDLE;
  private final Debouncer jamesWaitDebouncer = new Debouncer(0.5);

  private final Command autoElevatorCommand;

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

  private final LED led = new LED(Robot.isReal() ? new LEDIOCandle() : new LEDIO() {});
  private final Elevator elevator;

  private final LoggedMechanism2d mech;
  public final LoggedMechanismLigament2d elevatorDisplay;

  @AutoLogOutput(key = "Superstructure/Sim Intake")
  private Trigger simIntakeTrigger = new Trigger(() -> false);

  public Superstructure(
      Drive drive,
      Elevator elevator,
      Outtake outtake,
      Hopper hopper,
      Gripper gripper,
      Climb climb,
      ControllerLayout layout) {
    for (state kState : state.values()) {
      stateMap.put(
          kState, new Trigger(() -> (kCurrentState == kState) && DriverStation.isEnabled()));
    }

    this.autoElevatorCommand =
        (Commands.sequence(
            elevator.setElevatorHeight(() -> (kCoralTarget.height)).until(elevator::nearSetpoint),
            Commands.parallel(
                outtake
                    .setVoltage(() -> (OuttakeConstants.voltageMap.get(elevator.getSetpoint())))
                    .until(() -> !(outtake.getDetected())),
                outtake.setDetected(false)),
            this.setState(state.IDLE)));

    this.layout = layout;
    this.elevator = elevator;

    // Setting Up Display
    mech = new LoggedMechanism2d(1, 1);
    elevatorDisplay = new LoggedMechanismLigament2d("ElevatorMechanism", 0, 85);
    mech.getRoot("ElevatorRoot", Units.inchesToMeters(24 - 3), Units.inchesToMeters(4.087))
        .append(elevatorDisplay);

    // Setting Up Controls
    layout.L1.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L1));

    layout.L2.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L2));

    layout.L3.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L3));

    layout.L4.and(layout.manualElevator.negate()).onTrue(this.setCoralTarget(coralTarget.L4));

    // Manual Elevator Setpoints
    layout
        .L1
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .onTrue(elevator.setElevatorHeight(coralTarget.L1.height).until(elevator::nearSetpoint));

    layout
        .L2
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .onTrue(elevator.setElevatorHeight(coralTarget.L2.height).until(elevator::nearSetpoint));

    layout
        .L3
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .onTrue(elevator.setElevatorHeight(coralTarget.L3.height).until(elevator::nearSetpoint));

    // layout
    //     .L4
    //     .and(stateMap.get(state.MANUAL_ELEVATOR))
    //     .onTrue(elevator.setElevatorHeight(coralTarget.L4.height).until(elevator::nearSetpoint));

    layout
        .L4
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .onTrue(elevator.setElevatorHeight(coralTarget.L4.height).withTimeout(10.0));

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

    // Coral State Triggers
    layout
        .intakeRequest
        .and(() -> (!outtake.getDetected()))
        .and(() -> (!gripper.getDetected()))
        .and(stateMap.get(state.ALGAE_INTAKE).negate())
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.SOURCE))
        .onTrue(
            Commands.parallel(
                setState(state.CORAL_INTAKE),
                elevator
                    .setElevatorHeight(FieldConstants.SourceConstants.elevatorSetpoint)
                    .until(elevator::nearSetpoint)));

    stateMap
        .get(state.CORAL_INTAKE)
        .whileTrue(
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
        .whileTrue(Commands.parallel(this.setState(state.CORAL_READY), new PrintCommand("Intaked")))
        .onFalse(this.rumbleCommand(layout.driveController, 5.0, 1.0));

    stateMap
        .get(state.CORAL_READY)
        .and(
            () ->
                (AutoAlign.isNear(AutoAlign.getBestLeftBranch(drive::getPose), drive.getPose())
                    || AutoAlign.isNear(
                        AutoAlign.getBestRightBranch(drive::getPose), drive.getPose())))
        .onTrue(this.setState(state.CORAL_PRESCORE));

    stateMap.get(state.CORAL_READY).and(layout.scoreRequest).whileTrue(outtake.setVoltage(() -> 6));

    // Auto Scoring, Disabled for now
    // stateMap
    //     .get(state.CORAL_PRESCORE)
    //     .and(
    //         () ->
    //
    // (jamesWaitDebouncer.calculate(stateMap.get(state.CORAL_PRESCORE).getAsBoolean())))
    //     .onTrue(Commands.parallel(new PrintCommand("You are too slow"), autoElevatorCommand));

    // Manual Coral Scoring
    layout
        .scoreRequest
        .and(stateMap.get(state.CORAL_PRESCORE))
        .and(outtake::getDetected)
        .and(() -> (kCoralTarget != coralTarget.L1))
        .onTrue(autoElevatorCommand);

    layout
        .scoreRequest
        .and(outtake::getDetected)
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .and(() -> (kCoralTarget == coralTarget.L1))
        .onTrue(
            Commands.sequence(
                elevator.setElevatorHeight(coralTarget.L1).until(elevator::nearSetpoint),
                Commands.parallel(
                        outtake.setVoltage(() -> (OuttakeConstants.L1)),
                        elevator.setVoltage(() -> (0.5)))
                    .until(() -> !(outtake.getDetected())),
                elevator.setElevatorHeight(() -> 0.0)));

    layout
        .setPrescoreCoral
        .and(stateMap.get(state.CORAL_READY))
        .and(outtake::getDetected)
        .onTrue(this.setState(state.CORAL_PRESCORE));

    stateMap
        .get(state.CORAL_READY)
        .whileTrue(
            Commands.parallel(
                this.rumbleCommand(layout.driveController, 0.5, 1.0),
                elevator.setElevatorHeight(coralTarget.L1).until(elevator::nearSetpoint)));

    // Elevator goes to level when level has been selected and it is in the prescore.
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

    // Algae

    // stateMap
    //     .get(state.ALGAE_INTAKE)
    //     .and(elevator::nearSetpoint)
    //     .and(() -> (elevator.getSetpoint() > 0))
    //     .whileTrue(gripper.setVoltage(() -> (GripperConstants.intake)));

    stateMap.get(state.ALGAE_INTAKE).whileTrue(gripper.setVoltage(() -> (GripperConstants.intake)));

    layout
        .intakeRequest
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .and(stateMap.get(state.IDLE))
        .onTrue(this.setState(state.ALGAE_INTAKE));

    layout
        .L2
        .and(stateMap.get(state.IDLE))
        .and(() -> (!outtake.getDetected()))
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .onTrue(
            Commands.parallel(
                this.setState(state.ALGAE_INTAKE),
                elevator.setElevatorHeight(
                    () -> (FieldConstants.ReefConstants.algaeTarget.L2.height))));

    layout
        .L3
        .and(stateMap.get(state.IDLE))
        .and(() -> (!outtake.getDetected()))
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .onTrue(
            Commands.parallel(
                this.setState(state.ALGAE_INTAKE),
                elevator.setElevatorHeight(
                    () -> (FieldConstants.ReefConstants.algaeTarget.L3.height))));

    layout
        .L2
        .and(stateMap.get(state.ALGAE_INTAKE))
        .onTrue(
            elevator.setElevatorHeight(() -> FieldConstants.ReefConstants.algaeTarget.L2.height));

    layout
        .L3
        .and(stateMap.get(state.ALGAE_INTAKE))
        .onTrue(
            elevator.setElevatorHeight(() -> FieldConstants.ReefConstants.algaeTarget.L3.height));

    stateMap
        .get(state.ALGAE_INTAKE)
        .and(layout.intakeRequest)
        .whileTrue(
            AutoAlign.alignToPose(() -> (AutoAlign.getBestAlgaePose(drive::getPose)), drive));

    stateMap
        .get(state.ALGAE_INTAKE)
        .and(gripper::getDetected)
        .whileTrue(this.setState(state.ALGAE_READY));

    stateMap
        .get(state.ALGAE_READY)
        .and(() -> !(gripper.getDetected()))
        .whileTrue(this.setState(state.IDLE));

    stateMap
        .get(state.ALGAE_PRESCORE)
        .and(() -> !(gripper.getDetected()))
        .whileTrue(this.setState(state.IDLE));

    stateMap
        .get(state.ALGAE_READY)
        .and(() -> (BargeConstants.nearNet(drive::getPose)))
        .whileTrue(this.setState(state.ALGAE_PRESCORE));

    stateMap
        .get(state.ALGAE_PRESCORE)
        .and(() -> !(BargeConstants.nearNet(drive::getPose)))
        .whileTrue(this.setState(state.ALGAE_READY))
        .onTrue(
            elevator
                .setElevatorHeight(ReefConstants.coralTarget.L1.height)
                .until(elevator::nearSetpoint));

    layout
        .scoreRequest
        .and(stateMap.get(state.ALGAE_PRESCORE))
        .and(gripper::getDetected)
        .onTrue(
            Commands.sequence(
                elevator
                    .setElevatorHeight(() -> (BargeConstants.elevatorSetpoint))
                    .until(elevator::nearSetpoint),
                Commands.parallel(
                        gripper.setVoltage(() -> (GripperConstants.net)),
                        gripper.setDetected(false))
                    .until(() -> !(gripper.getDetected())),
                this.setState(state.IDLE)));

    stateMap
        .get(state.ALGAE_READY)
        .whileTrue(
            Commands.parallel(
                this.rumbleCommand(layout.driveController, 0.25, 1.0),
                elevator.setElevatorHeight(coralTarget.L1).until(elevator::nearSetpoint)));

    // Climb
    layout
        .climbRequest
        .and(() -> (Timer.getMatchTime() == -1))
        .whileTrue(Commands.parallel(this.setState(state.CLIMB_READY), climb.extend()));

    layout
        .autoAlignCage
        .and(stateMap.get(state.CLIMB_READY))
        .and(() -> (Timer.getMatchTime() < 25))
        .whileTrue(AutoAlign.alignToPose(() -> (AutoAlign.getBestCagePose(drive::getPose)), drive));

    layout
        .scoreRequest
        .and(stateMap.get(state.CLIMB_PULL))
        .and(() -> (Timer.getMatchTime() < 25))
        .whileTrue(climb.retract());

    // Idle State Triggers
    stateMap.get(state.IDLE).and(outtake::getDetected).whileTrue(this.setState(state.CORAL_READY));
    stateMap.get(state.IDLE).and(gripper::getDetected).whileTrue(this.setState(state.ALGAE_READY));

    // Sim State Triggers
    stateMap
        .get(state.CORAL_INTAKE)
        .and(elevator::nearSetpoint)
        .and(Robot::isSimulation)
        .and(
            () ->
                (AutoAlign.isNearWaypoint(
                    AutoAlign.getBestSource(drive.getPose()), drive.getPose())))
        .onTrue(Commands.parallel(outtake.setDetected(true), new PrintCommand("Intaked")));

    stateMap
        .get(state.ALGAE_INTAKE)
        .and(Robot::isSimulation)
        .and(() -> (AutoAlign.isNear(AutoAlign.getBestAlgaePose(drive::getPose), drive.getPose())))
        .onTrue(Commands.parallel(gripper.setDetected(true).until(gripper::getDetected)));

    simIntakeTrigger =
        stateMap
            .get(state.CORAL_INTAKE)
            .and(Robot::isSimulation)
            .and(
                () ->
                    (AutoAlign.isNearWaypoint(
                        AutoAlign.getBestSource(drive.getPose()), drive.getPose())));

    simIntakeTrigger.onTrue(outtake.setDetected(true).until(outtake::getDetected));

    // Non State Stuff
    layout
        .cancelRequest
        .and(() -> !(gripper.getDetected()))
        .and(() -> !(outtake.getDetected()))
        .onTrue(
            Commands.parallel(
                this.setState(state.IDLE),
                outtake.setVoltage(() -> 0.0).withTimeout(0.1),
                hopper.setVoltage(0.0).withTimeout(0.1),
                gripper.setVoltage(() -> 0.0).withTimeout(0.1),
                elevator.setElevatorHeight(0.0).until(elevator::nearSetpoint)));

    layout
        .cancelRequest
        .and(outtake::getDetected)
        .onTrue(
            Commands.parallel(
                this.setState(state.CORAL_READY),
                outtake.setVoltage(() -> 0.0).withTimeout(0.1),
                hopper.setVoltage(0.0).withTimeout(0.1),
                gripper.setVoltage(() -> 0.0).withTimeout(0.1)));

    layout
        .cancelRequest
        .and(gripper::getDetected)
        .onTrue(
            Commands.parallel(
                this.setState(state.ALGAE_READY),
                outtake.setVoltage(() -> 0.0).withTimeout(0.1),
                hopper.setVoltage(0.0).withTimeout(0.1),
                gripper.setVoltage(() -> 0.0).withTimeout(0.1)));

    layout
        .manualElevator
        .whileTrue(this.setState(state.MANUAL_ELEVATOR))
        .onFalse(this.setState(state.IDLE));

    layout.resetGyro.onTrue(
        Commands.runOnce(
            () -> {
              drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
            }));

    layout.revFunnel.whileTrue(hopper.setVoltage(-OuttakeConstants.intake));

    layout.dejamCoral.whileTrue(
        Commands.parallel(
            hopper.setVoltage(OuttakeConstants.intake),
            outtake.setVoltage(() -> (OuttakeConstants.intake)),
            gripper.setVoltage(() -> (GripperConstants.net))));
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
            })
        .andThen(led.setState(newState).withTimeout(0.1));
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
    Logger.recordOutput(
        "Superstructure/Layout/Auto Align Cage", layout.autoAlignCage.getAsBoolean());
    Logger.recordOutput("Superstructure/Layout/Dejam Coral", layout.dejamCoral.getAsBoolean());

    Logger.recordOutput("Superstructure/State", kCurrentState);
    elevatorDisplay.setLength(elevator.getHeight());
    Logger.recordOutput("Superstructure/Mechanism", mech);

    Logger.recordOutput("Auto Align/Waypoints", AutoAlign.waypointsLogged);
    Logger.recordOutput("Auto Align/Generated Path", testPath);
    // Logger.recordOutput("Superstructure/Layout/Cancel Request",
    // layout.cancelRequest.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
  }

  public Command rumbleCommand(CommandXboxController controller, double seconds, double intensity) {
    return Commands.run(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, intensity);
            })
        .withTimeout(seconds)
        .finallyDo(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 0.0);
            });
  }
}
