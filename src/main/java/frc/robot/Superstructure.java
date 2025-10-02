// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.IntakeLocation;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbConstants;
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
import frc.robot.util.FieldConstants.ReefConstants;
import frc.robot.util.FieldConstants.ReefConstants.CoralTarget;
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

  private CoralTarget kCoralTarget = CoralTarget.L4;

  @AutoLogOutput(key = "Superstructure/Current State")
  public State kCurrentState = State.IDLE;

  public enum State {
    IDLE, // Has Nothing and no Subsystems are preforming anything
    CORAL_INTAKE, // Going to intake coral, Hopper and Outake always runs. If it has a coral then
    // go
    // to CORAL_READY. This is triggered when Driver runs intake request
    CORAL_READY, // Has Coral. If it is near a scoring Location go to CORAL_PRESCORE, but if it
    // doesn't have coral go to IDLE
    CORAL_PRESCORE, // Has Coral and is in scoring location, Goes to idle if it doesn't have a
    // coral. If it leaves the reef then go to CORAL_READY
    ALGAE_INTAKE, // Intaking Algae, should only be triggered when near the reef and desired level
    // is pressed. This is triggered when driver hits L2 or L3 without a coral and
    // algae, and robot is near the reef. Goes to ALGAE_READY when it gets an ALGAE
    ALGAE_READY, // Has Algae. If it loses an algae go to IDLE, but if it is in the net zone then
    // go
    // to ALGAE_PRESCORE
    ALGAE_PRESCORE, // Has Algae and is in Netzone. If it loses the ALGAE go to IDLE. If it leaves
    // Netzone go to ALGAE_READY
    CLIMB_READY, // Robot is in Idle and Driver pov down is pressed Climb will move from IDLE to
    // CLIMB_READY
    // And climb will move to the right angle for it to be in contact with cage
    CLIMB_PULL, // Robot is in CLIMB_READY and score request is selected than Climb arm will
    // pull
    // back -- CLIMB_READY TO CLIMB_PULL
    MANUAL_ELEVATOR // Driver has Manual Control over elevator
  }

  private final HashMap<State, Trigger> stateMap = new HashMap<State, Trigger>();
  private final ControllerLayout layout;

  private final LED led = new LED(Robot.isReal() ? new LEDIOCandle() : new LEDIO() {});

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Hopper hopper;
  private final Gripper gripper;
  private final Climb climb;

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
    for (State kState : State.values()) {
      stateMap.put(
          kState, new Trigger(() -> (kCurrentState == kState) && DriverStation.isEnabled()));
    }

    this.layout = layout;
    this.drive = drive;
    this.elevator = elevator;
    this.outtake = outtake;
    this.hopper = hopper;
    this.gripper = gripper;
    this.climb = climb;
    // Setting Up Display
    mech = new LoggedMechanism2d(1, 1);
    elevatorDisplay = new LoggedMechanismLigament2d("ElevatorMechanism", 0, 85);
    mech.getRoot("ElevatorRoot", Units.inchesToMeters(24 - 3), Units.inchesToMeters(4.087))
        .append(elevatorDisplay);

    // State Machine

    // Algae
    setAlgaeBindings();

    // Coral
    setCoralBindings();

    // Climb
    setClimbBindings();

    // Why can I control the entire robot using on a single controller and sensor
    // readings?
    // Manual Elevator Stuff
    // Setting the bindings
    setManualBindings();

    // Non State Stuff
    setNonStateBindings();
  }

  // A set of bindings for the Gripper subsystem and algae states (ALGAE_INTAKE,
  // ALGAE_READY,
  // ALGAE_PRESCORE)
  private void setAlgaeBindings() {
    layout
        .L2
        .and(() -> !(outtake.getDetected()))
        .and(() -> (DriveCommands.getBestIntake(drive) == IntakeLocation.REEF))
        .onTrue(
            Commands.parallel(
                this.setState(State.ALGAE_INTAKE),
                Commands.sequence(
                    elevator.setTarget(() -> (FieldConstants.ReefConstants.AlgaeTarget.L2.height)),
                    elevator.setExtension())));

    layout
        .L3
        .and(() -> !(outtake.getDetected()))
        .and(() -> (DriveCommands.getBestIntake(drive) == IntakeLocation.REEF))
        .onTrue(
            Commands.parallel(
                this.setState(State.ALGAE_INTAKE),
                Commands.sequence(
                    elevator.setTarget(() -> (FieldConstants.ReefConstants.AlgaeTarget.L3.height)),
                    elevator.setExtension())));

    stateMap.get(State.ALGAE_INTAKE).whileTrue(gripper.setVoltage(() -> (GripperConstants.intake)));

    stateMap
        .get(State.ALGAE_INTAKE)
        .and(gripper::getDetected)
        .onTrue(this.setState(State.ALGAE_READY));

    stateMap.get(State.ALGAE_READY).onTrue(gripper.setVoltage(() -> GripperConstants.intake));

    stateMap
        .get(State.ALGAE_READY)
        .and(() -> (FieldConstants.BargeConstants.nearNet(drive::getPose)))
        .onTrue(this.setState(State.ALGAE_PRESCORE));

    stateMap
        .get(State.ALGAE_PRESCORE)
        .and(() -> !(FieldConstants.BargeConstants.nearNet(drive::getPose)))
        .onTrue(this.setState(State.ALGAE_READY));

    layout
        .scoreRequest
        .and(stateMap.get(State.ALGAE_PRESCORE))
        .onTrue(
            Commands.sequence(
                elevator.setTarget(() -> (FieldConstants.BargeConstants.elevatorSetpoint)),
                elevator.setExtension(),
                Commands.waitUntil(() -> elevator.atSetpoint()),
                gripper.setVoltage(() -> GripperConstants.net)));

    stateMap
        .get(State.ALGAE_PRESCORE)
        .and(() -> !gripper.getDetected())
        .onTrue(this.setState(State.IDLE));
  }

  // A set of bindings for the Outtake, and Hopper subsystems and coral states
  // (CORAL_INTAKE,
  // CORAL_READY, CORAL_PRESCORE)
  private void setCoralBindings() {

    layout.intakeRequest.and(stateMap.get(State.IDLE)).onTrue(this.setState(State.CORAL_INTAKE));

    // Always run intake when in coral intake state
    stateMap
        .get(State.CORAL_INTAKE)
        .whileTrue(
            Commands.parallel(
                hopper.setVoltage(OuttakeConstants.intake),
                outtake.setVoltage(() -> (OuttakeConstants.intake))));

    // Switch to Coral Ready when it is in the intake state and has coral.
    stateMap
        .get(State.CORAL_INTAKE)
        .and(outtake::getDetected)
        .onTrue(this.setState(State.CORAL_READY));

    // Rumble when it has coral and is in teleop.
    stateMap
        .get(State.CORAL_READY)
        .and(DriverStation::isTeleop)
        .onTrue(
            Commands.parallel(
                rumbleCommand(layout.driveController, 0.5, 1.0),
                elevator.setTarget(() -> (kCoralTarget.height))));

    // Auto Align
    layout
        .autoAlignLeft
        .or(layout.autoAlignRight)
        .and(stateMap.get(State.CORAL_READY))
        .whileTrue(
            DriveCommands.autoAlign(
                drive,
                () ->
                    (ReefConstants.getBestBranch(
                        drive::getPose,
                        layout.autoAlignLeft.getAsBoolean())))); // Add Auto Align Command Here

    layout
        .autoAlignLeft
        .or(layout.autoAlignRight)
        .and(stateMap.get(State.CORAL_PRESCORE))
        .whileTrue(
            DriveCommands.autoAlign(
                drive,
                () ->
                    (ReefConstants.getBestBranch(
                        drive::getPose,
                        layout.autoAlignLeft.getAsBoolean())))); // Add Auto Align Command Here

    layout
        .scoreRequest
        .and(stateMap.get(State.CORAL_READY))
        .whileTrue(outtake.setVoltage(() -> (OuttakeConstants.L1)));

    layout
        .L1
        .and(stateMap.get(State.CORAL_READY))
        .onTrue(
            elevator
                .setTarget(() -> (CoralTarget.L1.height))
                .andThen(setCoralTarget(CoralTarget.L1)));

    layout
        .L2
        .and(stateMap.get(State.CORAL_READY))
        .onTrue(
            elevator
                .setTarget(() -> (CoralTarget.L2.height))
                .andThen(setCoralTarget(CoralTarget.L2)));

    layout
        .L3
        .and(stateMap.get(State.CORAL_READY))
        .onTrue(
            elevator
                .setTarget(() -> (CoralTarget.L3.height))
                .andThen(setCoralTarget(CoralTarget.L3)));

    layout
        .L4
        .and(stateMap.get(State.CORAL_READY))
        .onTrue(
            elevator
                .setTarget(() -> (CoralTarget.L4.height))
                .andThen(setCoralTarget(CoralTarget.L4)));

    stateMap
        .get(State.CORAL_READY)
        .and(() -> ReefConstants.nearReef(drive::getPose))
        .onTrue(this.setState(State.CORAL_PRESCORE));

    stateMap
        .get(State.CORAL_PRESCORE)
        .and(() -> !ReefConstants.nearReef(drive::getPose))
        .onTrue(this.setState(State.CORAL_READY));

    layout
        .scoreRequest
        .and(stateMap.get(State.CORAL_PRESCORE))
        .and(() -> (kCoralTarget != CoralTarget.L1))
        .onTrue(
            Commands.sequence(
                elevator.setExtension(),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(elevator::atSetpoint),
                outtake
                    .setVoltage(() -> (OuttakeConstants.voltageMap.get(elevator.getSetpoint())))
                    .until(() -> !(outtake.getDetected())),
                elevator.setTarget(() -> (0.0)),
                elevator.setExtension()));

    layout
        .scoreRequest
        .and(stateMap.get(State.CORAL_PRESCORE))
        .and(() -> (kCoralTarget == CoralTarget.L1))
        .onTrue(Commands.parallel());

    layout
        .L1
        .and(stateMap.get(State.CORAL_PRESCORE))
        .onTrue(
            Commands.sequence(
                elevator.setTarget(() -> (CoralTarget.L1.height)),
                elevator.setExtension(),
                this.setCoralTarget(CoralTarget.L1)));

    layout
        .L2
        .and(stateMap.get(State.CORAL_PRESCORE))
        .onTrue(
            Commands.sequence(
                elevator.setTarget(() -> (CoralTarget.L2.height)),
                elevator.setExtension(),
                this.setCoralTarget(CoralTarget.L2)));

    layout
        .L3
        .and(stateMap.get(State.CORAL_PRESCORE))
        .onTrue(
            Commands.sequence(
                elevator.setTarget(() -> (CoralTarget.L3.height)),
                elevator.setExtension(),
                this.setCoralTarget(CoralTarget.L3)));

    layout
        .L4
        .and(stateMap.get(State.CORAL_PRESCORE))
        .onTrue(
            Commands.sequence(
                elevator.setTarget(() -> (CoralTarget.L4.height)),
                elevator.setExtension(),
                this.setCoralTarget(CoralTarget.L4)));

    stateMap
        .get(State.CORAL_READY)
        .and(() -> !(outtake.getDetected()))
        .onTrue(this.setState(State.IDLE));

    stateMap
        .get(State.CORAL_PRESCORE)
        .and(() -> !outtake.getDetected())
        .onTrue(this.setState(State.IDLE));

    stateMap.get(State.IDLE).and(outtake::getDetected).onTrue(this.setState(State.CORAL_READY));
    // Sim States
    stateMap
        .get(State.CORAL_INTAKE)
        .and(
            () ->
                (FieldConstants.inTolerance(
                    () -> (SourceConstants.getNearestSource(drive::getPose)),
                    drive::getPose,
                    0.5,
                    Math.PI / 4.0)))
        .onTrue(outtake.setDetected(true));
  }

  // A set of bindings for the Climb subsystem and climb states (CLIMB_READY,
  // CLIMB_PULL)
  private void setClimbBindings() {
    layout.climbRequest.onTrue(
        Commands.parallel(climb.setPosition(ClimbConstants.ready), setState(State.CLIMB_READY)));

    layout
        .scoreRequest
        .and(stateMap.get(State.CLIMB_READY))
        .onTrue(
            Commands.parallel(
                climb.setPosition(ClimbConstants.climbed), setState(State.CLIMB_PULL)));
  }

  // Manual Elevator Bindings only runs Outtake, Gripper, Hopper, and Elevator.
  // Only Runs during
  // ELEVATOR_MANUAL state
  private void setManualBindings() {
    layout
        .manualElevator
        .whileTrue(this.setState(State.MANUAL_ELEVATOR))
        .onFalse(this.setState(State.IDLE));
    // Manual Coral Intake if near source
    layout
        .intakeRequest
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(() -> !(outtake.getDetected()))
        .and(() -> (DriveCommands.getBestIntake(drive) == IntakeLocation.SOURCE))
        .whileTrue(
            Commands.parallel(
                hopper.setVoltage(OuttakeConstants.intake),
                outtake.setVoltage(() -> (OuttakeConstants.intake))));

    // Manual Algae Intake if near Reef
    layout
        .intakeRequest
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(() -> !(gripper.getDetected()))
        .and(() -> (DriveCommands.getBestIntake(drive) == IntakeLocation.REEF))
        .whileTrue(gripper.setVoltage(() -> (GripperConstants.intake)));

    // Manual Coral Score if it has coral
    layout
        .scoreRequest
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .whileTrue(outtake.setVoltage(() -> (5.0)));

    // Manual Algae Score if it has algae
    layout
        .scoreRequest
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(gripper::getDetected)
        .whileTrue(gripper.setVoltage(() -> (GripperConstants.net)));

    // Setting Setpoints
    // Set L1 to 0 if it doesn't have coral
    layout
        .L1
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(() -> !(outtake.getDetected()))
        .onTrue(elevator.setTarget(() -> (0.0)).andThen(elevator.setExtension()));

    // Algae L2 Setpoint
    layout
        .L2
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(() -> !(gripper.getDetected()))
        .and(() -> !(outtake.getDetected()))
        .onTrue(
            elevator
                .setTarget(() -> (FieldConstants.ReefConstants.AlgaeTarget.L2.height))
                .andThen(elevator.setExtension()));

    // Algae L3 Setpoint
    layout
        .L3
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(() -> !(gripper.getDetected()))
        .and(() -> !(outtake.getDetected()))
        .onTrue(
            elevator
                .setTarget(() -> (FieldConstants.ReefConstants.AlgaeTarget.L3.height))
                .andThen(elevator.setExtension()));

    stateMap
        .get(State.MANUAL_ELEVATOR)
        .and(gripper::getDetected)
        .whileTrue(gripper.setVoltage(() -> (GripperConstants.intake)));

    // Algae Net Setpoint
    layout
        .L4
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(gripper::getDetected)
        .onTrue(
            elevator
                .setTarget(() -> (FieldConstants.BargeConstants.elevatorSetpoint))
                .andThen(elevator.setExtension()));

    // Coral Auto Align
    layout
        .autoAlignLeft
        .or(layout.autoAlignRight)
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .whileTrue(
            DriveCommands.autoAlign(
                drive, () -> (FieldConstants.ReefConstants.getBestBranch(drive::getPose, layout.autoAlignLeft.getAsBoolean()))));

    // Coral Setpoints
    // L1 Setpoint
    layout
        .L1
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator
                .setTarget(() -> (ReefConstants.CoralTarget.L1.height))
                .andThen(elevator.setExtension()));

    // L2 Setpoint
    layout
        .L2
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator
                .setTarget(() -> (ReefConstants.CoralTarget.L2.height))
                .andThen(elevator.setExtension()));
    // L3 setpoint
    layout
        .L3
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator
                .setTarget(() -> (ReefConstants.CoralTarget.L3.height))
                .andThen(elevator.setExtension()));

    // L4 setpoint
    layout
        .L4
        .and(stateMap.get(State.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator
                .setTarget(() -> (ReefConstants.CoralTarget.L4.height))
                .andThen(elevator.setExtension()));
  }

  // A set of bindings that isn't tied to a specific State.
  private void setNonStateBindings() {
    // Cancel Request and robot doesn't have an algae.
    layout
        .cancelRequest
        .and(() -> !(gripper.getDetected()))
        .onTrue(
            Commands.parallel(
                outtake.setVoltage(() -> 0.0),
                hopper.setVoltage(0),
                gripper.setVoltage(() -> 0.0),
                elevator.setTarget(() -> 0.0).andThen(elevator.setExtension()),
                this.setState(State.IDLE)));

    // Cancel Request but robot does have an algae.
    layout
        .cancelRequest
        .and(gripper::getDetected)
        .onTrue(
            Commands.parallel(
                outtake.setVoltage(() -> 0.0),
                hopper.setVoltage(0),
                gripper.setVoltage(() -> 0.0),
                this.setState(State.IDLE)));

    // Reverse Funnel and Outtake
    layout.revFunnel.whileTrue(
        Commands.parallel(
            hopper.setVoltage(-OuttakeConstants.intake),
            outtake.setVoltage(() -> -(OuttakeConstants.intake))));

    // Dejam Coral / Force Coral into shooter
    layout.dejamCoral.whileTrue(
        Commands.parallel(
                hopper.setVoltage(OuttakeConstants.intake),
                outtake.setVoltage(() -> (OuttakeConstants.intake)))
            .until(outtake::getDetected));

    // Reset Gyro to 0 Degrees when pressed
    layout.resetGyro.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive)
            .ignoringDisable(true));
  }

  public Command setCoralTarget(CoralTarget target) {
    return Commands.runOnce(
        () -> {
          this.kCoralTarget = target;
        });
  }

  public Command setState(State newState) {
    return Commands.runOnce(
            () -> {
              if (kCurrentState != newState) {
                System.out.println(newState.toString());
              }
              this.kCurrentState = newState;
            })
        .andThen(led.setState(newState).withTimeout(0.1).unless(Robot::isSimulation))
        .ignoringDisable(true);
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

    elevatorDisplay.setLength(elevator.getSetpoint());
    Logger.recordOutput("Superstructure/Mechanism", mech);
  }

  public static Command rumbleCommand(
      CommandXboxController controller, double seconds, double intensity) {
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
