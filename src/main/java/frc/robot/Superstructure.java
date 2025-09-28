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
import frc.robot.subsystems.autoAlign.AutoAlign;
import frc.robot.subsystems.autoAlign.AutoAlign.IntakeLocation;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoint;
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

  public enum state {
    IDLE, // Has Nothing and no Subsystems are preforming anything
    CORAL_INTAKE, // Going to intake coral, Hopper and Outake always runs
    CORAL_READY, // Has Coral
    CORAL_PRESCORE, // Has Coral and is in scoring location
    ALGAE_INTAKE, // Intaking Algae, should only be triggered when near the reef and desired level is pressed
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
  private final AutoAlign autoAlign;
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
      ControllerLayout layout,
      AutoAlign autoAlign) {
    for (state kState : state.values()) {
      stateMap.put(
          kState, new Trigger(() -> (kCurrentState == kState) && DriverStation.isEnabled()));
    }

    this.layout = layout;
    this.elevator = elevator;
    this.autoAlign = autoAlign;
    // Setting Up Display
    mech = new LoggedMechanism2d(1, 1);
    elevatorDisplay = new LoggedMechanismLigament2d("ElevatorMechanism", 0, 85);
    mech.getRoot("ElevatorRoot", Units.inchesToMeters(24 - 3), Units.inchesToMeters(4.087))
        .append(elevatorDisplay);

    // State Machine

    // Why can I control the entire robot using on a single controller and sensor readings?
    // Manual Elevator Stuff
    layout
        .manualElevator
        .whileTrue(this.setState(state.MANUAL_ELEVATOR))
        .onFalse(this.setState(state.IDLE));

    // Manual Coral Intake if near source
    layout
        .intakeRequest
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(() -> !(outtake.getDetected()))
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.SOURCE))
        .whileTrue(Commands.parallel(
            hopper.setVoltage(OuttakeConstants.intake),
            outtake.setVoltage(() -> (OuttakeConstants.intake))
        ));

    // Manual Algae Intake if near Reef
    layout
        .intakeRequest
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(() -> !(gripper.getDetected()))
        .and(() -> (AutoAlign.getBestIntake(drive) == IntakeLocation.REEF))
        .whileTrue(gripper.setVoltage(() -> (GripperConstants.intake)));
    
    // Manual Coral Score if it has coral
    layout
        .scoreRequest
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .whileTrue(outtake.setVoltage(() -> (5.0)));

    // Manual Algae Score if it has algae
    layout
        .scoreRequest
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(gripper::getDetected)
        .whileTrue(gripper.setVoltage(() -> (GripperConstants.net)));
    
    // Setting Setpoints
    // Set L1 to 0 if it doesn't have coral
    layout
        .L1
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(() -> !(outtake.getDetected()))
        .onTrue(
            elevator.setTarget(() -> (0.0)).andThen(elevator.setExtension()));

    // Algae L2 Setpoint
    layout
        .L2
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(() -> !(gripper.getDetected()))
        .and(() -> !(outtake.getDetected()))
        .onTrue(
            elevator.setTarget(() -> (FieldConstants.ReefConstants.algaeTarget.L2.height))
            .andThen(elevator.setExtension())
        );

    // Algae L3 Setpoint
    layout
        .L3
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(() -> !(gripper.getDetected()))
        .and(() -> !(outtake.getDetected()))
        .onTrue(
            elevator.setTarget(() -> (FieldConstants.ReefConstants.algaeTarget.L3.height))
            .andThen(elevator.setExtension())
        );

    // Algae Net Setpoint
    layout
        .L4
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(gripper::getDetected)
        .onTrue(
            elevator.setTarget(() -> (FieldConstants.BargeConstants.elevatorSetpoint))
            .andThen(elevator.setExtension()));
    
    // Coral Setpoints
    // L1 Setpoint
    layout
        .L1
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator.setTarget(() -> (ReefConstants.coralTarget.L1.height))
            .andThen(elevator.setExtension())
        );

    // L2 Setpoint
    layout
        .L2
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator.setTarget(() -> (ReefConstants.coralTarget.L2.height))
            .andThen(elevator.setExtension())
        );
    // L3 setpoint
    layout
        .L3
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator.setTarget(() -> (ReefConstants.coralTarget.L3.height))
            .andThen(elevator.setExtension())
        );
    
    // L4 setpoint
    layout
        .L4
        .and(stateMap.get(state.MANUAL_ELEVATOR))
        .and(outtake::getDetected)
        .onTrue(
            elevator.setTarget(() -> (ReefConstants.coralTarget.L4.height))
            .andThen(elevator.setExtension())
        );
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
    elevatorDisplay.setLength(elevator.getSetpoint());
    Logger.recordOutput("Superstructure/Mechanism", mech);

    // Logger.recordOutput("Superstructure/Layout/Cancel Request",
    // layout.cancelRequest.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
    // Logger.recordOutput("Superstructure/Layout/L1", layout.L1.getAsBoolean());
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
