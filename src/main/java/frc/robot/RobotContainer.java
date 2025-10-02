// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Superstructure.ControllerLayout;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.gripper.GripperIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOSim;
import frc.robot.subsystems.outtake.OuttakeIOTalonFX;
import frc.robot.subsystems.proximity.ProximityIOCanAndColor;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AutoAlignConstants;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Outtake outtake;
  private final Hopper hopper;
  private final Gripper gripper;
  private final Climb climb;
  private final Vision vision;
  public final Superstructure superstructure;
  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // private final ControllerLayout compLayout = new ControllerLayout();
  private final ControllerLayout simLayout = new ControllerLayout();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Vision Camera Transforms
  private final Transform3d[] cameraTransforms =
      new Transform3d[] {
        new Transform3d(
            Units.inchesToMeters(12.066),
            Units.inchesToMeters(11.906),
            Units.inchesToMeters(8.355),
            new Rotation3d(0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(-30))),
        new Transform3d(
            Units.inchesToMeters(12.066),
            Units.inchesToMeters(-11.906),
            Units.inchesToMeters(8.355),
            new Rotation3d(0.0, -Units.degreesToRadians(13.125000), Units.degreesToRadians(30)))
      };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOTalonFX());
        outtake = new Outtake(new OuttakeIOTalonFX(new ProximityIOCanAndColor(21, 0.2)));
        hopper = new Hopper(new HopperIOTalonFX());
        gripper = new Gripper(new GripperIOTalonFX(new ProximityIOCanAndColor(41, 0.15)));
        climb = new Climb(new ClimbIOTalonFX());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                FieldConstants.fieldLayout,
                new VisionIOPhotonVision(
                    "CamLeft", cameraTransforms[1], FieldConstants.fieldLayout),
                new VisionIOPhotonVision(
                    "CamRight", cameraTransforms[0], FieldConstants.fieldLayout));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        hopper = new Hopper(new HopperIOSim());
        gripper = new Gripper(new GripperIOSim());
        climb = new Climb(new ClimbIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                FieldConstants.fieldLayout,
                new VisionIOPhotonVisionSim(
                    "Left Cam", cameraTransforms[1], drive::getPose, FieldConstants.fieldLayout),
                new VisionIOPhotonVisionSim(
                    "Right Cam", cameraTransforms[0], drive::getPose, FieldConstants.fieldLayout));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        outtake = new Outtake(new OuttakeIO() {});
        hopper = new Hopper(new HopperIO() {});
        gripper = new Gripper(new GripperIO() {});
        climb = new Climb(new ClimbIO() {});
        vision =
            new Vision(drive::addVisionMeasurement, FieldConstants.fieldLayout, new VisionIO[] {});
        break;
    }

    AutoAlignConstants.getAprilTagPoses();

    // Setting Trajectory Following
    AutoRoutines.poseGetter = drive::getPose;
    AutoRoutines.driveFunction = drive::runVelocity;
    AutoRoutines.driveSubsystem = drive;
    AutoRoutines.autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive.driveController(),
            true,
            drive,
            (traj, edge) -> {
              Logger.recordOutput(
                  "Autos/Active Trajectory",
                  DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)
                      ? traj.flipped().getPoses()
                      : traj.getPoses());
            });
    // Setting Up Superstructure
    // Defining Axises
    simLayout.driveX = () -> driver.getLeftY();
    simLayout.driveY = () -> driver.getLeftX();

    // Requests
    simLayout.intakeRequest = driver.leftTrigger();
    simLayout.scoreRequest = driver.rightTrigger();
    simLayout.manualElevator = driver.back();

    // Coral Presets
    simLayout.L1 = driver.a();
    simLayout.L2 = driver.b();
    simLayout.L3 = driver.x();
    simLayout.L4 = driver.y();

    // Climb Setup
    simLayout.climbRequest = driver.povDown();
    simLayout.autoAlignCage = new Trigger(() -> false);

    // Auto Align
    simLayout.autoAlignLeft = driver.leftBumper();
    simLayout.autoAlignRight = driver.rightBumper();

    // Basic Functions
    simLayout.cancelRequest = driver.povLeft();
    simLayout.resetGyro = driver.povRight();
    simLayout.revFunnel = driver.povUp();
    simLayout.dejamCoral = driver.start();

    // Maybe Useless Stuff?
    simLayout.setPrescoreCoral = driver.leftStick();
    simLayout.setPrescoreAlgae = driver.rightStick();
    simLayout.driveController = driver;
    simLayout.operatorController = operator;

    superstructure =
        new Superstructure(drive, elevator, outtake, hopper, gripper, climb, simLayout);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Homing Sequence", elevator.homeElevator());
    autoChooser.addOption(
        "aCtoG",
        AutoRoutines.followTrajectory(
            AutoRoutines.loadTrajectory("aCtoG").get(), drive::getPose, drive::runVelocity, drive));

    autoChooser.addDefaultOption(
        "Double L4", Autos.DoubleL4(drive, elevator, outtake, hopper, superstructure));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> -driver.getRightX()));

        controller
            .x()
            .onTrue(
                PoseUtils.getOffsets(
                    () -> drive.getPose().nearest(AutoAlignConstants.offsetList),
                    () -> drive.getPose()));
    
  }

  public Command controllerRumble(double time, double strength) {
    return Commands.run(
            () -> {
              driver.setRumble(RumbleType.kBothRumble, strength);
            })
        .withTimeout(time)
        .finallyDo(
            () -> {
              driver.setRumble(RumbleType.kBothRumble, 0.0);
            });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    // return new PrintCommand("To be Removed");
    return Autos.DoubleL4(drive, elevator, outtake, hopper, superstructure);
    // return Autos.testMultiPath();
    // return TrajectoryFollower.followTrajectory(TrajectoryFollower.loadTrajectory("Test"));
    // return outtake.setDetected(true).andThen(autoChooser.get());
  }
}
