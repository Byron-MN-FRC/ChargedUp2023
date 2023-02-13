// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ClawGrab;
import frc.robot.commands.ClawRelease;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveLift;
import frc.robot.commands.DriveToEncoder;
import frc.robot.commands.DropAndRelease;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.GrabAndRaise;
import frc.robot.commands.Path1;
import frc.robot.commands.Path3;
import frc.robot.commands.PlaceCargo;
import frc.robot.commands.RetractArm;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  private static RobotContainer m_RobotContainer = new RobotContainer();

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
  // The robot's subsystems
  public final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  public final LiftSubsystem m_liftSubsystem = new LiftSubsystem();

  // Joysticks
  private final XboxController attachmentController = new XboxController(1);
  private final XboxController driveController = new XboxController(0);

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

  private final PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem(photonCamera,
      m_drivetrainSubsystem);
  // private final XboxController m_controller = new XboxController(0);
  // private final Joystick m_controller1 = new Joystick(0);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    // m_drivetrainSubsystem,
    // () -> -modifyAxis(m_controller1.getY()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(m_controller1.getX()) *
    // DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(m_controller1.getTwist()) *
    // DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -modifyAxis(driveController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_liftSubsystem.setDefaultCommand(new DriveLift(m_liftSubsystem));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.addOption("Path1", new Path1(m_drivetrainSubsystem, m_liftSubsystem, m_clawSubsystem));
    m_chooser.addOption("Path3", new Path3(m_drivetrainSubsystem, m_liftSubsystem, m_clawSubsystem));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_RobotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    // Create some buttons
    final JoystickButton aDropAndRelease = new JoystickButton(attachmentController, XboxController.Button.kA.value);
    aDropAndRelease.onTrue(new DropAndRelease(m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton yGrabAndRaise = new JoystickButton(attachmentController, XboxController.Button.kY.value);
    yGrabAndRaise.onTrue(new GrabAndRaise(m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton startPlaceCargo = new JoystickButton(driveController, XboxController.Button.kStart.value);
    startPlaceCargo.onTrue(new PlaceCargo(m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton yclawGrab = new JoystickButton(driveController, XboxController.Button.kY.value);
    yclawGrab.onTrue(new ClawGrab(m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton aclawRelease = new JoystickButton(driveController, XboxController.Button.kA.value);
    aclawRelease.onTrue(new ClawRelease(m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton lTextend = new JoystickButton(driveController, XboxController.Button.kLeftStick.value);
    lTextend.onTrue(new ExtendArm(false, m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton rBretract = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    rBretract.onTrue(new RetractArm(m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    // Back button zeros the gyroscope
    // new Button(m_controller::getBackButton)
    // No requirements because we don't need to interrupt anything
    // .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    final JoystickButton xboxButton1 = new JoystickButton(driveController, XboxController.Button.kX.value);
    xboxButton1.onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    final JoystickButton btnChaseTag = new JoystickButton(driveController, XboxController.Button.kBack.value);
    btnChaseTag
        .whileTrue(new ChaseTagCommand(photonCamera, m_drivetrainSubsystem, m_PoseEstimatorSubsystem::getCurrentPose)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // Command resetGyroCommand = new
    // InstantCommand(m_drivetrainSubsystem::zeroGyroscope);
  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public XboxController getDriveController() {
    return driveController;
  }

  public XboxController getAttachmentController() {
    return attachmentController;
  }

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // var thetaController =
    // new ProfiledPIDController(
    // AutoConstants.kPThetaController, 0, 0,
    // AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // // The selected command will be run in autonomous
    // // The selected command will be run in autonomous
    // //return m_chooser.getSelected();
    // // Create a voltage constraint to ensure we don't accelerate too fast

    // // Create config for trajectory
    // TrajectoryConfig config =
    // new TrajectoryConfig
    // (
    // AutoConstants.kMaxSpeedMetersPerSecond,
    // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(DriveConstants.kDriveKinematics);
    // // Apply the voltage constraint

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
       TrajectoryGenerator.generateTrajectory(
           // Start at the origin facing the +X direction
           new Pose2d(0, 0, new Rotation2d(0)),
           // Pass through these two interior waypoints, making an 's' curve path
           List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          // List.of(new Translation2d(1, 0)),
           // End 3 meters straight ahead of where we started, facing forward
          //  new Pose2d(5.2+.1778, -0.3810-.0762, new Rotation2d(0)),
          new Pose2d(3, 0, new Rotation2d(0)),
           // Pass config
           config);

    // SwerveControllerCommand swerveControllerCommand =
    // new SwerveControllerCommand(
    // exampleTrajectory,
    // m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
    // DriveConstants.kDriveKinematics,

    // // Position controllers
    // new PIDController(AutoConstants.kPXController, 0, 0),
    // new PIDController(AutoConstants.kPYController, 0, 0),
    // thetaController,
    // m_drivetrainSubsystem::setModuleStates,
    // m_drivetrainSubsystem);

    // // Reset odometry to the starting pose of the trajectory.
    // m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_drivetrainSubsystem.drive(new
    // ChassisSpeeds(0, 0, 0)));
    return m_chooser.getSelected();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

}
