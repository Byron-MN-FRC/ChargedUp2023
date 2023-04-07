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

import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AccessoryBButton;
import frc.robot.commands.AccessoryLeftBumper;
import frc.robot.commands.AccessoryRightBumper;
import frc.robot.commands.AccessoryXButton;
import frc.robot.commands.AutonBalance;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.ClawGrab;
import frc.robot.commands.ClawRelease;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveLift;
import frc.robot.commands.DriveToEncoderBody;
import frc.robot.commands.DriveToEncoderOuter;
import frc.robot.commands.DriveToLight;
import frc.robot.commands.DropAndRelease;
import frc.robot.commands.EnableLift;
import frc.robot.commands.GrabAndRaise;
import frc.robot.commands.HighGrab;
import frc.robot.commands.HighGrabPt2;
import frc.robot.commands.PlaceCargoPrt2;
import frc.robot.commands.RetractArm;
import frc.robot.commands.ShakeLift;
import frc.robot.commands.ZeroLiftSequential;
import frc.robot.commands.manualExtend;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriverInterface;
import frc.robot.subsystems.DriverInterface.LateralPosition;
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

  public final PhotonCamera photonCamera = new PhotonCamera("4859-rear-cam");
  // The robot's subsystems and commands are defined here...
  public final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final DriverInterface m_driverInterface = new DriverInterface();
  private final PoseEstimatorSubsystem m_PoseEstimatorSubsystem = new PoseEstimatorSubsystem(photonCamera,
      m_drivetrainSubsystem);
  // private final XboxController m_controller = new XboxController(0);
  // private final Joystick m_controller1 = new Joystick(0);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  
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
      () -> -modifyAxis(driveController.getLeftY(),m_drivetrainSubsystem.getYLimiter()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driveController.getLeftX(),m_drivetrainSubsystem.getXLimiter()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driveController.getRightX(),m_drivetrainSubsystem.getTurnLimiter()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_liftSubsystem.setDefaultCommand(new DriveLift( m_liftSubsystem ));


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

    // Configure autonomous sendable chooser
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
    m_chooser.setDefaultOption("Default", new InstantCommand().withName("Default"));
    m_chooser.addOption("Bump", new InstantCommand().withName("Bump"));
    m_chooser.addOption("Middle", new InstantCommand().withName("Middle"));
    m_chooser.addOption("Wall", new InstantCommand().withName("Wall"));
    // m_chooser.addOption("Middle", new InstantCommand().withName("Middle"));
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
final JoystickButton yClawGrab = new JoystickButton(driveController, XboxController.Button.kY.value);        
yClawGrab.onTrue(new ClawGrab( m_clawSubsystem ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                        
final JoystickButton aClawRelease = new JoystickButton(driveController, XboxController.Button.kA.value);        
aClawRelease.onTrue(new ClawRelease( m_clawSubsystem ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
                        
// final JoystickButton lbRetract = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);        
// lbRetract.onTrue(new RetractArm( m_liftSubsystem ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
      




    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    // final JoystickButton lsExtend = new JoystickButton(driveController, XboxController.Button.kLeftStick.value);        
    // lsExtend.onTrue(new ExtendArm( false,m_liftSubsystem ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // Back button zeros the gyroscope
    // new Button(m_controller::getBackButton)
    // No requirements because we don't need to interrupt anything
    // .whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    final JoystickButton xZeroGyro = new JoystickButton(driveController, XboxController.Button.kX.value);
    xZeroGyro.onTrue(new InstantCommand(m_drivetrainSubsystem::zeroGyroscope)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    final JoystickButton lbChaseTag = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    lbChaseTag
        .whileTrue(new ChaseTagCommand(photonCamera, m_drivetrainSubsystem, m_PoseEstimatorSubsystem::getCurrentPose)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    // final JoystickButton startPlaceCargo = new JoystickButton(driveController, XboxController.Button.kStart.value);
    // startPlaceCargo.whileTrue(new PlaceCargo(m_clawSubsystem, m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    // final JoystickButton finishPlaceCargo = new JoystickButton(driveController, XboxController.Button.kStart.value);
    // finishPlaceCargo.onFalse(new PlaceCargoPrt2(m_clawSubsystem, m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton startPlaceCargoAt = new JoystickButton(attachmentController, XboxController.Button.kRightBumper.value);
    startPlaceCargoAt.whileTrue(new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, false).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    final JoystickButton finishPlaceCargoAt = new JoystickButton(attachmentController, XboxController.Button.kRightBumper.value);
    finishPlaceCargoAt.onFalse(new PlaceCargoPrt2(m_clawSubsystem, m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    final JoystickButton armShake = new JoystickButton(attachmentController, XboxController.Button.kX.value);
    armShake.onTrue(new ShakeLift(m_liftSubsystem));

    final JoystickButton aDropAndRelease = new JoystickButton(attachmentController, XboxController.Button.kA.value);
    aDropAndRelease.onTrue(new DropAndRelease(m_liftSubsystem, m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    final JoystickButton yGrabAndRaise = new JoystickButton(attachmentController, XboxController.Button.kY.value);
    yGrabAndRaise.onTrue(new GrabAndRaise(m_liftSubsystem, m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    final JoystickButton rbZeroLift = new JoystickButton(attachmentController, XboxController.Button.kBack.value);
    rbZeroLift.onTrue(new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
    final JoystickButton lsLiftEnable = new JoystickButton(attachmentController, XboxController.Button.kLeftStick.value);
    lsLiftEnable.onTrue(new EnableLift(m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    final JoystickButton leftBumpSetAprilTarget = new JoystickButton(attachmentController, XboxController.Button.kLeftBumper.value);
    leftBumpSetAprilTarget.onTrue(new AccessoryLeftBumper().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // final JoystickButton RightBumpSetAprilTarget = new JoystickButton(attachmentController, XboxController.Button.kRightBumper.value);
    // RightBumpSetAprilTarget.onTrue(new AccessoryRightBumper().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    final JoystickButton driveToLight = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    driveToLight.whileTrue(new DriveToLight(m_drivetrainSubsystem, photonCamera).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    final JoystickButton AccessoryBButton = new JoystickButton(attachmentController, XboxController.Button.kB.value);
    AccessoryBButton.onTrue(new AccessoryBButton().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    final JoystickButton AccessoryXButton = new JoystickButton(attachmentController, XboxController.Button.kX.value);
    AccessoryXButton.onTrue(new AccessoryXButton().withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    final JoystickButton manualExtend = new JoystickButton(attachmentController, XboxController.Button.kRightStick.value);
    manualExtend.whileTrue(new manualExtend(m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    
    final JoystickButton highGrab = new JoystickButton(attachmentController, XboxController.Button.kLeftBumper.value);
    highGrab.whileTrue(new HighGrab(m_clawSubsystem, m_liftSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    final JoystickButton highGrabPt2 = new JoystickButton(attachmentController, XboxController.Button.kLeftBumper.value);
    highGrabPt2.onFalse(new HighGrabPt2(m_liftSubsystem, m_clawSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
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
    double negate;
    if (DriverStation.getAlliance()==Alliance.Blue){
      negate = -1;
    }
    else{
      negate = 1;
    }
    double autoOffset = 0;
    LateralPosition aprilPosition= m_driverInterface.AprilTagPosition();
    switch (aprilPosition) {
      case Right: autoOffset = -Units.inchesToMeters(28);
        break;
      case Left: autoOffset = Units.inchesToMeters(28);
        break;
      default: autoOffset = 0;
    }

    SmartDashboard.putNumber("Auto Extend?", autoOffset);
    boolean autoExtend = SmartDashboard.getBoolean("Auto Extend?", false);

    double autoDelay = SmartDashboard.getNumber("Auto Delay", 0);
    var thetaController =
    new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0,
    AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


    TrajectoryConfig config =
    new TrajectoryConfig
    (
             AutoConstants.kMaxSpeedMetersPerSecond,
             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(DriveConstants.kDriveKinematics);
         // Apply the voltage constraint

    TrajectoryConfig configSlow =
    new TrajectoryConfig
    (
             1.6,
             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
         // Add kinematics to ensure max speed is actually obeyed
         .setKinematics(DriveConstants.kDriveKinematics);
         // Apply the voltage constraint

  // An example trajectory to follow.  All units in meters.
  Trajectory bumpPathTrajectoryOne =
     TrajectoryGenerator.generateTrajectory(
         // Start at the origin facing the +X direction
         new Pose2d(0, autoOffset, new Rotation2d(0)),
         // Pass through these two interior waypoints, making an 's' curve path
        //  List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        List.of(new Translation2d(.5, .5*negate)),
         // End 3 meters straight ahead of where we started, facing forward
         new Pose2d(5.2+.1778, .65*negate, new Rotation2d(0)),
        // new Pose2d(3, 0, new Rotation2d(0)),
         // Pass config
         config);
    
  Trajectory wallPathTrajectoryOne =

    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, autoOffset, new Rotation2d(0)),
      // starting point
      List.of(new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(-15)*negate),
      new Translation2d(Units.inchesToMeters(175), Units.inchesToMeters(-15)*negate)
      // new Translation2d(Units.inchesToMeters(220.404), Units.inchesToMeters(-8)*negate)
      ),
        // middle point
        new Pose2d(Units.inchesToMeters(200), Units.inchesToMeters(-14)*negate, new Rotation2d(0)),
      //end point (untestedlast coords, should be right tho)
        config);

    Trajectory pathMiddleTrajectory =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.feetToMeters(0), 0, new Rotation2d(0)),
        List.of(
          // new Translation2d(Units.feetToMeters(3),Units.feetToMeters(.1)),
          new Translation2d(Units.feetToMeters(5), Units.feetToMeters(-.1)*negate)
        ),
        new Pose2d(Units.feetToMeters(7), Units.feetToMeters(.1)*negate, new Rotation2d(0)),
        configSlow);
             
    Trajectory pathThreeTrajectoryTwo = 
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(240.404), Units.inchesToMeters(-8)*negate, new Rotation2d(0)),
        List.of(new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(-8.1)*negate)),
        new Pose2d(Units.inchesToMeters(12), Units.inchesToMeters(-8)*negate, new Rotation2d(0)),
        config);


    SwerveControllerCommand BumpPartOne =
        new SwerveControllerCommand(
        bumpPathTrajectoryOne,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem);

    SwerveControllerCommand pathMiddlePartOne =
        new SwerveControllerCommand(
        pathMiddleTrajectory,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem);
    
    SwerveControllerCommand WallPathPartOne =
        new SwerveControllerCommand(
        wallPathTrajectoryOne,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem);

    SwerveControllerCommand pathThreePartTwo =
        new SwerveControllerCommand(
        pathThreeTrajectoryTwo,
        m_drivetrainSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_drivetrainSubsystem::setModuleStates,
        m_drivetrainSubsystem);
    // // Reset odometry to the starting pose of the trajectory.
    // m_drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_drivetrainSubsystem.drive(new
    // ChassisSpeeds(0, 0, 0)));
    

if (m_chooser.getSelected()==null){
  return new SequentialCommandGroup(
    new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem),
    new ClawGrab(m_clawSubsystem),
    new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, autoExtend),
    new ClawRelease(m_clawSubsystem),
    new WaitCommand(.5),
    new RetractArm(m_liftSubsystem),
    new DriveToEncoderBody(m_liftSubsystem.storedPos, m_liftSubsystem));
}
System.out.println("!!!!!Autonomous Selected!!!!! " + m_chooser.getSelected().getName());
    m_drivetrainSubsystem.resetOdometry(bumpPathTrajectoryOne.getInitialPose());

    if (m_chooser.getSelected().getName() =="Default"){
      return new SequentialCommandGroup(
        new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem),
        new ClawGrab(m_clawSubsystem),
        new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, autoExtend),
        new ClawRelease(m_clawSubsystem),
        new WaitCommand(.5),
        new RetractArm(m_liftSubsystem),
        new DriveToEncoderBody(m_liftSubsystem.storedPos, m_liftSubsystem)
      );
    }
    if (m_chooser.getSelected().getName() =="Bump"){
      return new SequentialCommandGroup(
        new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem),
        new ClawGrab(m_clawSubsystem),
        new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, autoExtend),
        new ClawRelease(m_clawSubsystem),
        new WaitCommand(.5),
        new RetractArm(m_liftSubsystem),
        new DriveToEncoderBody(m_liftSubsystem.storedPos, m_liftSubsystem),
        new WaitCommand(autoDelay),
        BumpPartOne
        // new ClawGrab(m_clawSubsystem)
      );
    }
    if (m_chooser.getSelected().getName() =="Middle"){
      return new SequentialCommandGroup(
        new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem),
        new ClawGrab(m_clawSubsystem),
        // new PlaceCargo(m_clawSubsystem, m_liftSubsystem),
        new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, autoExtend),
        new ClawRelease(m_clawSubsystem),
        new WaitCommand(.5),
        new RetractArm(m_liftSubsystem),
        new ParallelCommandGroup(
          new DriveToEncoderBody(m_liftSubsystem.storedPos, m_liftSubsystem),
          pathMiddlePartOne
        ),
        new AutonBalance(m_drivetrainSubsystem)
      );
    }
    if (m_chooser.getSelected().getName() == "Wall"){
      return new SequentialCommandGroup(
        new ClawGrab(m_clawSubsystem),
        new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem),
        new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, autoExtend),
        new ClawRelease(m_clawSubsystem),
        new WaitCommand(.5),
        new RetractArm(m_liftSubsystem),
        new DriveToEncoderBody(m_liftSubsystem.storedPos, m_liftSubsystem),
        new WaitCommand(autoDelay),
        WallPathPartOne
        // new GrabAndRaise(m_liftSubsystem, m_clawSubsystem),
        // pathThreePartTwo
      );
    }

    else {
      return new SequentialCommandGroup(
        new ZeroLiftSequential(m_liftSubsystem, m_clawSubsystem),
        new ClawGrab(m_clawSubsystem),
        new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, autoExtend),
        new ClawRelease(m_clawSubsystem),
        new WaitCommand(.5),
        new RetractArm(m_liftSubsystem),
        new DriveToEncoderBody(m_liftSubsystem.storedPos, m_liftSubsystem));
    }
    // return m_chooser.getSelected();
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

  public static double modifyAxis(double value, SlewRateLimiter limiter) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = limiter.calculate(Math.copySign(value * value, value));

    return value;
  }

}
