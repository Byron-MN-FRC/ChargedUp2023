// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.CANBUS_DRIVETRAIN;
import static frc.robot.Constants.DRIVETRAIN_PIGEON_ID;
import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is
   * useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  // By default we use a Pigeon for our gyroscope. But if you use another
  // gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating
  // the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  private final WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID, CANBUS_DRIVETRAIN);
  // FIXME Uncomment if you are using a NavX
  // private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX
  // connected over MXP

  private SwerveDriveOdometry m_odometry;

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  // Slew rate limiters to make joystick inputs more gentle.
  // A value of .1 will requier 10 seconds to get from 0 to 1. It is calculated as 1/rateLimitPerSecond to go from 0 to 1
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter balanceXLimiter = new SlewRateLimiter(1);

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // private final DifferentialDriveOdometry odometry;

// photon vision
ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
// Constants such as camera and target height stored. Change per robot and goal!

    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(14);
    final double TARGET_HEIGHT_METERS = Units.inchesToMeters(14.9);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(1);
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("4859-rear-cam");
    // PID constants should be tuned per robot
    final double LINEAR_P = 0.4;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.05;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    private double rollOffset;
    // april tag red field position
    public int middleAprilTag = 2;
    public int rightAprilTag = 3;
    public int leftAprilTag = 1;

    public double driveSpeed = 1;

    public final Map<String, double[]> position2Start = new HashMap<String, double[]>();
    public double AprilTagOffset = 1;


    public DrivetrainSubsystem() {
        
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            layout.setOrigin(Alliance.Blue == alliance ? 
                OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
        } catch(IOException e){
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
        }
        // LEFT
        position2Start.put("LL", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), AprilTagOffset, leftAprilTag});
        position2Start.put("LM", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), 0, leftAprilTag});
        position2Start.put("LR", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), -AprilTagOffset, leftAprilTag});
        // MIDDLE
        position2Start.put("ML", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), AprilTagOffset, middleAprilTag});
        position2Start.put("MM", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), 0, middleAprilTag});
        position2Start.put("MR", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), -AprilTagOffset, middleAprilTag});
        // RIGHT
        position2Start.put("RL", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), AprilTagOffset, rightAprilTag});
        position2Start.put("RL", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), 0, rightAprilTag});
        position2Start.put("RL", new double[] {
            layout.getTagPose(leftAprilTag).get().getX(),
            layout.getTagPose(leftAprilTag).get().getY(), -AprilTagOffset, rightAprilTag});

        
        MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
        m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Front Left Module",
                // BuiltInLayouts.kList))
                // .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();

        // We will do the same for the other modules
        m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Front Right Module",
                // BuiltInLayouts.kList))
                // .withPosition(3, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();

        m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Back Left Module",
                // BuiltInLayouts.kList))
                // .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET)
                .build();

        m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
                // .withLayout(getSMLayout(tab.getLayout("Back Right Module",
                // BuiltInLayouts.kList))
                // .withPosition(9, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR, CANBUS_DRIVETRAIN)
                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN)
                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();
        // Constants.tab_subsystems.add("Field", Field2d)

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        m_odometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics,
                m_pigeon.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeftModule.getPosition(),
                        m_frontRightModule.getPosition(),
                        m_backLeftModule.getPosition(),
                        m_backRightModule.getPosition()
                });
        rollOffset = m_pigeon.getRoll();
    }

    public boolean zeroGyroscope() {
        // FIXME Remove if you are using a Pigeon
        m_pigeon.reset();
        m_pigeon.setYaw(0);
        return true;

        // FIXME Uncomment if you are using a NavX
        // m_navx.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        // FIXME Remove if you are using a Pigeon
        return Rotation2d.fromDegrees(m_pigeon.getYaw());

        // FIXME Uncomment if you are using a NavX
        // if (m_navx.isMagnetometerCalibrated()) {
        // // We will only get valid fused headings if the magnetometer is calibrated
        // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
        // }
        //
        // // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        if (RobotContainer.getInstance().getDriveController().getBButton()) {

            forwardController.setP(.028);
            double x;
            double pitch = getPitch();
            x = forwardController.calculate(pitch, 0);
            // pitch is current value and setpoint is desired value
            // SmartDashboard.putNumber("x = ", x * MAX_VELOCITY_METERS_PER_SECOND);
            // SmartDashboard.putNumber("Pigeon Pitch", pitch);
            if (m_pigeon.getYaw() > 90 && m_pigeon.getYaw() < 270) {
                x = -x;
            }

            
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    -RobotContainer.modifyAxis(-x, balanceXLimiter) * MAX_VELOCITY_METERS_PER_SECOND,
                    -RobotContainer.modifyAxis(0, yLimiter) * MAX_VELOCITY_METERS_PER_SECOND,
                    -RobotContainer.modifyAxis(0, turnLimiter) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    getGyroscopeRotation());
        } else {
            m_chassisSpeeds = chassisSpeeds;
            
        }


    }

    @Override
    public void periodic() {
       SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        setModuleStates(states);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
       return m_odometry.getPoseMeters();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
       SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, MAX_VELOCITY_METERS_PER_SECOND);
       m_frontLeftModule.set(desiredStates[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                desiredStates[0].angle.getRadians());
       m_frontRightModule.set(desiredStates[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                desiredStates[1].angle.getRadians());
       m_backLeftModule.set(desiredStates[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                desiredStates[2].angle.getRadians());
       m_backRightModule.set(desiredStates[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                desiredStates[3].angle.getRadians());
       m_odometry.update(
               m_pigeon.getRotation2d(),
               new SwerveModulePosition[] {
                       m_frontLeftModule.getPosition(),
                       m_frontRightModule.getPosition(),
                       m_backLeftModule.getPosition(),
                       m_backRightModule.getPosition()
                });
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                m_pigeon.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeftModule.getPosition(),
                        m_frontRightModule.getPosition(),
                        m_backLeftModule.getPosition(),
                        m_backRightModule.getPosition()
                },
                pose);
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
        };
    }

    public void autoBalanceDrive() {
        forwardController.setP(.03);
        // forwardController.setI(.02);
            double x;
            double pitch = getPitch();
            x = forwardController.calculate(pitch, 0);
            // pitch is current value and setpoint is desired value
            // SmartDashboard.putNumber("x = ", x * MAX_VELOCITY_METERS_PER_SECOND);
            // SmartDashboard.putNumber("Pigeon Pitch", pitch);
            if (m_pigeon.getYaw() > 90 && m_pigeon.getYaw() < 270) {
                x = -x;
            }
            
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            -RobotContainer.modifyAxis(-x, balanceXLimiter) * MAX_VELOCITY_METERS_PER_SECOND,
            -RobotContainer.modifyAxis(0, yLimiter) * MAX_VELOCITY_METERS_PER_SECOND,
            -RobotContainer.modifyAxis(0, turnLimiter) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            getGyroscopeRotation());
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        setModuleStates(states);
    }

    private double getPitch() {
        // return m_pigeon.getPitch() - pitchOffset;
        return m_pigeon.getPitch();

    }

    public void setColor() {
        if (DriverStation.getAlliance()==Alliance.Blue){
            middleAprilTag = 7;
            leftAprilTag = 6;
            rightAprilTag = 8;
        }
        if (DriverStation.getAlliance()==Alliance.Red){
            middleAprilTag = 2;
            leftAprilTag = 1;
            rightAprilTag = 3;
        }
    }

public SlewRateLimiter getXLimiter() {
    return xLimiter;
}

public SlewRateLimiter getYLimiter() {
    return yLimiter;
}

public SlewRateLimiter getTurnLimiter() {
    return turnLimiter;
}

}
