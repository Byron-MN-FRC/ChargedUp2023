// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .584; // FIXME Measure and set trackwidth
    /**
     * The front-tolokiuy-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = .5715; // FIXME Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 8; // FIXME Set Pigeon ID
    public static final String CANBUS_DRIVETRAIN = "CANivore1";
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(334.775390625); // FIXME Measure and set
                                                                                                // front left steer
                                                                                                // offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(53.96484375); // FIXME Measure and set
                                                                                               // front right steer
                                                                                               // offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(91.845703125); // FIXME Measure and set
                                                                                              // back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(163.828125); // FIXME Measure and set
                                                                                             // back right steer offset

    public static final ShuffleboardTab tab_subsystems = Shuffleboard.getTab("Subsytems");
    public static final ShuffleboardTab tab_commands = Shuffleboard.getTab("Commands");

    static public final class DriveConstants {

        // Autonomous
        public static final double kTrackWidth = DRIVETRAIN_TRACKWIDTH_METERS; // ??
        public static final double kWheelBase = DRIVETRAIN_WHEELBASE_METERS;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kWheelDiameterMeters = 0.15; // ??

        // Values from Robot Characterization Toolsuite
        public static final double ksVolts = 0.127957; // 0.22
        public static final double kvVoltSecondsPerMeter = 2.7085; // 1.98
        public static final double kaVoltSecondsSquaredPerMeter = 0.31875; // .2

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
                SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class VisionConstants {
        /**
         * 
         * Physical location of the camera on the robot, relative to the center of the
         * robot.
         */
        public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(Units.inchesToMeters(-10), 0, Units.inchesToMeters(14)), new Rotation3d(0,0,Math.PI));
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
    }

    public static class LifterConstants {
        /**
         * 
         */
        //Current limit values
         public static final boolean kEnableCurrentLimiting_BS = true;
        public static final double currentLimit = 12;
        public static final double thresholdLimit = 18;
        public static final double thresholdTime = .1;

        //Arm values
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /**
         * Gains used in Motion Magic, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final Gains kGains_lifterMotor = new Gains(0.6, 0.0, 0.0, 0.2, 0, 1.0);
    }

}
