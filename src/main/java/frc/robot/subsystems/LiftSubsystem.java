// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 *
 */
public class LiftSubsystem extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
private DigitalInput bodySwitch;
private DigitalInput outerSwitch;
private WPI_TalonFX leftLifter;
private WPI_TalonFX rightLifter;
private DoubleSolenoid armExtender;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public final StatorCurrentLimitConfiguration currentLimiting = new StatorCurrentLimitConfiguration(
        LifterConstants.kEnableCurrentLimiting_BS, LifterConstants.currentLimit,
        LifterConstants.thresholdLimit,LifterConstants.thresholdTime
    );
    

    /**
    *
    */
    public LiftSubsystem() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
bodySwitch = new DigitalInput(0);
 addChild("bodySwitch", bodySwitch);
 

outerSwitch = new DigitalInput(1);
 addChild("outerSwitch", outerSwitch);
 

leftLifter = new WPI_TalonFX(13);
 
 

rightLifter = new WPI_TalonFX(14);
 
 

armExtender = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 2, 3);
 addChild("armExtender", armExtender);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        armExtender.set(Value.kReverse);
        liftMotorConfig();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isBodyTriggered", isBodyTriggered());

        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem

    public void liftWithJoystick(XboxController controller2) {
        double target = LifterConstants.storedPos;
        double speed = controller2.getLeftX();
        // negitive values are isOuterTriggered, and postive values are isBodyTriggered.
        if (isBodyTriggered() && speed > 0) {
            speed = 0;
        } else if (isOuterTriggered() && speed < 0) {
            speed = 0;
        } else {
            // rightLifter.set(speed);
            // leftLifter.set(speed);
            // SmartDashboard.putNumber("speed", speed);

            // if (controller2.getPOV() == -1) {
            // target = LifterConstants.middleDrop;
            // }else if (controller2.getPOV() <= 45 || controller2.getPOV() >= 315){
            // target = LifterConstants.highDrop;
            // }else if (controller2.getPOV() <= 225 || controller2.getPOV() >= 135){
            // target = LifterConstants.lowDrop;
            // }
            // SmartDashboard.putNumber("Target", target);
        }

    }

    public void toggleArm() {
        armExtender.toggle();
    }

    public void extendArm() {
        armExtender.set(Value.kForward);
    }

    public void retractArm() {
        armExtender.set(Value.kReverse);
    }

    public void liftToPoint() {

    }

    public boolean isBodyTriggered() {
        if (bodySwitch.get()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isOuterTriggered() {
        if (outerSwitch.get()) {
            return true;
        } else {
            return false;
        }
    }

    public void liftMotorConfig() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        leftLifter.configFactoryDefault();
        rightLifter.configFactoryDefault();

        /* Configure Sensor Source for Primary PID */
        leftLifter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, LifterConstants.kPIDLoopIdx,
                LifterConstants.kTimeoutMs);
        rightLifter.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, LifterConstants.kPIDLoopIdx,
                LifterConstants.kTimeoutMs);

        /*
         * set deadband to super small 0.001 (0.1 %).
         * The default deadband is 0.04 (4 %)
         */
        leftLifter.configNeutralDeadband(0.001, LifterConstants.kTimeoutMs);
        rightLifter.configNeutralDeadband(0.001, LifterConstants.kTimeoutMs);

        /**
         * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
         * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
         * sensor to have positive increment when driving Talon Forward (Green LED)
         */
        leftLifter.setSensorPhase(false);
        leftLifter.setInverted(false);
        rightLifter.setSensorPhase(false);
        rightLifter.setInverted(false);

        /* Set relevant frame periods to be at least as fast as periodic rate */
        leftLifter.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, LifterConstants.kTimeoutMs);
        leftLifter.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, LifterConstants.kTimeoutMs);
        rightLifter.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, LifterConstants.kTimeoutMs);
        rightLifter.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, LifterConstants.kTimeoutMs);

        /* Set the peak and nominal outputs */
        leftLifter.configNominalOutputForward(0, LifterConstants.kTimeoutMs);
        leftLifter.configNominalOutputReverse(0, LifterConstants.kTimeoutMs);
        leftLifter.configPeakOutputForward(1, LifterConstants.kTimeoutMs);
        leftLifter.configPeakOutputReverse(-1, LifterConstants.kTimeoutMs);
        rightLifter.configNominalOutputForward(0, LifterConstants.kTimeoutMs);
        rightLifter.configNominalOutputReverse(0, LifterConstants.kTimeoutMs);
        rightLifter.configPeakOutputForward(1, LifterConstants.kTimeoutMs);
        rightLifter.configPeakOutputReverse(-1, LifterConstants.kTimeoutMs);

        /* Set Motion Magic gains in slot0 - see documentation */
        leftLifter.selectProfileSlot(LifterConstants.kSlotIdx, LifterConstants.kPIDLoopIdx);
        leftLifter.config_kF(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kF,
                LifterConstants.kTimeoutMs);
        leftLifter.config_kP(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kP,
                LifterConstants.kTimeoutMs);
        leftLifter.config_kI(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kI,
                LifterConstants.kTimeoutMs);
        leftLifter.config_kD(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kD,
                LifterConstants.kTimeoutMs);
        rightLifter.selectProfileSlot(LifterConstants.kSlotIdx, LifterConstants.kPIDLoopIdx);
        rightLifter.config_kF(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kF,
                LifterConstants.kTimeoutMs);
        rightLifter.config_kP(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kP,
                LifterConstants.kTimeoutMs);
        rightLifter.config_kI(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kI,
                LifterConstants.kTimeoutMs);
        rightLifter.config_kD(LifterConstants.kSlotIdx, LifterConstants.kGains_lifterMotor.kD,
                LifterConstants.kTimeoutMs);

        /* Set acceleration and vcruise velocity - see documentation */
        leftLifter.configMotionCruiseVelocity(15000, LifterConstants.kTimeoutMs);
        leftLifter.configMotionAcceleration(6000, LifterConstants.kTimeoutMs);
        rightLifter.configMotionCruiseVelocity(15000, LifterConstants.kTimeoutMs);
        rightLifter.configMotionAcceleration(6000, LifterConstants.kTimeoutMs);

        /* Zero the sensor once on robot boot up */
        leftLifter.setSelectedSensorPosition(0, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
        rightLifter.setSelectedSensorPosition(0, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
        leftLifter.configStatorCurrentLimit(currentLimiting);
    }

    public Boolean targetEncoder(double target) {
        final double currentEncoderUnits = leftLifter.getSelectedSensorPosition(0);
        if (Math.abs(currentEncoderUnits - target) < 190) {
            SmartDashboard.putBoolean("done", true);
            return true;
        }
        SmartDashboard.putBoolean("done", true);

        return false;
    }

    public void setLiftPos(double pos) {
        rightLifter.set(TalonFXControlMode.MotionMagic, pos);
        // if (leftLifter.getSelectedSensorPosition() >= 5000)
        leftLifter.set(ControlMode.Disabled, 0);
        // else
        // leftLifter.set(TalonFXControlMode.MotionMagic, pos);

        // rightLifter.follow(leftLifter);
        // rightMaster.set(ControlMode.MotionMagic, target_sensorUnits,
        // DemandType.AuxPID, 0);
        // leftMaster.follow(rightMaster, FollowerType.AuxOutput1);
        SmartDashboard.putNumber("leftEncoder", leftLifter.getSelectedSensorPosition());
        SmartDashboard.putNumber("rightEncoder", rightLifter.getSelectedSensorPosition());
    }

    public void stopLift() {
        leftLifter.set(ControlMode.Disabled, 0);
        rightLifter.set(ControlMode.Disabled, 0);
    }

}
