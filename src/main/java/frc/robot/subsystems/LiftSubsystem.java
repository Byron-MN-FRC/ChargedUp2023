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
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverInterface.VerticalPosition;

/**
 *
 */
public class LiftSubsystem extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
public boolean LiftEnabled = false;
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
// public double offset = 0;
public double highPos = 198000.000000;
public double storedPos = 500;
public double lowPos = 22000;
public double zeroPos = 500;
public enum LiftPosition{
    Stored, High, Low
}
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
    
    

    armExtender = new DoubleSolenoid(21, PneumaticsModuleType.REVPH, 2, 3);
    addChild("armExtender", armExtender);
    


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    armExtender.set(Value.kForward);
    liftMotorConfig();
    // SmartDashboard.putNumber("HighPosition", highPos);

}

    @Override
    public void periodic() {
        if (RobotContainer.getInstance().getAttachmentController().getPOV() == 270){storedPos = 100;}
        if (RobotContainer.getInstance().getAttachmentController().getPOV() == 90){storedPos = 12500;}
        
        SmartDashboard.putBoolean("isBodyTriggered", isBodyTriggered());
        SmartDashboard.putBoolean("isOuterTriggered", isOuterTriggered());
        // SmartDashboard.putNumber("leftLimitSwitchFwd", rightLifter.isFwdLimitSwitchClosed());
        // SmartDashboard.putNumber("leftLimitSwitchRev", rightLifter.isRevLimitSwitchClosed());
        SmartDashboard.putNumber("EncoderUnits", leftLifter.getSelectedSensorPosition());
        SmartDashboard.putNumber("LeftLifterVelocity", leftLifter.getSelectedSensorVelocity());
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("StoredPositionNew", getStoredPosition());
       // if (isBodyTriggered()) {System.out.println("!!!!!Body Limit Switch!!!!!");}
       // if (isOuterTriggered()) {System.out.println("!!!!!Outer Limit Switch!!!!!");}
        // put arm positions to the dashboard
        reportArmPose();

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }
 
    // Put methods for controlling this subsystem

    public void liftWithJoystick(XboxController controller2) {

        if (LiftEnabled) {
        double speed = controller2.getLeftX();
        rightLifter.set(speed*.3);
        leftLifter.follow(rightLifter);
        
        }
    }



    

    public void toggleArm() {
        armExtender.toggle();
    }

    public void extendArm() {
        armExtender.set(Value.kReverse);
    }

    public void retractArm() {
        armExtender.set(Value.kForward);
    }

    public void liftToPoint() {

    }

    public boolean isBodyTriggered() {
        return (rightLifter.isRevLimitSwitchClosed() == 0);

    }

    public boolean isOuterTriggered() {
        return (rightLifter.isFwdLimitSwitchClosed() == 0);

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
        rightLifter.setInverted(true);
        leftLifter.follow(rightLifter);

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
        leftLifter.configMotionCruiseVelocity(20000, LifterConstants.kTimeoutMs);
        leftLifter.configMotionAcceleration(24000, LifterConstants.kTimeoutMs);
        // rightLifter.configMotionCruiseVelocity(12000, LifterConstants.kTimeoutMs);
        rightLifter.configMotionCruiseVelocity(18000, LifterConstants.kTimeoutMs);
        // rightLifter.configMotionAcceleration(10000, LifterConstants.kTimeoutMs);
        rightLifter.configMotionAcceleration(22000, LifterConstants.kTimeoutMs);


        /* Zero the sensor once on robot boot up */
        leftLifter.setSelectedSensorPosition(0, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
        rightLifter.setSelectedSensorPosition(0, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
        leftLifter.configStatorCurrentLimit(currentLimiting);

        // Set default for limit switches to closed so don't move motors unless detected
        rightLifter.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed,LifterConstants.kTimeoutMs);
        rightLifter.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed,LifterConstants.kTimeoutMs);

        // rightLifter.setInverted(true);
        leftLifter.setNeutralMode(NeutralMode.Brake);
        rightLifter.setNeutralMode(NeutralMode.Brake);
    }

    public Boolean targetEncoder(double target) {
        final double currentEncoderUnits = rightLifter.getSelectedSensorPosition(0);
        if (Math.abs(currentEncoderUnits - target) < 190) {
            SmartDashboard.putBoolean("done", true);
            return true;
        }
        // NOT moving  so indicate at target -- dangerous for timeing?
        // doesn't work - need to test in neutral mode or just create separate commands
        // for outer vs inner
        //if (leftLifter.getSelectedSensorVelocity() == 0) { return true; }
 
        return false;
    }

     public void setLiftPos(double pos) {       
        rightLifter.set(TalonFXControlMode.MotionMagic, pos);
        leftLifter.follow(rightLifter);

   }

    public void stopLift() {
        rightLifter.set(ControlMode.Disabled, 0);
        leftLifter.follow(rightLifter);
       // rightLifter.set(ControlMode.Disabled, 0);
    }
    public boolean isArmExtended(){
        return (armExtender.get()==Value.kReverse);
    }

    public void zeroLift() {
        // offset = leftLifter.getSelectedSensorPosition();
        leftLifter.setSelectedSensorPosition(0, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
        rightLifter.setSelectedSensorPosition(0, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
    }


    public void startLift(double speed) {
        rightLifter.set(speed);
    }
    public void setHighLift(){
        // offset = -Math.abs(leftLifter.getSelectedSensorPosition()-highPos);
        leftLifter.setSelectedSensorPosition(highPos, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
        rightLifter.setSelectedSensorPosition(highPos, LifterConstants.kPIDLoopIdx, LifterConstants.kTimeoutMs);
    }
    public double getLeftLifterVelocity(){
        return rightLifter.getSelectedSensorVelocity();
    }
    public double liftVelocity(){
        return leftLifter.getSelectedSensorVelocity();
    }
    public double getLifttEncoder(){
        return rightLifter.getSelectedSensorPosition();
    }   

    private void reportArmPose(){
        final double currentEncoderUnits = leftLifter.getSelectedSensorPosition(0);
        SmartDashboard.putNumber("arm position", currentEncoderUnits);
    } 
    public double getStoredPosition(){
        return storedPos;
    }
}
