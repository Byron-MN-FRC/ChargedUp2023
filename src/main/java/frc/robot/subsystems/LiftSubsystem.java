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


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
    import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
 
 

armExtender = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 2, 3);
 addChild("armExtender", armExtender);
 


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    armExtender.set(Value.kForward);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    
    
    public void liftWithJoystick(XboxController controller2) {
        double speed = controller2.getLeftX();
        //negitive values are isOuterTriggered, and postive values are isBodyTriggered.
        if (isBodyTriggered() && speed > 0){
            speed = 0;
        } else if (isOuterTriggered() && speed < 0) {
            speed = 0;
        } else
             rightLifter.set(speed);
             leftLifter.set(speed);
             SmartDashboard.putNumber("speed", speed);
    }

    public void toggleArm () {
        armExtender.toggle();
    }
    public void liftToPoint () {

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
}




