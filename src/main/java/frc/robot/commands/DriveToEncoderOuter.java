// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.LiftSubsystem;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class DriveToEncoderOuter extends CommandBase {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
        private final LiftSubsystem m_liftSubsystem;
    private double m_targetEncoders;
    private boolean m_armExtender;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    private double startingPosition;

    public DriveToEncoderOuter(double targetEncoders, LiftSubsystem subsystem, boolean armExtender) {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        m_targetEncoders = targetEncoders;
        m_armExtender = armExtender;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        m_liftSubsystem = subsystem;
        addRequirements(m_liftSubsystem);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if (m_targetEncoders == -1) {
        //     if (RobotContainer.getInstance().getAttachmentController().getPOV() == -1) {
        //         m_targetEncoders = m_liftSubsystem.storedPos;
        //     } else if (RobotContainer.getInstance().getAttachmentController().getPOV() <= 45
        //             || RobotContainer.getInstance().getAttachmentController().getPOV() >= 315) {
        //         m_targetEncoders = m_liftSubsystem.highPos;
        //     } else if (RobotContainer.getInstance().getAttachmentController().getPOV() <= 225
        //             || RobotContainer.getInstance().getAttachmentController().getPOV() >= 135) {
        //         m_targetEncoders = m_liftSubsystem.lowPos;
        //     }
        // }
        startingPosition = m_liftSubsystem.getLifttEncoder();
        SmartDashboard.putNumber("StartingPos", startingPosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_liftSubsystem.setLiftPos(m_targetEncoders);
        double POV = RobotContainer.getInstance().getAttachmentController().getPOV();

        if ((POV <= 45 || POV >= 315) && POV != -1)  {
                m_armExtender = true;
        }
        if (m_armExtender && (m_liftSubsystem.getLifttEncoder()>=(m_liftSubsystem.highPos*2/3)) && Math.abs(startingPosition-m_liftSubsystem.getLifttEncoder())>=100000){
            m_liftSubsystem.extendArm();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if(m_liftSubsystem.isOuterTriggered()){
            m_liftSubsystem.stopLift();
            m_liftSubsystem.setHighLift();
        }
        m_armExtender = false;

        // m_targetEncoders = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
            return m_liftSubsystem.targetEncoder(m_targetEncoders)||m_liftSubsystem.isOuterTriggered();
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
