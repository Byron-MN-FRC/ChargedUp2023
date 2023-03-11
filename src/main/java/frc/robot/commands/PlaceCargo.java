// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.LifterConstants;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class PlaceCargo extends SequentialCommandGroup {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    public PlaceCargo(ClawSubsystem clawSubsystem, LiftSubsystem liftSubsystem) {

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        addCommands(

                new DriveToEncoderOuter(liftSubsystem.highPos, liftSubsystem, false),
                new DriveToEncoderOuter(liftSubsystem.highPos-5000, liftSubsystem, false),
                new DriveToEncoderOuter(liftSubsystem.highPos, liftSubsystem, false)

                // new ExtendArm(false, liftSubsystem)


          //      new ClawRelease(clawSubsystem)

        // Add Commands here:
        // Also add parallel commands using the
        //
        // addCommands(
        // new command1(argsN, subsystem),
        // parallel(
        // new command2(argsN, subsystem),
        // new command3(argsN, subsystem)
        // )
        // );

        );
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
