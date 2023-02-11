// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LifterConstants;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LiftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Path1 extends SequentialCommandGroup {
  /** Creates a new Path1. */
  public Path1(DrivetrainSubsystem dSubsystem, LiftSubsystem liftSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToEncoder(LifterConstants.highPos,liftSubsystem),
    //  new ArmToDropPosition(liftSubsystem),
      new ExtendArm(liftSubsystem), 
      new ToggleClaw(clawSubsystem),
      new RetractArm(liftSubsystem),
      new DriveToEncoder(LifterConstants.lowPos,liftSubsystem),
      new DriveToPoint()
    );
  }
}
