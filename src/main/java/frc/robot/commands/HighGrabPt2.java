// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LiftSubsystem.LiftPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighGrabPt2 extends SequentialCommandGroup {
  /** Creates a new HighGrab. */
  public HighGrabPt2(LiftSubsystem liftSubsystem, ClawSubsystem clawSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawGrab(clawSubsystem),
      new RetractArm(liftSubsystem),
      new DriveToEncoderBody(liftSubsystem.lowPos, liftSubsystem)
    );
  }
}
