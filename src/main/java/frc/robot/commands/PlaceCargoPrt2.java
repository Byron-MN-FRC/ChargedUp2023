// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LiftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCargoPrt2 extends SequentialCommandGroup {
  /** Creates a new PlaceCargoPrt2. */
  public PlaceCargoPrt2(ClawSubsystem clawSubsystem, LiftSubsystem liftSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawRelease(clawSubsystem),
      new RetractArm(liftSubsystem),
      new WaitCommand(.5),
      new DriveToEncoderBody(liftSubsystem.storedPos, liftSubsystem)
    );
  }
}
