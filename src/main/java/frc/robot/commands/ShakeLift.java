// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LiftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShakeLift extends SequentialCommandGroup {
  /** Creates a new ShakeLift. */
  public ShakeLift(LiftSubsystem m_liftSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (RobotContainer.getInstance().getDriveController().getBackButton()){
      addCommands(
        new DriveToEncoderBody(m_liftSubsystem.highPos-5000, m_liftSubsystem),
        new DriveToEncoderOuter(m_liftSubsystem.highPos, m_liftSubsystem, false)
        );

    }
  }
}
