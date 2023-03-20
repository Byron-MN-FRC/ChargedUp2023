// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriverInterface.LateralPosition;

public class AccessoryXButton extends CommandBase {

  public AccessoryXButton() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LateralPosition pos = RobotContainer.getInstance().m_driverInterface.AprilTagOffset();

    switch (pos) {
      case Right:
        SmartDashboard.putBoolean("roM", true); // Robot Position Middle April Tag;
        break;
      case Center:
        SmartDashboard.putBoolean("roL", true); // Robot Position Left April Tag;
        break;
      default: // do nothing
        SmartDashboard.putBoolean("roL", true); // Robot Position Left April Tag;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
