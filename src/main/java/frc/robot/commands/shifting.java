// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
public class shifting extends CommandBase {
  private double m_increment;
  private double speed = 1;
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  /** Creates a new shifting. */
  public shifting(DrivetrainSubsystem DrivetrainSubsystem, double increment) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_increment = increment;
    m_DrivetrainSubsystem = DrivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((speed==1) && (m_increment>0)) {

    }else{
      speed = speed+m_increment;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_DrivetrainSubsystem.setDriveSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
