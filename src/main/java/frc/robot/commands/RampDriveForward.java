// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RampDriveForward extends CommandBase {
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private boolean rampUp=false;
  private boolean rampDown=false;
  private double balanceAngle;
  private double balanceSpeed;
  /** Creates a new RampDrive. */
  public RampDriveForward(DrivetrainSubsystem drivetrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.getNumber("balanceAngle", 15);
    SmartDashboard.getNumber("balanceSpeed", .35);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds((balanceSpeed) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, (0), (0),m_DrivetrainSubsystem.getGyroscopeRotation()));
    if (m_DrivetrainSubsystem.getPitch()>=balanceAngle||m_DrivetrainSubsystem.getPitch()<=-balanceAngle){
      rampUp=true;
    }
    
    // // Consider < -5 instead of +15
    // // Need Timeout   
    // if (m_DrivetrainSubsystem.getPitch()<=-15 && rampUp){
    //   // reduce speed here?
    //   rampDown = true;
    // }
    SmartDashboard.putBoolean("RampUp", rampUp);
    SmartDashboard.putBoolean("RampDown", rampDown);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rampUp;
  }
}
