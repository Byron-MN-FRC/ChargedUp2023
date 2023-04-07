// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToLight extends CommandBase {
  /** Creates a new DriveToLight. */
  private PhotonCamera m_PhotonCamera;
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);

  private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);

  public DriveToLight(DrivetrainSubsystem drivetrainSubsystem, PhotonCamera photonCamera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PhotonCamera = photonCamera;
    m_DrivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PhotonCamera.setPipelineIndex(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_PhotonCamera.getLatestResult().hasTargets()){
    double targetYaw = m_PhotonCamera.getLatestResult().getBestTarget().getYaw();
    SmartDashboard.putNumber("Light Target??", targetYaw);
    
    m_DrivetrainSubsystem.drive(new ChassisSpeeds(0, targetYaw, 0));
    }
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
