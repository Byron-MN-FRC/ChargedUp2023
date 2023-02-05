package frc.robot.commands;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

import java.util.function.Supplier;

import javax.swing.text.TabExpander;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ChaseTagCommand extends CommandBase {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  private double offset = 0;
  private int TAG_TO_CHASE = 3;
  private Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(1, 0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));
  private final PhotonCamera photonCamera;
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final XboxController controller;

  private final ProfiledPIDController xController = new ProfiledPIDController(2.75, .5, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(2.75, .5, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(3, .5, 0, OMEGA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;
  public ChaseTagCommand(
        PhotonCamera photonCamera, 
        DrivetrainSubsystem drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        XboxController controller) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.controller = controller;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());

    //Checks for tag to chase
    if (controller.getLeftBumper()){
      TAG_TO_CHASE = 1;
    }else if (controller.getRightBumper()){
      TAG_TO_CHASE = 3;
    }else{
      TAG_TO_CHASE = 2;
    }

    // Checking the offset
    if (controller.getBButton()){
      offset = 1;
    }
    else if (controller.getXButton()){
      offset = -1;
    }
    else {
      offset = 0;
    }
    System.out.println("offset " + offset);
    System.out.println("Tag To Chase " + TAG_TO_CHASE);
    TAG_TO_GOAL = 
    new Transform3d(
        new Translation3d(1, offset, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI));
  }

  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      // var targetOpt = photonRes.getBestTarget();
          var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE).findFirst();
          // .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          // .findFirst();
          System.out.println("Seeing Target");
      if (targetOpt.isPresent()) {
        PhotonTrackedTarget target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget == null) {
      // No target has been visible
      drivetrainSubsystem.stop();
      System.out.println("No Target");
    } else {
      // Drive to the target
      var xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }
      System.out.println("This xSpeed" + xSpeed);
      System.out.println("ySpeed"+ySpeed);
      System.out.println("omegaSpeed"+omegaSpeed);
      drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
    }
  }
  @Override
  public boolean isFinished(){
    if (xController.atGoal()&yController.atGoal()&omegaController.atGoal()){
      return true;
    }else{
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}
