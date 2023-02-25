// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;



public class GoToPositionOld extends CommandBase {
  /** Creates a new GoToPosition. */
  private final Vision vision;
  private final DriveTrain driveTrain;

  ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();

  

  public GoToPositionOld(Vision vision, DriveTrain driveTrain) {
    this.vision = vision;
    this.driveTrain = driveTrain;
    
    addRequirements(vision, driveTrain);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final Pose2d robotPosition = vision.getEstimatedPose().get().estimatedPose.toPose2d();
    waypoints.add(robotPosition);

    if (robotPosition.getY() > 0) {
      Pose2d farEntryPoint1 = Constants.entryWaypoints[0][0];
      Pose2d farEntryPoint2 = Constants.entryWaypoints[0][1];

      //TODO Change Scroing Point

      if(driveTrain.isDistanceOneGreater(robotPosition, farEntryPoint1, farEntryPoint2)){
        waypoints.add(farEntryPoint1);
      }else{
        waypoints.add(farEntryPoint2);
      } 

      } 
      if(robotPosition.getY() > 0){
        Pose2d closeEntryPoint1 = Constants.entryWaypoints[1][0];
        Pose2d closeEntryPoint2 = Constants.entryWaypoints[1][1];
        
        if(driveTrain.isDistanceOneGreater(robotPosition, closeEntryPoint1, closeEntryPoint2)){
          waypoints.add(closeEntryPoint1);
        }else{
          waypoints.add(closeEntryPoint2);
        } 
      }

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
