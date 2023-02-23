// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;



public class GoToPosition extends CommandBase {
  /** Creates a new GoToPosition. */
  private final Vision vision;
  private final Pigeon pigeon;

  private final DriveTrain driveTrain;

  Pose2d[] listOfEntryLocations = {new Pose2d(0.0, 0.0, new Rotation2d(0)), new Pose2d(0.0, 0.0, new Rotation2d(0))};
  // Pose2d[] scoringLocation = {new Pose2d(0.0, 0.0, new Rotation2d(0)), new Pose2d(0.0, 0.0, new Rotation2d(0))};
  

  public GoToPosition(Vision vision, Pigeon pigeon, DriveTrain driveTrain) {
    this.vision = vision;
    this.pigeon = pigeon;
    this.driveTrain = driveTrain;
    
    addRequirements(vision, pigeon, driveTrain);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final Pose2d robotPosition = vision.getEstimatedPose().get().estimatedPose.toPose2d();

    if (robotPosition.getY() > )

    double distanceFromEntryOne = Math.sqrt(
      Math.pow(listOfEntryLocations[0].getX() - robotPosition.getX(), 2) 
      + 
      Math.pow(listOfEntryLocations[0].getY() - robotPosition.getY(), 2));

    double distanceFromEntryTwo = Math.sqrt(
      Math.pow(listOfEntryLocations[1].getX() - robotPosition.getX(), 2) 
      + 
      Math.pow(listOfEntryLocations[1].getY() - robotPosition.getY(), 2));

    //TODO Change Scroing Point
    if (distanceFromEntryOne <= distanceFromEntryTwo){
      driveTrain.generateThreePointTrajectory(robotPosition, listOfEntryLocations[0], new Pose2d());
    }else{
      driveTrain.generateThreePointTrajectory(robotPosition, listOfEntryLocations[1], new Pose2d());
  
    }

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ;

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
