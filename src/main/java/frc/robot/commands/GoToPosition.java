// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.scoringLocation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToPosition extends SequentialCommandGroup {
  /** Creates a new GoToPosition. */
  
  ArrayList<PathPoint> waypoints = new ArrayList<PathPoint>();

  public GoToPosition(   
     DriveTrain driveTrain,
     PoseEstimator poseEstimator,
     scoringLocation preset
    ) {
      
      final Pose2d robotPosition = poseEstimator.getEstimatedPose();
      waypoints.add(new PathPoint(robotPosition.getTranslation(), robotPosition.getRotation()));
  
      if (robotPosition.getY() > 0) {
        Pose2d farEntryPoint1 = Constants.entryWaypoints[0][0];
        Pose2d farEntryPoint2 = Constants.entryWaypoints[0][1];
  
        //TODO Change Scroing Point
  
        if(driveTrain.isDistanceOneGreater(robotPosition, farEntryPoint1, farEntryPoint2)){
          waypoints.add(new PathPoint(farEntryPoint1.getTranslation(), farEntryPoint1.getRotation()));
        }else{
          waypoints.add(new PathPoint(farEntryPoint2.getTranslation(), farEntryPoint2.getRotation()));
        } 
  
        } 
        if(robotPosition.getY() > 0){
          Pose2d closeEntryPoint1 = Constants.entryWaypoints[1][0];
          Pose2d closeEntryPoint2 = Constants.entryWaypoints[1][1];
          
          
          if(driveTrain.isDistanceOneGreater(robotPosition, closeEntryPoint1, closeEntryPoint2)){
            waypoints.add(new PathPoint(closeEntryPoint1.getTranslation(), closeEntryPoint1.getRotation()));
          }else{
            waypoints.add(new PathPoint(closeEntryPoint2.getTranslation(), closeEntryPoint2.getRotation()));
          } 
        }

        waypoints.add(preset.positions);



        

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
      DriveTrainConstants.PATH_CONSTRAINTS, 
      waypoints);

    addCommands(
      driveTrain.followTrajectoryCommand(trajectory, poseEstimator, false)
      );
  }
}
