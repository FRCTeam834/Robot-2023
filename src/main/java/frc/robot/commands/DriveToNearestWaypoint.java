// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToNearestWaypoint extends SequentialCommandGroup {
  public DriveToNearestWaypoint(DriveTrain driveTrain, PoseEstimator poseEstimator, List<Pose2d> waypoints) {
    Pose2d currentPose = poseEstimator.getEstimatedPose();
    Pose2d nearestWaypoint = currentPose.nearest(waypoints);
    addCommands(new DriveToWaypoint(driveTrain, poseEstimator, nearestWaypoint));
  }
}
