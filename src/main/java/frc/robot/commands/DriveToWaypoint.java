// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
public class DriveToWaypoint extends SequentialCommandGroup {
  public DriveToWaypoint(DriveTrain driveTrain, PoseEstimator poseEstimator, Pose2d waypoint) {
    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
      new PathConstraints(
        DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
        DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
      ),
      PathPoint.fromCurrentHolonomicState(poseEstimator.getEstimatedPose(), driveTrain.getLastChassisSpeeds()),
      new PathPoint(waypoint.getTranslation(), waypoint.getRotation())
    );

    addCommands(driveTrain.followTrajectoryCommand(trajectory, poseEstimator, false));
  }
}
