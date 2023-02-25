// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LinearPath extends SequentialCommandGroup {
  /** Temporary test auton */
  public LinearPath(
    DriveTrain driveTrain,
    PoseEstimator poseEstimator
  ) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath(
      "Linear Path",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );
    addCommands(
      driveTrain.followTrajectoryCommand(trajectory, poseEstimator, true)
    );
  }
}
