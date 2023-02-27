// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainConstants.OnTheFlyConstants;

public class DriveToPreset extends CommandBase {

  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;
  private String presetName;
  
  private Command followTrajectoryCommand;

  public DriveToPreset(
    DriveTrain driveTrain,
    PoseEstimator poseEstimator,
    String presetName
  ) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;
    this.presetName = presetName;
  }

  /**
   * Choose the best waypoint for a step in a path
   * @param robotPose
   * @param waypoints
   * @param destination
   * @return
   */
  private static Pose2d chooseWaypointFromStep (Pose2d robotPose, Pose2d[] waypoints, Pose2d destination) {
    Pose2d candidate = null;
    double candidateDistance = Double.POSITIVE_INFINITY;
    
    for (Pose2d waypoint : waypoints) {
      // Bad waypoint if it brings us farther away from the final destination
      if (destination.getX() < robotPose.getX() != waypoint.getX() < robotPose.getX()) continue;
      // Choose closest waypoint
      double distance = Math.hypot(waypoint.getX() - robotPose.getX(), waypoint.getY() - robotPose.getY());
      if(distance < candidateDistance){
        candidate = waypoint;
        candidateDistance = distance;
      }
    }
    return candidate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pair<Pose2d, Pose2d[][]> preset = OnTheFlyConstants.PRESETS.get(presetName);

    Pose2d robotPose = poseEstimator.getEstimatedPose();
    List<PathPoint> pathPoints = new ArrayList<PathPoint>();
    
    // Path begins at current robot position
    pathPoints.add(PathPoint.fromCurrentHolonomicState(robotPose, driveTrain.getLastChassisSpeeds()));

    for (Pose2d[] step : preset.getSecond()){
      Pose2d chosenWaypoint = DriveToPreset.chooseWaypointFromStep(robotPose, step, preset.getFirst());
      if(chosenWaypoint == null) continue;

      pathPoints.add(new PathPoint(
        chosenWaypoint.getTranslation(),
        chosenWaypoint.getRotation()
      ));
      robotPose = chosenWaypoint;
    }

    // Add final destination to path
    pathPoints.add(new PathPoint(
      preset.getFirst().getTranslation(),
      preset.getFirst().getRotation()
    ));

    PathPlannerTrajectory trajectory = PathPlanner.generatePath(
      new PathConstraints(
        DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
        DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
      ),
      pathPoints
    );

    followTrajectoryCommand = driveTrain.followTrajectoryCommand(trajectory, poseEstimator, false);
    CommandScheduler.getInstance().schedule(followTrajectoryCommand);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    followTrajectoryCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
