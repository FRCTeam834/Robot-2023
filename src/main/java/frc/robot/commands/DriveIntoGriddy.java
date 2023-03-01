// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrainConstants.OnTheFlyConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PoseEstimator;

public class DriveIntoGriddy extends CommandBase {
  /** Creates a new DriveIntoGriddy. */
  private final DriveTrain driveTrain;
  private final PoseEstimator poseEstimator;
  private final String columnName;
  private Pose2d alignmentPose;
  private SequentialCommandGroup currentCommand = new SequentialCommandGroup();
  
  public DriveIntoGriddy(DriveTrain driveTrain, PoseEstimator poseEstimator, String columnName) {
    this.driveTrain = driveTrain;
    this.poseEstimator = poseEstimator;
    this.columnName = columnName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    alignmentPose = OnTheFlyConstants.WAYPOINTS.get(columnName);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = poseEstimator.getEstimatedPose();

    List<Pose2d> OuterChargingStation = new ArrayList<>();
    OuterChargingStation.add(OnTheFlyConstants.WAYPOINTS.get("OuterChargingStationTop"));
    OuterChargingStation.add(OnTheFlyConstants.WAYPOINTS.get("OuterChargingStationBottom"));

    List<Pose2d> InnerChargingStation = new ArrayList<>();
    OuterChargingStation.add(OnTheFlyConstants.WAYPOINTS.get("InnerChargingStationTop"));
    OuterChargingStation.add(OnTheFlyConstants.WAYPOINTS.get("InnerChargingStationBottom"));

    currentCommand.cancel();

    switch (DriverStation.getAlliance()) {
      case Blue: {
        if (currentPose.getX() > OnTheFlyConstants.WAYPOINTS.get("OuterChargingStationTop").getX()) {
          // Robot is behind charging station
          currentCommand = new DriveToNearestWaypoint(driveTrain, poseEstimator, OuterChargingStation);
        } else if (
          currentPose.getX() <= OnTheFlyConstants.WAYPOINTS.get("OuterChargingStationTop").getX() &&
          currentPose.getX() > OnTheFlyConstants.WAYPOINTS.get("InnerChargingStationTop").getX()
        ) {
          // Robot is with the charging station
          currentCommand = new DriveToNearestWaypoint(driveTrain, poseEstimator, InnerChargingStation);
        } else if (currentPose.getX() <= OnTheFlyConstants.WAYPOINTS.get("InnerChargingStationTop").getX()) {
          // Robot is past the charging station
          currentCommand = new DriveToWaypoint(driveTrain, poseEstimator, alignmentPose);
        }
        break;
      }
      
      case Red: {
        if (currentPose.getX() < OnTheFlyConstants.WAYPOINTS.get("OuterChargingStationTop").getX()) {
          currentCommand = new DriveToNearestWaypoint(driveTrain, poseEstimator, OuterChargingStation);
        } else if (
          currentPose.getX() >= OnTheFlyConstants.WAYPOINTS.get("OuterChargingStationTop").getX() &&
          currentPose.getX() < OnTheFlyConstants.WAYPOINTS.get("InnerChargingStationTop").getX()) 
        {
          currentCommand = new DriveToNearestWaypoint(driveTrain, poseEstimator, InnerChargingStation);
        } else if (currentPose.getX() >= OnTheFlyConstants.WAYPOINTS.get("InnerChargingStationTop").getX()) {
          currentCommand = new DriveToWaypoint(driveTrain, poseEstimator, alignmentPose);
        }
            
        
        break;
      }
      case Invalid: break;
    }

    CommandScheduler.getInstance().schedule(currentCommand);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
      Math.abs(poseEstimator.getEstimatedPose().getX() - alignmentPose.getX()) < 0.1 &&
      Math.abs(poseEstimator.getEstimatedPose().getY() - alignmentPose.getY()) < 0.1 &&
      Math.abs(poseEstimator.getEstimatedPose().getRotation().minus(alignmentPose.getRotation()).getRadians()) < 0.1;
  }
}