// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  
  private final AprilTagFieldLayout fieldLayout;
  private final RobotPoseEstimator poseEstimator;
  private Pose3d lastEstimatedPose;
  
  public Vision() throws IOException {
    fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    poseEstimator = new RobotPoseEstimator(
      fieldLayout,
      VisionConstants.POSE_ESTIMATION_STRATEGY,
      VisionConstants.CAMERAS
    );
  }

  /**
   * 
   * @return Robot global pose2d and camera latency in ms
   */
  public Pair<Pose2d, Double> getEstimatedPose () {
    Optional<Pair<Pose3d, Double>> result = null;
    double currentTime = Timer.getFPGATimestamp();

    switch (VisionConstants.POSE_ESTIMATION_STRATEGY) {
      case CLOSEST_TO_LAST_POSE: {
        poseEstimator.setLastPose(lastEstimatedPose);
        result = poseEstimator.update();
        break;
      }
      default: break;
    }

    if (result == null || !result.isPresent()) {
      return new Pair<Pose2d, Double> (null, 0.0);
    }
    return new Pair<Pose2d, Double> (
      result.get().getFirst().toPose2d(),
      (currentTime - result.get().getSecond()) / 1000.0
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
