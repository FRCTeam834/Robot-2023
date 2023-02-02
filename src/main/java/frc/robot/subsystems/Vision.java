// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private Optional<EstimatedRobotPose> estimatedPose;

  private final Field2d telemetryField = new Field2d();

  private static AprilTagFieldLayout loadFieldLayout () {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  public Vision() {
    camera = new PhotonCamera(VisionConstants.PHOTON_CAMERA_NAME);
    photonPoseEstimator = new PhotonPoseEstimator(
      Vision.loadFieldLayout(),
      VisionConstants.POSE_ESTIMATION_STRATEGY,
      camera,
      VisionConstants.ROBOT_TO_CAMERA_TRANSFORM
    );

    SmartDashboard.putData("Vision Pose", telemetryField);
  }

  public void setReferencePose (Pose2d pose) {
    photonPoseEstimator.setReferencePose(pose);
  }

  public Optional<EstimatedRobotPose> getEstimatedPose () {
    return estimatedPose;
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (!Constants.telemetryMode) return;
    
    builder.setSmartDashboardType("Vision");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    estimatedPose = photonPoseEstimator.update();

    if (estimatedPose.isPresent()) {
      telemetryField.setRobotPose(estimatedPose.get().estimatedPose.toPose2d());
    }
  }
}