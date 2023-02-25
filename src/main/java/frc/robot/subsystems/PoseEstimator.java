// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseEstimatorConstants;

public class PoseEstimator extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final DriveTrain driveTrain;
  private final Pigeon gyro;
  private final Vision vision;

  private final Field2d telemetryField = new Field2d();

  public PoseEstimator(
    SwerveDriveKinematics kinematics,
    DriveTrain driveTrain,
    Pigeon gyro,
    Vision vision
  ) {
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      gyro.getYawAsRotation2d(),
      driveTrain.getModulePositions(),
      new Pose2d(),
      PoseEstimatorConstants.STATE_STDDEVS,
      PoseEstimatorConstants.VISION_STDDEVS
    );

    this.driveTrain = driveTrain;
    this.gyro = gyro;
    this.vision = vision;

    SmartDashboard.putData("Pose Estimator Pose", telemetryField);
  }

  /**
   * 
   * Reset odometry to a known pose
   * @param pose
   */
  public void resetOdometry (Pose2d pose) {
    gyro.resetYaw(pose.getRotation().getRadians());
    poseEstimator.resetPosition(gyro.getYawAsRotation2d(), driveTrain.getModulePositions(), pose);
  }

  public Pose2d getEstimatedPose () {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    /* Update the odometry */
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getYawAsRotation2d(), driveTrain.getModulePositions());

    /* Update with vision measurements */
    vision.setReferencePose(this.getEstimatedPose());
    Optional<EstimatedRobotPose> visionMeasurement = vision.getEstimatedPose();
    if (visionMeasurement.isPresent()) {
      poseEstimator.addVisionMeasurement(
        visionMeasurement.get().estimatedPose.toPose2d(),
        visionMeasurement.get().timestampSeconds
      );
    }

    telemetryField.setRobotPose(this.getEstimatedPose());
  }
}
