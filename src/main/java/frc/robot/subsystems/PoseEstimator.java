// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
  
  private final DriveTrain driveTrain;
  private final Pigeon gyro;
  private final Vision vision;
  private final SwerveDrivePoseEstimator poseEstimator;

  public PoseEstimator(
    DriveTrain driveTrain,
    Pigeon gyro,
    Vision vision
  ) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    this.vision = vision;

    poseEstimator = new SwerveDrivePoseEstimator(
      null,
      null,
      null,
      null
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
