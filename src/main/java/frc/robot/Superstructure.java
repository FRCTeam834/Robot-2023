// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.utility.PoseEstimator;

/** */
public class Superstructure extends SubsystemBase {

  private final DriveTrain driveTrain;
  private final Pigeon gyro;

  private final PoseEstimator poseEstimator;

  public Superstructure(
    DriveTrain driveTrain,
    Pigeon gyro
  ) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;

    this.poseEstimator = new PoseEstimator(driveTrain, gyro);
  }


  @Override
  public void periodic() {
    this.poseEstimator.update();
  }
}
