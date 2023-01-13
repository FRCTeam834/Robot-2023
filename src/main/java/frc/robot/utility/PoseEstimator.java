// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

/** Add your docs here. */
public class PoseEstimator {
    private final DriveTrain driveTrain;
    private final Pigeon gyro;

    private final SwerveDrivePoseEstimator poseEstimator;
    private Pose2d estimatedPose;

    public PoseEstimator (DriveTrain driveTrain, Pigeon gyro) {
        this.driveTrain = driveTrain;
        this.gyro = gyro;

        poseEstimator = new SwerveDrivePoseEstimator(
            this.driveTrain.getKinematics(),
            this.gyro.getYawAsRotation2d(),
            this.driveTrain.getModulePositions(),
            new Pose2d(),
            Constants.POSEESTIMATOR.STATE_STDDEVS,
            Constants.POSEESTIMATOR.VISION_STDDEVS
        );
    }

    public Pose2d getEstimatedPose () {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void update () {
        this.poseEstimator.update(this.gyro.getYawAsRotation2d(), this.driveTrain.getModulePositions());
    }
}
