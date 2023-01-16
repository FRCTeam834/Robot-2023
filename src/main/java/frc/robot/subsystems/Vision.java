// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  
  private final PhotonCamera camera;
  private final AprilTagFieldLayout fieldLayout;
  
  public Vision() {
    camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
    fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
  }

  public Pose3d getPoseFromVision () {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
