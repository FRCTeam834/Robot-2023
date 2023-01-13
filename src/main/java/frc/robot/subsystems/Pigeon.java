// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pigeon extends SubsystemBase {
  /** TEMPORARY; NAVX FOR TESTING PURPOSES ONLY */
  AHRS gyro;
  public Pigeon() {
    gyro = new AHRS(SPI.Port.kMXP);
  }

  public double getYaw () {
    return -gyro.getYaw();
  }

  public Rotation2d getYawAsRotation2d () {
    return Rotation2d.fromDegrees(this.getYaw());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
