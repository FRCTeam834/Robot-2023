// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;

public class Pigeon extends SubsystemBase {
  Pigeon2 gyro;
  
  public Pigeon() {
    gyro = new Pigeon2(CANIDS.PIGEON);
  }

  public double getYaw() {
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
