// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  public DriveTrain() {
    frontLeft = SwerveModule.buildFrontLeft();
    frontRight = SwerveModule.buildFrontRight();
    backLeft = SwerveModule.buildBackLeft();
    backRight = SwerveModule.buildBackRight();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
