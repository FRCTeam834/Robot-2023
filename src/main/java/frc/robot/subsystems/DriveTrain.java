// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.SwerveModuleFactory;

public class DriveTrain extends SubsystemBase {
  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  SwerveDriveKinematics kinematics;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeft = SwerveModuleFactory.getFrontLeft();
    frontRight = SwerveModuleFactory.getFrontRight();
    backLeft = SwerveModuleFactory.getBackLeft();
    backRight = SwerveModuleFactory.getBackRight();

    kinematics = new SwerveDriveKinematics(
      new Translation2d(0.28448, 0.28448),
      new Translation2d(0.28448, -0.28448),
      new Translation2d(-0.28448, 0.28448),
      new Translation2d(-0.28448, -0.28448)
    );
  }

  /**
   * Field centric by default
   */
  public void drive(double vx, double vy, double omega) {
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 3);

    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void halt() {
    frontLeft.halt();
    frontRight.halt();
    backLeft.halt();
    backRight.halt();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
