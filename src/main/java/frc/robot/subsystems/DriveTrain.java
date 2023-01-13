// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private final Pigeon gyro;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;

  public DriveTrain(Pigeon gyro) {
    this.gyro = gyro;

    frontLeft = SwerveModule.buildFrontLeft();
    frontRight = SwerveModule.buildFrontRight();
    backLeft = SwerveModule.buildBackLeft();
    backRight = SwerveModule.buildBackRight();

    kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DRIVETRAIN.WIDTH / 2.0, Constants.DRIVETRAIN.LENGTH / 2.0),
      new Translation2d(-Constants.DRIVETRAIN.WIDTH / 2.0, Constants.DRIVETRAIN.LENGTH / 2.0),
      new Translation2d(Constants.DRIVETRAIN.WIDTH / 2.0, -Constants.DRIVETRAIN.LENGTH / 2.0),
      new Translation2d(-Constants.DRIVETRAIN.WIDTH / 2.0, -Constants.DRIVETRAIN.LENGTH / 2.0)
    );
  }

  public SwerveDriveKinematics getKinematics () {
    return this.kinematics;
  }

  public SwerveModulePosition[] getModulePositions () {
    return new SwerveModulePosition [] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    };
  }

  /** Set desired module states */
  public void setModuleStates (SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Set desired module states open loop */
  public void setModuleStatesOpenLoop (SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredStateOpenLoop(desiredStates[0]);
    frontRight.setDesiredStateOpenLoop(desiredStates[1]);
    backLeft.setDesiredStateOpenLoop(desiredStates[2]);
    backRight.setDesiredStateOpenLoop(desiredStates[3]);
  }

  /**
   * Field centric drive
   * @param vx - x velocity in m/s
   * @param vy - y velocity in m/s
   * @param omega - angular velocity in rad/s (note: ccw is positive)
   */
  public void drive (
    double vx,
    double vy,
    double omega
  ) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, this.gyro.getYawAsRotation2d());
    SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(speeds);

    this.setModuleStates(desiredStates);
  }

  /**
   * Field centric drive
   * @param vx - x velocity in m/s
   * @param vy - y velocity in m/s
   * @param omega - angular velocity in rad/s (note: ccw is positive)
   */
  public void driveOpenLoop (
    double vx,
    double vy,
    double omega
  ) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, this.gyro.getYawAsRotation2d());
    SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(speeds);

    this.setModuleStatesOpenLoop(desiredStates);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
