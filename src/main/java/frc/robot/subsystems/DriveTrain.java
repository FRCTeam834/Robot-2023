// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.SwerveModuleFactory;

public class DriveTrain extends SubsystemBase {
  SwerveModule frontLeft;
  SwerveModule frontRight;
  SwerveModule backLeft;
  SwerveModule backRight;

  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  AHRS gyro;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.zeroYaw();

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

    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(-gyro.getYaw()), this.getModulePositions());
  }

  /**
   * Field centric by default
   */
  public void drive(double vx, double vy, double omega) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(gyro.getYaw()));
    this.setChassisSpeeds(speeds);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d newPose) {
    odometry.resetPosition(Rotation2d.fromDegrees(-this.gyro.getYaw()), getModulePositions(), newPose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** For pathplanner auton api */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 3);

    this.setModuleStates(desiredStates);
  }

  public SwerveModulePosition[] getModulePositions () {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = frontLeft.getModulePosition();
    modulePositions[1] = frontRight.getModulePosition();
    modulePositions[2] = backLeft.getModulePosition();
    modulePositions[3] = backRight.getModulePosition();
    return modulePositions;
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
    odometry.update(Rotation2d.fromDegrees(-gyro.getYaw()), this.getModulePositions());
  }
}
