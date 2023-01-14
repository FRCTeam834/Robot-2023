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



public class DriveTrain extends SubsystemBase {
  SwerveModule frontLeftMotor;
  SwerveModule frontRightMotor;
  SwerveModule backLeftMotor;
  SwerveModule backRightMotor;

  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  
  Pose2d currentPose;

  AHRS gyro;
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeftMotor = new SwerveModule(1,2);
    frontRightMotor = new SwerveModule(3,4);
    backLeftMotor = new SwerveModule(5,6);
    backRightMotor = new SwerveModule(7,8);
    
    kinematics = new SwerveDriveKinematics(
      new Translation2d(0, 0),
      new Translation2d(0, 0),
      new Translation2d(0, 0),
      new Translation2d(0, 0)
    );
    
    odometry = new SwerveDriveOdometry(
      kinematics, new Rotation2d(),
      new SwerveModulePosition[] {
        frontLeftMotor.getModulePosition(),
        frontRightMotor.getModulePosition(),
        backLeftMotor.getModulePosition(),
        backRightMotor.getModulePosition()
      }, new Pose2d(0, 0, new Rotation2d())
    );

    gyro = new AHRS(SPI.Port.kMXP);
  }

  public void drive(double vx, double vy, double omega){
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 3);

    frontLeftMotor.setDesiredState(desiredStates[0]);
    frontRightMotor.setDesiredState(desiredStates[1]);
    backLeftMotor.setDesiredState(desiredStates[2]);
    backRightMotor.setDesiredState(desiredStates[3]);

  }

  public Pose2d getPose() {
    return currentPose;
  }

  @Override
  public void periodic() {

    Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getYaw());

    currentPose = odometry.update(gyroAngle, 
      new SwerveModulePosition [] {
        frontLeftMotor.getModulePosition(),
        frontRightMotor.getModulePosition(),
        backLeftMotor.getModulePosition(),
        backRightMotor.getModulePosition()
      }
    );
    
    // This method will be called once per scheduler run
  }
}