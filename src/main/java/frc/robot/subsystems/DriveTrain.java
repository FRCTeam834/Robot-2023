// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrain extends SubsystemBase {
  SwerveModule frontLeftMotor;
  SwerveModule frontRightMotor;
  SwerveModule backLeftMotor;
  SwerveModule backRightMotor;


  SwerveDriveKinematics kinematics;
  
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
  }




  public void drive(double vx, double vy, double omega){
    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    SwerveModuleState[] desiredStates = this.kinematics.toSwerveModuleStates(speeds);

    frontLeftMotor.setDesiredState(desiredStates[0]);
    frontRightMotor.setDesiredState(desiredStates[1]);
    backLeftMotor.setDesiredState(desiredStates[2]);
    backRightMotor.setDesiredState(desiredStates[3]);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
