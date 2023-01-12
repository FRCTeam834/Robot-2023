// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxPIDController;


public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  
  CANSparkMax steerMotor;
  CANSparkMax driveMotor;
  SparkMaxPIDController steerController;
  SparkMaxPIDController driveController;
  RelativeEncoder steerEncoder;
  RelativeEncoder driveEncoder;
  


  public SwerveModule(int SteerID, int DriveID) {
    steerMotor = new CANSparkMax(SteerID, CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(DriveID, CANSparkMaxLowLevel.MotorType.kBrushless);

    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();
    
    steerController = steerMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    steerController.setP(0);
    steerController.setI(0);
    steerController.setD(0);

    driveController.setP(0);
    driveController.setI(0);
    driveController.setD(0);

  }

  public void setDesiredState(SwerveModuleState state) {
    // driveMotor.set(state.speedMetersPerSecond / 3);
    
    if(state.speedMetersPerSecond <= 0.1){
      driveMotor.set(0);
      return;
    }
    
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(steerEncoder.getPosition()));
    
    steerController.setReference(state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    driveController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    
  }

  public void stopModule(){
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(steerEncoder.getPosition()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
