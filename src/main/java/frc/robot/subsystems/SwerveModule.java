// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  
  CANSparkMax steerMotor;
  CANSparkMax driveMotor;
  
  public SwerveModule(int SteerID, int DriveID) {
    steerMotor = new CANSparkMax(SteerID, CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(DriveID, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  public void setDesiredState(SwerveModuleState state) {
    steerMotor.setPosition(state.angle);
    driveMotor.setSpeed(state.speedMetersPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
