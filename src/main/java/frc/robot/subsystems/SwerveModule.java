// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax steerMotor;
  private final CANSparkMax driveMotor;
  private final SparkMaxPIDController steerController;
  private final SparkMaxPIDController driveController;
  private final RelativeEncoder steerEncoder;
  private final RelativeEncoder driveEncoder;
  /** Creates a new SwerveModule. */
  public SwerveModule(int STEERID, int DRIVEID) {
    steerMotor = new CANSparkMax(STEERID, CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(DRIVEID, CANSparkMaxLowLevel.MotorType.kBrushless);

    steerController = steerMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setPositionPIDWrappingMaxInput(359);
    steerController.setPositionPIDWrappingMinInput(0);

    steerController.setP(0.05);
    steerController.setI(0);
    steerController.setD(0);
    driveController.setP(0.5);
    driveController.setI(0);
    driveController.setD(0);

    steerEncoder = steerMotor.getEncoder();
    steerEncoder.setPosition(-Math.PI / 2);
    steerEncoder.setPositionConversionFactor(360.0 / 12.8);
    
    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(Math.PI * 0.127 / 8.14);
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) <= 0.1) {
      this.halt();
      return;
    }
    state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(steerEncoder.getPosition()));
    driveMotor.set(state.speedMetersPerSecond / 3);
    steerController.setReference(state.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(steerEncoder.getPosition()));
  }

  public void halt() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
