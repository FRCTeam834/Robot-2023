// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.PIDGains;

public class SwerveModule extends SubsystemBase {
  
  private final CANSparkMax steerMotor;
  private final CANSparkMax driveMotor;

  private final SparkMaxPIDController steerController;
  private final SparkMaxPIDController driveController;

  private final RelativeEncoder steerEncoder;
  private final RelativeEncoder driveEncoder;

  /* Factory */
  public final static SwerveModule buildModule (
    int STEERID, int DRIVEID,
    PIDGains steerPID,
    PIDGains drivePID
  ) {
    SwerveModule module = new SwerveModule(STEERID, DRIVEID);
    SparkMaxPIDController steerController = module.getSteerController();
    SparkMaxPIDController driveController = module.getDriveController();
    RelativeEncoder steerEncoder = module.getSteerEncoder();
    RelativeEncoder driveEncoder = module.getDriveEncoder();

    steerPID.bindToController(steerController);
    drivePID.bindToController(driveController);

    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setPositionPIDWrappingMaxInput(359);
    steerController.setPositionPIDWrappingMaxInput(0);

    steerEncoder.setPositionConversionFactor(360.0 / Constants.DRIVETRAIN.STEER_GEAR_RATIO);
    driveEncoder.setPositionConversionFactor(Math.PI * Constants.DRIVETRAIN.WHEEL_DIAMETER / Constants.DRIVETRAIN.DRIVE_GEAR_RATIO);

    return module;
  }

  public final static SwerveModule buildFrontLeft () {
    return SwerveModule.buildModule(
      Constants.CANIDS.FL_STEER,
      Constants.CANIDS.FL_DRIVE,
      Constants.PIDGAINS.MODULE_STEER,
      Constants.PIDGAINS.MODULE_DRIVE
    );
  }

  public SwerveModule(int STEERID, int DRIVEID) {
    steerMotor = new CANSparkMax(STEERID, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  /* Getters */
  public CANSparkMax getSteerMotor () {
    return this.steerMotor;
  }

  public CANSparkMax getDriveMotor () {
    return this.driveMotor;
  }

  public SparkMaxPIDController getSteerController () {
    return this.steerController;
  }

  public SparkMaxPIDController getDriveController () {
    return this.driveController;
  }

  public RelativeEncoder getSteerEncoder () {
    return this.steerEncoder;
  }

  public RelativeEncoder getDriveEncoder () {
    return this.driveEncoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
