// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveModule extends SubsystemBase {
  private final String name;

  private final CANSparkMax steerMotor;
  private final CANSparkMax driveMotor;

  private final RelativeEncoder steerEncoder;
  private final RelativeEncoder driveEncoder;

  private final SparkMaxPIDController steerController;
  private final SparkMaxPIDController driveController;

  public SwerveModule(int ID) {
    name = DriveTrainConstants.MODULE_NAMES[ID];

    steerMotor = new CANSparkMax(DriveTrainConstants.STEERIDS[ID], CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(DriveTrainConstants.DRIVEIDS[ID], CANSparkMaxLowLevel.MotorType.kBrushless);

    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    steerController = steerMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    driveEncoder.setMeasurementPeriod(8);
    driveEncoder.setAverageDepth(8);

    if (Constants.competitionMode) {
      steerMotor.burnFlash();
      driveMotor.burnFlash();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
