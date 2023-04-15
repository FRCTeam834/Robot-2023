// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCone;

public class Intake extends SubsystemBase {
  
  public final CANSparkMax motor;
  private final RelativeEncoder encoder;

  public GamePieceType currentGamePiece = GamePieceType.NONE;
  
  public Intake() {
    motor = new CANSparkMax(IntakeConstants.CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();

    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    encoder.setAverageDepth(2);
    encoder.setMeasurementPeriod(5);

    encoder.setVelocityConversionFactor(1.0 / IntakeConstants.GEAR_RATIO);

    if (Constants.competitionMode) {
      motor.burnFlash();
    }
  }

  public double getRPM () {
    return encoder.getVelocity();
  }

  public void setVoltage (double voltage) {
    motor.setVoltage(voltage);
  }

  public void stop () {
    motor.set(0.0);
  }

  public boolean hasCube () {
    return currentGamePiece == GamePieceType.CUBE;
  }

  public boolean hasCone () {
    return currentGamePiece == GamePieceType.CONE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
