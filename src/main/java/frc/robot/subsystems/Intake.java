// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final LinearFilter filter;
  private boolean isStalled;

  public Intake() {
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, CANSparkMaxLowLevel.motorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    filter = LinearFilter.movingAverage(IntakeConstants.NUM_SAMPLES);
  }

  public boolean isMotorStalled(){
    return isStalled;
  }

  public void startMotorForward(){
    intakeMotor.set(IntakeConstants.MOTOR_SPEED);
  }

  public void startMotorBackward() {
    intakeMotor.set(-IntakeConstants.MOTOR_SPEED);
  }

  public void stopMotor(){
    intakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    isStalled = filter.calculate(intakeMotor.getOutputCurrent()) > IntakeConstants.CURRENT_THRESHOLD;
  }
}
