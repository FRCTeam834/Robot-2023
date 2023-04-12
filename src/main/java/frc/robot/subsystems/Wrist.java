// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final CANSparkMax motor;
  public Wrist() {
    motor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor.setSmartCurrentLimit(20);
    motor.enableVoltageCompensation(12);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
