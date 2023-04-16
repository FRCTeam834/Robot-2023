// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final CANSparkMax motor;
  private final SparkMaxPIDController controller;
  public final RelativeEncoder encoder;
  private double setpoint = 0.0;

  public Wrist() {
    motor = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(true);
    motor.setSmartCurrentLimit(20);
    motor.enableVoltageCompensation(12);
    controller = motor.getPIDController();
    encoder = motor.getEncoder();
    controller.setP(1);
    controller.setI(0);
    controller.setD(0);

    encoder.setPositionConversionFactor(2 * Math.PI / 100);
    encoder.setVelocityConversionFactor(2 * Math.PI / (100 * 60));

    if (Constants.competitionMode) {
      motor.burnFlash();
    }

    SmartDashboard.putData(this);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setPosition(double position) {
    setpoint = position;
    controller.setReference(position, ControlType.kPosition);
  }
  
  public double getPosition () {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.telemetryMode == false) return;

    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Position", () -> Units.radiansToDegrees(this.getPosition()), null);
    builder.addDoubleProperty("Setpoint", () -> Units.radiansToDegrees(setpoint), null);
  }
}
