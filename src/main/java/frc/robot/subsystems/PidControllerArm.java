// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PidControllerArm extends SubsystemBase {
  /** Creates a new PidControllerArm. */
  CANSparkMax armMotor;
  RelativeEncoder armEncoder;
  ArmFeedforward feedforward;
  ProfiledPIDController armController;

  boolean isStopped = true;

  
  public PidControllerArm() {
    armMotor = new CANSparkMax(15, MotorType.kBrushless);
    armEncoder = armMotor.getEncoder();

    armMotor.setSmartCurrentLimit(30);
    armMotor.enableVoltageCompensation(12);
    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


    armEncoder.setPositionConversionFactor(2.0 * Math.PI / 100.0);
    armEncoder.setVelocityConversionFactor(2.0 * Math.PI / (60.0 * 100.0));

    armEncoder.setPosition(0);


    feedforward = new ArmFeedforward(1, 0.59, 1.95, 0.09);
    armController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    
  }
  
  public void setGoal(double x){
    isStopped = false;
    armController.setGoal(x);
  }

  public void stop(){
    armMotor.set(0.0);
    isStopped = true;


  }

  @Override
  public void periodic() {
    if (isStopped)
      return;
    
    // This method will be called once per scheduler run
      armMotor.setVoltage(feedforward.calculate(armEncoder.getPosition(), armEncoder.getVelocity())
                          + armController.calculate(armEncoder.getPosition()));
  }

    
}
