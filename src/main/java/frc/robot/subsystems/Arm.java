// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  // You thought this was a useful comment, but it was me, DIO!
  
  CANSparkMax pivotMotor;
  CANSparkMax rizzMotor;

  RelativeEncoder pivotMotorEncoder;
  RelativeEncoder rizzMotorEncoder;

  public Arm() {
    pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    rizzMotor = new CANSparkMax(Constants.RIZZ_MOTOR_ID, CANSparkMax.MotorType.kBrushless);

    pivotMotorEncoder = pivotMotor.getEncoder();
    rizzMotorEncoder = rizzMotor.getEncoder();

    

  }

  public void setPivotSpeed(double speed){
    pivotMotor.set(speed);
  }
  
  public double getPivotSpeed(){
    return pivotMotor.get();
  }
  
  public double getRizzPostion(){
    return rizzMotorEncoder.getPosition();
  }

  public void closeClaw(){
    rizzMotor.set(Constants.CLAW_CLOSE_SPEED);
  }
  
  public void openClaw(){
    rizzMotor.set(Constants.CLAW_OPEN_SPEED);
  }

  public void stopClaw(){
    rizzMotor.set(0);
  }

  public void stopPivot(){
    pivotMotor.set(0);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
