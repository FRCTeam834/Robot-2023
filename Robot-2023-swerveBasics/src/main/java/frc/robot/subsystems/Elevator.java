// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax m_elevator;
  SparkMaxPIDController eController;  
  RelativeEncoder eEncoder;
  ElevatorFeedforward feedforward;
  public DigitalInput bottomLimitSwitch;
  public DigitalInput topLimitSwitch;
  
  public Elevator() {
    m_elevator = new CANSparkMax(ElevatorMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    eController = m_elevator.getPIDController();
    eEncoder = m_elevator.getEncoder();
    feedforward = new ElevatorFeedforward(0,0,0,0);
    bottomLimitSwitch = new DigitalInput(Constants.ElevatorBottomLimitSwitchID);
    topLimitSwitch = new DigitalInput(Constants.ElevatorTopLimitSwitchID);
    
    eController.setP(0);
    eController.setI(0);
    eController.setD(0);

    
    /*

    ! Things to do:

    setReference(implement) PID
    Desired State
    ! Problem: What class are we using for the desired state? 
    = double
    eController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    

    ! feedforward.calculate(0,0); Do we need this?

    Calculate Feedforward
    

    Command to home the elevator
    ^ don't we have that? 
    * We don't have reference

    */


    

  }

  public void setDesiredPosition(double position){

    eController.setReference(position, CANSparkMax.ControlType.kPosition, 0, feedforward.calculate(0, 0));
  

  }
  
  public void setElevatorMotor(double speed){
    m_elevator.set(speed);

  }
  
  public void stopElevator() {
    m_elevator.set(0);
  }

  @Override
  public void periodic() {
    
    

    // This method will be called once per scheduler run
  }
}
