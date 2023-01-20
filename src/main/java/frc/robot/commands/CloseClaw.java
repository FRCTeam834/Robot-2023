// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CloseClaw extends CommandBase {
  /** Creates a new CloseClaw. */
  public CloseClaw() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.arm.closeClaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.arm.stopClaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double position = RobotContainer.arm.getRizzPostion();
    return (position >= -0.1 && position <= 0.1);
  }
}
