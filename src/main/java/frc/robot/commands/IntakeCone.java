// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCone extends CommandBase {
  
  private final Intake intake;
  private final LinearFilter rpmFilter;
  private double rpm;
  
  public IntakeCone(Intake intake) {
    this.intake = intake;
    rpmFilter = LinearFilter.movingAverage(IntakeConstants.RPM_FILTER_TAPS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Artifically inflate filter so intake has time to spin up */
    rpmFilter.reset();
    rpmFilter.calculate(-834834);
    intake.runBackward();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = rpmFilter.calculate(intake.getRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rpm) < IntakeConstants.CONE_RPM_THRESHOLD;
  }
}
