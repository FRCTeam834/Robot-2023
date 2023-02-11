// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.IntakeConstants;

public class Outtake extends CommandBase {
  /** Creates a new Outtake. */
  private Intake intake;
  private final LinearFilter rpmFilter;
  private double rpm;

  public Outtake(Intake intake) {
    this.intake = intake;
    rpmFilter = LinearFilter.movingAverage(IntakeConstants.RPM_FILTER_TAPS);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpmFilter.reset();
    if(Intake.currentGamePiece == GamePieceType.CUBE) {
      intake.runBackward();
    } else if (Intake.currentGamePiece == GamePieceType.CONE){
      intake.runForward();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = rpmFilter.calculate(intake.getRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (this.isFinished()) {
      Intake.currentGamePiece = GamePieceType.NONE;
    }
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rpm) > IntakeConstants.FREE_RPM_THRESHOLD;
  }
}
