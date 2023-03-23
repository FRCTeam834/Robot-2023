// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;
import frc.robot.Superstructure;

public class Outtake extends CommandBase {
  /** Creates a new Outtake. */
  private Intake intake;
  private Arm arm;
  private final LinearFilter rpmFilter;
  private double rpm;
  private GamePieceType lastGamePiece;

  public Outtake(Intake intake, Arm arm) {
    this.intake = intake;
    this.arm = arm;
    rpmFilter = LinearFilter.movingAverage(IntakeConstants.RPM_FILTER_TAPS + 30);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpmFilter.reset();
    if(intake.currentGamePiece == GamePieceType.CUBE) {
      lastGamePiece = intake.currentGamePiece;
      intake.setVoltage(-6);
      if (arm.getPositionSetpoint() == ArmPositionPresets.STOW.position) {
        intake.setVoltage(-12);
      }
    } else if (intake.currentGamePiece == GamePieceType.CONE){
      lastGamePiece = intake.currentGamePiece;
      intake.setVoltage(6);
    } else if (intake.currentGamePiece == GamePieceType.NONE) {
      if (lastGamePiece == GamePieceType.CONE) {
        intake.setVoltage(6);
      } else if (lastGamePiece == GamePieceType.CUBE) {
        intake.setVoltage(-6);
        if (arm.getPositionSetpoint() == ArmPositionPresets.STOW.position || arm.getPositionSetpoint() == ArmPositionPresets.L1.position) {
          intake.setVoltage(-12);
        }
      }
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
      intake.currentGamePiece = GamePieceType.NONE;
      Superstructure.leds.defaultColor();
    }
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rpm) > IntakeConstants.FREE_RPM_THRESHOLD;
  }
}
