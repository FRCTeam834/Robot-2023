// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Superstructure;
import frc.robot.subsystems.Intake;
import frc.robot.utility.LEDColors;

public class IntakeCube extends CommandBase {
  /** Creates a new IntakeCube. */
  private final Intake intake;
  private final LinearFilter rpmFilter;
  private double rpm;
  private Timer timer = new Timer();
  
  public IntakeCube(Intake intake) {
    this.intake = intake;
    rpmFilter = LinearFilter.movingAverage(IntakeConstants.RPM_FILTER_TAPS);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    rpmFilter.reset();
    //rpmFilter.calculate(834834834);
    intake.setVoltage(10);
    Superstructure.leds.setColor(LEDColors.PURPLE);
    Superstructure.desiredGamePiece = GamePieceType.CUBE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = rpmFilter.calculate(intake.getRPM());
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.currentGamePiece = GamePieceType.CUBE;
    if (this.isFinished()) {
      intake.setVoltage(6);
      Superstructure.leds.setColor(LEDColors.GREEN);
    } else {
      intake.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.5 && Math.abs(rpm) < IntakeConstants.CUBE_RPM_THRESHOLD;
  }
}
