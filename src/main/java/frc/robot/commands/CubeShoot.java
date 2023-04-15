// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Superstructure;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class CubeShoot extends CommandBase {
  /** Creates a new CubeShoot. */
  public final Intake intake;
  public final int current;
  public final Timer timer = new Timer();

  public CubeShoot(Intake intake, int current) {
    this.intake = intake;
    this.current = current;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!intake.hasCube()) return;

    timer.reset();
    timer.start();
    intake.motor.setSmartCurrentLimit(current);
    intake.setVoltage(-12);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setVoltage(0);
    intake.motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.5;
  }
}
