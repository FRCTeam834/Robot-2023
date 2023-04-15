// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Superstructure;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;
import frc.robot.subsystems.Arm;

public class ArmToPreset extends CommandBase {
  /** Creates a new ArmToPreset. */
  private final Arm arm;
  private final ArmPositionPresets preset;
  private boolean finished = false;

  public ArmToPreset(Arm arm, ArmPositionPresets preset) {
    this.arm = arm;
    this.preset = preset;
    // nvm dont want this this.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
    addRequirements(arm, Superstructure.wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    //arm.setDesiredState(preset.position, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Superstructure.wrist.getPosition() < WristConstants.STOW_POSITION - Units.degreesToRadians(5)) {
      Superstructure.wrist.setPosition(WristConstants.STOW_POSITION);
    } else {
      finished = true;
      arm.setDesiredState(preset.position, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished; //return Math.abs(arm.getPosition() - preset.position) < ArmConstants.SETPOINT_TOLERANCE;
  }
}
