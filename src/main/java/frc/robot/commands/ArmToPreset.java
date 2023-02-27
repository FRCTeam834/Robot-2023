// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;
import frc.robot.subsystems.Arm;

public class ArmToPreset extends CommandBase {
  /** Creates a new ArmToPreset. */
  private final Arm arm;
  private final ArmPositionPresets preset;

  public ArmToPreset(Arm arm, ArmPositionPresets preset) {
    this.arm = arm;
    this.preset = preset;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setDesiredState(preset.position, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.getPosition() - preset.position) < ArmConstants.SETPOINT_TOLERANCE;
  }
}
