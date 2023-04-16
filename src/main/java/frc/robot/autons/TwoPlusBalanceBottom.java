// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.ArmToPreset;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.Outtake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPlusBalanceBottom extends SequentialCommandGroup {
  /** Creates a new OnePlusOne. */
  public TwoPlusBalanceBottom(
    DriveTrain driveTrain,
    Arm arm,
    Intake intake,
    PoseEstimator poseEstimator
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup(
      "2 + Balance Bottom",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );

    addCommands(
      driveTrain.followEventTrajectoryCommand(trajectory, poseEstimator, true)
    );
  }
}
