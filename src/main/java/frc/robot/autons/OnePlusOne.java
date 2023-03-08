// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

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
public class OnePlusOne extends SequentialCommandGroup {
  /** Creates a new OnePlusOne. */
  public OnePlusOne(
    DriveTrain driveTrain,
    Arm arm,
    Intake intake,
    PoseEstimator poseEstimator
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath(
      "1 + 1 Part 1",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath(
      "1 + 1 Part 2",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );

    addCommands(
      new IntakeCone(intake),
      //new ArmToPreset(arm, ArmPositionPresets.ESCAPE),
      new ArmToPreset(arm, ArmPositionPresets.L3).until(() -> arm.getPosition() > ArmPositionPresets.L2.position),
      driveTrain.followTrajectoryCommand(trajectory1, poseEstimator, true),
      new Outtake(intake)
      
      /*new ParallelCommandGroup(
        driveTrain.followTrajectoryCommand(trajectory2, poseEstimator, true),
        new IntakeCone(intake),
        new SequentialCommandGroup(
          new RepeatCommand(new InstantCommand())
            .until(() -> poseEstimator.getEstimatedPose().getX() > 3.5)
            .andThen(new ArmToPreset(arm, ArmPositionPresets.STOW)),
          new RepeatCommand(new InstantCommand())
            .until(() -> poseEstimator.getEstimatedPose().getX() > 5.5)
            .andThen(new ArmToPreset(arm, ArmPositionPresets.L1)),
          new RepeatCommand(new InstantCommand())
            .until(() -> poseEstimator.getEstimatedPose().getX() < 5.5)
            .andThen(new ArmToPreset(arm, ArmPositionPresets.STOW))
        )
       
      ),

      new ArmToPreset(arm, ArmPositionPresets.L3).until(() -> arm.getPosition() > ArmPositionPresets.L2.position),
      new Outtake(intake)*/
    );
  }
}
