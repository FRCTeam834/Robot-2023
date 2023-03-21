// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.ArmToPreset;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.Outtake;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LinearPath extends SequentialCommandGroup {
  /** Temporary test auton */
  public LinearPath(
    DriveTrain driveTrain,
    Arm arm,
    Intake intake,
    PoseEstimator poseEstimator
  ) {
    /*PathPlannerTrajectory trajectory = PathPlanner.loadPath(
      "Linear Path",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );
    addCommands(
      driveTrain.followTrajectoryCommand(trajectory, poseEstimator, true)
    );*/

    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath(
      "Cone Test",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath(
      "Cone Test Part 2",
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxVelocity,
      DriveTrainConstants.AUTON_DRIVE_CONSTRAINTS.maxAcceleration
    );

    addCommands(
      new ParallelCommandGroup(
        new IntakeCone(intake),
        driveTrain.followTrajectoryCommand(trajectory1, poseEstimator, true)
      ),
      new ArmToPreset(arm, ArmPositionPresets.L2).until(() -> arm.getPosition() > 1),
      driveTrain.followTrajectoryCommand(trajectory2, poseEstimator, false),
      new Outtake(intake, arm)
    );
  }
}
