// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class followPath extends CommandBase {
  /** Creates a new followPath. */
  private final Timer timer = new Timer();
  private PathPlannerTrajectory transformedTrajectory; 
  private PathPlannerTrajectory trajectory;
  private Pose2d pose;
  private boolean mirrorIfRed;
  DriveTrain m_drive;
  private PPHolonomicDriveController controller;
  private PathConstraints constraints;

  public followPath(PathPlannerTrajectory traj, Pose2d m_pose, PIDController xController, PIDController yController, PIDController rController, PathConstraints constraints, boolean mirrorIfRed, DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.trajectory = traj;
    this.pose = m_pose;
    this.mirrorIfRed = mirrorIfRed;
    this.m_drive = m_drive;
    this.controller = new PPHolonomicDriveController(xController, yController, rController);
    this.constraints = constraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mirrorIfRed) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    pose = m_drive.m_pose;
    SmartDashboard.putNumber("desired x", desiredState.poseMeters.getX());
    SmartDashboard.putNumber("desired y", desiredState.poseMeters.getY());

    ChassisSpeeds targetChassisSpeeds = this.controller.calculate(pose, desiredState);
    double fwd = targetChassisSpeeds.vxMetersPerSecond;
    double str = -targetChassisSpeeds.vyMetersPerSecond;
    double rot = targetChassisSpeeds.omegaRadiansPerSecond;
    //rot /= 2;
    //rot *= SwerveConstants.r;
    SmartDashboard.putNumber("auto fwd", fwd);
    SmartDashboard.putNumber("auto str", str);
    SmartDashboard.putNumber("auto rot", rot);
    m_drive.m_controller.setSwerveDrive(false, fwd, str, rot, m_drive.getGyroAngle());
    if (fwd == 0 && str == 0) {
      m_drive.m_controller.brake(0.5);
    } else {
      m_drive.m_controller.brake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop();
    if (interrupted) {
      m_drive.m_controller.setSwerveDrive(false, 0,0,0,m_drive.getGyroAngle());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }
}
