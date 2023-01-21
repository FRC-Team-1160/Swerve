// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain.DriveTrain;

public class TestWheelSpeed extends CommandBase {
  /** Creates a new TestWheelSpeed. */
  private double max;
  private double lastSpeed;
  private DriveTrain m_drive;
  public TestWheelSpeed(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.max = 0;
    this.lastSpeed = 0;
    this.m_drive = m_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("tester", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Front Right max", max);
    double speed = m_drive.m_controller.testFrontRightWheel(0.95);
    double accel = (speed-lastSpeed)/0.02;
    if (accel > max) {
      max = accel;
    }
    lastSpeed = speed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("tester", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
