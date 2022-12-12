// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain.DriveTrain;

import frc.robot.Constants.OIConstants;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */
  private double angle;
  DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  public SwerveDrive(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAngle = m_mainStick.getRawAxis(0);
    double yAngle = m_mainStick.getRawAxis(1);
    
    double mag = Math.sqrt(xAngle*xAngle + yAngle*yAngle);
    if (mag > 1) {
      mag = 1;
    }
    
    double turn = m_mainStick.getRawAxis(4);
    //field oriented
    /*
    if (mag < 0.02) {
      angle = angle;
    } else {
      angle = Math.toDegrees(Math.atan((yAngle/xAngle)/2)) + 90;
      if (xAngle < 0) {
        angle += 180;
      }
      double gyroAngle = m_drive.getGyroAngle();
      if (gyroAngle < 0) {
        gyroAngle += 360;
      }

      angle -= m_drive.getGyroAngle();
      if (angle < 0) {
        angle += 360;
      }
    }
    mag *= 0.3;
    */
    double temp = xAngle * Math.cos(turn) + yAngle*Math.sin(turn);
    yAngle = -1*xAngle * Math.sin(turn) + yAngle*Math.cos(turn);
    xAngle = temp;
    m_drive.m_controller.setSwerveDrive(xAngle, yAngle, turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
