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
  private double x;
  private double y;
  private double spd;
  private double turnspd;
  DriveTrain m_drive;
  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  public SwerveDrive(DriveTrain m_drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    this.m_drive = m_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = m_mainStick.getRawAxis(0);
    y = -m_mainStick.getRawAxis(1);
    spd = 0.3;
    turnspd = 0.3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double joystickX = m_mainStick.getRawAxis(0);
    double joystickY = -m_mainStick.getRawAxis(1);
    double joystickBrake = m_mainStick.getRawAxis(2);

    double mag = Math.sqrt(joystickX*joystickX + joystickY*joystickY);
    if (mag > 1) {
      mag = 1;
    }

    double turn = m_mainStick.getRawAxis(4);
    double gyroAngle = Math.toRadians(m_drive.getGyroAngle());
        //field oriented
    if (mag > 0.02) {
      x = m_mainStick.getRawAxis(0);
      y = -m_mainStick.getRawAxis(1);
      double temp = y * Math.cos(gyroAngle) + x*Math.sin(gyroAngle);
      x = -1*y * Math.sin(gyroAngle) + x*Math.cos(gyroAngle);
      y = temp;
      spd = 0.3;
    } else {
      spd = 0.0001;
    }
    
    m_drive.m_controller.setSwerveDrive(true, spd*y, spd*x, turnspd*turn, gyroAngle);
    m_drive.m_controller.brake(joystickBrake);

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
