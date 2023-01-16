/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PortConstants;


public class DriveTrain extends SubsystemBase{
  /** 
   * Creates a new DriveTrain.
   */

  private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
  
  private static DriveTrain m_instance;

  private TalonFX m_frontLeftRotationMotor, m_frontRightRotationMotor, m_backLeftRotationMotor, m_backRightRotationMotor;

  private TalonFXSensorCollection m_frontLeftRotationEncoder, m_frontRightRotationEncoder, m_backLeftRotationEncoder, m_backRightRotationEncoder;

  private TalonFX m_frontLeftDirectionMotor, m_frontRightDirectionMotor, m_backLeftDirectionMotor, m_backRightDirectionMotor;

  private TalonFXSensorCollection m_frontLeftDirectionEncoder, m_frontRightDirectionEncoder, m_backLeftDirectionEncoder, m_backRightDirectionEncoder;

  private SwerveDriveWheel m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel;
  
  public SwerveDriveController m_controller;

  private CANCoder m_frontLeftCoder, m_frontRightCoder, m_backLeftCoder, m_backRightCoder;

  private AHRS m_gyro;

  private Translation2d m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation;
  private SwerveDriveKinematics m_kinematics;

  private Solenoid m_gate;

  double wkP, wkI, wkD;

  //initializes the drive train
  
  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  private static initSwerveModule(double P, double I, double D, int direction_drive, int rotation_drive, int coder_drive ) {
    TalonFX directionMotor = new TalonFX(direction_drive);
    CANCoder directionEncoder = m_frontLeftDirectionMotor.getSensorCollection();
    TalonFX rotationMotor = new TalonFX(rotation_drive);
    CANCOder coder = new CANCoder(coder_drive);
    return new SwerveDriveWheel(P, I, D, rotationMotor, coder, directionMotor, directionEncoder);
  }

  public DriveTrain() {

    //swerve wheel PID values
    wkP = 0.005;
    wkI = 0.000001;
    wkD = 0;

    //swerve modules

    m_frontLeftWheel = initSwerveModule(wkP, wkI, wkD, PortConstants.FRONT_LEFT_DIRECTION_DRIVE, PortConstants.FRONT_LEFT_ROTATION_DRIVE, PortConstants.FRONT_LEFT_CODER_DRIVE);
    m_frontRightWheel = initSwerveModule(wkP, wkI, wkD, PortConstants.FRONT_RIGHT_DIRECTION_DRIVE, PortConstants.FRONT_RIGHT_ROTATION_DRIVE, PortConstants.FRONT_RIGHT_CODER_DRIVE);
    m_backLeftWheel = initSwerveModule(wkP, wkI, wkD, PortConstants.BACK_LEFT_DIRECTION_DRIVE, PortConstants.BACK_LEFT_ROTATION_DRIVE, PortConstants.BACK_LEFT_CODER_DRIVE);
    m_backRightWheel = initSwerveModule(wkP, wkI, wkD, PortConstants.BACK_RIGHT_DIRECTION_DRIVE, PortConstants.BACK_RIGHT_ROTATION_DRIVE, PortConstants.BACK_RIGHT_CODER_DRIVE);

    m_gyro = new AHRS(Port.kMXP);

    m_gyro.zeroYaw();
    m_controller = new SwerveDriveController(m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel, m_gyro);
  }

  public double getGyroAngle() {
    return m_gyro.getYaw();
  }

  public void resetGyro() {
    m_gyro.zeroYaw();
  }

  public void turnBackRight(double speed) {
    m_backRightRotationMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setGate(boolean b) {
    m_gate.set(b);
  }

  public boolean getGateStatus() {
    return m_gate.get();
  }
  
  @Override
  public void periodic() {
    double xAngle = m_mainStick.getRawAxis(0);
    double yAngle = -m_mainStick.getRawAxis(1);
    double angle = Math.toDegrees(Math.atan((yAngle/xAngle)/2)) + 90;
    if (xAngle < 0) {
      angle += 180;
    }
    double mag = Math.sqrt(xAngle*xAngle + yAngle*yAngle);
    if (mag > 1) {
      mag = 1;
    }
    double turn = m_mainStick.getRawAxis(4);
    SmartDashboard.putNumber("x", xAngle);
    SmartDashboard.putNumber("y", yAngle);
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Mag", mag);
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putNumber("Gyro Yaw", getGyroAngle());

    SmartDashboard.putNumber("FLCoder", m_frontLeftCoder.getAbsolutePosition());
    SmartDashboard.putNumber("FRCoder", m_frontRightCoder.getAbsolutePosition());
    SmartDashboard.putNumber("BLCoder", m_backLeftCoder.getAbsolutePosition());
    SmartDashboard.putNumber("BRCoder", m_backRightCoder.getAbsolutePosition());

    var gyroAngle = m_gyro.getRotation2d();

    m_controller.m_pose = m_controller.m_odometry.update(gyroAngle,
            new SwerveModulePosition[] {
              m_controller.frontLeftWheel.getModule(),
              m_controller.frontRightWheel.getModule(),
              m_controller.backLeftWheel.getModule(),
              m_controller.backRightWheel.getModule()
            });
    SmartDashboard.putNumber("Pose2DY", m_controller.m_pose.getY());
    SmartDashboard.putNumber("Pose2DX", m_controller.m_pose.getX());
    SmartDashboard.putNumber("Pose2DRotation", m_controller.m_pose.getRotation().getDegrees());
  }
}
