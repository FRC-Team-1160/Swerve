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
//

  //initializes the drive train
  
  public static DriveTrain getInstance(){
    if (m_instance == null){
      m_instance = new DriveTrain();
    }
    return m_instance;
  }

  public DriveTrain() {
    //directional motors
    m_frontLeftDirectionMotor = new TalonFX(PortConstants.FRONT_LEFT_DIRECTION_DRIVE);
    m_frontRightDirectionMotor = new TalonFX(PortConstants.FRONT_RIGHT_DIRECTION_DRIVE);
    m_backLeftDirectionMotor = new TalonFX(PortConstants.BACK_LEFT_DIRECTION_DRIVE);
    m_backRightDirectionMotor = new TalonFX(PortConstants.BACK_RIGHT_DIRECTION_DRIVE);

    //directional encoders
    m_frontLeftDirectionEncoder = m_frontLeftDirectionMotor.getSensorCollection();
    m_frontRightDirectionEncoder = m_frontRightDirectionMotor.getSensorCollection();
    m_backLeftDirectionEncoder = m_backLeftDirectionMotor.getSensorCollection();
    m_backRightDirectionEncoder = m_backRightDirectionMotor.getSensorCollection();


    //rotational motors
    m_frontLeftRotationMotor = new TalonFX(PortConstants.FRONT_LEFT_ROTATION_DRIVE);
    m_frontRightRotationMotor = new TalonFX(PortConstants.FRONT_RIGHT_ROTATION_DRIVE);
    m_backLeftRotationMotor = new TalonFX(PortConstants.BACK_LEFT_ROTATION_DRIVE);
    m_backRightRotationMotor = new TalonFX(PortConstants.BACK_RIGHT_ROTATION_DRIVE);

    //rotational encoders
    m_frontLeftRotationEncoder = m_frontLeftRotationMotor.getSensorCollection();
    m_frontRightRotationEncoder = m_frontRightRotationMotor.getSensorCollection();
    m_backLeftRotationEncoder = m_backLeftRotationMotor.getSensorCollection();
    m_backRightRotationEncoder = m_backRightRotationMotor.getSensorCollection();

    //CAN coedrs
    m_frontLeftCoder = new CANCoder(PortConstants.FRONT_LEFT_CODER_DRIVE);
    m_frontRightCoder = new CANCoder(PortConstants.FRONT_RIGHT_CODER_DRIVE);
    m_backLeftCoder = new CANCoder(PortConstants.BACK_LEFT_CODER_DRIVE);
    m_backRightCoder = new CANCoder(PortConstants.BACK_RIGHT_CODER_DRIVE);

    /*m_frontLeftDirectionEncoder.setIntegratedSensorPosition(0, 0);
    m_frontRightDirectionEncoder.setIntegratedSensorPosition(0, 0);
    m_backLeftDirectionEncoder.setIntegratedSensorPosition(0, 0);
    m_backRightDirectionEncoder.setIntegratedSensorPosition(0, 0);

    m_frontLeftRotationEncoder.setIntegratedSensorPosition(0, 0);
    m_frontRightRotationEncoder.setIntegratedSensorPosition(0, 0);
    m_backLeftRotationEncoder.setIntegratedSensorPosition(0, 0);
    m_backRightRotationEncoder.setIntegratedSensorPosition(0, 0);*/

    //swerve wheel PID values
    wkP = 0.005;
    wkI = 0.000001;
    wkD = 0;

    //swerve wheels (controls the rotation and direction motors)
    m_frontLeftWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_frontLeftRotationMotor, m_frontLeftCoder, m_frontLeftDirectionMotor, m_frontLeftDirectionEncoder);
    m_frontRightWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_frontRightRotationMotor, m_frontRightCoder, m_frontRightDirectionMotor, m_frontRightDirectionEncoder);
    m_backLeftWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_backLeftRotationMotor, m_backLeftCoder, m_backLeftDirectionMotor, m_backLeftDirectionEncoder);
    m_backRightWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_backRightRotationMotor, m_backRightCoder, m_backRightDirectionMotor, m_backRightDirectionEncoder);

    m_controller = new SwerveDriveController(m_frontLeftWheel, m_frontRightWheel, m_backLeftWheel, m_backRightWheel);

    m_gyro = new AHRS(Port.kMXP);

    m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    m_gyro.zeroYaw();
    //m_gate = new Solenoid(PneumaticsModuleType.CTREPCM, PortConstants.GATE);
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
    double yAngle = m_mainStick.getRawAxis(1);
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
  }
}