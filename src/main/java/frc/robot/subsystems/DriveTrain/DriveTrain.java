/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private AHRS m_gyro;

  private Translation2d m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation;
  private SwerveDriveKinematics m_kinematics;

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

    //swerve wheel PID values
    wkP = 0.0001;
    wkI = 0;
    wkD = 0;

    //swerve wheels (controls the rotation and direction motors)
    m_frontLeftWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_frontLeftRotationMotor, m_frontLeftRotationEncoder, m_frontLeftDirectionMotor, m_frontLeftDirectionEncoder);
    m_frontRightWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_frontRightRotationMotor, m_frontRightRotationEncoder, m_frontRightDirectionMotor, m_frontRightDirectionEncoder);
    m_backLeftWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_backLeftRotationMotor, m_backLeftRotationEncoder, m_backLeftDirectionMotor, m_backLeftDirectionEncoder);
    m_backRightWheel = new SwerveDriveWheel(wkP, wkI, wkD, m_backRightRotationMotor, m_backRightRotationEncoder, m_backRightDirectionMotor, m_backRightDirectionEncoder);

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

    
  }

  public double getGyroAngle() {
    return m_gyro.getYaw();
  }

  
  @Override
  public void periodic() {
    double xAngle = (m_mainStick.getRawAxis(0)-0.5)*2;
    double yAngle = (m_mainStick.getRawAxis(1)-0.5)*2;
    double angle = Math.toDegrees(Math.atan(yAngle/xAngle)) + 90;
    double mag = Math.sqrt(xAngle*xAngle + yAngle*yAngle);
    double turn = (m_mainStick.getRawAxis(1)-0.5)*2;
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Mag", mag);
    SmartDashboard.putNumber("Turn", turn);
  }
}