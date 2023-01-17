package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveWheel
{
    public PIDController directionController;
    public TalonFX rotationMotor;
    public TalonFX directionMotor;
    public CANCoder rotationSensor;
    public TalonFXSensorCollection directionSensor;
    double kFF, kP, kI, kD, accumulator, maxA;

    public SwerveDriveWheel(double P, double I, double D, TalonFX rotationMotor, CANCoder rotationSensor, TalonFX directionMotor, TalonFXSensorCollection directionSensor)
    {
        this.rotationSensor = rotationSensor;
        this.rotationMotor = rotationMotor;
        this.directionSensor = directionSensor;
        this.directionMotor = directionMotor;

        kP = P;
        kI = I;
        kD = D;
        maxA = 100;
        accumulator = 0;
        kFF = 0.05;
    }

    public SwerveModulePosition getModule() {
        return new SwerveModulePosition(directionSensor.getIntegratedSensorPosition(), Rotation2d.fromDegrees(rotationSensor.getAbsolutePosition()));
    }

    private double normalizeAngle(double angle) {
      angle = ((angle % 360) + 360) % 360;
      if (angle > 180) {
        angle -= 360;
      }
      return angle;
    }

    public void set(double setpoint, double speed)
    {
        double currentAngle = rotationSensor.getAbsolutePosition();
        double error, output;
        error = normalizeAngle(setpoint - currentAngle);

        if (Math.abs(error)) > 90 {
          setpoint = (setpoint + 180) % 360;
          error = normalizeAngle(setpoint - currentAngle);
          speed = -speed
        }

        output = (kP * Math.abs(error)) + ((0.4/(1+3*(Math.exp(-0.01*Math.abs(error)))))-0.06);

        if (error < 0) {
          output = -output
        }

        rotationMotor.set(TalonFXControlMode.PercentOutput, output);
        directionMotor.set(TalonFXControlMode.PercentOutput, speed);

        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("speeed", speed);
        SmartDashboard.putNumber("mode", mode);
        SmartDashboard.putNumber("error", error);
        SmartDashboard.putNumber("setpoint", setpoint);
}
