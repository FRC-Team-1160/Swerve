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
    public double lastSetpointSpeed;
    double krP, krI, krD, ksP, ksI, ksD, rAccumulator, maxA;
    double currentOutputSpeed;
    double brakeConstant;

    public SwerveDriveWheel(double rP, double rI, double rD, double sP, double sI, double sD, TalonFX rotationMotor, CANCoder rotationSensor, TalonFX directionMotor)
    {
        this.rotationSensor = rotationSensor;
        this.rotationMotor = rotationMotor;
        this.directionMotor = directionMotor;
        this.lastSetpointSpeed = 0;
        this.currentOutputSpeed = 0;
        this.brakeConstant = 1;
        
        //this.directionController = new PIDController(P, I, D);
        krP = rP;
        krI = rI;
        krD = rD;
        ksP = sP;
        ksI = sI;
        ksD = sD;

        maxA = 100;
        rAccumulator = 0;
    }

    public SwerveModulePosition getModule() {
        return new SwerveModulePosition(getPosition(), Rotation2d.fromDegrees(rotationSensor.getAbsolutePosition()));
    }

    public double getVelocity() {
        double v = directionMotor.getSelectedSensorVelocity();
        v *= SwerveConstants.SWERVE_POSITION_RATIO*10;
        return v; //meters per second
    }

    public double getPosition() {
        double pos = directionMotor.getSelectedSensorPosition();
        pos *= SwerveConstants.SWERVE_POSITION_RATIO;
        return pos; //meters
    
    }

    public double getRotation() {
        return rotationSensor.getAbsolutePosition();
    }

    public void resetPosition() {
        directionMotor.setSelectedSensorPosition(0);
    }

    public void setOutput(double setpoint, double speed)
    {
        //choose the fastest rotation direction and wheel direction
        double currentAngle = rotationSensor.getAbsolutePosition();
        //choose the shorter direction
        //unflipped wheel direction RUN PID
        //divide by 180 to convert from angle to output volts
       // double error = Math.abs(setpoint - currentAngle);
        /*if (error < 30) {
            accumulator += error;
        }
    
        if (accumulator > maxA) {
            accumulator = maxA;
        } else if (accumulator < -maxA) {
            accumulator = -maxA;
        }*/
        double angle = (((setpoint - currentAngle) % 360) + 360 ) % 360;
        double error, output;
        error = getAngleError(currentAngle, setpoint);
        if (error < 20) {
            rAccumulator += error;
        }
        if (rAccumulator > maxA) {
            rAccumulator = maxA;
        } else if (rAccumulator < -maxA) {
            rAccumulator = -maxA;
        }
        output = (krP * error) + (krI*rAccumulator);
        if (angle % 180 > 90) {
            output = -output;
        }
        if (angle < 90 || angle > 270) {
            speed = -speed;
        }
        this.currentOutputSpeed = speed;
        rotationMotor.set(TalonFXControlMode.PercentOutput, output);
        directionMotor.set(TalonFXControlMode.PercentOutput, speed * this.brakeConstant);
    }

    public void setOutputSpeed(double spd) {
        rotationMotor.set(TalonFXControlMode.PercentOutput, 0);
        directionMotor.set(TalonFXControlMode.PercentOutput, spd * this.brakeConstant);
        this.currentOutputSpeed = spd;
    }

    public void setVelocity(double setpointAngle, double setpointSpeed) {
        
        double currentAngle = rotationSensor.getAbsolutePosition();
        double angle = (((setpointAngle - currentAngle) % 360) + 360 ) % 360;
        double error = getAngleError(currentAngle, setpointAngle);
        if (error < 20) {
            rAccumulator += error;
        }
        if (rAccumulator > maxA) {
            rAccumulator = maxA;
        } else if (rAccumulator < -maxA) {
            rAccumulator = -maxA;
        }
        double anglePID = (krP * error) + (krI*rAccumulator);
        double speedPID = (ksP * getSpeedError(getVelocity(), setpointSpeed));

        double kV = 1/(SwerveConstants.MAX_WHEEL_SPEED);
        double kA = 1/(SwerveConstants.MAX_WHEEL_ACCELERATION);

        double setpointAcceleration = (setpointSpeed-lastSetpointSpeed)/0.02;

        double angleOutput = anglePID;
        double speedOutput = (speedPID/SwerveConstants.MAX_WHEEL_SPEED + kV*setpointSpeed + kA*setpointAcceleration);

        if (angle % 180 > 90) {
            angleOutput = -angleOutput;
        }
        if (angle < 90 || angle > 270) {
            speedOutput = -speedOutput;
        }
        this.currentOutputSpeed = speedOutput;
        rotationMotor.set(TalonFXControlMode.PercentOutput, angleOutput);
        directionMotor.set(TalonFXControlMode.PercentOutput, speedOutput*this.brakeConstant);
        lastSetpointSpeed = setpointSpeed;

    }

    public void brake(double power) {
        this.brakeConstant = (2.25*(Math.pow((4/9), power))-1.25); //exponential decay
    }

    public double getAngleError(double currentAngle, double setpoint) {
        double setpointAngle = (setpoint % 360) - (currentAngle % 360); //the %360 for some of these really should not do anything, as setpoint and current_angle are always positive.
        double setpointFAngle = ((setpoint+180)%360) - (currentAngle % 360);

        if (Math.abs(setpointAngle) > 180) {
            setpointAngle = -(Math.signum(setpointAngle) * 360.0) + setpointAngle;
        }
        if (Math.abs(setpointFAngle) > 180) {
            setpointFAngle = -(Math.signum(setpointFAngle) * 360.0) + setpointFAngle;
        }
        double error = Math.abs(Math.min(setpointAngle, setpointFAngle));
        if (error > 90) {
            error = 180 - error;
        }
        return error;
    }

    public double getSpeedError(double currentSpeed, double setpointSpeed) {
        return Math.abs(Math.abs(currentSpeed) - Math.abs(setpointSpeed));
    }
    
}