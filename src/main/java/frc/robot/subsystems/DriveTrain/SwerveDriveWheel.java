package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
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
        //this.directionController = new PIDController(P, I, D);
        kP = P;
        kI = I;
        kD = D;
        maxA = 100;
        accumulator = 0;
        kFF = 0.05;
    }

    public void set(double setpoint, double speed)
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
        //increase output at low errors and decrease output at high errors

        //double output = (kP * error);// + (kI * accumulator);// + ((0.4/(1+3*(Math.pow(Math.E, -0.01))))-0.06);
        //double output = directionController.calculate(currentAngle, setpointAngle);
        int mode = closestAngle(setpoint, currentAngle);
        /*
        0-3 do not cross 0 degree mark
        0 = right forward
        1 = right back
        2 = left forward
        3 = left back
        4-7 do cross 0 degree mark
        4 = right forward
        5 = right back
        6 = left forward
        7 = left back
        */
        double error, output;
        if (mode == 0) {
            error = Math.abs(setpoint - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = Math.abs(output);
            speed = -speed;
        } else if (mode == 1) {
            error = Math.abs((setpoint+180) - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = Math.abs(output);
            speed = speed;
        } else if (mode == 2) {
            error = Math.abs(setpoint - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = -(Math.abs(output));
            speed = -speed;
        } else if (mode == 3) {
            error = Math.abs((setpoint-180) - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            speed = (speed);
            output = -(Math.abs(output));
        } else if (mode == 4) {
            error = Math.abs((setpoint+360) - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = Math.abs(output);
            speed = -speed;
        } else if (mode == 5) {
            error = Math.abs((setpoint+180) - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = Math.abs(output);
            speed = (speed);
        } else if (mode == 6) {
            error = Math.abs((setpoint-360) - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = -(Math.abs(output));
            speed = -speed;
        } else if (mode == 7) {
            error = Math.abs((setpoint-180) - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error) + ((0.4/(1+3*(Math.pow(Math.E, -0.01*error))))-0.06);
            output = -Math.abs(output);
            speed = speed;
        } else {
            error = Math.abs(setpoint - currentAngle);
            if (error > 360) {
                error -= 360;
            }
            output = (kP * error);
            output = Math.abs(output);
            speed = speed;
        }
        //output *= closestAngle(setpoint, currentAngle);
        SmartDashboard.putNumber("output", output);
        SmartDashboard.putNumber("speeed", speed);
        SmartDashboard.putNumber("mode", mode);
        rotationMotor.set(TalonFXControlMode.PercentOutput, output);
        directionMotor.set(TalonFXControlMode.PercentOutput, speed);
        SmartDashboard.putNumber("error", error);
        SmartDashboard.putNumber("setpoint", setpoint);
        
    }


    //returns true for turning right and false for turning left
    private static int closestAngle(double a, double b) {
        //a is setpoint, b is current Angle

        if (Math.abs(a - b) <= 180 && a >= b) {
            //turn right
            if (Math.abs(a - b) <= 90) {
                return 0;
            } else {
                return 3;
            }
        } else if (Math.abs(a - b) <= 180 && a < b) {
            //turn left
            if (Math.abs(a - b) <= 90) {
                return 2;
            } else {
                return 1;
            }
            //crossing over 0
        } else if (b >= a) {
            //turn right
            if (Math.abs(a+180-b) > 90) {
                return 4;
            } else {
                return 7;
            }

        } else {
            if (Math.abs(a-180-b) > 90) {
                return 6;
            } else {
                return 5;
            }
        }
    }
    
}