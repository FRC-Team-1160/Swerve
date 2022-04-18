package frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;

public class SwerveDriveWheel
{
    public PIDController directionController;
    public TalonFX rotationMotor;
    public TalonFX directionMotor;
    public TalonFXSensorCollection rotationSensor;
    public TalonFXSensorCollection directionSensor;

    public SwerveDriveWheel(double P, double I, double D, TalonFX rotationMotor, TalonFXSensorCollection rotationSensor, TalonFX directionMotor, TalonFXSensorCollection directionSensor)
    {
        this.rotationSensor = rotationSensor;
        this.rotationMotor = rotationMotor;
        this.directionSensor = directionSensor;
        this.directionMotor = directionMotor;
        this.directionController = new PIDController(P, I, D);
    }

    public void setSpeed(double speed) {
        directionMotor.set(ControlMode.Velocity, speed);
    }

    public void setDirection(double setpoint)
    {
        //choose the fastest rotation direction and wheel direction
        double currentAngle = rotationSensor.getIntegratedSensorPosition();

        double setpointAngle = closestAngle(currentAngle, setpoint);

        double setpointAngleFlip = closestAngle(currentAngle, setpoint + 180);

        //choose the shorter direction
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlip)) {
            //unflipped wheel direction RUN PID
            double output = directionController.calculate(rotationSensor.getIntegratedSensorPosition(), setpoint);
            rotationMotor.set(ControlMode.Velocity, output);
        } else {
            //flipped wheel direction RUN PID  
            double output = directionController.calculate(rotationSensor.getIntegratedSensorPosition(), setpoint);
            rotationMotor.set(ControlMode.Velocity, -output);

        }

        directionController.calculate(rotationSensor.getIntegratedSensorPosition(), setpoint);
    }

    private static double closestAngle(double a, double b)
    {
        // get direction
        double dir = b % 360.0 - a % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 180.0)
        {
                dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }
}