package frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveController {
    SwerveDriveWheel frontLeftWheel;
    SwerveDriveWheel frontRightWheel;
    SwerveDriveWheel backLeftWheel;
    SwerveDriveWheel backRightWheel;
    


    public SwerveDriveController(SwerveDriveWheel frontLeftWheel, SwerveDriveWheel frontRightWheel, SwerveDriveWheel backLeftWheel, SwerveDriveWheel backRightWheel) {
        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;
    }

    //chooses either turning in place or turning while driving
    public void setSwerveDrive(double driveDirection, double driveSpeed, double turnSpeed) {
        //this.driveTurn(driveDirection, driveSpeed, turnSpeed);
        if ((driveSpeed <= 0.01) && (turnSpeed != 0.0)) {
            //turns without driving if no drive speed
            this.turn(turnSpeed);
        } else {
            //either only drives if turnspeed is 0
            //or drives and turns at the same time if turnspeed is not 0
            this.driveTurn(driveDirection, driveSpeed, turnSpeed);
        }
    }

    //DRIVE WITHOUT TURNING
    //we don't really actually need this but its good to have
    public void drive(double direction, double speed) {
        frontLeftWheel.set(direction, speed);
        frontRightWheel.set(direction, speed);
        backLeftWheel.set(direction, speed);
        backRightWheel.set(direction, speed);
    }

    //TURN WITHOUT DRIVING
    public void turn(double speed) {
        //zero is facing forward
        speed *= 0.3;
        frontLeftWheel.set(45.0, speed);
        frontRightWheel.set(135.0, speed);
        backLeftWheel.set(315.0, speed);
        backRightWheel.set(225.0, speed);

    }

    //TURN AND DRIVE AT THE SAME TIME
    public void driveTurn(double driveDirection, double driveSpeed, double turnSpeed) {
        //turn Speed is value fron -1.0 to 1.0 with -1 being max left and 1 being max right
        double turnAngle = turnSpeed * 45.0;
        SmartDashboard.putNumber("turn Angle", turnAngle);

        //determine if wheel is in front or in back
        //for example: if the direction was to the right and the robot was facing forward,
        //the frontRight and backRight wheel would be front and the frontLeft and backLeft wheels would be in back
        //if driving direction is within 90 degrees of the direction of the wheel, wheel is in front

        //FRONT LEFT
        //directly at the front left wheel is 45 degrees
        frontLeftWheel.set(convert(driveDirection + turnAngle), driveSpeed);
        if (closestAngle(driveDirection, 45.0) <= 90) {
            //front
            //tilt to the right
            frontLeftWheel.set(driveDirection + turnAngle, driveSpeed);
        } else {
            //back
            //tilt to the left
            frontLeftWheel.set(driveDirection - turnAngle, driveSpeed);
        }
        //FRONT RIGHT
        //directly at the front right wheel is 315 degrees
        if (closestAngle(driveDirection, 315.0) < 90) {
            //front
            //tilt right
            frontRightWheel.set(driveDirection + turnAngle, driveSpeed);
        } else {
            //back
            //tilt left
            frontRightWheel.set(driveDirection - turnAngle, driveSpeed);
        }

        //BACK LEFT
        //directly at back left wheel is 135 degrees
        if (closestAngle(driveDirection, 135.0) < 90) {
            //front
            //tilt right
            backLeftWheel.set(driveDirection + turnAngle, driveSpeed);
        } else {
            //back
            //tilt left
            backLeftWheel.set(driveDirection - turnAngle, driveSpeed);
        }

        //BACK RIGHT
        //directly at back right wheel is 225 degrees
        if (closestAngle(driveDirection, 225.0) <= 90) {
            //front
            //tilt right
            backRightWheel.set(driveDirection + turnAngle, driveSpeed);
        } else {
            //back
            //tilt left
            backRightWheel.set(driveDirection - turnAngle, driveSpeed);
        }
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

    private static double convert(double a) {
        if (a > 360) {
            return (a-360.0);
        }
        if (a < 0) {
            return (a+360.0);
        }
        return a;
    }
}
