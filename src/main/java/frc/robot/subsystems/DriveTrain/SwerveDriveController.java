package frc.robot.subsystems.DriveTrain;

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
        if ((driveSpeed == 0.0) && (turnSpeed != 0.0)) {
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
        frontLeftWheel.setDirection(direction);
        frontRightWheel.setDirection(direction);
        backLeftWheel.setDirection(direction);
        backRightWheel.setDirection(direction);

        frontLeftWheel.setSpeed(speed);
        frontRightWheel.setSpeed(speed);
        backLeftWheel.setSpeed(speed);
        backRightWheel.setSpeed(speed);
    }

    //TURN WITHOUT DRIVING
    public void turn(double speed) {
        //zero is facing forward
        frontLeftWheel.setDirection(45.0);
        frontRightWheel.setDirection(135.0);
        backLeftWheel.setDirection(315.0);
        backRightWheel.setDirection(225.0);

        frontLeftWheel.setSpeed(speed);
        frontRightWheel.setSpeed(speed);
        backLeftWheel.setSpeed(speed);
        backRightWheel.setSpeed(speed);

    }

    //TURN AND DRIVE AT THE SAME TIME
    public void driveTurn(double driveDirection, double driveSpeed, double turnSpeed) {
        //turn Speed is value fron -1.0 to 1.0 with -1 being max left and 1 being max right
        double turnAngle = turnSpeed * 45.0;

        //determine if wheel is in front or in back
        //for example: if the direction was to the right and the robot was facing forward,
        //the frontRight and backRight wheel would be front and the frontLeft and backLeft wheels would be in back
        //if driving direction is within 90 degrees of the direction of the wheel, wheel is in front

        //FRONT LEFT
        //directly at the front left wheel is 315 degrees or -45 degrees
        if (closestAngle(driveDirection, 315.0) <= 90) {
            //front
            //tilt to the right
            frontLeftWheel.setDirection(driveDirection + turnAngle);
        } else {
            //back
            //tilt to the left
            frontLeftWheel.setDirection(driveDirection - turnAngle);
        }
        //FRONT RIGHT
        //directly at the front right wheel is 45 degrees
        if (closestAngle(driveDirection, 45.0) < 90) {
            //front
            //tilt right
            frontRightWheel.setDirection(driveDirection + turnAngle);
        } else {
            //back
            //tilt left
            frontRightWheel.setDirection(driveDirection - turnAngle);
        }

        //BACK LEFT
        //directly at back left wheel is 225 degrees
        if (closestAngle(driveDirection, 225.0) < 90) {
            //front
            //tilt right
            backLeftWheel.setDirection(driveDirection + turnAngle);
        } else {
            //back
            //tilt left
            backLeftWheel.setDirection(driveDirection - turnAngle);
        }

        //BACK RIGHT
        //directly at back right wheel is 135 degrees
        if (closestAngle(driveDirection, 135.0) <= 90) {
            //front
            //tilt right
            backRightWheel.setDirection(driveDirection + turnAngle);
        } else {
            //back
            //tilt left
            backRightWheel.setDirection(driveDirection - turnAngle);
        }

        frontLeftWheel.setSpeed(driveSpeed);
        frontRightWheel.setSpeed(driveSpeed);
        backLeftWheel.setSpeed(driveSpeed);
        backRightWheel.setSpeed(driveSpeed);
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
