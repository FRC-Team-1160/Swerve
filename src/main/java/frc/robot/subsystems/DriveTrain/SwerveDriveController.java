package frc.robot.subsystems.DriveTrain;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveDriveController {
    public SwerveDriveWheel frontLeftWheel;
    public SwerveDriveWheel frontRightWheel;
    public SwerveDriveWheel backLeftWheel;
    public SwerveDriveWheel backRightWheel;
    public double m_posx;
    public double m_posy;
    public SwerveDriveKinematics m_kinematics;
    public SwerveDriveOdometry m_odometry;

    public double angleToLoc(double ogRot)
    {
        if(ogRot < 0)
        {
            return ogRot + 360;
        }
        return ogRot;
    }
    public double locToAngle(double ogLoc) {
        if (ogLoc > 180) {
          ogLoc -= 360;
        }
        return ogLoc;
    }

    public SwerveDriveController(SwerveDriveWheel frontLeftWheel, SwerveDriveWheel frontRightWheel, SwerveDriveWheel backLeftWheel, SwerveDriveWheel backRightWheel, AHRS m_gyro) {
        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.backLeftWheel = backLeftWheel;
        this.backRightWheel = backRightWheel;
        Translation2d m_frontLeftLocation = new Translation2d(SwerveConstants.w/100.0, SwerveConstants.l/100.0);
        Translation2d m_frontRightLocation = new Translation2d(SwerveConstants.w/100.0, -SwerveConstants.l/100.0);
        Translation2d m_backLeftLocation = new Translation2d(-SwerveConstants.w/100.0, SwerveConstants.l/100.0);
        Translation2d m_backRightLocation = new Translation2d(-SwerveConstants.w/100.0, -SwerveConstants.l/100.0);

        m_kinematics = new SwerveDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );


        /*m_odometry = new SwerveDriveOdometry(
            m_kinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                this.frontLeftWheel.getModule(),
                this.frontRightWheel.getModule(),
                this.backLeftWheel.getModule(),
                this.backRightWheel.getModule()
            }, new Pose2d(0.0, 0.0, new Rotation2d()));*/
    }

    public double[] getSwerveOdometry(double gyroAngle) {
        double l = SwerveConstants.l;
        double w = SwerveConstants.w;
        double wa1 = locToAngle(frontRightWheel.getRotation());
        double wa2 = locToAngle(frontLeftWheel.getRotation());
        double wa3 = locToAngle(backLeftWheel.getRotation());
        double wa4 = locToAngle(backRightWheel.getRotation());

        double ws1 = frontRightWheel.getVelocity();
        double ws2 = frontLeftWheel.getVelocity();
        double ws3 = backLeftWheel.getVelocity();
        double ws4 = backRightWheel.getVelocity();
        
        double a3 = Math.sin(Math.toRadians(wa3))*ws3;
        double a4 = Math.sin(Math.toRadians(wa4))*ws4;
        double b2 = Math.sin(Math.toRadians(wa2))*ws2;
        double b1 = Math.sin(Math.toRadians(wa1))*ws1;
        double c1 = Math.cos(Math.toRadians(wa1))*ws1;
        double c4 = Math.cos(Math.toRadians(wa4))*ws4;
        double d2 = Math.cos(Math.toRadians(wa2))*ws2;
        double d3 = Math.cos(Math.toRadians(wa3))*ws3;

        double a = -(a3+a4)/2;
        double b = -(b2+b1)/2;
        double c = -(c1+c4)/2;
        double d = -(d2+d3)/2;

        double rot1 = (b-a)/l;
        double rot2 = (c-d)/w;
        double rot = (rot1+rot2)/2;

        double fwd1 = rot * (l/2) + c;
        double fwd2 = -rot * (l/2) + d;
        double tfwd = (fwd1 + fwd2)/2;

        double str1 = rot * (w/2) + a;
        double str2 = -rot * (w/2) + b;
        double tstr = (str1 + str2)/2;

        //field centric
        double fwd = tfwd * Math.cos(Math.toRadians(gyroAngle)) + tstr * Math.sin(Math.toRadians(gyroAngle));
        double str = tstr * Math.cos(Math.toRadians(gyroAngle)) - tfwd * Math.sin(Math.toRadians(gyroAngle));
        SmartDashboard.putNumber("a odom", a);
        SmartDashboard.putNumber("b odom", b);
        SmartDashboard.putNumber("c odom", c);
        SmartDashboard.putNumber("d odom", d);
        SmartDashboard.putNumber("FWD", fwd);
        SmartDashboard.putNumber("STR", str);
        SmartDashboard.putNumber("ROT", rot);
        double[] values = {fwd, str, rot};
        return values;

    }

    //chooses either turning in place or turning while driving
    public void setSwerveDrive(double fwd, double str, double rot, double gyroAngle) {

        double l = frc.robot.Constants.SwerveConstants.l;
        double r = frc.robot.Constants.SwerveConstants.r;
        double w = frc.robot.Constants.SwerveConstants.w;
        double a = str - (rot * (l / r));
		double b = str + (rot * (l / r));
		double c = fwd - (rot * (w / r));
		double d = fwd + (rot * (w / r));   
        SmartDashboard.putNumber("a kine", a);
        SmartDashboard.putNumber("b kine", b);
        SmartDashboard.putNumber("c kine", c);
        SmartDashboard.putNumber("d kine", d);

		double ws1 = Math.sqrt((b * b) + (c * c));//top right
		double ws2 = Math.sqrt((b * b) + (d * d));//top left
		double ws3 = Math.sqrt((a * a) + (d * d));//bittom left
		double ws4 = Math.sqrt((a * a) + (c * c));//bottom right

		double wa1 = (Math.atan2(b, c) * 180 / Math.PI);// + gyroAngle ;
		double wa2 = (Math.atan2(b, d) * 180 / Math.PI);// + gyroAngle;
		double wa3 = (Math.atan2(a, d) * 180 / Math.PI);// + gyroAngle;
		double wa4 = (Math.atan2(a, c) * 180 / Math.PI);// + gyroAngle;

		double max = ws1;
		max = Math.max(max, ws2);
		max = Math.max(max, ws3);
		max = Math.max(max, ws4);
		if (max > 1) {
			ws1 /= max;
			ws2 /= max;
			ws3 /= max;
			ws4 /= max;
		}
        frontRightWheel.set(angleToLoc(wa1), ws1);
        frontLeftWheel.set(angleToLoc(wa2), ws2);
        backLeftWheel.set(angleToLoc(wa3), ws3);
        backRightWheel.set(angleToLoc(wa4), ws4);
        //this.driveTurn(driveDirection, driveSpeed, turnSpeed);
        /*if ((driveSpeed <= 0.02) && (Math.abs(turnSpeed) != 0.0)) {
            //turns without driving if no drive speed
            this.turn(turnSpeed);
        } else {
            //either only drives if turnspeed is 0
            //or drives and turns at the same time if turnspeed is not 0
            this.driveTurn(driveDirection, driveSpeed, turnSpeed);

        }
        */

        
    }

    //DRIVE WITHOUT TURNING
    //we don't really actually need this but its good to have
    /*public void drive(double direction, double speed) {
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
        //while turning, can turn a maximum of 40 degrees
        driveSpeed *= (Math.log(Math.abs(turnSpeed)+1)/Math.log(3))*0.45 + 1;
        SmartDashboard.putNumber("turn Angle", turnAngle);
        //determine if wheel is in front or in back
        //for example: if the direction was to the right and the robot was facing forward,
        //the frontRight and backRight wheel would be front and the frontLeft and backLeft wheels would be in back
        //if driving direction is within 90 degrees of the direction of the wheel, wheel is in front

        //FRONT LEFT
        //directly at the front left wheel is 45 degrees
        if (closestAngle(driveDirection, 315.0) <= 90) {
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
        if (closestAngle(driveDirection, 45.0) < 90) {
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
        if (closestAngle(driveDirection, 225.0) < 90) {
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
        SmartDashboard.putNumber("BR closest angle", closestAngle(driveDirection, 135.0));
        if (closestAngle(driveDirection, 135.0) <= 90) {

            //front
            //tilt right
            backRightWheel.set(driveDirection + turnAngle, driveSpeed);
        } else {
            //back
            //tilt left
            backRightWheel.set(driveDirection - turnAngle, driveSpeed);
        }
    }*/

    private static double closestAngle(double a, double b)
    {
        if (Math.abs(a - b) <= 180) {
            return Math.abs(a-b);
        } else {
            if (a > b) {
                return Math.abs((a-360)-b);
            } else {
                return Math.abs((a+360)-b);
            }
        }
    }
}
