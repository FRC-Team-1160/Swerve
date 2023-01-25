package frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
        double[] values = {fwd, -str, rot};
        return values;

    }

    //chooses either turning in place or turning while driving
    public void setSwerveDrive(boolean isJoystick, double fwd, double str, double rot, double gyroAngle) {

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
        if (isJoystick)
        {
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
            frontRightWheel.setOutput(angleToLoc(wa1), ws1);
            frontLeftWheel.setOutput(angleToLoc(wa2), ws2);
            backLeftWheel.setOutput(angleToLoc(wa3), ws3);
            backRightWheel.setOutput(angleToLoc(wa4), ws4);
        } else {
            
            frontRightWheel.setVelocity(angleToLoc(wa1), ws1);
            frontLeftWheel.setVelocity(angleToLoc(wa2), ws2);
            backLeftWheel.setVelocity(angleToLoc(wa3), ws3);
            backRightWheel.setVelocity(angleToLoc(wa4), ws4);
        }
        SmartDashboard.putNumber("FR expected angle", angleToLoc(wa1));
        SmartDashboard.putNumber("FL expected angle", angleToLoc(wa2));
        SmartDashboard.putNumber("BL expected angle", angleToLoc(wa3));
        SmartDashboard.putNumber("BR expected angle", angleToLoc(wa4));
        SmartDashboard.putNumber("FR actual angle", frontRightWheel.getRotation());
        SmartDashboard.putNumber("FL actual angle", frontLeftWheel.getRotation());
        SmartDashboard.putNumber("BL actual angle", backLeftWheel.getRotation());
        SmartDashboard.putNumber("BR actual angle", backRightWheel.getRotation());
        SmartDashboard.putNumber("FR expected speed", ws1);
        SmartDashboard.putNumber("FL expected speed", ws2);
        SmartDashboard.putNumber("BL expected speed", ws3);
        SmartDashboard.putNumber("BR expected speed", ws4);
        SmartDashboard.putNumber("FR actual speed", frontRightWheel.getVelocity());
        SmartDashboard.putNumber("FL actual speed", frontLeftWheel.getVelocity());
        SmartDashboard.putNumber("BL actual speed", backLeftWheel.getVelocity());
        SmartDashboard.putNumber("BR actual speed", backRightWheel.getVelocity());
        SmartDashboard.putNumber("FR speed difference", Math.abs(frontRightWheel.getVelocity())- Math.abs(ws1));

        
    }

    public void brake(double power) {
        frontLeftWheel.brake(power);
        frontRightWheel.brake(power);
        backLeftWheel.brake(power);
        backRightWheel.brake(power);
    }

    public double testFrontRightWheel(double spd) {
        frontRightWheel.setOutputSpeed(spd);
        return frontRightWheel.getVelocity();
    }
}
