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
    public SwerveDriveKinematics m_kinematics;
    public SwerveDriveOdometry m_odometry;
    public Pose2d m_pose;

    public double angleToLoc(double ogRot)
    {
        if(ogRot < 0)
        {
            return ogRot + 360;
        }
        return ogRot;
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


        m_odometry = new SwerveDriveOdometry(
            m_kinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                this.frontLeftWheel.getModule(),
                this.frontRightWheel.getModule(),
                this.backLeftWheel.getModule(),
                this.backRightWheel.getModule()
            }, new Pose2d(0.0, 0.0, new Rotation2d()));
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
    }
}
