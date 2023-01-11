package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Units are m kg s unless otherwise specified

  public static final class PortConstants {
    // CAN ID 
    public static final int FRONT_LEFT_ROTATION_DRIVE = 1;
    public static final int FRONT_RIGHT_ROTATION_DRIVE = 3;
    public static final int BACK_LEFT_ROTATION_DRIVE = 5;
    public static final int BACK_RIGHT_ROTATION_DRIVE = 7;

    public static final int FRONT_LEFT_DIRECTION_DRIVE = 2;
    public static final int FRONT_RIGHT_DIRECTION_DRIVE = 4;
    public static final int BACK_LEFT_DIRECTION_DRIVE = 6;
    public static final int BACK_RIGHT_DIRECTION_DRIVE = 8;

    public static final int FRONT_LEFT_CODER_DRIVE = 1;
    public static final int FRONT_RIGHT_CODER_DRIVE = 3;
    public static final int BACK_LEFT_CODER_DRIVE = 5;
    public static final int BACK_RIGHT_CODER_DRIVE = 7;

    public static final int GATE = 0;
  }

  public static final class SwerveConstants{
    public static final double l = 24.25;
    public static final double w = 24.25;
    public static final double r = Math.sqrt(Math.pow(l, 2) + Math.pow(w, 2));
    public static final double centerToMotor = 0;//Distance from center of motor to center of robot
    public static final double unitsToRotations = (2048/360);
  }

  public static final class OIConstants {
    public static final int mainStickPort = 0;
    public static final int firstStickPort = 1;
    public static final int secondStickPort = 2;
    public static final int thirdStickPort = 3;

  }

}