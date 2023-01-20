package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Reset;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.TestRotator;
import frc.robot.commands.ToggleGate;
import frc.robot.commands.followPath;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Commands

/*
import frc.robot.commands.drive.Drive;
import frc.robot.commands.vision.LimelightCameraToggle;
import frc.robot.commands.vision.LimelightLightToggle;
import frc.robot.commands.vision.LimelightSnapshotToggle;
import frc.robot.commands.vision.LimelightStreamToggle;
*/

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveTrain m_driveTrain = DriveTrain.getInstance(); 
  
    // Controllers
    private Joystick m_mainStick = new Joystick(OIConstants.mainStickPort);
    private Joystick m_firstStick = new Joystick(OIConstants.firstStickPort);
    private Joystick m_secondStick = new Joystick(OIConstants.secondStickPort);



    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();
  
      // Configure default commands
      m_driveTrain.setDefaultCommand(new SwerveDrive(m_driveTrain));

    }

      
  
    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    
    private void configureButtonBindings() {
      Trigger startButton = new JoystickButton(m_mainStick, Button.kStart.value);
      startButton.onTrue(new Reset(m_driveTrain));

      /*ew JoystickButton(m_mainStick, Button.kX.value)
        .whenPressed(
          new ToggleGate(m_driveTrain)
        );*/

    }

/*
 * COMMANDS
 */
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, PathConstraints maxSpd) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              m_driveTrain.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new followPath(
            traj, 
            m_driveTrain.m_pose, // Pose supplier
            new PIDController(0.2, 0.01, 0),
            new PIDController(0.2, 0.01, 0),
            new PIDController(0.2, 0.01, 0),
            maxSpd,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_driveTrain // Requires this drive subsystem
        )
    );
  }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    
    public Command getAutonomousCommand() {
        /*PathConstraints max = new PathConstraints(4, 3);
        PathPlannerTrajectory path = PathPlanner.loadPath("1.5m square", max);
        return followTrajectoryCommand(path, true, max);*/
        return null;
    }
    
}