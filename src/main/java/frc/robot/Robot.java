// RomiBase - gyroDrivB       final 9/23                     Robot.j
// mod of AM Base, some syntax simpler; keeps Cmd/Subsys framework, 
// adds lambda 4 button cmds

// gyroDriv v. B extends gyroDrvA [Lesson 5,9,11 exercise]; goal here:
// use button w/ lambda to control Mode (from RC), mod auto TurnDegree()
// to use reaching gyro target instead of wheel travel distance to end cmd.

// Uses gyro to drive straight from initial heading (0). R bumper button (6)
// activates AD default cmd to call a modified arcaDriv() to correct heading
// with gyro (also disables manual turning); Button 1 (A) rezero's
// Gyro to drive straight from any position you get to in teleOp.

// Romi Problems: 60+ deg CPU temp, bad gyro drift (temp dependent?), 
// assymetry of motor response even w/ no load, consistent overshooting any
// end target condx that relies on distance -- proportional to speed?
// erratic reaching of any turn target, ? temp dependent

// For live vision, attach camera to any pi port, its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg stream,
// and (when/if Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Instantiate RobotContainer --> RC declares, instances, configs the
    // robot specific components and their functionality (methods).
    m_robotContainer = new RobotContainer();
  }

  // This function is called every robot packet, no matter the mode -- for
  // things that you want run during all modes, like diagnostics.
  // This runs after the mode specific periodic functions, but before
  // LiveWindow and SmartDashboard integrated updating.
  @Override
  public void robotPeriodic() {
    // Calls the Scheduler <-- this is responsible for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodics.
    // Must be here for anything in the Command-based framework to work.
    SmartDashboard.putNumber("Z axis Rot",
        m_robotContainer.m_drivetrain.m_gyro.getAngleZ());
    CommandScheduler.getInstance().run();
  } // end robotPeriodic

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
     }

  @Override //incessant (-) drift, so button A can reset manually in Disabled
  public void disabledPeriodic() {
    if (RobotContainer.m_controller.getRawButton(1))
      m_robotContainer.m_drivetrain.resetGyro();
  }

  // autoInit runs the autonomous command set in {@link RobotContainer}
  @Override
  public void autonomousInit() {
    // RC got selected routine from the SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // all auto methods reset encoder & gyro @ init, so not needed here
    // schedule the selected autonomous command (if not empty variable)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();    }
  } // end autoInit

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { // R bumper button hold activates
    // gyro mode to drive straight in auto as well as teleOp
    // ? possible to do this in RC with JoystkButt(cmd) yes but that
    // only works if button pressed after auto starts so kept here too
    if (RobotContainer.m_controller.getRawButton(6))
         m_robotContainer.m_drivetrain.setGyroMode(true);
    else
         m_robotContainer.m_drivetrain.setGyroMode(false);
  }
  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped,. If you want the
    // auto cmd to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_drivetrain.resetEncoders();
    m_robotContainer.m_drivetrain.resetGyro();
  } // end teleInit
  
  // This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() { // rt bumper button hold to
    // activate gyro mode to drive straight worked OK here but
    // moved to RC to use JoystkButton(cmd) structure
    // if (RobotContainer.m_controller.getRawButton(6))
    // m_robotContainer.m_drivetrain.setGyroMode(true);
    // else
    // m_robotContainer.m_drivetrain.setGyroMode(false);
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
} // end class
