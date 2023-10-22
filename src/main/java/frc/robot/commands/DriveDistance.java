// VSpace\romiBase - gyroDrivB    cmd to drive straight  DriveDistance.j 
// distance controlled by encoders reaching target (no PID); driving straight
// uses gyro feedback, w/ manually coded P factor

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase {
  private final Drivetrain m_drive;
  private final double m_distance;
  private final double m_speed;

  /*
   * Creates new DriveDistance command; goes straight fwd/bak at set
   * speed for distance per wheel encoders, using arcaGyve() correction.
   * 
   * @param speed The speed [0-1] for the robot to drive
   * 
   * @param inches The number of inches the robot will drive
   * 
   * @param drive The drivetrain subsystem for this cmd
   */
  public DriveDistance(double speed, double inches, Drivetrain drive) {
    m_speed = speed;
    m_distance = inches;
    m_drive = drive;
    addRequirements(drive);
  } // end constructor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcaDriv(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs it.
  @Override
  public void execute() {
    // gyro correction enabled by calling this method; i.e. Z setpt = 0
    // is default when using this method
    m_drive.arcaGyve(m_speed, 0);

    // to control use/not use gyro correction w/ button use this code:
    // if (m_drive.getGyroMode()) { // if it's set to true in the subsys
    // m_drive.arcaGyve(m_speed, 0);
    // // System.out.println("using gyroMode");
    // } else // normal aD using params from auto distance cmd
    // m_drive.arcaDriv(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
}
