// RomiBase - gyroDrivB     auto command   TurnDegrGyro.j 

// mod to use onboard gyro instead of distance wheels have turned;
// execute() has manually coded P & I turn correction, whose feedback
// params need tuning for each target angle and often don't work smoothly

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnDegrGyro extends CommandBase {
  private final Drivetrain m_drive;
  private double m_degreeTarg;
  private double m_speed;
  private double turn_speed = 0;
  private double Ifactor = 0;

  /**
   * Creates new TurnDegrGyro cmd -- auto turns Romi
   * (in degrees reported by gyro) at commanded z-speed
   * directly proportional to error (target minus now).
   * Degrees here all robot centric, 0 = bot's fwd pointing
   * 
   * @param speed The speed to turn (+ = CW, - = CCW)
   * @param drive The drive subsystem to operate on
   */
  // Constructor -- relies on this being an auto-called cmd
  public TurnDegrGyro(double speed, double degree, Drivetrain drive) {
    // watch for large continuous gyro drift, recalibrate PRN
    // auto init resets all, so initial z should be close to 0;
    // if target param is 0, stay where you are.
    m_degreeTarg = degree != 0 ? degree : drive.getGyroAngleZ();
    m_speed = speed;
    m_drive = drive;

    // // normalize target angle -- keep between -180<-0->180
    m_degreeTarg %= 360;

    if (m_degreeTarg >= 181) //
      m_degreeTarg -= 360;
    if (m_degreeTarg <= -181)
      m_degreeTarg += 360;

    addRequirements(drive);
  } // end constructor

  // runs x1 when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set motors to stop, reset encoder & gyro to 0
    m_drive.arcaDriv(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro(); // calculations all assume start pos = 0 deg
  }

  // Called every time cmd scheduler runs while this is scheduled.
  @Override
  public void execute() { // pure turning; gyro's + is CW, - is CCW;
    // ~headError's sign is same as direction turn needs to go; large #
    // --> full turn speed, which should diminish to 0 as error drops

    // target angle should always be <180, normal start position near 0
    double headingError = m_degreeTarg - m_drive.getGyroAngleZ();
    // if target = 60 and gyro reads 75 deg (overshot targ), hE = -15

    // not needed for small error as with straight driving
    // if (headingError < -180)
    // headingError += 360;
    // if (headingError > 180)
    // headingError -= 360;

    // handcrafted PID feedback processor:
    // P correction multiplier directly Proportional to angular error
    // -- feedsback to turning speed, reducing speed as error decreases
    if (Math.abs(headingError) > .25 * Math.abs(m_degreeTarg)) {
      // fixed speed more stable for 1st 3/4 of turn than P feedback
      turn_speed = Math.copySign(1, headingError) * m_speed;
    }
    // transition zone -- add P (.003) and I factor [starts @ 0]
    else if (Math.abs(headingError) > 0.15 * Math.abs(m_degreeTarg))
      turn_speed = (0.003 * headingError * m_speed) + .0001 * (Ifactor += headingError);
    else // just Integral factor used to finish turn -- last 15%
      turn_speed = .0002 * (Ifactor += headingError);

    // neg # sent to arcaDriv is inverted to pos there, makes CCW correction;
    // --i.e. I want neg hE via calc. of turnspeed to cause CCW turn, so I
    // can leave this rotate param sign as is
    m_drive.arcaDriv(0, turn_speed);

  } // end execute

  // Called when the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
  }

  // Returns true when the turn is completed
  @Override
  public boolean isFinished() {
    // Compare degree current to degree target,
    // if <= 1 degree different then turn is complete
    if ((Math.abs(m_degreeTarg) - Math.abs(m_drive.getGyroAngleZ())) <= 1.0) {
      System.out.println("TDG is fin");
      return true;
    } else
      return false;
  } // end isFin

} // end TDG class
