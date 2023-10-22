// VSpace\RomiBase - gyroDrivB    ArcadeDrive cmd, default for teleOp
            // gets mode from subsys, calls its drive method +/- gyro

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import static frc.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.Supplier;
// import edu.wpi.first.wpilibj2.command.PrintCommand;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain; // this is just a local pvt
  // variable for this class, but confusing to use same exact name
  // as RC's instance.

  /* orig.:
   * // private final Supplier<Double> m_xaxisSpeedSupplier;
   * // private final Supplier<Double> m_zaxisRotateSupplier;
   * 
   * Creates a new ArcadeDrive. This command will drive your robot using
   * the speed supplier lambdas. This command does not terminate.
   *
   * [@param] drivetrain The drivetrain subsystem on which this command runs
   * [@param] xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * [@param] zaxisRotateSupplier Lambda supplier of rotational speed
   */

  // CONSTRUCTOR uses simpler syntax than obscure lambda
  public ArcadeDrive(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  // orig. obscure constructor
  // public ArcadeDrive(
  // Drivetrain drivetrain,
  // Supplier<Double> xaxisSpeedSupplier,
  // Supplier<Double> zaxisRotateSupplier) {
  // m_drivetrain = drivetrain;
  // m_xaxisSpeedSupplier = xaxisSpeedSupplier;
  // m_zaxisRotateSupplier = zaxisRotateSupplier;
  // addRequirements(drivetrain);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // original:
    // m_drivetrain.arcadeDrive(m_xaxisSpeedSupplier.get(),
    // m_zaxisRotateSupplier.get());
  
    // mode [aD|gyro] var in subsys, set by button in telePeriodic, used here.
    if (m_drivetrain.getGyroMode()) {// no turn possible, 2nd param unused.
      m_drivetrain.arcaGyve(-m_controller.getRawAxis(1) * 0.5,
            m_controller.getRawAxis(0) * 0.4);
      // System.out.println("using gyroMode");
    } else // normal control using left button speed, rt button turn
      m_drivetrain.arcaDriv(-m_controller.getRawAxis(1) * 0.6,
            m_controller.getRawAxis(0)* 0.6);
  } // end execute

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // default does not finish when interrupted
  }
} // end ArcaDriv cmd class


