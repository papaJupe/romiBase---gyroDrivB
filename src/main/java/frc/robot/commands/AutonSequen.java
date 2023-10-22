// VSpace\romiBase - gyroDrivB     auto sequence   AutonSequen.j   

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonSequen extends SequentialCommandGroup {
  /*
   *  Autonomous sequence using distance and turn cmds. This will drive
   * a specified distance, turn CCW 180, drive back to start, and turn 180.
   * 'manually' coded P + I feedback turn control -- mostly "accurate"
   * @param drivetrain -- the subsystem to be controlled
   */
  public AutonSequen(Drivetrain drivetrain) {
    addCommands(
        new DriveDistance(0.6, 36, drivetrain),
        new WaitCommand(5.0),
        new TurnDegrGyro(0.6, -178, drivetrain), // turn CCW
        new WaitCommand(5.0),
        new DriveDistance(0.6, 33, drivetrain),
        new WaitCommand(5.0),
        new TurnDegrGyro(0.6, 178, drivetrain));// turn CW
  }
}  // end class
