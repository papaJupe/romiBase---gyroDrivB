// VSpace\romiBase - gyroDrivB      auto sequence  AutoSequRotate.j 

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSequRotate extends SequentialCommandGroup {
  /*
   * An Autonomous sequence of 2 turns -- will turn CW  X degr, wait 5 sec,
   then reverse turn (whatever params are). Use to tune manual PI coeffic.
   *  -- to 'manually' code P + I feedback for various angle,
   * since they will differ
   * @param drivetrain -- the subsystem to be controlled
   */
  public AutoSequRotate(Drivetrain drivetrain) {
    addCommands(
        new TurnDegrGyro(0.6, 170, drivetrain), // turn CW
        new WaitCommand(5.0),
        new TurnDegrGyro(0.6, -170, drivetrain));// turn CCW
  }
}  // end class
