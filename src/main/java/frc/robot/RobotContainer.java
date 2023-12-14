// RomiBase - gyroDrivB                 RobotContainer.j

package frc.robot;

//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutoSequRotate;
import frc.robot.commands.AutonSequen;
import frc.robot.commands.TurnDegrGyro;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/* RC is where this robot specifics are defined. Since Command-based is a
 * "declarative" paradigm, very little robot logic should be handled in
 * the {@link Robot} periodic methods other than the scheduler calls.
 */
public class RobotContainer {
  // instance the two subsystems
  protected final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO

  (ChannelMode.INPUT, ChannelMode.INPUT);
  // NOTE: re: I/O pin function config possible in web interface; v. base code

  // instance joystick --assumes stick or gamepad plugged into USB 0
  // numerous get()s in AD cmd require public stick
  public static final XboxController m_controller = new XboxController(0);

  // put chooser on SmartDashboard to pick autonomous routine
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * CONSTRUCT container for robot: its single method, configBB() sets
   * Drivetrain [subsystem's] default Cmd, OperatorInterface (OI) actions,
   * Smart Dashbd Autonomous chooser options.
   */
  public RobotContainer() {
    // Configure joystick button bindings et. al.
    configureButtonBindings();
  } // end constructor

  /*
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {GenericHID} or one of its subclasses
   * edu.wpi.first.wpilibj.Joystick} or {XboxController}, and then passing
   * it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command -- ArcadeDrive -- runs unless another command
    // is scheduled over it.(e.g. runs in teleOp unless overridden)
    // [orig. code] m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
    // now less obscure syntax, with simpler constructor
    m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain));

    // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA // just prints once
        .whileTrue(new PrintCommand("Button A Press"))
        .whileFalse(new PrintCommand("Button A Release"));

    // Button A press resets Gyro (to 0) (instanced in drive subsystem)
    // --- two ways to make a subsys Runnable method into instant cmd
    new JoystickButton(m_controller, 1)
         .onTrue(Commands.runOnce(m_drivetrain::resetGyro, m_drivetrain))
        // .onTrue(new InstantCommand(() -> m_drivetrain.resetGyro()))
        .onTrue(new PrintCommand("Button 1 Press"));

    new JoystickButton(m_controller, 2)
        .onTrue(new TurnDegrGyro(0.4, -90, m_drivetrain)
                                 .withTimeout(5));

    // in teleOp works any time; must press p autonomous starts
    new JoystickButton(m_controller, 6) // R bumper
        .onTrue(new InstantCommand(() -> m_drivetrain.setGyroMode(true)))
        .onFalse(new InstantCommand(() -> m_drivetrain.setGyroMode(false)));

    // Set SmartDashboard auto options
    m_chooser.setDefaultOption("AutoSequ180", new AutonSequen(m_drivetrain));
    m_chooser.addOption("AutoSequRota", new AutoSequRotate(m_drivetrain));
    SmartDashboard.putData(m_chooser);

  } // end configBB()

  // ... passes selected auto command to the scheduling {@link Robot} class.
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  } // end get.AutoCmd

} // end RC class
