// romiBase - gyroDrivB                  Drivetrain.j subsystem

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.756; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Use differential drive controller for its arcade drive method
  // [applies default deadband 0.02, squares inputs by default]
  private final DifferentialDrive m_diffDrive = new 
         DifferentialDrive(m_leftMotor, m_rightMotor);

  // to select normal arcaDriv() or subsys's arcaGyve()<-- set by button in auto
  // & teleoPeriod,  got by ArcadeDrive cmd
  public boolean gyroMode = false;

  // instance the RomiGyro
  public final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  // private final BuiltInAccelerometer m_accelerometer = 
  //  newBuiltInAccelerometer();

  /** Constructs a new Drivetrain subsyst */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
       kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
       kCountsPerRevolution);
    resetEncoders();
    resetGyro();
  }  //  end constructor

// if this gyroMode variable is true, then calls to this subsystem can use
// its arcaGyve() method to drive straight with gyro assist.
  public void setGyroMode(boolean maybe) {
    gyroMode = maybe;
  }

  public boolean getGyroMode() {
    return gyroMode;
  }

  // sloppy naming by WPI: top aD() is subsys. method, second is inherited
  // diffDrive method; [originally] using same name confused the issue
  public void arcaDriv(double xaxisSpeed, double zaxisRotate) {
    // diffDrive class method processes positive zaxis-rot as CCW rotation
    // so I invert to keep consistent w/ gyro reading CCW as negative degr
    m_diffDrive.arcadeDrive(xaxisSpeed, -zaxisRotate, true);
  }

  // aG() method uses Gyro to keep straight course, set in teleInit to 0;
  // also rezeroed by button A press;  no manual turn input possible.
  // AD cmd calls this method when button 6 held, works in auto too.
  // aG() and normal aD() get same params from AD cmd; non-squared aG faster.
  public void arcaGyve(double xaxisSpeed, double zaxisRotate) {
    // proportional feedback of error, refresh angle q loop,
    // angular deviation%mod * scaling factor. This P multiple
    // works well for small deviation while driving straight (only)
   double gyroAdjust = getAngleMod() * 0.015; 
    // if no z input from stick and target angle == 0, feedback example:
    // +5 deg angle * .02 = -.10 power applied to turn
    // z turn cmd corrects angle error
    double zaxisRot = gyroAdjust;  // stick z input totally ignored
    // double zaxisRot = zaxisRotate - gyroAdjust ; // not ignored,? problem;
    // number may be small so I don't want to square it (diffDrv's default)
       m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRot, false);
  } // end arcaGyve
     // N.B. error > few deg. will likely cause continuous oscillation
  public double getAngleMod() {
    // normalize large turn number 
    // assumes target heading is 0
    // [in DD class, positive rotation cmd goes CCW, but our gyro reads
    // CCW as negative deg., hence need to invert turn param in DD's method.
    double anglError = getGyroAngleZ() ;

    anglError %= 360;    
    if (anglError < -180)
      anglError += 360;
    if (anglError > 180)
      anglError -= 360;

    return anglError;
  } // end getAngleMod

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }
  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }
  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }
    
  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }
  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }
// Re'zero' the gyro. 
  public void resetGyro() {
    m_gyro.reset();
    System.out.println("gyro reset");
  }

   // @return The current X angle of the Romi in degrees
public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }
   // @return The current Y angle of the Romi in degrees
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }
   // @return The current Z angle of the Romi in degrees
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }
       // @return The acceleration of the Romi along the X-axis in Gs
//   public double getAccelX() {
//     return m_accelerometer.getX();
//   }
//    // @return The acceleration of the Romi along the Y-axis in Gs
//   public double getAccelY() {
//     return m_accelerometer.getY();
//   }
//    // @return The acceleration of the Romi along the Z-axis in Gs 
//   public double getAccelZ() {
//     return m_accelerometer.getZ();
//   }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}  // end class
