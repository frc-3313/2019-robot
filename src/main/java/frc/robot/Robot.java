/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // Default configuration values
  private static final double DEFAULT_TURN_MULTIPLIER = .5;
  private static final double DEFAULT_DRIVE_MULTIPLIER = 1;

  Joystick thrustmaster = new Joystick(0);
  Joystick logitech = new Joystick(1);

  Compressor compressor = new Compressor(11);

  // Solenoids
  DualValveSolenoid liftSolenoid = new DualValveSolenoid(11, 0, 1);
  DualValveSolenoid armSolenoid = new DualValveSolenoid(11, 2, 3);
  DualValveSolenoid ejectorSolenoid = new DualValveSolenoid(11, 4, 5);

  // Limit Switches
  DigitalInput tiltLimit1 = new DigitalInput(0);

  // Motor Controllers
  Talon intakeMotor = new Talon(5);
  Talon tiltMotor = new Talon(6);

  WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(1);
  WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(2);
  WPI_VictorSPX backLeftMotor = new WPI_VictorSPX(3);
  WPI_VictorSPX backRightMotor = new WPI_VictorSPX(4);

  MecanumDrive drive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  // Shuffleboard code
  ShuffleboardTab robotStatusTab = Shuffleboard.getTab("Robot Status");
  NetworkTableEntry pressureSwitchStatus = robotStatusTab.add("Pneumatic Pressure", false).withPosition(0, 0)
      .withSize(2, 1).withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon")).getEntry();

  NetworkTableEntry tilt1Status = robotStatusTab.add("Tilt up", true).withPosition(2, 0).withSize(2, 1)
      .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon")).getEntry();

  NetworkTableEntry turnSpeedMultiplier = robotStatusTab.add("Turn Speed Multiplier", DEFAULT_TURN_MULTIPLIER)
      .withPosition(4, 0).withSize(2, 1).getEntry();

  NetworkTableEntry driveSpeedMultiplier = robotStatusTab.add("Drive Speed Multiplier", DEFAULT_DRIVE_MULTIPLIER)
      .withPosition(6, 0).withSize(2, 1).getEntry();

  NetworkTableEntry resolvedDriveMultiplier = robotStatusTab.add("Resolved Drive Speed", 0).withPosition(8, 0)
      .withSize(2, 1).getEntry();

  // Camera
  UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture();
  UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // CameraServer.getInstance().startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    pressureSwitchStatus.setBoolean(compressor.getPressureSwitchValue());
    resolvedDriveMultiplier
        .setDouble(driveSpeedMultiplier.getDouble(DEFAULT_DRIVE_MULTIPLIER) * (-thrustmaster.getRawAxis(2) + 1));
    tilt1Status.setBoolean(tiltLimit1.get());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopPeriodic() {
    drive.driveCartesian(thrustmaster.getRawAxis(0) * driveSpeedMultiplier.getDouble(DEFAULT_DRIVE_MULTIPLIER),
        -thrustmaster.getRawAxis(1) * driveSpeedMultiplier.getDouble(DEFAULT_DRIVE_MULTIPLIER)
            * (-thrustmaster.getRawAxis(2) + 1),
        thrustmaster.getRawAxis(3) * turnSpeedMultiplier.getDouble(DEFAULT_TURN_MULTIPLIER)
            * (-thrustmaster.getRawAxis(2) + 1));

    // Lift Position
    if (logitech.getRawButton(10)) { //
      liftSolenoid.set(1);
    } else if (logitech.getRawButton(11)) { //
      liftSolenoid.set(0);
    } else {
      liftSolenoid.set(2);
    }

    // Arm Position
    if (thrustmaster.getRawButton(7) || logitech.getRawButton(7)) { // Down
      armSolenoid.set(1);
    } else if (thrustmaster.getRawButton(6) || logitech.getRawButton(6)) { // Up
      armSolenoid.set(0);
    } else {
      armSolenoid.set(2);
    }

    // Shoot/Intake
    if (thrustmaster.getRawButton(1) || logitech.getRawButton(1)) { // Shoot
      intakeMotor.set((-logitech.getRawAxis(2) + 1) / 2);
    } else if (thrustmaster.getRawButton(2) || logitech.getRawButton(4) || logitech.getRawButton(5)) { // Intake
      intakeMotor.set((-logitech.getRawAxis(2) + 1) / -2);
    } else { // No Movement
      intakeMotor.set(0);
    }

    // Tilt
    if (thrustmaster.getRawButton(9) || logitech.getRawButton(2)) {
      tiltMotor.set(1);
    } else if (thrustmaster.getRawButton(10) && !tiltLimit1.get() || logitech.getRawButton(3) && !tiltLimit1.get()) {
      tiltMotor.set(-1);
    } else {
      tiltMotor.set(0);
    }

    // Ejector
    if (logitech.getRawButton(8)) {
      ejectorSolenoid.set(0);
    } else {
      ejectorSolenoid.set(1);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
