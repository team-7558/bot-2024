// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.ampside.AmpSeries;
import frc.robot.auto.ampside.DefaultMovingWhileShooting;
import frc.robot.commands.RobotTeleop;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private final AutoSelector AS =
      new AutoSelector()
          .add(new DefaultMovingWhileShooting(), 0, 24, 24)
          .add(new AmpSeries(0), 16, 16, 16)
          .add(new AmpSeries(1), 8, 32, 8)
          .add(new AmpSeries(2), 32, 0, 8);

  private Command autonomousCommand;
  private Drive drive;
  private Elevator elevator;
  private Shooter shooter;
  private Intake intake;
  private Vision vision;
  private LED led;

  boolean lastState = false;

  boolean resetPose = false;

  private Timer t = new Timer();
  private Timer tt = new Timer();

  protected Robot() {
    super(Constants.globalDelta_sec);

    elevator = Elevator.getInstance();
    intake = Intake.getInstance();
    drive = Drive.getInstance();
    shooter = Shooter.getInstance();
    vision = Vision.getInstance();
    led = LED.getInstance();
  }

  @Override
  public void disabledPeriodic() {
    boolean buttonPressed = RobotController.getUserButton();
    if (buttonPressed && !lastState) {
      System.out.println("Toggling Brakes");
      drive.toggleBrake();
      shooter.toggleBrake();
      elevator.toggleBrake();
      intake.toggleBrake();
    }

    lastState = buttonPressed;

    if (!resetPose && t.get() > 0.2) {
      resetPose = true;
      drive.resetPose();
    }

    int d0 = (int) (16 + 16 * Math.sin(6.2 * t.get()));
    int d1 = (int) (16 + 16 * Math.sin(5.6 * t.get()));
    int d2 = (int) (16 + 16 * Math.sin(7.9 * t.get()));
    int d3 = (int) (16 + 16 * Math.sin(6.0 * t.get()));

    led.draw7(0, d0, d0, d0);
    led.draw5(8, d1, d1, d1);
    led.draw5(16, d2, d2, d2);
    led.draw8(24, d3, d3, d3);

    int r = (int) (LED.HEIGHT * (0.5 + 0.5 * Math.sin(6.9 * t.get())));
    int c = (int) (LED.WIDTH * (0.5 + 0.5 * Math.sin(4.3 * t.get())));
    boolean red = G.isRedAlliance();
    led.scaleRow(r, red ? 3 : 0, 0, red ? 0 : 3);
    led.scaleCol(c, red ? 3 : 0, 0, red ? 0 : 3);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT ans local repo
        Logger.addDataReceiver(new WPILOGWriter("simlogs/"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();
    t.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    drive.setDefaultCommand(new RobotTeleop());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    boolean blinkinBlack = false;
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.

    if (OI.DR.getPOV() == 180) {
      drive.hardSetPose(
          new Pose2d(
              drive.getPose().getTranslation(),
              Rotation2d.fromRotations(G.isRedAlliance() ? 0.5 : 0)));
    }

    if (OI.XK.get(5, 3)) {
      LED.getInstance().drawRow(0, 255, 255, 0);
      LED.getInstance().setBlinkin(0.71);
    } else if (OI.XK.get(6, 3)) {
      LED.getInstance().drawRow(0, 128, 0, 128);
      LED.getInstance().setBlinkin(0.91);
    } else if (shooter.beamBroken()
        && !(shooter.getCurrentState() == shooter.TRACKING
            || shooter.getCurrentState() == shooter.SHOOTING)) {
      led.setBlinkin(-0.57);
    } else {
      LED.getInstance().setBlinkin(0.99);
    }

    SS.getInstance().periodic();
    SS2d.periodic();
    vision.periodic();
    CommandScheduler.getInstance().run();
    led.render();
    PerfTracker.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    OI.DR.setRumble(RumbleType.kBothRumble, 0);
    t.reset();
    t.start();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    AS.getCommand().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {    
    tt.reset();
    tt.start();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double tv = 135.0 - tt.get();
    if (tv < 10.0) led.drawNumber(tv, 48, 0, 0);
    else if (shooter.beamBroken()) {
      led.drawNumber(tv, 255, 25, 0);
    } else led.drawNumber(tv, 16, 16, 16);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
