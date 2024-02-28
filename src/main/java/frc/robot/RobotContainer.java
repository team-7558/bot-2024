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

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.Calibration;
import frc.robot.auto.RunAltAutoCommand;
import frc.robot.auto.Test;
import frc.robot.commands.RobotTeleop;
import frc.robot.subsystems.drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = Drive.getInstance();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(new RobotTeleop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //   return new FollowPathHolonomic(
    //         PathPlannerPath.fromChoreoTrajectory("LeftStart"),
    //         drive::getPose, // Robot pose supplier
    //         drive::getChassisSpeedsFromModuleStates, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    //         drive::runVelocity, // Method that will drive the robot given ROBOT RELATIVE
    // ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
    // live in your Constants class
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                 4.5, // Max module speed, in m/s
    //                 0.4, // Drive base radius in meters. Distance from robot center to furthest
    // module.
    //                 new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    //         ),
    //         () -> {
    //             // Boolean supplier that controls when the path will be mirrored for the red
    // alliance
    //             // This will flip the path being followed to the red side of the field.
    //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //             var alliance = DriverStation.getAlliance();
    //             if (alliance.isPresent()) {
    //                 return alliance.get() == DriverStation.Alliance.Red;
    //             }
    //             return false;
    //         },
    //         drive // Reference to this subsystem to set requirements
    // );

    return new RunAltAutoCommand(new Calibration());

    /*PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("4note");
    List<PathPoint> points = path.getAllPathPoints();
    PathPoint[] array = (PathPoint[]) points.toArray(new PathPoint[0]);
    Translation2d[] translationArray = new Translation2d[array.length];
    for (int i = 0; i < array.length; i++) {
      translationArray[i] = array[i].position;
    }
    Logger.recordOutput("Drive/Path", translationArray);

    List<PathPoint> pointsF = path.flipPath().getAllPathPoints();
    PathPoint[] arrayF = pointsF.toArray(new PathPoint[0]);
    Translation2d[] translationArrayF = new Translation2d[arrayF.length];
    for (int i = 0; i < arrayF.length; i++) {
      translationArrayF[i] = arrayF[i].position;
    }
    Logger.recordOutput("Drive/PathFlipped", translationArrayF);

    Pose2d pose = path.getPreviewStartingHolonomicPose();
    pose = pose.rotateBy(Rotation2d.fromDegrees(180));

    drive.setPose(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path.flipPath());*/
  }
}
