package frc.robot.auto;

import java.util.Collections;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class SwerveFollower {

    private final PPHolonomicDriveController con;

    private final Drive drive;
    private final IFollowable toFollow;

    public SwerveFollower(IFollowable toFollow){
        this.toFollow = toFollow;

        this.drive = Drive.getInstance();

        con = new PPHolonomicDriveController(Drive.HPFG.translationConstants, Drive.HPFG.rotationConstants, 
                Constants.globalDelta_sec, Drive.MAX_LINEAR_SPEED, Drive.DRIVE_BASE_RADIUS);

    }

    public void start(){
        if(!toFollow.isGenerated()){
            toFollow.generate();
        }

        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentSpeeds = drive.getChassisSpeedsFromModuleStates();

        con.reset(currentPose, currentSpeeds);

        ChassisSpeeds fieldSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
        Rotation2d currentHeading =
            new Rotation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Rotation2d targetHeading = toFollow.getInitState().heading; //TODO: check if this is correct
        Rotation2d headingError = currentHeading.minus(targetHeading);
        boolean onHeading =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.25
            || Math.abs(headingError.getDegrees()) < 30;

        //TODO: Figure this out
        /*if (!path.isChoreoPath()
            && replanningConfig.enableInitialReplanning
            && (currentPose.getTranslation().getDistance(path.getPoint(0).position) > 0.25
                || !onHeading)) {
          replanPath(currentPose, currentSpeeds);
        } else {
          generatedTrajectory = path.getTrajectory(currentSpeeds, currentPose.getRotation());
          PathPlannerLogging.logActivePath(path);
          PPLibTelemetry.setCurrentPath(path);
        }*/

    }

    public void step(double t){
        PathPlannerTrajectory.State targetState = toFollow.sample(t);

        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentSpeeds = drive.getChassisSpeedsFromModuleStates();

        //TODO: figure this out too
        /*if (!path.isChoreoPath() && replanningConfig.enableDynamicReplanning) {
          double previousError = Math.abs(controller.getPositionalError());
          double currentError = currentPose.getTranslation().getDistance(targetState.positionMeters);

          if (currentError >= replanningConfig.dynamicReplanningTotalErrorThreshold
              || currentError - previousError
                  >= replanningConfig.dynamicReplanningErrorSpikeThreshold) {
            replanPath(currentPose, currentSpeeds);
            timer.reset();
            targetState = generatedTrajectory.sample(0);
          }
        }*/

        ChassisSpeeds targetSpeeds = con.calculateRobotRelativeSpeeds(currentPose, targetState);

        double currentVel =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        PPLibTelemetry.setCurrentPose(currentPose);
        PathPlannerLogging.logCurrentPose(currentPose);

        PPLibTelemetry.setTargetPose(targetState.getTargetHolonomicPose());
        PathPlannerLogging.logTargetPose(targetState.getTargetHolonomicPose());

        PPLibTelemetry.setVelocities(
            currentVel,
            targetState.velocityMps,
            currentSpeeds.omegaRadiansPerSecond,
            targetSpeeds.omegaRadiansPerSecond);
        PPLibTelemetry.setPathInaccuracy(con.getPositionalError());

        drive.runVelocity(targetSpeeds);

    }

    public boolean elapsed(double t) {
        return t >= toFollow.endTime();
    }

    public void end(boolean interrupted) {
        // Only output 0 speeds when ending a path that is supposed to stop, this allows interrupting
        // the command to smoothly transition into some auto-alignment routine
        if (!interrupted && toFollow.getEndState().velocityMps < 0.05) {
            drive.runVelocity(new ChassisSpeeds());
        }

        PathPlannerLogging.logActivePath(null);
    }

}
