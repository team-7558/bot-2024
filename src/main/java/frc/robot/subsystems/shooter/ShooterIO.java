package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs{
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double linearActuatorPositionLeft = 0.0;
        public double linearActuatorPositionRight = 0.0;
        public double turretVelocityRadPerSec = 0.0;
        public double turretAppliedVolts = 0.0;
        public double turretPositionRad = 0.0;
        public double[] currentAmps = new double[] {};
        public boolean beamBreakActivated =  true;

    }
            /** Updates the set of loggable inputs. */
        public default void updateInputs(ShooterIOInputs inputs) {}

        /** Run open loop at the specified voltage. */
        public default void setFlywheelVoltage(double volts) {}

        /** Run closed loop at the specified velocity. */
        public default void setFlywheelVelocity(double velocityRadPerSec, double ffVolts) {}

        /** Run open loop at the specified voltage. */
        public default void setTurretVoltage(double volts) {}

        /** Run closed loop at the specified velocity. */
        public default void setTurretVelocity(double velocityRadPerSec, double ffVolts) {}

        public default void setLeftLinearActuatorPosition (double position) {}

        public default void setRightLinearActuatorPosition (double position) {}

        
        public default void setTilt(double angle) {}

        /** Stop in open loop. */
        public default void stop() {}

        /** Set velocity PID constants. */
        public default void flywheelConfigurePID(double kP, double kI, double kD) {}

        public default void turretConfigurePID(double kP, double kI, double kD) {}

        public default void hoodConfigurePID(double kP, double kI, double kD) {}

        public default void setTurretAngle(double angle) {}
} 
