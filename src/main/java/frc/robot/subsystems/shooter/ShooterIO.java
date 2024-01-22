package frc.robot.subsystems.shooter;

public interface ShooterIO {

    public static class ShooterIOInputs{
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelAppliedVolts = 0.0;
        // public double hoodPositionRad = 0.0;
        // public double hoodAppliedVolts = 0.0;
        // public double hoodVelocityRadPerSec = 0.0;
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

        /** Stop in open loop. */
        public default void stop() {}

        /** Set velocity PID constants. */
        public default void flywheelConfigurePID(double kP, double kI, double kD) {}

        public default void turretConfigurePID(double kP, double kI, double kD) {}

        public default void hoodConfigurePID(double kP, double kI, double kD) {}

        public default void setTurretAngle() {}
} 
