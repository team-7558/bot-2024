package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;

public class ElevatorIOSim implements ElevatorIO{
    private ElevatorSim sim = new ElevatorSim(DCMotor.getKrakenX60Foc(2), 0, 0, 0, 0, 0, false, 0, null);
    private edu.wpi.first.math.controller.PIDController pid = new PIDController(0.0, 0.0, 0.0);
    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
        appliedVolts =
            MathUtil.clamp(appliedVolts, ffVolts, appliedVolts);
        sim.setInputVoltage(appliedVolts);
    }
    sim.update(Constants.globalDelta_sec);
    inputs.vel_radps = sim.getVelocityMetersPerSecond();
    inputs.volts_V = appliedVolts;
    inputs.currents_A = new double[] {sim.getCurrentDrawAmps()};
    }
    
    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = 0.0;
        sim.setInputVoltage(volts);
    }
    
    @Override
    public void setVel_radps(double velocity_v, double ffVolts) {
        closedLoop = true;
        pid.setSetpoint(velocity_v);
        this.ffVolts = ffVolts;
    }
    
    @Override
    public void stop() {
        setVoltage(0.0);
    }
    
    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }
}
