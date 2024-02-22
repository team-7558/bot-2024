package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

  private ElevatorSim sim =
      new ElevatorSim(DCMotor.getFalcon500Foc(2), 1, 0, 0, 0, 0, false, 0, null);
  private PIDController velPid = new PIDController(0.0, 0.0, 0.0);
  private PIDController posPid = new PIDController(0.0, 0.0, 0.0);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(Constants.globalDelta_sec);
    inputs.posMeters = sim.getPositionMeters();
    inputs.velMPS = sim.getVelocityMetersPerSecond();
    inputs.volts = appliedVolts;
    inputs.currents = new double[] {sim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVel(double velocity_mps) {
    velPid.setSetpoint(velocity_mps);
    setVoltage(velPid.calculate(sim.getVelocityMetersPerSecond()));
  }

  @Override
  public void holdPos(double pos_m) {
    posPid.setSetpoint(pos_m);
    setVoltage(posPid.calculate(sim.getPositionMeters()));
  }

  @Override
  public void travelToPos(double pos_m) {
    posPid.setSetpoint(pos_m);
    setVoltage(posPid.calculate(sim.getPositionMeters()));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
