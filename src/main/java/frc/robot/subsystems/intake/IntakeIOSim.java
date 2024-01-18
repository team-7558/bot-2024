package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(3), 1.5, 0.004);

  // private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  /** Updates the set of loggable inputs. */
  public void updateInputs(IntakeIOInputs inputs) {
    // if (closedLoop) {
    //   appliedVolts =
    //       MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
    //   sim.setInputVoltage(appliedVolts);
    // }

    sim.update(Constants.globalDelta_sec);

    inputs.intakeVelocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.intakeAppliedVolts = appliedVolts;
    inputs.intakeCurrentAmps = new double[]{sim.getCurrentDrawAmps()};

  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    // closedLoop = false;
    appliedVolts = 0.0;
    sim.setInputVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(double velocityRadPerSec) {
    //closedLoop = false;
    appliedVolts = 0;
    sim.setState(1);
  }

  /** Stop in open loop. */
  public void stop() {
    setVelocity(0);
  }

  /** Set velocity PID constants. */
//   public void configurePID(double kP, double kI, double kD) {
//     pid.setPID(kP, kI, kD);
//   }

}