package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {

  private FlywheelSim simIntake = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);
  private FlywheelSim simDirection = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);


  // private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  /** Updates the set of loggable inputs. */
  public void updateInputs(IntakeIOInputs inputs) {
    // if (closedLoop) {
    //   appliedVolts =
    //       MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
    //   sim.setInputVoltage(appliedVolts);
    // }

    simIntake.update(Constants.globalDelta_sec);
    simDirection.update(Constants.globalDelta_sec);

    inputs.intakeVelocityRadPerSec = simIntake.getAngularVelocityRadPerSec();
    inputs.intakeAppliedVolts = appliedVolts;
    inputs.currentAmps = new double[]{simIntake.getCurrentDrawAmps(), simDirection.getCurrentDrawAmps()};

    inputs.directionVelocityRadPerSec = simDirection.getAngularVelocityRadPerSec();
    inputs.directionAppliedVolts = appliedVolts;
  }

  /** Run open loop at the specified voltage. */
  public void setIntakeVoltage(double volts) {
    // closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    simIntake.setInputVoltage(appliedVolts);
  }

  /** Run closed loop at the specified velocity. */
  public void setIntakeVelocity(double velocity) {
    simIntake.setState(velocity);
  }


  public void setDirectionVoltage(double volts) {
    // closedLoop = false;
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    simDirection.setInputVoltage(appliedVolts);
  }
  
  public void setDirectionSpeed(double velocity) {
    simDirection.setState(velocity);
  }

  /** Stop in open loop. */
  public void stop() {
    setIntakeVoltage(0);
    setDirectionVoltage(0);
  }

  /** Set velocity PID constants. */
//   public void configurePID(double kP, double kI, double kD) {
//     pid.setPID(kP, kI, kD);
//   }

}