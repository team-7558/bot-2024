package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leftFalcon;
  private final TalonFX rightFalcon;

  private final StatusSignal<Double> elevatorPosition;
  private final StatusSignal<Double> elevatorVelocity;
  private final StatusSignal<Double> elevatorAppliedVolts;
  private final StatusSignal<Double> elevatorCurrent;

  private final boolean isLeftMotorInverted = false;
  private final VoltageOut volts_V;
  private final VelocityVoltage vel_mps;
  private final PositionVoltage pos_m;

  public ElevatorIOReal() {
    var leaderConfig = new TalonFXConfiguration();
    var followerConfig = new TalonFXConfiguration();

    volts_V = new VoltageOut(0);
    vel_mps = new VelocityVoltage(0);
    pos_m = new PositionVoltage(Elevator.MIN_HEIGHT_M);

    leftFalcon = new TalonFX(25);
    rightFalcon = new TalonFX(26);

    elevatorPosition = leftFalcon.getPosition();
    elevatorVelocity = leftFalcon.getVelocity();
    elevatorCurrent = leftFalcon.getStatorCurrent();
    elevatorAppliedVolts = leftFalcon.getMotorVoltage();

    leaderConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.Feedback.SensorToMechanismRatio = 1; // Figure out how to scale this to real height
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Position control gains
    leaderConfig.Slot0.kP = 0.0;

    // MotionMagic gains
    leaderConfig.Slot1.kV = 0.0;

    // Velocity control
    leaderConfig.Slot2.kV = 0.0;

    leftFalcon.getConfigurator().apply(leaderConfig);

    rightFalcon.setControl(new Follower(25, true).withUpdateFreqHz(50));
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        elevatorPosition, elevatorAppliedVolts, elevatorCurrent, elevatorVelocity);
    inputs.pos_m = elevatorPosition.getValueAsDouble();
    inputs.volts_V = elevatorAppliedVolts.getValueAsDouble();
    inputs.vel_mps = elevatorVelocity.getValueAsDouble();
    inputs.currents_A = new double[] {elevatorCurrent.getValueAsDouble()};
  }

  @Override
  public void setVel(double vel_mps) {
    double v = MathUtil.clamp(vel_mps, -Elevator.MAX_SPEED_V, Elevator.MAX_SPEED_V);

    leftFalcon.setControl(this.vel_mps.withVelocity(v));
  }

  @Override
  public void setVoltage(double volts_V) {
    leftFalcon.setControl(this.volts_V.withOutput(volts_V));
  }
}
