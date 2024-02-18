package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityTorqueCurrentFOC mmVelControl;
  private final PositionVoltage posControl;
  private final MotionMagicTorqueCurrentFOC mmPosControl;

  public ElevatorIOReal() {
    var leaderConfig = new TalonFXConfiguration();
    var followerConfig = new TalonFXConfiguration();

    voltageControl = new VoltageOut(0);
    mmVelControl = new MotionMagicVelocityTorqueCurrentFOC(0, 0, true, 0, 1, false, false, false);
    posControl = new PositionVoltage(Elevator.MIN_HEIGHT_M, 0, true, 0, 0, false, false, false);
    mmPosControl =
        new MotionMagicTorqueCurrentFOC(Elevator.MIN_HEIGHT_M, 0, 2, false, false, false);

    leftFalcon = new TalonFX(1);
    rightFalcon = new TalonFX(3);

    elevatorPosition = leftFalcon.getPosition();
    elevatorVelocity = leftFalcon.getVelocity();
    elevatorCurrent = leftFalcon.getStatorCurrent();
    elevatorAppliedVolts = leftFalcon.getMotorVoltage();

    leaderConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.Feedback.SensorToMechanismRatio = 1; // Figure out how to scale this to real height
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
    leaderConfig.MotionMagic.MotionMagicAcceleration = 1;
    leaderConfig.MotionMagic.MotionMagicJerk = 200;
    leaderConfig.MotionMagic.MotionMagicExpo_kV = 200;
    leaderConfig.MotionMagic.MotionMagicExpo_kA = 200;

    // Position control gains
    leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.Slot0.kG = 0;
    leaderConfig.Slot0.kP = 0.0;
    leaderConfig.Slot0.kI = 0;
    leaderConfig.Slot0.kD = 0;

    // MotionMagic Position gains
    leaderConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.Slot1.kG = 0;
    leaderConfig.Slot1.kV = 0.0;
    leaderConfig.Slot1.kS = 0.0;
    leaderConfig.Slot1.kA = 0.0;
    leaderConfig.Slot1.kP = 0.0;
    leaderConfig.Slot1.kI = 0;
    leaderConfig.Slot1.kD = 0.0;

    // MotionMagic Velocity control
    leaderConfig.Slot2.GravityType = GravityTypeValue.Elevator_Static;
    leaderConfig.Slot2.kG = 0;
    leaderConfig.Slot2.kV = 0.0;
    leaderConfig.Slot2.kS = 0.0;
    leaderConfig.Slot2.kA = 0.0;
    leaderConfig.Slot2.kP = 0.0;
    leaderConfig.Slot2.kI = 0.0;
    leaderConfig.Slot2.kD = 0.0;

    leftFalcon.getConfigurator().apply(leaderConfig);

    rightFalcon.setControl(new Follower(leftFalcon.getDeviceID(), true).withUpdateFreqHz(50));
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
    leftFalcon.setControl(mmVelControl.withVelocity(v));
  }

  @Override
  public void setPos(double position) {
    leftFalcon.setControl(mmPosControl.withPosition(position));
  }

  @Override
  public void climb(double position) {
    leftFalcon.setControl(mmPosControl.withPosition(position));
  }

  @Override
  public void setVoltage(double volts_V) {
    leftFalcon.setControl(voltageControl.withOutput(volts_V));
  }

  public void stop() {
    leftFalcon.stopMotor();
  }
}
