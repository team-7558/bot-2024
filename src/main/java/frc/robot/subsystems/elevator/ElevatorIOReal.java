package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leftFalcon;
  private final TalonFX rightFalcon;

  private static final double MIN_HEIGHT_R = -86.379;
  private static final double MAX_HEIGHT_R = -28.326;
  private static final double STROKE_R = MAX_HEIGHT_R - MIN_HEIGHT_R;
  private static final double METERS_TO_ROTATIONS = STROKE_R / Elevator.STROKE_M;
  private static final double ROTATIONS_TO_METERS = 1.0 / METERS_TO_ROTATIONS;
  private final StatusSignal<Double> pos_m;
  private final StatusSignal<Double> vel_mps;
  private final StatusSignal<Double> acc_mps2;
  private final StatusSignal<Double> volts_V;
  private final StatusSignal<Double> leftCurrent_A, rightCurrent_A;

  private final boolean isLeftMotorInverted = false;
  private final VoltageOut voltageControl;
  private final MotionMagicVelocityTorqueCurrentFOC mmVelControl;
  private final PositionVoltage posControl;
  private final MotionMagicTorqueCurrentFOC mmPosControl;

  private double posOffset_m = 0.0;

  public ElevatorIOReal() {
    var leaderConfig = new TalonFXConfiguration();
    var followerConfig = new TalonFXConfiguration();

    voltageControl = new VoltageOut(0);
    mmVelControl = new MotionMagicVelocityTorqueCurrentFOC(0, 0, true, 0, 2, false, false, false);
    posControl = new PositionVoltage(Elevator.MIN_HEIGHT_M, 0, true, 0, 0, false, false, false);
    mmPosControl =
        new MotionMagicTorqueCurrentFOC(Elevator.MIN_HEIGHT_M, 0, 1, false, false, false);

    leftFalcon = new TalonFX(1);
    rightFalcon = new TalonFX(4);

    pos_m = leftFalcon.getPosition();
    vel_mps = leftFalcon.getVelocity();
    acc_mps2 = leftFalcon.getAcceleration();
    leftCurrent_A = leftFalcon.getStatorCurrent();
    rightCurrent_A = rightFalcon.getStatorCurrent();
    volts_V = leftFalcon.getMotorVoltage();

    leaderConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.Feedback.SensorToMechanismRatio =
        METERS_TO_ROTATIONS; // Figure out how to scale this to real height
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leaderConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
    leaderConfig.MotionMagic.MotionMagicAcceleration = 1;
    leaderConfig.MotionMagic.MotionMagicJerk = 1;
    leaderConfig.MotionMagic.MotionMagicExpo_kV = 1;
    leaderConfig.MotionMagic.MotionMagicExpo_kA = 1;

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

    // leftFalcon.setPosition(0, 0.01);
    resetPos(Elevator.MIN_HEIGHT_M);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(pos_m, vel_mps, acc_mps2, volts_V, leftCurrent_A, rightCurrent_A);
    inputs.posMeters = pos_m.getValueAsDouble() + posOffset_m;
    inputs.volts = volts_V.getValueAsDouble();
    inputs.velMetersPerSecond = vel_mps.getValueAsDouble();
    inputs.accMetersPerSecond2 = acc_mps2.getValueAsDouble();
    inputs.currents =
        new double[] {leftCurrent_A.getValueAsDouble(), rightCurrent_A.getValueAsDouble()};
  }

  @Override
  public void setVel(double vel_mps) {
    double v = MathUtil.clamp(vel_mps, -Elevator.MAX_VEL_MPS, Elevator.MAX_VEL_MPS);
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

  @Override
  public void resetPos(double pos_m) {
    posOffset_m = -this.pos_m.getValueAsDouble() + pos_m;
    Logger.recordOutput("Elevator/posOffset", posOffset_m);
  }

  @Override
  public void stop() {
    leftFalcon.stopMotor();
  }

  @Override
  public void setBrake(boolean brake) {
    var config = new MotorOutputConfigs();

    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    leftFalcon.getConfigurator().apply(config);
  }
}
