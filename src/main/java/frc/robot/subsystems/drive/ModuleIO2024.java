package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIO2024 implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  private final VoltageOut driveVoltageSetpoint = new VoltageOut(0, true, false, false, false);
  private final VoltageOut turnVoltageSetpoint = new VoltageOut(0);

  private final VelocityVoltage driveVelocitySetpoint_v =
      new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final PositionVoltage turnPositionSetpoint_v =
      new PositionVoltage(0, 0, true, 0, 0, false, false, false);

  private final double index;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private final boolean isLeftSideDriveInverted = true;
  private boolean braked = true;
  private final Rotation2d absoluteEncoderOffset;

  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100.0).withKI(0).withKD(0).withKS(0).withKV(0.0).withKA(0);
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(2.3).withKI(0).withKD(0).withKS(0).withKV(0.85).withKA(0);
  private static final Slot1Configs steerGainsTorque =
      new Slot1Configs().withKP(10).withKI(0).withKD(0.2).withKS(0).withKV(0.0).withKA(0);
  private static final Slot1Configs driveGainsTorque =
      new Slot1Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0.3).withKA(0);

  public static final double kSpeedAt12VoltsMps = 4.73;

  public ModuleIO2024(int index) {
    this.index = index;
    switch (index) {
      case Drive.FL:
        turnTalon = new TalonFX(1, "Optimus");
        cancoder = new CANcoder(2, "Optimus"); // todo change
        driveTalon = new TalonFX(3, "Optimus");
        absoluteEncoderOffset = Rotation2d.fromRadians(0.934); // MUST BE CALIBRATED
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.148); // MUST BE CALIBRATED
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.3564453125); // MUST BE CALIBRATED
        break;
      case Drive.FR:
        driveTalon = new TalonFX(6, "Optimus");
        cancoder = new CANcoder(5, "Optimus"); // todo change
        turnTalon = new TalonFX(4, "Optimus");
        absoluteEncoderOffset = Rotation2d.fromRadians(1.786); // MUST BE CALIBRATED
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.283); // MUST BE CALIBRATED
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.210205078125); // MUST BE CALIBRATED
        break;
      case Drive.BR:
        driveTalon = new TalonFX(9, "Optimus");
        cancoder = new CANcoder(8, "Optimus"); // todo change
        turnTalon = new TalonFX(7, "Optimus");
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.20751953125); // MUST BE CALIBRATED
        absoluteEncoderOffset = Rotation2d.fromRadians(1.418); // MUST BE CALIBRATED
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.294); // MUST BE CALIBRATED
        break;
      case Drive.BL:
        driveTalon = new TalonFX(12, "Optimus");
        cancoder = new CANcoder(11, "Optimus"); // todo change
        turnTalon = new TalonFX(10, "Optimus");
        // absoluteEncoderOffset = Rotation2d.fromRotations(0.20751953125); // MUST BE CALIBRATED
        absoluteEncoderOffset = Rotation2d.fromRadians(-0.038); // MUST BE CALIBRATED
        // absoluteEncoderOffset = Rotation2d.fromRotations(-0.014); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.MagnetOffset = -absoluteEncoderOffset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        isTurnMotorInverted
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    var driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0 = driveGains;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        isLeftSideDriveInverted && (index == Drive.BL || index == Drive.FL)
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    driveConfig.Slot0 = driveGains;
    driveConfig.Slot1 = driveGainsTorque;
    driveTalon.getConfigurator().apply(driveConfig);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    turnConfig.Slot0 = steerGains;
    turnConfig.Slot1 = steerGainsTorque;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // turnConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.3;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnConfig.Feedback.RotorToSensorRatio = TURN_GEAR_RATIO;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnTalon.getConfigurator().apply(turnConfig);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();

    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePos_r = drivePosition.getValueAsDouble();
    inputs.driveVel_mps = driveVelocity.getValueAsDouble() * Module.WHEEL_RADIUS;
    inputs.driveVolts_V = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrent_A = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsPos_rot2d =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.turnPos_rot2d = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVel_rps = turnVelocity.getValueAsDouble();
    inputs.turnVolts_V = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrent_A = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePos_r =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPos_rot2d =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveVoltageSetpoint.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnVoltageSetpoint.withOutput(volts));
  }

  @Override
  public void toggleBrake() {
    var dconfig = new MotorOutputConfigs();
    var tconfig = new MotorOutputConfigs();
    dconfig.Inverted =
        isLeftSideDriveInverted && (index == Drive.BL || index == Drive.FL)
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tconfig.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    dconfig.NeutralMode = braked ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    tconfig.NeutralMode = braked ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    braked = !braked;
    driveTalon.getConfigurator().apply(dconfig);
    turnTalon.getConfigurator().apply(tconfig);
  }

  @Override
  public void setDriveVelocity(double velocity) {
    // driveTalon.setControl(drivevVelocitySetpoint_v.withVelocity(velocity));

    // velocity control with foc

    // driveTalon.setControl(new PositionVoltage(velocity, 0, true, velocity, 0, false, false,
    // false));
    driveTalon.setControl(
        driveVelocitySetpoint_v
            .withVelocity(velocity / Module.WHEEL_RADIUS)
            .withEnableFOC(true)
            .withSlot(0));
  }

  @Override
  public void setTurnAngle(double pos_r) {
    turnTalon.setControl(turnPositionSetpoint_v.withPosition(pos_r).withSlot(0));

    Logger.recordOutput("Drive/" + index + "/angleSetpoint", pos_r);

    // turnTalon.setControl(turnPositionSetpoint.withPosition(pos_r));
  }
}
