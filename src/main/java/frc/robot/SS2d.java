package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class SS2d {

  private static final double SCENE_WIDTH_M = 7;
  private static final double SCENE_HEIGHT_M = 5;

  public static final double TARGET_HEIGHT_FROM_GROUND_M = 1.9812;
  public static final double TARGET_HEIGHT = 2.105025 - TARGET_HEIGHT_FROM_GROUND_M;
  public static final double TARGET_SLOPE = 14;

  public static final double CENTER_TO_TURRET_M = 0.1016;
  public static final double TURRET_TO_PIVOT_M = 0.0508;
  public static final double CENTER_TO_INTAKE_M = Units.inchesToMeters(3.75);

  public static final double GROUND_TO_ELEVATOR_BASE_M = Units.inchesToMeters(16.5);
  public static final double ELEVATOR_BASE_TO_ELEVATOR_TOP_M = Units.inchesToMeters(8.125);
  public static final double GROUND_TO_MIN_HEIGHT =
      GROUND_TO_ELEVATOR_BASE_M + ELEVATOR_BASE_TO_ELEVATOR_TOP_M;
  public static final double GROUND_TO_MAX_HEIGHT = 0.8763;

  public static final double GROUND_TO_SHOOTER_PIVOT = 0.254; // Poofs reference?
  public static final double SHOOTER_PIVOT_TO_TIP = 0.3683; // DAVE referece?
  public static final double SHOOTER_TIP_TO_ROOF = Units.inchesToMeters(8.5);
  public static final double SHOOTER_ROOF_LENGTH = Units.inchesToMeters(7.75);

  public static final double TURRET_MECH_LIMIT_A = 90.0;
  public static final double TURRET_MECH_LIMIT_B = -90.0;
  public static final double TURRET_FEED_LIMIT_A = 30.0;
  public static final double TURRET_FEED_LIMIT_B = -30.0;

  private static final Color8Bit COLOR_BG = new Color8Bit(0, 0, 0);
  private static final Color8Bit COLOR_REF = new Color8Bit(10, 10, 10);
  private static final Color8Bit COLOR_FRAME = new Color8Bit(100, 100, 100);
  private static final Color8Bit COLOR_NEUTRAL = new Color8Bit(255, 255, 255);
  private static final Color8Bit COLOR_FWD = new Color8Bit(0, 255, 0);
  private static final Color8Bit COLOR_REV = new Color8Bit(255, 0, 0);

  public static final SS2d S, M;

  static {
    S = new SS2d();
    M = new SS2d();
  }

  private final Mechanism2d mech;
  private final MechanismLigament2d yref;
  private final MechanismLigament2d xref;
  private final MechanismLigament2d intakeBase;
  private final MechanismLigament2d shooterBase;
  private final MechanismLigament2d pivotBase;
  private final MechanismLigament2d lowerIntake;
  private final MechanismLigament2d elevatorBase;
  private final MechanismLigament2d upperIntake;
  private final MechanismLigament2d shooter;
  private final MechanismLigament2d turretBase;
  private final MechanismLigament2d turret;

  private SS2d() {
    mech = new Mechanism2d(SCENE_WIDTH_M, SCENE_HEIGHT_M, COLOR_BG);

    MechanismRoot2d root = mech.getRoot("scene", 0, 0);

    yref = root.append(new MechanismLigament2d("yref", 0.5, 90, 1.0, COLOR_REF));
    xref = yref.append(new MechanismLigament2d("xref", SCENE_WIDTH_M * 0.5, -90, 1.0, COLOR_REF));

    MechanismLigament2d target =
        yref.append(
            new MechanismLigament2d("target height", TARGET_HEIGHT_FROM_GROUND_M, 0, 1, COLOR_REF));
    target.append(
        new MechanismLigament2d(
            "target",
            TARGET_HEIGHT * Math.cos(Units.degreesToRadians(TARGET_SLOPE)),
            -TARGET_SLOPE,
            1,
            COLOR_NEUTRAL));

    intakeBase =
        xref.append(
            new MechanismLigament2d("driveintake", CENTER_TO_INTAKE_M, 0.0, 1.0, COLOR_FRAME));
    shooterBase =
        xref.append(
            new MechanismLigament2d("driveshooter", CENTER_TO_TURRET_M, 180.0, 1.0, COLOR_REF));
    pivotBase =
        shooterBase.append(
            new MechanismLigament2d("pivot base", TURRET_TO_PIVOT_M, 180.0, 1.0, COLOR_FRAME));

    MechanismLigament2d shooterHeight =
        pivotBase.append(
            new MechanismLigament2d("shooter height", GROUND_TO_SHOOTER_PIVOT, 90, 1, COLOR_REF));

    shooter =
        shooterHeight.append(
            new MechanismLigament2d("shooter", SHOOTER_PIVOT_TO_TIP, 90, 1, COLOR_NEUTRAL));

    MechanismLigament2d shooterTip =
        shooter.append(
            new MechanismLigament2d("shooter tip", SHOOTER_TIP_TO_ROOF, -90, 1.0, COLOR_FRAME));
    shooterTip.append(
        new MechanismLigament2d("shooter roof", SHOOTER_ROOF_LENGTH, -90, 1.0, COLOR_FRAME));

    lowerIntake =
        intakeBase.append(
            new MechanismLigament2d(
                "lower intake", GROUND_TO_ELEVATOR_BASE_M, 90, 4, COLOR_NEUTRAL));

    elevatorBase =
        intakeBase.append(
            new MechanismLigament2d("elevator base", GROUND_TO_MIN_HEIGHT, 90, 1, COLOR_REF));

    upperIntake =
        elevatorBase.append(
            new MechanismLigament2d(
                "upper intake", ELEVATOR_BASE_TO_ELEVATOR_TOP_M, 0, 2, COLOR_NEUTRAL));

    MechanismLigament2d turretOffset =
        xref.append(
            new MechanismLigament2d("turret offset", SCENE_HEIGHT_M * 0.5, 90, 1, COLOR_BG));
    turretBase =
        turretOffset.append(new MechanismLigament2d("turret base", 0.01, 180, 1, COLOR_REF));
    turret = turretBase.append(new MechanismLigament2d("turret", 0.4, 0, 2.5, COLOR_NEUTRAL));
    turretBase.append(
        new MechanismLigament2d("mech limit A", 0.5, TURRET_MECH_LIMIT_A, 1, COLOR_FRAME));
    turretBase.append(
        new MechanismLigament2d("mech limit B", 0.5, TURRET_MECH_LIMIT_B, 1, COLOR_FRAME));
    turretBase.append(
        new MechanismLigament2d("feed limit A", 0.6, TURRET_FEED_LIMIT_A, 1, COLOR_FWD));
    turretBase.append(
        new MechanismLigament2d("feed limit B", 0.6, TURRET_FEED_LIMIT_B, 1, COLOR_REV));
  }

  public void setIntakeMotors(double intake, double dir) {
    lowerIntake.setColor(intake > 0.0 ? COLOR_FWD : intake < 0.0 ? COLOR_REV : COLOR_NEUTRAL);
    upperIntake.setColor(dir > 0.0 ? COLOR_FWD : dir < 0.0 ? COLOR_REV : COLOR_NEUTRAL);
  }

  public void setElevatorHeight(double height) {
    elevatorBase.setLength(height - ELEVATOR_BASE_TO_ELEVATOR_TOP_M);
  }

  public void setShooterTilt(double angle) {
    shooter.setAngle(90.0 - angle);
  }

  public void setDistance(double x) {
    xref.setLength(x);
  }

  public void setTurretAngle(double angle_deg) {
    turret.setAngle(angle_deg);
  }

  public void setTurretBaseAngle(Rotation2d angle_r2d) {
    turretBase.setAngle(
        G.isRedAlliance() ? angle_r2d : angle_r2d.minus(Rotation2d.fromRotations(0.5)));
  }

  public static void periodic() {
    Logger.recordOutput("SS/Setpoint", S.mech);
    Logger.recordOutput("SS/Measured/", M.mech);
  }
}
