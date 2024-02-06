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



  public static final double CENTER_TO_SHOOTER = 0.1;
  public static final double CENTER_TO_INTAKE = 0.611 * 0.5;

  public static final double GROUND_TO_ELEVATOR_BASE_M = 0.594;
  public static final double ELEVATOR_BASE_TO_ELEVATOR_TOP_M = 0.586;

  public static final double GROUND_TO_SHOOTER_PIVOT = 0.28;
  public static final double SHOOTER_PIVOT_TO_TIP = 0.38;

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

  static{
    S = new SS2d();
    M = new SS2d();

  }

  private final Mechanism2d side;
  private final Mechanism2d top;
  private final MechanismLigament2d yref;
  private final MechanismLigament2d xref;
  private final MechanismLigament2d intakeBase;
  private final MechanismLigament2d shooterBase;
  private final MechanismLigament2d lowerIntake;
  private final MechanismLigament2d elevatorBase;
  private final MechanismLigament2d upperIntake;
  private final MechanismLigament2d shooter;
  private final MechanismLigament2d turretBase;
  private final MechanismLigament2d turret;

  private SS2d(){
    side = new Mechanism2d(SCENE_WIDTH_M, SCENE_HEIGHT_M, COLOR_BG);
    top = new Mechanism2d(SCENE_WIDTH_M, SCENE_HEIGHT_M, COLOR_BG);

    MechanismRoot2d side_root = side.getRoot("scene", 0, 0);

    yref = side_root.append(new MechanismLigament2d("yref", 0.5, 90, 1.0, COLOR_REF));
    xref = yref.append(new MechanismLigament2d("xref", SCENE_WIDTH_M * 0.5, -90, 1.0, COLOR_REF));
    
    MechanismLigament2d target = yref.append(new MechanismLigament2d("target height", TARGET_HEIGHT_FROM_GROUND_M, 0, 1, COLOR_REF));
    target.append(new MechanismLigament2d("target", TARGET_HEIGHT*Math.cos(Units.degreesToRadians(TARGET_SLOPE)), -TARGET_SLOPE, 1, COLOR_NEUTRAL));


    intakeBase =
        xref.append(
            new MechanismLigament2d("driveintake", CENTER_TO_INTAKE, 0.0, 1.0, COLOR_FRAME));
    shooterBase =
        xref.append(
            new MechanismLigament2d("driveshooter", CENTER_TO_SHOOTER, 0.0, 1.0, COLOR_FRAME));

    MechanismLigament2d shooterHeight =
        shooterBase.append(
            new MechanismLigament2d("shooter height", GROUND_TO_SHOOTER_PIVOT, 90, 1, COLOR_REF));

    shooter =
        shooterHeight.append(
            new MechanismLigament2d("shooter", SHOOTER_PIVOT_TO_TIP, 90, 1, COLOR_NEUTRAL));

    lowerIntake =
        intakeBase.append(
            new MechanismLigament2d(
                "lower intake", GROUND_TO_ELEVATOR_BASE_M, 90, 4, COLOR_NEUTRAL));

    elevatorBase =
        intakeBase.append(
            new MechanismLigament2d("elevator base", GROUND_TO_ELEVATOR_BASE_M, 90, 1, COLOR_REF));

    upperIntake =
        elevatorBase.append(
            new MechanismLigament2d(
                "upper intake", ELEVATOR_BASE_TO_ELEVATOR_TOP_M, 0, 2, COLOR_NEUTRAL));

    MechanismRoot2d top_root = top.getRoot("scene", SCENE_WIDTH_M * 0.5, SCENE_HEIGHT_M * 0.75);
    turretBase = top_root.append(new MechanismLigament2d("turret base", 0.01, -90, 1, COLOR_REF));
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

  public void setTurretAngle(double angle_deg){
    turret.setAngle(angle_deg);
  }

  public void setTurretBaseAngle(Rotation2d angle_r2d){
    turretBase.setAngle(angle_r2d.minus(Rotation2d.fromRotations(0.25)));
  }

  public static void periodic() {
    Logger.recordOutput("SS/Setpoint/TopProfile", S.top);    
    Logger.recordOutput("SS/Setpoint/SideProfile", S.side);

    Logger.recordOutput("SS/Measured/TopProfile", M.top);
    Logger.recordOutput("SS/Measured/SideProfile", M.side);

  }
}
