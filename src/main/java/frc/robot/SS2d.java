package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class SS2d {

  private static final double SCENE_WIDTH_M = 10;
  private static final double SCENE_HEIGHT_M = 10;

  public static final double CENTER_TO_SHOOTER = 0.1;
  public static final double CENTER_TO_INTAKE = 0.611 * 0.5;

  public static final double GROUND_TO_ELEVATOR_BASE_M = 0.594;
  public static final double ELEVATOR_BASE_TO_ELEVATOR_TOP_M = 0.586;

  public static final double GROUND_TO_SHOOTER_PIVOT = 0.28;
  public static final double SHOOTER_PIVOT_TO_TIP = 0.38;

  private static final Color8Bit COLOR_BG = new Color8Bit(0, 0, 0);
  private static final Color8Bit COLOR_REF = new Color8Bit(10, 10, 10);
  private static final Color8Bit COLOR_FRAME = new Color8Bit(100, 100, 100);
  private static final Color8Bit COLOR_NEUTRAL = new Color8Bit(255, 255, 255);
  private static final Color8Bit COLOR_FWD = new Color8Bit(0, 255, 0);
  private static final Color8Bit COLOR_REV = new Color8Bit(255, 0, 0);

  private static final Mechanism2d ss;
  private static final MechanismLigament2d yref;
  private static final MechanismLigament2d xref;
  private static final MechanismLigament2d intakeBase;
  private static final MechanismLigament2d shooterBase;
  private static final MechanismLigament2d lowerIntake;
  private static final MechanismLigament2d elevatorBase;
  private static final MechanismLigament2d upperIntake;
  private static final MechanismLigament2d shooter;

  static {
    ss = new Mechanism2d(SCENE_WIDTH_M, SCENE_HEIGHT_M, COLOR_BG);

    MechanismRoot2d root = ss.getRoot("scene", 0, 0);
    yref = root.append(new MechanismLigament2d("yref", 0.5, 90, 1.0, COLOR_REF));
    xref = yref.append(new MechanismLigament2d("xref", SCENE_WIDTH_M * 0.5, -90, 1.0, COLOR_REF));
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
  }

  public static void setIntakeMotors(double intake, double dir) {
    lowerIntake.setColor(intake > 0.0 ? COLOR_FWD : intake < 0.0 ? COLOR_REV : COLOR_NEUTRAL);
    upperIntake.setColor(dir > 0.0 ? COLOR_FWD : dir < 0.0 ? COLOR_REV : COLOR_NEUTRAL);
  }

  public static void setElevatorHeight(double height) {
    elevatorBase.setLength(height - ELEVATOR_BASE_TO_ELEVATOR_TOP_M);
  }

  public static void setShooterTilt(double angle) {
    shooter.setAngle(90.0 - angle);
  }

  public static void setDistance(double x) {
    xref.setLength(x);
  }

  public static void periodic() {
    Logger.recordOutput("SS/Setpoint2d", ss);
  }
}
