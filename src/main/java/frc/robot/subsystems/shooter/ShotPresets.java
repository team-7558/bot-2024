package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class ShotPresets {
  public static final Setpoints RED_BUDGET_AMP = new Setpoints(13.5, 0, 0, 0.12);
  public static final Setpoints RED_FENDER = new Setpoints(30, 0, 0, 0.145);
  public static final Setpoints RED_FRONT_POST = new Setpoints(40, 0, 0.09, 0.105);
  public static final Setpoints RED_SIDE_POST = new Setpoints(40, 0, 0.09, 0.105);
  public static final Setpoints RED_AMP_BOX = new Setpoints(40, 0, -0.134, 0.102);
  public static final Setpoints RED_FRONT_COURT = new Setpoints(40, 0, 0, 0.1);
  public static final Setpoints RED_WING_POST =
      new Setpoints(45, 0, -0.023, Units.degreesToRotations(40));
  public static final Setpoints RED_WING_WALL = new Setpoints(45, 0, -0.080, 0.078);
  public static final Setpoints RED_CLEAR_WALL = new Setpoints(28, 0, 0.075, 0.135);
  public static final Setpoints RED_CLEAR_MID = new Setpoints(28, 0, 0.0, 0.135);
  public static final Setpoints RED_CLEAR_CLOSE =
      new Setpoints(32, 0, 0.0, Shooter.PIVOT_MIN_POS_r);

  public static final Setpoints BLUE_BUDGET_AMP = new Setpoints().copyAndFlipTurret(RED_BUDGET_AMP);
  public static final Setpoints BLUE_FENDER = new Setpoints().copyAndFlipTurret(RED_FENDER);
  public static final Setpoints BLUE_FRONT_POST = new Setpoints().copyAndFlipTurret(RED_FRONT_POST);
  public static final Setpoints BLUE_SIDE_POST = new Setpoints().copyAndFlipTurret(RED_SIDE_POST);
  public static final Setpoints BLUE_AMP_BOX = new Setpoints().copyAndFlipTurret(RED_AMP_BOX);
  public static final Setpoints BLUE_FRONT_COURT =
      new Setpoints().copyAndFlipTurret(RED_FRONT_COURT);
  public static final Setpoints BLUE_WING_POST = new Setpoints().copyAndFlipTurret(RED_WING_POST);
  public static final Setpoints BLUE_WING_WALL = new Setpoints().copyAndFlipTurret(RED_WING_WALL);
  public static final Setpoints BLUE_CLEAR_WALL = new Setpoints().copyAndFlipTurret(RED_CLEAR_WALL);
  public static final Setpoints BLUE_CLEAR_MID = new Setpoints().copyAndFlipTurret(RED_CLEAR_MID);
  public static final Setpoints BLUE_CLEAR_CLOSE =
      new Setpoints().copyAndFlipTurret(RED_CLEAR_CLOSE);
}
