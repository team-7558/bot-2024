package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.Shooter.Setpoints;

public class ShotPresets {

  public static final Setpoints TRAP_SHOT = new Setpoints(16, 0, 0, 0.14);

  public static final Setpoints RED_BUDGET_AMP = new Setpoints(10.5, 0, 0, 0.14);
  public static final Setpoints RED_FENDER = new Setpoints(29, 0, 0, 0.14);
  public static final Setpoints RED_FRONT_POST =
      new Setpoints(38, 0, 0.087, 0.085); // potentially make higher
  public static final Setpoints RED_SIDE_POST = new Setpoints(45, 0, 0.092, 0.080);
  public static final Setpoints RED_AMP_BOX = new Setpoints(38, 0, -0.134, 0.11);
  public static final Setpoints RED_FRONT_COURT = new Setpoints(36, 0, 0, 0.103);
  public static final Setpoints RED_WING_POST = new Setpoints(49, 0, -0.02, 0.068);
  public static final Setpoints OP_SHOT = new Setpoints(43, 0, 0.02, 0.076);
  public static final Setpoints RED_CLEAR_WALL = new Setpoints(25, 0, 0.08, 0.1);
  public static final Setpoints SNIPE = new Setpoints(49, 0, 0.095, 0.054);
  public static final Setpoints RED_CLEAR_CLOSE =
      new Setpoints(36, 0, 0.1, Shooter.PIVOT_MIN_POS_r);
  public static final Setpoints CLEAR_FLAT = new Setpoints(49, 0, 0, 0.04);

  public static final Setpoints BLUE_BUDGET_AMP = new Setpoints().copyAndFlipTurret(RED_BUDGET_AMP);
  public static final Setpoints BLUE_FENDER = new Setpoints().copyAndFlipTurret(RED_FENDER);
  public static final Setpoints BLUE_FRONT_POST = new Setpoints().copyAndFlipTurret(RED_FRONT_POST);
  public static final Setpoints BLUE_SIDE_POST = new Setpoints().copyAndFlipTurret(RED_SIDE_POST);
  public static final Setpoints BLUE_AMP_BOX = new Setpoints().copyAndFlipTurret(RED_AMP_BOX);
  public static final Setpoints BLUE_FRONT_COURT =
      new Setpoints().copyAndFlipTurret(RED_FRONT_COURT);
  public static final Setpoints BLUE_WING_POST = new Setpoints().copyAndFlipTurret(RED_WING_POST);
  public static final Setpoints BLUE_SNIPE = new Setpoints().copyAndFlipTurret(SNIPE);
  public static final Setpoints BLUE_CLEAR_WALL = new Setpoints().copyAndFlipTurret(RED_CLEAR_WALL);
  public static final Setpoints BLUE_OP_SHOT = new Setpoints().copyAndFlipTurret(OP_SHOT);
  public static final Setpoints BLUE_CLEAR_CLOSE =
      new Setpoints().copyAndFlipTurret(RED_CLEAR_CLOSE);
  public static final Setpoints BLUE_CLEAR_FLAT = new Setpoints().copyAndFlipTurret(CLEAR_FLAT);
}
