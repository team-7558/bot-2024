package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public final class RunAltAutoCommand extends Command {

  private AltAuto aa;
  private boolean generated;
  private SwerveFollower follower;

  private Timer t;

  public RunAltAutoCommand(AltAuto auto) {
    this.aa = auto;
    t = new Timer();
    follower = new SwerveFollower(aa.trajstack);

    addRequirements(Drive.getInstance());
  }

  public void generate() {}

  @Override
  public void initialize() {
    t.reset();
    t.start();

    follower.start();

    aa.init();
  }

  @Override
  public void execute() {
    follower.step(t.get());
    aa.execute();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interuppted) {}
}
