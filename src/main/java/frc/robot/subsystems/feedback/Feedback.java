
package frc.robot.subsystems.feedback;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;

public class Feedback extends SubsystemBase {
  public Feedback() {}

  @Override
  public void periodic() {
    /**
     * if(RobotContainer.intake.pieceDetectedRecently()) {
     *  OI.DR.setRumble(RumbleType.kLeftRumble,1);
     * } else {
     *  OI.DR.setRumble(RumbleType.kLeftRumble,0);
     * }
     */

    if(DriverStation.getMatchTime() <= 30 && DriverStation.getMatchTime() > 28) {
      OI.DR.setRumble(RumbleType.kBothRumble, 1);
    } else {
      OI.DR.setRumble(RumbleType.kBothRumble, 0);
    }

  }
}
