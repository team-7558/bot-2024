package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAL extends ElevatorIO.ElevatorIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Pos_m", pos_m);
    table.put("Vel_mps", vel_mps);
    table.put("Volts_V", volts_V);
    table.put("Currents_A", currents_A);
  }

  @Override
  public void fromLog(LogTable table) {
    pos_m = table.get("Pos_m", pos_m);
    vel_mps = table.get("Vel_mps", vel_mps);
    volts_V = table.get("Volts_V", volts_V);
    currents_A = table.get("Currents_A", currents_A);
  }

  public ElevatorIOInputsAL clone() {
    ElevatorIOInputsAL copy = new ElevatorIOInputsAL();
    copy.pos_m = this.pos_m;
    copy.vel_mps = this.vel_mps;
    copy.volts_V = this.volts_V;
    copy.currents_A = this.currents_A.clone();
    return copy;
  }
}
