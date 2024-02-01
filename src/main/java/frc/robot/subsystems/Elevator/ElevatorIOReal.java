package frc.robot.subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX rightTalon;
  private final TalonFX leftTalon;
  private final DutyCycleEncoder absEncoder;

  private boolean isRightMotorInverted = true;
  private final boolean isLeftMotorInverted = false;
  private final boolean isAbsEncoderInverted = false;

  private final volts_V volts_V
  private final vel_radps velocity_V;
  private final pos_rad pos_rad;

  public ElevatorIOReal(int index) {
    var elevatorConfig = new TalonFXConfiguration();

    switch (index) {
      case Elevator.RM:
        rightTalon = new TalonFX(1);
        absEncoder = new DutyCycleEncoder(1);
        break;
      case Elevator.LM:
        leftTalon = new TalonFX(2);
        absEncoder = new DutyCycleEncoder(2);
        break;

      default:
        break;
    }

    absEncoder.setDutyCycleRange(index, index);

    private
  }
}
