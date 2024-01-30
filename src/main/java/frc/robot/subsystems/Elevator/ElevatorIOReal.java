package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX rightTalon;
    private final TalonFX leftTalon;
}