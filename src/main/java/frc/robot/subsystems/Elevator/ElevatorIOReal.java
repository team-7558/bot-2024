package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public class ElevatorIOReal implements ElevatorIO {
  private final TalonFX leftFalcon;
  private final TalonFX rightFalcon;

  private final boolean isRightMotorInverted = false;
  private final boolean isLeftMotorInverted = false;
  private final boolean isAbsEncoderInverted = false;

  public ElevatorIOReal() {
    var leaderConfig = new TalonFXConfiguration();
    var followerConfig = new TalonFXConfiguration();


    leftFalcon = new TalonFX(25);
    rightFalcon = new TalonFX(26);

    leaderConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leaderConfig.Feedback.SensorToMechanismRatio = 1; //Figure out how to scale this to real height
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    //Position control gains
    leaderConfig.Slot0.kP = 0.0;   
    
    //MotionMagic gains
    leaderConfig.Slot1.kV = 0.0;

    //Velocity control
    leaderConfig.Slot2.kV = 0.0;

    leftFalcon.getConfigurator().apply(leaderConfig);

    rightFalcon.setControl(new Follower(25, true).withUpdateFreqHz(50));

  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(ModuleIOInputs inputs) {

  }
}
