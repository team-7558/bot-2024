package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOTalonFx implements ShooterIO {
    //these talons are for testing shooter on wood thing
     private final TalonFX talonL = new TalonFX(31);
     private final TalonFX talonR = new TalonFX(32);

        //getting stuff
        private final StatusSignal<Double> LVelocity = talonL.getVelocity();
        private final StatusSignal<Double> LAppliedVolts = talonL.getMotorVoltage();
        private final StatusSignal<Double> LCurrent = talonL.getStatorCurrent();
        
        private final StatusSignal<Double> RVelocity = talonR.getVelocity();
        private final StatusSignal<Double> RAppliedVolts = talonR.getMotorVoltage();
        private final StatusSignal<Double> RCurrent = talonR.getStatorCurrent();



        //sensors??? ethan told me to put but prob dont need (prob do)
       private final DigitalInput bottomSensor = new DigitalInput(0);
        private final DigitalInput topSensor = new DigitalInput(1);
    public ShooterIOTalonFx(){
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 30.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        talonL.getConfigurator().apply(config);
        talonR.getConfigurator().apply(config);

        talonL.setControl(new Follower(talonR.getDeviceID(), false));
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, LVelocity, LAppliedVolts, LCurrent, RVelocity, RAppliedVolts, RCurrent);
        talonL.optimizeBusUtilization();
        talonR.optimizeBusUtilization();
        
    }

    @Override
    public void flywheelConfigurePID(double kP, double kI, double kD) {
       var config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        talonL.getConfigurator().apply(config);
        
    }

    @Override
    public void hoodConfigurePID(double kP, double kI, double kD) {
        
    }

    @Override
    public void setFlywheelVelocity(double velocityRadPerSec, double ffVolts) {
         double ACCLERATION = 2.0;
     talonL.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec), ACCLERATION, true, ffVolts, 0, false, false, false));

    }

    @Override
    public void setFlywheelVoltage(double volts) {
        talonL.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurretAngle() {
   
    }

    @Override
    public void stop() {
        talonL.stopMotor();
        
    }

    @Override
    public void turretConfigurePID(double kP, double kI, double kD) {
     
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      
    }
    
}
