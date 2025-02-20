package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;

public class ArmIOHardware implements ArmIO{
        private TalonFX shoulder1;
        private TalonFX shoulder2;
        private TalonFX shoulder3;
        private TalonFX shoulder4;

        private TalonFX telescope1;
        private TalonFX telescope2;

        private SparkFlex wrist;
        
        public ArmIOHardware(){

                shoulder1 = new TalonFX(0);
                shoulder2 = new TalonFX(0);
                shoulder3 = new TalonFX(0);
                shoulder4 = new TalonFX(0);

                TalonFXConfigurator s1Configurator = shoulder1.getConfigurator();
                TalonFXConfigurator s2Configurator = shoulder2.getConfigurator();
                TalonFXConfigurator s3Configurator = shoulder3.getConfigurator();
                TalonFXConfigurator s4Configurator = shoulder4.getConfigurator();

                telescope1 = new TalonFX(0);
                telescope2 = new TalonFX(0);

                TalonFXConfigurator t1Configurator = telescope1.getConfigurator();
                TalonFXConfigurator t2Configurator = telescope2.getConfigurator();


                wrist = new SparkFlex(0, null);

                TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
                shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                shoulderConfig.CurrentLimits.SupplyCurrentLimit = 25;
                shoulderConfig.Slot0.kP = 0.0;
                shoulderConfig.Slot0.kI = 0.0;
                shoulderConfig.Slot0.kD = 0.0;
                
                s1Configurator.apply(shoulderConfig);
                s2Configurator.apply(shoulderConfig);
                s3Configurator.apply(shoulderConfig);
                s4Configurator.apply(shoulderConfig);

                shoulder2.setControl(new Follower(shoulder1.getDeviceID(), false));
                shoulder3.setControl(new Follower(shoulder1.getDeviceID(), false));
                shoulder4.setControl(new Follower(shoulder1.getDeviceID(), false));

                TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
                telescopeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                telescopeConfig.CurrentLimits.StatorCurrentLimit = 30;
                telescopeConfig.Slot0.kP = 0.0;
                telescopeConfig.Slot0.kI = 0.0;
                telescopeConfig.Slot0.kD = 0.0;

                t1Configurator.apply(telescopeConfig);
                t2Configurator.apply(telescopeConfig);

        }

}
