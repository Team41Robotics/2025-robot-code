package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ArmIO{


        public static void setWristVoltage(double voltage) {};

        public static void setShoulderVoltage(double leftVoltage, double rightVoltage){}

        public static void setTargetExtension(double target) {};

        public static void setTargetShoulderAngle(Rotation2d target) {};

        public static void setTargetWristAngle(Rotation2d target) {};

        public static void updateInputs(ArmInputs inputs){}
        

        @AutoLog
        public static class ArmInputs{

                public Rotation2d shoulderRotation = new Rotation2d(); 
                public double shoulderPivotVoltage = 0.0;
                public double[] shoulderPivotCurrentAmps = new double[] {};
                public double shoulderAngVel = 0.0; // rads / s

                public double telescopePosition = 0.0; // m
                public double telescopeVelocity = 0.0; // m/s
                public double telescopeVoltage = 0.0;
                public double[] telescopeCurrent = new double[] {};

                public Rotation2d wristRotation = new Rotation2d();
                public double wristPivotVoltage = 0.0; 
                public double[] wristPivotCurrent = new double[] {}; // amps
                public double wristAngVel = 0.0; // rad / s

                public boolean shoulderTooFast;

        }

}
