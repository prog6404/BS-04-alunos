/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {

    // MOTOR VELOCITY
    public static final double SHOOTER_ANGLE_SPEED = 0.5;
    public static final double CLIMB_SPEED = 1;
    public static final double TELESCOPIC_SPEED_RAISE = 0.9;
    public static final double TELESCOPIC_SPEED_LOWER = 0.2;
    public static final double SHOOTING_SPEED = 1;
    public static final double INTAKE_SPEED = .8;
    public static final double SHOOTER_BELT_SPEED = 1;
    public static final double STORAGE_BELT_SPEED = -0.4;

    //PID
    public static final double DRIVE_kP = 0.03, DRIVE_kI = 0.00003, DRIVE_kD = 0.006;

    public static final double SHOOTER_kP = 0, SHOOTER_kI=0, SHOOTER_kD = 0;

    // PIXY CAM
    public static final double STORAGE_MIN_PIXY_AREA = 100;
    
    public static class Ports{
        public static class Sensors {
            
            public static final int CLIMB_I2C_COLOR = 0;            
            public static final int INTAKE_OPTIC = 8;
            public static final int SHOOTER_OPTIC = 4;
            public static final int STORAGE_OPTIC = 9; 
            public static final int SHOOTER_LIMIT_HIGH = 7;
            public static final int SHOOTER_LIMIT_LOW = 6;
            public static final int DRIVE_ENC_LEFT_A = 0;
            public static final int DRIVE_ENC_LEFT_B = 1;
            public static final int DRIVE_ENC_RIGHT_A = 2;
            public static final int DRIVE_ENC_RIGHT_B = 3;
            public static final int PDP_PORT = 20;

        }

        public static class Motors{
            
            // PWM
            public static final int DRIVE_LEFT_FRONT = 3;
            public static final int DRIVE_LEFT_BACK = 2;
            public static final int DRIVE_RIGHT_FRONT = 1;
            public static final int DRIVE_RIGHT_BACK = 0;

            public static final int SHOOTER_BELT = 9; 

            //CAN
            public static final int INTAKE_ANGLE = 4; 
            public static final int INTAKE_COLLECTOR = 1;//6; 

            public static final int STORAGE_BELT = 10; 
            public static final int SHOOTER_SHOOT = 11;

            public static final int SHOOTER_ANGLE = 4;
            
            public static final int CLIMB_TELESCOPIC = 3;
            public static final int CLIMB_BACK = 2;
            public static final int CLIMB_FRONT = 1;

        }
    }
    
    public static class OI_Map {

        public static final int PILOT = 0;
        public static final int COPILOT = 1; 

        public static final int BUTTON_A = 1; 
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int BUTTON_LEFT = 5;
        public static final int BUTTON_RIGHT = 6;
        
    }

    public static class Sensors_Values {
        public static final boolean ENC_RIGHT_REVERSE = false;
        public static final boolean ENC_LEFT_REVERSE = true;
        public static final double ENC_DISTANCE_PER_PULSE = Math.PI * 2.54 / 6000; //Math.PI * 4 *2.54 / 360
    }
    
    public static class Autonomous{
        
        // DISTANCE TO SHOT
        public static final double MIN_DIST_TO_SHOOT = 0, MAX_DIST_TO_SHOOT = 0;
        
        // Constantes provindas do FRC-Characterization
        // Não foram definidas ainda.
        public static final double ksVolts = 1.85;//2.28;//0.833;
        public static final double kvVoltSecondsPerMeter = 2.82;//2.74;//3.19;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0239;//0.0164;//0.294;
        public static final double kPDriveVel = 0.0204;//0.204;//0.00804;//11;
        public static final double kTrackwidthMeter = 1.37342683;//0.65;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeter);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        // WPILIB recomenda esses 2 valores para o Ramsete
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

}
