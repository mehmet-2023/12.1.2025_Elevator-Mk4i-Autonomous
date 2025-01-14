package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;

public final class Constants {
    public static final class Swerve {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0); // MK4i Standard
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 2;
        public static final double DRIVE_GEAR_RATIO = 8.14; // MK4i Standard
        public static final double STEER_GEAR_RATIO = 12.8; // MK4i Standard
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3.5;

        public static final double FRONT_LEFT_MODULE_DRIVE_OFFSET = 0;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0;
        public static final double FRONT_RIGHT_MODULE_DRIVE_OFFSET = 0;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0;
        public static final double BACK_LEFT_MODULE_DRIVE_OFFSET = 0;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 0;
        public static final double BACK_RIGHT_MODULE_DRIVE_OFFSET = 0;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 0;

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0,0.0,0.0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5.0,0.0,0.0);

        public static final double LIMELIGHT_ALIGN_KP = 0.35;
    }

    public static final class LedSubsystem{
        public static final int LED_PWM_PORT = 0;
        public static final int LED_LENGTH = 60; //Default
        public static final LEDPattern RED_ALLIANCE_COLOR = LEDPattern.solid(Color.kRed);
        public static final LEDPattern BLUE_ALLIANCE_COLOR = LEDPattern.solid(Color.kBlue);
        public static final LEDPattern ELEVATOR_PROCESS_COLOR = LEDPattern.solid(Color.kPurple);
        public static final LEDPattern TARGET_FOCUS_COLOR = LEDPattern.solid(Color.kGreen);
        public static final LEDPattern INTAKE_COLOR = LEDPattern.solid(Color.kYellow);
        public static final LEDPattern BREATHE_COLOR = LEDPattern.solid(Color.kWhite);
    }

    public static final class Elevator{
        //TODO Encoder Values Wİll Be Updated
        public static final double ELEVATOR_START_VALUE = 0; //Generally True
        public static final double ELEVATOR_END_VALUE = 450;
        public static final double ELEVATOR_REEFSCAPE_VALUE = 200;
        public static final double ELEVATOR_L1_VALUE = 250;
        public static final double ELEVATOR_L2_VALUE = 300;
        public static final double ELEVATOR_L3_VALUE = 350;
        public static final int ELEVATOR_LEADER_MOTOR_PORT = 0;
        public static final int ELEVATOR_FOLLOWER_MOTOR_PORT = 1;

    }

    public static final class OI {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static double ELEVATOR_SPEED = 0.5;
        public static boolean IS_TEST = false;
        public static boolean IS_PROCESSING = false;
        public static boolean IS_PID_ENDED = false;
        public static boolean IS_SWERVE_FOCUSED = false;
    }

    public static final class SwervePorts {
        // TODO Offset And Ports Will Be Entered
        // Front Left Module
        public static final int FRONT_LEFT_DRIVE_MOTOR = 0;
        public static final int FRONT_LEFT_STEER_MOTOR = 1;
        public static final int FRONT_LEFT_DRIVE_ENCODER_A = 0; // DIO 0
        public static final int FRONT_LEFT_DRIVE_ENCODER_B = 1; // DIO 1
        public static final int FRONT_LEFT_STEER_ENCODER_A = 2; // DIO 2
        public static final int FRONT_LEFT_STEER_ENCODER_B = 3; // DIO 3

        // Front Right Module
        public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR = 3;
        public static final int FRONT_RIGHT_DRIVE_ENCODER_A = 4; // DIO 4
        public static final int FRONT_RIGHT_DRIVE_ENCODER_B = 5; // DIO 5
        public static final int FRONT_RIGHT_STEER_ENCODER_A = 6; // DIO 6
        public static final int FRONT_RIGHT_STEER_ENCODER_B = 7; // DIO 7

        // Back Left Module
        public static final int BACK_LEFT_DRIVE_MOTOR = 4;
        public static final int BACK_LEFT_STEER_MOTOR = 5;
        public static final int BACK_LEFT_DRIVE_ENCODER_A = 8; // DIO 8
        public static final int BACK_LEFT_DRIVE_ENCODER_B = 9; // DIO 9
        public static final int BACK_LEFT_STEER_ENCODER_A = 10; // DIO 10
        public static final int BACK_LEFT_STEER_ENCODER_B = 11; // DIO 11

        // Back Right Module
        public static final int BACK_RIGHT_DRIVE_MOTOR = 6;
        public static final int BACK_RIGHT_STEER_MOTOR = 7;
        public static final int BACK_RIGHT_DRIVE_ENCODER_A = 12; // DIO 12
        public static final int BACK_RIGHT_DRIVE_ENCODER_B = 13; // DIO 13
        public static final int BACK_RIGHT_STEER_ENCODER_A = 14; // DIO 14
        public static final int BACK_RIGHT_STEER_ENCODER_B = 15; // DIO 15
    }
}
