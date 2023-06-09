package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.util.Color8Bit;

public final class Constants {

    public static final class ActiveVariables {
        public static boolean isCone = true;
        public static boolean clawRunning = false;
        public static boolean isRed = false;
        public static boolean AprilTag = false;

        //0 = moving
        //1 = low scoring
        //2 = mid scoring
        //3 = high scoring
        //4 = intake
    }

    public static final class Swerve {
        private Swerve(){}

        public static final double M1Offset = 125.85;
        public static final double M2Offset = 236.25;
        public static final double M3Offset = 10.37;
        public static final double M4Offset = 50.267;

        public static final double DriverEncoderRatio  = (1 / 2048.) * (1 / 6.55) * (0.1016 * Math.PI); //4 inch to 0.1016 meters
        public static final double RotatorEncoderRatio = (1 / 2048.) * (1 / 10.29) * (2 * Math.PI);

    }

    public static final class IDs {
        private IDs(){}

        public static final int Swerve1Drive = 1;
        public static final int Swerve1Rot   = 2;
        public static final int Swerve2Drive = 3;
        public static final int Swerve2Rot   = 4;
        public static final int Swerve3Drive = 5;
        public static final int Swerve3Rot   = 6;
        public static final int Swerve4Drive = 7;
        public static final int Swerve4Rot   = 8;

        public static final int Swerve1Enc = 9;
        public static final int Swerve2Enc = 10;
        public static final int Swerve3Enc = 11;
        public static final int Swerve4Enc = 12;

        public static final int ArmPivotL = 13;
        public static final int ArmPivotR = 14;
        public static final int ElevatorMotorL = 15;
        public static final int ElevatorMotorR = 16;
        public static final int WristMotor = 18;
        public static final int PneumaticHub = 19;
        public static final int ClawL = 20;
        public static final int ClawR = 21;

        public static final int ClawPiston = 0;
        public static final int ArmPiston = 15;
    }

    public static final class SetPoints {
        //intake:
        //platform:
        public static final double PIWrist = Math.PI * 16 / 32;//23/32
        public static final double PIArm = Math.PI * .23;
        public static final double PIElevator = 0;
        public static final boolean PIExtension = false;
        //floor:
        public static final double FIWrist = 0.28;
        public static final double FIArm = Math.PI * 0.7;
        public static final double FIElevator = 27;
        public static final boolean FIExtension = false;

        //movement:
        public static final double MWrist = Math.PI * 0.22;
        public static final double MArm = 0;
        public static final double MElevator = 0;
        public static final boolean MExtension = false;

        //scoring:
        //ground:
        public static final double LSWrist = 0.5;
        public static final double LSArm = Math.PI * 2 / 3;
        public static final double LSElevator = 50;
        public static final boolean LSExtension = false;
        //mid node:
        public static final double MSWrist = Math.PI * 8 / 16;
        public static final double MSArm = Math.PI / 4;
        public static final double MSElevator = 0;
        public static final boolean MSExtension = false;
        //high node:
        public static final double HSWrist = Math.PI * 0.34;
        public static final double HSArm = Math.PI * 0.33;//1
        public static final double HSElevator = 60;//55
        public static final boolean HSExtension = true;
    }

    public static class RobotInfo {
        public static final double MaxRobotSpeed = 2.3;//change
        public static final double MAX_ROBOT_ROTATION_SPEED = Math.PI / 2;//change

        public static final double TrackWidth = 0.25;
        public static final double WheelBase = 0.25;
        public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, -TrackWidth / 2),
                new Translation2d(WheelBase / 2, TrackWidth / 2),
                new Translation2d(-WheelBase / 2, -TrackWidth / 2),
                new Translation2d(-WheelBase / 2, TrackWidth / 2));

        public static class Auton {
            public static final double maxSpeed = 1;//change
            public static final double maxAcceleration = 1; // change

            public static final double maxRotSpeed = Math.PI; //change
            public static final double maxRotAccel = Math.PI + 1; //change
        }
    }

    public static class LED {
        public static int LEDStatus = 0;
        // 0 = Blue and orange stirps, disabled status
        // 1 = Green, has object
        // 2 = Purple, need Cube
        // 3 = Yellow, need Cone
        // 4 = Red, does not have object
        public static final Color8Bit Blue = new Color8Bit(3, 23, 252);
        public static final int BlueR = 3;
        public static final int BlueG = 23;
        public static final int BlueB = 252;

        public static final Color8Bit Orange = new Color8Bit(252, 61, 3);
        public static final int OrangeR = 252;
        public static final int OrangeG = 61;
        public static final int OrangeB = 3;

        public static final Color8Bit Purple = new Color8Bit(252, 6, 244);
        public static final int PurpleR = 252;
        public static final int PurpleG = 6;
        public static final int PurpleB = 244;

        public static final Color8Bit Yellow = new Color8Bit(255, 255, 255);
        public static final int YellowR = 255;
        public static final int YellowG = 255;
        public static final int YellowB = 0;

        public static final Color8Bit Green = new Color8Bit(19, 252, 3);
        public static final int GreenR = 19;
        public static final int GreenG = 252;
        public static final int GreenB = 3;

        public static final Color8Bit Red = new Color8Bit(252, 3, 3);
        public static final int RedR = 252;
        public static final int RedG = 3;
        public static final int RedB = 3;
    }

    public static class ControllerConstants {
        public static final double DEADZONE_VALUE = 0.01;
        public static final int NUMBER_OF_CONTROLLERS = 2;

        public enum Axes {
            LEFT_STICK_X(0), LEFT_STICK_Y(1),
            LEFT_TRIGGER(2), RIGHT_TRIGGER(3),
            RIGHT_STICK_X(4), RIGHT_STICK_Y(5);

            private final int value;

            Axes(int value) {
                this.value = value;
            }

            public int getValue() {
                return value;
            }
        }

        public enum Buttons {
            A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
                    7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

            private final int value;

            private Buttons(int value) {
                this.value = value;
            }

            public int getValue() {
                return value;
            }
        }
    }
}
