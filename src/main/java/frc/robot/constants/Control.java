package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public class Control {
    public static final double kDriverDeadband = 0.08;

    public class arm {
        public static final double ANGLE_GEAR_RATIO = 0; //TODO find
        public static final double ENCODER_POS_UNIT_PER_DEGREE = 0; //TODO find (rotation/degree)
        public static final double ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND = 0; //TODO find (rpm/deg/s)
        public static final double EXTEND_GEAR_RATIO = 0; //TODO find
        public static final double ENCODER_POS_UNIT_PER_INCH = 0; //TODO FIND
        public static final double ENCODER_VEL_UNIT_PER_INCH_PER_SECOND = 0; //TODO FIND

        public static final double MAX_ARM_EXTENSION = 30; //TODO check (in.)
        public static final double MIN_ARM_EXTENSION = 10.75; //TODO check (in.)
        public static final double MAX_ARM_EXTENSION_SIDE = 16; //TODO check/revise (in.)
        public static final double MAX_ARM_EXTENSION_TOP = 10.5; //TODO check/revise (in.)
        public static final Translation2d ARM_POSITION = new Translation2d(12.865, 23.663); //TODO check (inches)

        public static final double MAX_ANGLE = 0; //TODO find
        public static final double MIN_ANGLE = 17; //TODO find
        public static final double kStartAngle = 20; //TODO find
        public static final double kNeutralAngle = 90; //TODO find
        public static final double kDropZone = 0; //TODO find
        public static final double kPickup = 0; //TODO find
        public static final double kStartExtension = 0; //TODO find
        public static final double kNeutralExtension = 0; //TODO find
        
        public static final double kAngleP = 0.001;
        public static final double kAngleD = 0.001;
        public static final double kAngleFF = 0.001;

        public static final double kExtendP = 0.001;
        public static final double kExtendD = 0.001;
        public static final double kExtendFF = 0.001;
    }

    public class drivetrain {
        public static final double GEAR_RATIO = 0; //TODO find
        public static final double WHEEL_DIAMETER = 6; //inches
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0,0); //TODO find
        public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(0,0); //TODO find
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0,0); //TODO find
        public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(0,0); //TODO find
        public static final MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics
            (FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, REAR_LEFT_LOCATION, REAR_RIGHT_LOCATION);

        public static final double kVelocityConversionFactor = 0; //TODO find
        public static final double kMaxVel = 0; //TODO find
        
        public static final double kP = 0.001;
        public static final double kD = 0.001;
        public static final double kFF = 0.001;
    }
}
