package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public class Control {
    public static final double kDriverDeadband = 0.08;

    public class arm {
        public static final double GEAR_RATIO = 0; //TODO find
        public static final double ENCODER_POS_UNIT_PER_DEGREE = 0; //TODO find (rotation/degree)
        public static final double ENCODER_VEL_UNIT_PER_DEGREE_PER_SECOND = 0; //TODO find (rpm/deg/s)
        
        public static final Translation2d ARM_POSITION = new Translation2d(12.865, 23.663); //TODO check (inches)

        public static final double kMaxExtension = 0; //TODO find
        public static final double kMaxAngle = 0; //TODO find
        public static final double kMinAngle = 0; //TODO find
        public static final double kDropZone = 0; //TODO find
        public static final double kPickup = 0; //TODO find
        
        public static final double kP = 0.001;
        public static final double kD = 0.001;
        public static final double kFF = 0.001;
    }

    public class drivetrain {
        public static final double GEAR_RATIO = 0; //TODO find
        public static final double WHEEL_DIAMETER = 6; //inches

        public static final double kVelocityConversionFactor = 0; //TODO find
        public static final double kMaxVel = 0; //TODO find
        
        public static final double kP = 0.001;
        public static final double kD = 0.001;
        public static final double kFF = 0.001;
    }
}
