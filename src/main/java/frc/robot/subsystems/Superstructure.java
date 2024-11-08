package frc.robot.subsystems;

public class Superstructure {
    private Drivetrain drivetrain;
    private Arm arm;

    private ISubsystem[] subsystems;

    private static Superstructure instance;

    private RobotState state;

    private Superstructure(){
        this.drivetrain = Drivetrain.getInstance();
        this.arm = Arm.getInstance();

        this.subsystems = new ISubsystem[]{ drivetrain };
    }

    public static Superstructure getInstance(){
        if (instance == null){
            instance = new Superstructure();
        }
        return instance;
    }

    public enum RobotState {
        MOVING_TO_BALLOON, READY_TO_GRAB_BALLOON, HOLDING_BALLOON, DROPPING_BALLOON,
        MOVING_TO_TOTE, READY_TO_PICKUP_TOTE, HOLDING_TOTE, DROPPING_TOTE 
    }

    public void handleStates(){
        switch (this.state) {
            case MOVING_TO_BALLOON:
                arm.setSetPoint(30);
            case READY_TO_GRAB_BALLOON:
            case HOLDING_BALLOON:
            case DROPPING_BALLOON:
            case MOVING_TO_TOTE:
            case READY_TO_PICKUP_TOTE:
            case HOLDING_TOTE:
            case DROPPING_TOTE:
        }
    }
}
