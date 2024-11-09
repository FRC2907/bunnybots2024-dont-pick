package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.constants.Ports;

public class Superstructure implements ISubsystem{
    private Drivetrain drivetrain;
    private Arm arm;
    private ISubsystem[] subsystems;

    private static Superstructure instance;

    private RobotState state;

    private PS5Controller driver;
    private PS5Controller operator;

    private Superstructure(){
        this.drivetrain = Drivetrain.getInstance();
        this.arm = Arm.getInstance();

        this.subsystems = new ISubsystem[]{ arm, drivetrain };

        this.driver = new PS5Controller(Ports.HID.DRIVER);
        this.operator = new PS5Controller(Ports.HID.OPERATOR);
    }

    public static Superstructure getInstance(){
        if (instance == null){
            instance = new Superstructure();
        }
        return instance;
    }

    public enum RobotState {
        START, NEUTRAL, 
        MOVING_TO_BALLOON, READY_TO_GRAB_BALLOON, HOLDING_BALLOON, DROPPING_BALLOON,
        MOVING_TO_TOTE, READY_TO_PICKUP_TOTE, HOLDING_TOTE, DROPPING_TOTE 
    }

    public RobotState getState() { return this.state; }

    public void start()             { this.state = RobotState.START; }
    public void neutral()           { this.state = RobotState.NEUTRAL; }
    public void movingToBalloon()   { this.state = RobotState.MOVING_TO_BALLOON; }
    public void readyToGrabBalloon(){ this.state = RobotState.READY_TO_GRAB_BALLOON; }
    public void holdingBalloon()    { this.state = RobotState.HOLDING_BALLOON; }
    public void droppingBalloon()   { this.state = RobotState.DROPPING_BALLOON; }
    public void movingToTote()      { this.state = RobotState.MOVING_TO_TOTE; }
    public void readyToPickupTote() { this.state = RobotState.READY_TO_PICKUP_TOTE; }
    public void holdingTote()       { this.state = RobotState.HOLDING_TOTE; }
    public void droppingTote()      { this.state = RobotState.DROPPING_TOTE; }


    public void handleStates(){
        switch (this.state) {
            case START:
                break;
            case NEUTRAL:
                break;

            
            case MOVING_TO_BALLOON:
                arm.setSetPoint(30);
                break;
            case READY_TO_GRAB_BALLOON:
                break;
            case HOLDING_BALLOON:
                break;
            case DROPPING_BALLOON:
                break;

            
            case MOVING_TO_TOTE:
                break;
            case READY_TO_PICKUP_TOTE:
                break;
            case HOLDING_TOTE:
                break;
            case DROPPING_TOTE:
                break;
        }
    }

    @Override
    public void onLoop(){
        handleStates();

        receiveOptions();
        submitTelemetry();
    }
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
