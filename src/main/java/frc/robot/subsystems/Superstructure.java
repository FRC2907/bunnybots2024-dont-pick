package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.constants.Ports;

public class Superstructure implements ISubsystem{
    private Drivetrain drivetrain;
    private Arm arm;
    private ISubsystem[] subsystems;

    private static Superstructure instance;

    private ManipulatorState manipulatorState;
    private TransportState transportState;

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

    public enum ManipulatorState {
        START, NEUTRAL, 
        MOVING_TO_BALLOON, READY_TO_GRAB_BALLOON, MOVING_TO_DROPZONE, IN_DROPZONE, 
        HOLDING_BALLOON, DROPPING_BALLOON,
    }

    public enum TransportState {
        START, NEUTRAL,
        MOVING_TO_TOTE, READY_TO_PICKUP_TOTE, 
        EXTENDED, RETRACTED, HOOKED, 
        HOLDING_TOTE, DROPPING_TOTE
    }

    public ManipulatorState getManipulatorState() { return this.manipulatorState; }
    public TransportState getTransportState() { return this.transportState; }

    public void start()             { this.manipulatorState = ManipulatorState.START; }
    public void neutral()           { this.manipulatorState = ManipulatorState.NEUTRAL; }
    public void movingToBalloon()   { this.manipulatorState = ManipulatorState.MOVING_TO_BALLOON; }
    public void readyToGrabBalloon(){ this.manipulatorState = ManipulatorState.READY_TO_GRAB_BALLOON; }
    public void holdingBalloon()    { this.manipulatorState = ManipulatorState.HOLDING_BALLOON; }
    public void droppingBalloon()   { this.manipulatorState = ManipulatorState.DROPPING_BALLOON; }
    public void movingToTote()      { this.transportState = TransportState.MOVING_TO_TOTE; }
    public void readyToPickupTote() { this.transportState = TransportState.READY_TO_PICKUP_TOTE; }
    public void holdingTote()       { this.transportState = TransportState.HOLDING_TOTE; }
    public void droppingTote()      { this.transportState = TransportState.DROPPING_TOTE; }


    public void handleManipulatorState(){
        switch (this.manipulatorState) {
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
            case MOVING_TO_DROPZONE:
                break;
            case IN_DROPZONE:
                break;
        }
    }

    @Override
    public void onLoop(){
        handleManipulatorState();

        receiveOptions();
        submitTelemetry();
    }
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
