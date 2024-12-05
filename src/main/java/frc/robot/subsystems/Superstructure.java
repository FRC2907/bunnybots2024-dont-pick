package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.constants.Control;
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
        //this.arm = Arm.getInstance();

        this.subsystems = new ISubsystem[]{ drivetrain };

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
        START, NEUTRAL, MANUAL_CONTROL,
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

    public void manipulatorStart()         { this.manipulatorState = ManipulatorState.START; }
    public void manipulatorNeutral()       { this.manipulatorState = ManipulatorState.NEUTRAL; }
    public void movingToBalloon()          { this.manipulatorState = ManipulatorState.MOVING_TO_BALLOON; }
    public void readyToGrabBalloon()       { this.manipulatorState = ManipulatorState.READY_TO_GRAB_BALLOON; }
    public void movingToDropzone()         { this.manipulatorState = ManipulatorState.MOVING_TO_DROPZONE; }
    public void inDropzone()               { this.manipulatorState = ManipulatorState.IN_DROPZONE; }
    public void holdingBalloon()           { this.manipulatorState = ManipulatorState.HOLDING_BALLOON; }
    public void droppingBalloon()          { this.manipulatorState = ManipulatorState.DROPPING_BALLOON; }
    public void transportStart()           { this.transportState = TransportState.START; }
    public void transportNeutral()         { this.transportState = TransportState.NEUTRAL; }
    public void movingToTote()             { this.transportState = TransportState.MOVING_TO_TOTE; }
    public void readyToPickupTote()        { this.transportState = TransportState.READY_TO_PICKUP_TOTE; }
    public void extended()                 { this.transportState = TransportState.EXTENDED; }
    public void retracted()                { this.transportState = TransportState.RETRACTED; }
    public void hooked()                   { this.transportState = TransportState.HOOKED; }
    public void holdingTote()              { this.transportState = TransportState.HOLDING_TOTE; }
    public void droppingTote()             { this.transportState = TransportState.DROPPING_TOTE; }


    private void handleManipulatorState(){
        switch (this.manipulatorState) {
            case START:
                arm.start();
                break;
            case NEUTRAL:
                arm.neutral();
                break;
            case MANUAL_CONTROL:
                arm.manualMove(driver.getRightY());
                break;
            case MOVING_TO_BALLOON:
                arm.pickup();
                break;
            case READY_TO_GRAB_BALLOON:
                arm.pickup();
                break;
            case HOLDING_BALLOON:
                break;
            case DROPPING_BALLOON:
                arm.drop();
                break;
            case MOVING_TO_DROPZONE:
                arm.drop();
                break;
            case IN_DROPZONE:
                arm.drop();
                break;
        }
    }

    private void handleTransportState(){
        switch (this.transportState) {
            case START:
                break;
            case NEUTRAL:
                break;
            case MOVING_TO_TOTE:
                break;
            case READY_TO_PICKUP_TOTE:
                break;
            case EXTENDED:
                break;
            case RETRACTED:
                break;
            case HOOKED:
                break;
            case HOLDING_TOTE:
                break;
            case DROPPING_TOTE:
                break;
        }
    }

    private void drive(){
        drivetrain.setLocalDriveInputs(driver.getLeftY()  * Control.drivetrain.kMaxVel,
                                       driver.getLeftX()  * Control.drivetrain.kMaxVel, 
                                       driver.getRightX() * Control.drivetrain.kMaxVel); 
        //TODO change to m/s
        //TODO add drive states
    }

    @Override
    public void onLoop(){
        //handleManipulatorState();
        //handleTransportState();
        //drive();

        for (ISubsystem s : subsystems){ s.onLoop(); }
        
        receiveOptions();
        submitTelemetry();
    }
    @Override
    public void submitTelemetry(){}
    @Override
    public void receiveOptions(){}
}
