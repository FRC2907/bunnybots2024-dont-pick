package frc.robot.subsystems;

public class Superstructure {
    private Drivetrain drivetrain;

    private ISubsystem[] subsystems;

    private Superstructure(){
        this.drivetrain = Drivetrain.getInstance();

        this.subsystems = new ISubsystem[]{ drivetrain };
    }
}
