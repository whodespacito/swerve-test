package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

//George's comment #3

public class elevatorTest extends Command {

    private final DriveSubsystem driveSubsystem;
    private final BooleanSupplier button;
    private final boolean inverse; //instead of stopping when true, stop on false

    public elevatorTest(DriveSubsystem driveSubsystem, BooleanSupplier button, boolean inverse) {
        this.driveSubsystem = driveSubsystem;
        this.button = button;
        this.inverse = inverse;
    }
    
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        SwerveModuleState[] state = {new SwerveModuleState(0,new Rotation2d(0)),
                            new SwerveModuleState(0,new Rotation2d(0)),
                            new SwerveModuleState(0,new Rotation2d(0)),
                            new SwerveModuleState(2,new Rotation2d(0))
                            };
        
        driveSubsystem.setModuleStates(state);
    }

    @Override
    public boolean isFinished() {
        if (inverse) {
            return !button.getAsBoolean();
        } else {
            return button.getAsBoolean();
        }
    }
}
