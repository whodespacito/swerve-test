package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class intakeActiveTest extends Command {

    private final DriveSubsystem m_DriveSubsystem;
    private final double m_speed;
    private final BooleanSupplier m_condition;

    public intakeActiveTest(DriveSubsystem driveSubsystem, double speed, BooleanSupplier condition) {
        m_DriveSubsystem = driveSubsystem;
        m_speed = speed;
        m_condition = condition;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SwerveModuleState[] state = {new SwerveModuleState(0,new Rotation2d(0)),
                            new SwerveModuleState(0,new Rotation2d(0)),
                            new SwerveModuleState(m_speed, new Rotation2d(0)),
                            new SwerveModuleState(0,new Rotation2d(0))
                            };
        m_DriveSubsystem.setModuleStates(state);
    }

    @Override
    public boolean isFinished() {
        return m_condition.getAsBoolean();
    }
}
