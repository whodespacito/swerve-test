package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class intakeNeutralTest extends Command {

    private final DriveSubsystem m_DriveSubsystem;
    private final double m_speed;

    public intakeNeutralTest(DriveSubsystem driveSubsystem, double speed) {
        m_DriveSubsystem = driveSubsystem;
        m_speed = speed;
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
        return false;
    }
}
