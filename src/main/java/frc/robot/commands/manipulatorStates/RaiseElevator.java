package frc.robot.commands.manipulatorStates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class RaiseElevator extends Command {

    private ManipulatorSubsystem m_manipulator;

    private Timer m_ampTimer;
    private double m_desiredHeight;

    public RaiseElevator(ManipulatorSubsystem manipulator, double desiredHeight) {
        m_manipulator = manipulator;


        addRequirements(manipulator);
        setName("Raise Elevator");
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
        m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteOff);

        m_manipulator.elevatorSetPosition(ManipulatorConstants.kElevatorUpPos);

        m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxOff);

        SmartDashboard.putNumber("amp timer", m_ampTimer.get());
        SmartDashboard.putString("command:", "Raise Elevator");
    
    }

    @Override
    public void end(boolean interrupted) {
        m_ampTimer.restart();
    }

    @Override
    public boolean isFinished() {
//        return m_elevatorUp.getAsBoolean();
        return Math.abs(m_manipulator.getElevatorPosition() - m_desiredHeight) < ManipulatorConstants.kElevatorPosTolerance;
    }
}