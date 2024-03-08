package frc.robot.commands.manipulatorStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class BringElevatorDown extends Command {
    private ManipulatorSubsystem m_manipulator;

    public BringElevatorDown(ManipulatorSubsystem manipulator) {
        m_manipulator = manipulator;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
        m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteOff);

        m_manipulator.elevatorMotorSpeed(ManipulatorConstants.kElevatorDown);
        m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxOff);

        SmartDashboard.putString("command:", "Lower Elevator");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_manipulator.getElevatorPosition() < 0.01;
    }
}
