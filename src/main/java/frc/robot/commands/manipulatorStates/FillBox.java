package frc.robot.commands.manipulatorStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class FillBox extends Command {
    private ManipulatorSubsystem m_manipulator;

    //also known as box full on the paper
    private BooleanSupplier m_outletTrailing = () -> m_manipulator.outletSensorDetect(true) && !m_manipulator.outletSensorDetect(false);

    private int m_observedEdge = 0;

    public FillBox(ManipulatorSubsystem manipulatorSubsystem) {
        m_manipulator = manipulatorSubsystem;

        addRequirements(manipulatorSubsystem);
        setName("Fill Box");
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
        m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteIn);

        //m_manipulator.elevatorMotorSpeed(ManipulatorConstants.kElevatorOff);
        m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxOff);
        
        if (m_outletTrailing.getAsBoolean()) {
            m_observedEdge++;
        }
        SmartDashboard.putNumber("box observed edge", m_observedEdge);
        SmartDashboard.putString("command:", "Fill Box");

    }

    @Override
    public void end(boolean interrupted) {
        m_observedEdge = 0;
        if (!interrupted) {
            m_manipulator.chuteSetState(false);
            m_manipulator.boxSetState(true);
        }
    }

    @Override
    public boolean isFinished() {
        return m_observedEdge >= 2;
    }
}
