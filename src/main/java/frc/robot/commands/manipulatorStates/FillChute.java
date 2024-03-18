package frc.robot.commands.manipulatorStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class FillChute extends Command {
    private ManipulatorSubsystem m_manipulator;

    //also known as chute full on the paper
    private BooleanSupplier m_inletTrailing = () -> m_manipulator.inletSensorDetect(true) && !m_manipulator.inletSensorDetect(false);

    private int m_observedEdge;

    public FillChute(ManipulatorSubsystem manipulatorSubsystem) {
        m_manipulator = manipulatorSubsystem;

        addRequirements(manipulatorSubsystem);
        setName("Fill Chute");
    }

    @Override
    public void initialize() {
        //for some reason, the command remembers observedEdge when the command is finished so you need to set it back to 0 yourself
        m_observedEdge = 0;
    }

    @Override
    public void execute() {
        m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeIn);
        m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteIn);

        //m_manipulator.elevatorMotorSpeed(ManipulatorConstants.kElevatorOff);

        m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxOff);

        if (m_inletTrailing.getAsBoolean()) {
            m_observedEdge++;
        }
        SmartDashboard.putNumber("chute observed edge", m_observedEdge);
        SmartDashboard.putString("command:", "Fill Chute");
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            m_manipulator.chuteSetState(true);
        }
    }

    @Override
    public boolean isFinished() {
        return m_observedEdge >= 2;
    }



}
