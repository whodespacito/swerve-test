package frc.robot.commands.manipulatorStates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreInAmp extends Command {
    private Timer m_ampTimer;
    private ManipulatorSubsystem m_manipulator;

  public ScoreInAmp(Timer ampTimer, ManipulatorSubsystem manipulator) {
    m_ampTimer = ampTimer;
    m_manipulator = manipulator;

    addRequirements(manipulator);
    setName("Score In Amp");
  }

  @Override
  public void initialize() {
    m_ampTimer.restart();
  }

  @Override
  public void execute() {
    m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
    m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteOff);

    //m_manipulator.elevatorMotorSpeed(ManipulatorConstants.kElevatorOff);

    m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxOut);

    SmartDashboard.putNumber("amp timer", m_ampTimer.get());
    SmartDashboard.putString("command:", "Score In Amp");
  }

  @Override
  public void end(boolean interrupted) {
    m_manipulator.boxSetState(false);
    m_ampTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_ampTimer.hasElapsed(ManipulatorConstants.kEmptyBoxToAmpTime);
  }
}
