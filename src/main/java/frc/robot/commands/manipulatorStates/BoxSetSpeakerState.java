
package frc.robot.commands.manipulatorStates;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ManipulatorSubsystem;

public class BoxSetSpeakerState extends Command {

    private ManipulatorSubsystem m_manipulator;
    private Timer m_timer;

  public BoxSetSpeakerState(Timer timer, ManipulatorSubsystem manipulatorSubsystem) {
    m_manipulator = manipulatorSubsystem;
    m_timer = timer;
    addRequirements(manipulatorSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    m_manipulator.intakeMotorSpeed(ManipulatorConstants.kIntakeOut);
    m_manipulator.boxMotorSpeed(ManipulatorConstants.kBoxInFast);
    m_manipulator.chuteMotorSpeed(ManipulatorConstants.kChuteOff);
    m_manipulator.CNDASetPosition(ManipulatorConstants.kCNDASpeakerPos);

  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
//    private double CNDAPosition = m_manipulator.getCNDAPosition();
//    private double CNDALowerRange;
//    private double CNDAHigherRange = ManipulatorConstants.kCNDASpeakerPos

//    private boolean AcceptableCNDARange = (CNDAPosition >) && ()
    return m_timer.hasElapsed(ManipulatorConstants.kBoxMotorFastTime) && m_manipulator.getCNDAPosition() > 9.5;
  }
}
