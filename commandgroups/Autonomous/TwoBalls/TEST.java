package frc.robot.commandgroups.Autonomous.TwoBalls;

//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.ShootAuto;
import frc.robot.commands.Autonomous.AutoAngleTurn;
import frc.robot.commands.Autonomous.AutoStraightDrive;
//import frc.robot.commands.Autonomous.TakeAim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.Intake.IntakePneumaticPush;
import frc.robot.commands.Intake.IntakeTurn;
import frc.robot.commands.Feeder.FeederTurn;

public class TEST extends SequentialCommandGroup {
  public TEST(
      DriveSubsystem m_drive,
      FeederSubsystem m_feeder,
      IntakeSubsystem m_intake,
      ShooterSubsystem m_shooter,
      VisionSubsystem m_vision,
      LEDSubsystem m_led) {
    addCommands(
      new ShootAuto(m_shooter, m_vision, m_led)
      .alongWith(
      new IntakePneumaticPush(m_intake).withTimeout(0.5)
      .andThen(new AutoStraightDrive(m_drive, 1, false))
      .deadlineWith(new IntakeTurn(m_intake, -1))
      //.raceWith(new IntakeTurn(m_intake, -1))
      .andThen(new WaitCommand(0.5))
      .andThen(new IntakeTurn(m_intake, -1)).withTimeout(1)
      .andThen(new AutoAngleTurn(m_drive, 180))
      .andThen(new WaitCommand(0.5))
      .andThen(new FeederTurn(m_feeder, 1))));
  
    }
}
