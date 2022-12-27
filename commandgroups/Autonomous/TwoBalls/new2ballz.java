// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups.Autonomous.TwoBalls;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commandgroups.ShootAuto;
import frc.robot.commands.Autonomous.AutoAngleTurn;
import frc.robot.commands.Autonomous.AutoStraightDrive;
//import frc.robot.commands.Autonomous.TakeAim;
//import frc.robot.commands.Feeder.FeederTurn;
import frc.robot.commands.Intake.IntakeTurn;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class new2ballz extends SequentialCommandGroup {
  /** Creates a new new2ballz. */
  public new2ballz(
    DriveSubsystem m_drive,
    FeederSubsystem m_feeder,
    IntakeSubsystem m_intake,
    ShooterSubsystem m_shooter,
    VisionSubsystem m_vision,
    LEDSubsystem m_led   
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new InstantCommand(
        new Runnable() {
          @Override
          public void run() {
            m_intake.pushPneumatic();
          }
        }
      )

.andThen(new AutoStraightDrive(m_drive, 1.5, false))
.deadlineWith(new IntakeTurn(m_intake,-1))
.andThen(new IntakeTurn(m_intake, -1).withTimeout(1))
.andThen(new AutoAngleTurn(m_drive, 180))
.andThen(new ShootAuto(m_shooter, m_vision, m_led).deadlineWith(new WaitCommand(2).andThen(new InstantCommand(
  new Runnable() {
    @Override
    public void run() {
      m_feeder.runFeeder(1);
    }
  }))))


    );
  }
}
