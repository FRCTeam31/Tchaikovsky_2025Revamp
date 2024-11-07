package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class CombinedCommands {
    /**
     * Runs a sequence to score a note in the speaker
     * @return
     */
    public SequentialCommandGroup scoreInSpeakerSequentialGroup(ShooterSubsystem shooter, Intake intake) {
      return shooter
        .startShootingNoteCommand()
        .andThen(new WaitCommand(0.75))
        .andThen(intake.ejectNoteCommand())
        .andThen(new WaitCommand(0.75))
        .andThen(shooter.stopMotorsCommand())
        .andThen(intake.stopRollersCommand());
    }

    /**
     * Runs a sequence to load a note into the shooter for scoring in the amp
     * @return
     */
    public SequentialCommandGroup loadNoteForAmp(ShooterSubsystem shooter, Intake intake) {
      return Commands
        .runOnce(() -> intake.runIntakeRollers(-0.7)) // Eject from the intake
        .alongWith(Commands.runOnce(() -> shooter.runShooter(0.1))) // Load into the shooter
        .andThen(new WaitUntilCommand(shooter::isNoteLoaded).withTimeout(1)) // Wait until the note is loaded
        .andThen(new WaitCommand(0.045)) // Give the note time to get into the shooter
        .andThen(stopShooterAndIntakeCommand(shooter, intake)); // Stop both the shooter and intake
    }

    /**
     * Runs a sequence to stop both the shooter and intake
     * @return
     */
    public SequentialCommandGroup stopShooterAndIntakeCommand(ShooterSubsystem shooter, Intake intake) {
      return shooter.stopMotorsCommand().andThen(intake.stopRollersCommand());
    }

    /**
     * Returns a map of named commands that use multiple subsystems
     * @return
     */
    public Map<String, Command> getNamedCommands(ShooterSubsystem shooter, Intake intake) {
      return Map.of(
        "Score_In_Speaker",
        scoreInSpeakerSequentialGroup(shooter, intake),
        "Load_Note_For_Amp",
        loadNoteForAmp(shooter, intake),
        "Stop_Shooter_And_Intake",
        stopShooterAndIntakeCommand(shooter, intake)
      );
    }
}
