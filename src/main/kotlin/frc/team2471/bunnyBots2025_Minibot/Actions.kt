package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.runCommand

fun spit(): Command {
    return runCommand {
        if (Intake.currentIntakeState != Intake.IntakeState.SHOOTING) {
            Intake.currentIntakeState = Intake.IntakeState.REVERSING
            Shooter.motor.setControl(VelocityVoltage(Shooter.spittingVelocity))
        }
    }.finallyRun {
        Intake.currentIntakeState = Intake.IntakeState.HOLDING
        Shooter.motor.setControl(VelocityVoltage(0.0))
    }
}