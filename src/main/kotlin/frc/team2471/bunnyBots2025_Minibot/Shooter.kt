package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted

// Single falcon using phoenix 6 velocity control
object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    private val shootingVelocityEntry = table.getEntry("Shooting Velocity")

    val shootingVelocity = shootingVelocityEntry.getDouble(5.0)

    val motor = TalonFX(Falcons.SHOOTER)

    init {
        if (!shootingVelocityEntry.exists()) {
            shootingVelocityEntry.setDouble(shootingVelocity)
        }

        motor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            brakeMode()
        }
    }

    fun shoot(): Command {
        return run {
            Intake.currentIntakeState = Intake.IntakeState.SHOOTING
            motor.setControl(VelocityVoltage(shootingVelocity))
        }.finallyRun {
            Intake.currentIntakeState = Intake.IntakeState.HOLDING
            motor.setControl(VelocityVoltage(0.0))
        }

    }
}