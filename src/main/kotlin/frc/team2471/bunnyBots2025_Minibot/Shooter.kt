package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted

// Single falcon using phoenix 6 velocity control
object Shooter: SubsystemBase("Shooter") {
    val motor = TalonFX(Falcons.SHOOTER)

    init {
        motor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            brakeMode()
        }
    }
}