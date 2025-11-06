package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v
import kotlin.math.absoluteValue

// Single falcon using phoenix 6 velocity control
object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    private val shootingVelocityEntry = table.getEntry("Shooting Velocity")
    private val spittingVelocityEntry = table.getEntry("Spitting Velocity")

    const val SHOOT_ERROR_THRESHOLD = 3.0

    val shootingVelocity get() = shootingVelocityEntry.getDouble(27.0)
    val spittingVelocity get() = spittingVelocityEntry.getDouble(-10.0)

    val motor = TalonFX(Falcons.SHOOTER)

    init {
        if (!shootingVelocityEntry.exists()) {
            shootingVelocityEntry.setDouble(shootingVelocity)
        }
        if (!spittingVelocityEntry.exists()) {
            spittingVelocityEntry.setDouble(spittingVelocity)
        }

        motor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            coastMode()

            p(0.3)
            v(0.12)
            s(0.17, StaticFeedforwardSignValue.UseVelocitySign)

//            p(5.0)
////            d(0.5)
//            i(5.0)
////            v(0.4)
//            s(9.0, StaticFeedforwardSignValue.UseVelocitySign)
        }
    }

    override fun periodic() {
        Logger.recordOutput("Shooter Velocity", motor.velocity.valueAsDouble)
    }

    fun shoot(): Command {
        var hasStartedShooting = false
        return runCommand {
            motor.setControl(VelocityVoltage(shootingVelocity))
            hasStartedShooting = false
            if ((shootingVelocity - motor.velocity.valueAsDouble).absoluteValue < SHOOT_ERROR_THRESHOLD) {
                if (hasStartedShooting) {
                    println("shooting at ${motor.velocity.valueAsDouble} rps")
                    hasStartedShooting = false
                }
                Intake.currentState = Intake.State.SHOOTING
            }
        }.finallyRun {
            Intake.currentState = Intake.State.HOLDING
            motor.setControl(VoltageOut(0.0))
        }

    }}