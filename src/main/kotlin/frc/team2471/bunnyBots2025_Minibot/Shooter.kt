package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.onlyRunWhileTrue
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
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

    const val SHOOT_ERROR_THRESHOLD = 2.0
    var withinShootErrorFrames = 0
    val reverseFrameAmount = 5

    var shootingVelocity get() = shootingVelocityEntry.getDouble(26.0)
        set(value) {shootingVelocityEntry.setDouble(value)}
    val spittingVelocity get() = spittingVelocityEntry.getDouble(-10.0)

    val motor = TalonFX(Falcons.SHOOTER)

    init {
        if (!spittingVelocityEntry.exists()) {
            spittingVelocityEntry.setDouble(spittingVelocity)
        }
        if (!shootingVelocityEntry.exists()) {
            shootingVelocityEntry.setDouble(shootingVelocity)
        }

        motor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            coastMode()

            p(0.25)
            d(0.005)
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

    // Ramps up,  rumbles, then waits for the shoot button to be pressed
    fun rampUp(): Command {
        var reverseFrames = 0
        return runCommand {

            if (reverseFrames < reverseFrameAmount) {
                Intake.currentState = Intake.State.INDEXERREVERSING
                reverseFrames++
            } else if (reverseFrames == reverseFrameAmount) {
                Intake.currentState = Intake.State.HOLDING
                reverseFrames++
            }
            motor.setControl(VelocityVoltage(shootingVelocity))
            if ((shootingVelocity - motor.velocity.valueAsDouble).absoluteValue < SHOOT_ERROR_THRESHOLD) withinShootErrorFrames++
            else withinShootErrorFrames = 0
            if (withinShootErrorFrames > 20) {
                OI.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.5)
            }

            if (OI.driverController.rightTriggerAxis > 0.95) {
                Intake.currentState = Intake.State.SHOOTING
            }
        }.finallyRun {
            Intake.currentState = Intake.State.HOLDING
            motor.setControl(VoltageOut(0.0))
            reverseFrames = 0
            OI.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        }

    }}