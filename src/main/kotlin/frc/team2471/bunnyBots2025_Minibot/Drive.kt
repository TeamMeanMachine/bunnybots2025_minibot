package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted

object Drive: SubsystemBase("Drive") {
    val leftMotor = TalonFX(Falcons.LEFT_DRIVE)
    val rightMotor = TalonFX(Falcons.RIGHT_DRIVE)

    init {
        leftMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            brakeMode()
        }
        rightMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            brakeMode()
        }
    }

    override fun periodic() {
        joystickDrive()
    }

    fun joystickDrive() {
        val forwardStick = -OI.driverController.leftY
        val steerStick = OI.driverController.rightX

        val leftPower = forwardStick + steerStick
        val rightPower = forwardStick - steerStick

        leftMotor.setVoltage(leftPower * 12.0)
        rightMotor.setVoltage(rightPower * 12.0)
    }

}