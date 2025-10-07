package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXConfigurator
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.bunnyBots2025_Minibot.util.ctre.applyConfiguration
import frc.team2471.bunnyBots2025_Minibot.util.ctre.brakeMode
import frc.team2471.bunnyBots2025_Minibot.util.ctre.currentLimits
import frc.team2471.bunnyBots2025_Minibot.util.ctre.inverted

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