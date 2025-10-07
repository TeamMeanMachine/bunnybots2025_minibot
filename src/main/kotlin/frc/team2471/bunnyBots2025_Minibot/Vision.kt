package frc.team2471.bunnyBots2025_Minibot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.bunnyBots2025_Minibot.util.control.commands.finallyRun
import frc.team2471.bunnyBots2025_Minibot.util.control.commands.runCommand
import frc.team2471.bunnyBots2025_Minibot.util.units.asDegrees
import frc.team2471.bunnyBots2025_Minibot.util.units.degrees
import frc.team2471.bunnyBots2025_Minibot.util.units.radians
import frc.team2471.bunnyBots2025_Minibot.util.vision.limelight.LimelightMode
import frc.team2471.bunnyBots2025_Minibot.util.vision.limelight.VisionIO
import frc.team2471.bunnyBots2025_Minibot.util.vision.limelight.VisionIOLimelight
import org.littletonrobotics.junction.Logger
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.sign

object Vision : SubsystemBase() {
    val io: Array<VisionIO> = arrayOf(
    )
    
    val inputs = Array(io.size) { VisionIO.VisionIOInputs() }

    override fun periodic() {
        for (i in io.indices) {
            io[i].updateInputs(inputs[i])
        }
    }

    fun gyroReset() {
        io.forEach { it.gyroReset() }
    }


    fun onEnable() = io.forEach { it.enable() }

    fun onDisable() = io.forEach { it.disable() }
}