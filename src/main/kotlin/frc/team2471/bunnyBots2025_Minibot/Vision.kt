package frc.team2471.bunnyBots2025_Minibot

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.vision.limelight.VisionIO

// Single limelight 3g
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