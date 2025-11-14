package frc.team2471.bunnyBots2025_Minibot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.bunnyBots2025_Minibot.Drive.poseEstimator
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight

// Single limelight 3g
object Vision : SubsystemBase() {
    val io: Array<VisionIO> = arrayOf(
        VisionIOLimelight("limelight-shooter", false, {Drive.heading})
    )
    
    val inputs = Array(io.size) { VisionIO.VisionIOInputs() }

    override fun periodic() {
        for (i in io.indices) {
            io[i].updateInputs(inputs[i])
            if (inputs[i].aprilTagPoseEstimate != Pose2d()) {
                poseEstimator.addVisionMeasurement(inputs[i].aprilTagPoseEstimate, inputs[i].aprilTagTimestamp)
            }
        }
    }

    fun gyroReset() {
        io.forEach { it.gyroReset() }
    }


    fun onEnable() = io.forEach { it.enable() }

    fun onDisable() = io.forEach { it.disable() }
}