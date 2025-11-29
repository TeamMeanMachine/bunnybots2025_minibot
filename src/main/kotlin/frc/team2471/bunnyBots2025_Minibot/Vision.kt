package frc.team2471.bunnyBots2025_Minibot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team2471.bunnyBots2025_Minibot.Drive.poseEstimator
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.vision.limelight.VisionIO
import org.team2471.frc.lib.vision.limelight.VisionIOLimelight
import kotlin.math.sqrt


// Single limelight 3g
object Vision : SubsystemBase() {
    val io: VisionIO = VisionIOLimelight("limelight", false, {Drive.gyroAngle})
    
    val inputs = VisionIO.VisionIOInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.recordOutput("Limelight Timestamp", inputs.aprilTagTimestamp)
        Logger.recordOutput("Robot Timestamp", Timer.getFPGATimestamp())
        if (inputs.aprilTagPoseEstimate != Pose2d()) {
            poseEstimator.addVisionMeasurement(inputs.aprilTagPoseEstimate, inputs.aprilTagTimestamp)
        }

        updateCrop(inputs.trimmedFiducials)
    }

    fun gyroReset() {
        io.gyroReset()
    }

    fun updateCrop(fiducials: List<Triple<Double, Pair<Double, Double>, Double>>) {

        if (fiducials.isEmpty()) {
//            println("No fiducials found")
            io.updateCropping(-1.0, 1.0, -1.0, 1.0)
            return
        }

        val overshootPercentage = 1.75

        var minY = 1.0
        var maxY = -1.0


        for (i in fiducials.indices) {

            val fiducial = fiducials[i]

            val normalizedTy = fiducial.second.second / 28.1

            // half a side length               area of image (when x and y are between -1 and 1)
            val targetRadius = sqrt(fiducial.third) / 2.0 * 4.0

            Logger.recordOutput("targetRadius", targetRadius )


            val maybeMinY = (normalizedTy - targetRadius * overshootPercentage).coerceIn(-1.0, 1.0)
            val maybeMaxY = (normalizedTy + targetRadius * overshootPercentage).coerceIn(-1.0, 1.0)

            if (maybeMaxY > maxY) maxY = maybeMaxY
            if (maybeMinY < minY) minY = maybeMinY


        }

        Logger.recordOutput("Crop", Translation2d(-1.0, minY), Translation2d(1.0, minY), Translation2d(-1.0, maxY), Translation2d(1.0, maxY))

        io.updateCropping(-1.0, 1.0, minY, maxY)

    }


    fun onEnable() = io.enable()

    fun onDisable() = io.disable()
}