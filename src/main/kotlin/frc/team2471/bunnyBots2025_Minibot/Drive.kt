package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix6.hardware.TalonFX
import com.studica.frc.AHRS
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.PDVelocityController
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.brakeMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.math.deadband
import org.team2471.frc.lib.math.toPose2d
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.units.wrap
import kotlin.math.abs

object Drive: SubsystemBase("Drive") {

    val leftMotor = TalonFX(Falcons.LEFT_DRIVE)
    val rightMotor = TalonFX(Falcons.RIGHT_DRIVE)

    private val navx = AHRS(AHRS.NavXComType.kMXP_SPI)
    @get:AutoLogOutput(key = "Drive/Heading")
    val heading get() = navx.angle.degrees

    @get:AutoLogOutput(key = "Drive/Pose")
    var pose = Pose2d()

    @get:AutoLogOutput(key = "Drive/Distance Right")
    var distRight = 0.0
    @get:AutoLogOutput(key = "Drive/Distance Left")
    var distLeft = 0.0

    val wheelRadius = (1.0 + 15.0/16.0).inches
    val kinematics = DifferentialDriveKinematics(17.0.inches)
    val poseEstimator = DifferentialDrivePoseEstimator(kinematics, Rotation2d(), 0.0, 0.0, Pose2d(), VecBuilder.fill(0.5, 0.5, 0.5), VecBuilder.fill(0.01, 0.01, 0.01))

    val aimPIDControler = PIDController(0.0075, 0.0, 0.0)

    init {
        leftMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(true)
            brakeMode()
            this.OpenLoopRamps.apply {
                DutyCycleOpenLoopRampPeriod = 1.0
            }

        }
        rightMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            inverted(false)
            brakeMode()
            this.OpenLoopRamps.apply {
                DutyCycleOpenLoopRampPeriod = 1.0
            }
        }
    }

    override fun periodic() {
        updateOdometry()
    }

    fun joystickDrive(turnOverride: Double? = null) {
        val forwardStick = -OI.driverController.leftY.deadband(0.1)
        val steerStick = OI.driverController.rightX.deadband(0.1)

        val turn = turnOverride ?: if (abs(forwardStick) > 0.1) abs(forwardStick) * steerStick else steerStick

        val leftPower = forwardStick + turn
        val rightPower = forwardStick - turn

        leftMotor.setVoltage(leftPower * 12.0)
        rightMotor.setVoltage(rightPower * 12.0)
    }

    fun updateOdometry() {
        val distanceRight = rightMotor.position.value.asRadians * wheelRadius.asMeters / 5.8
        val distanceLeft = leftMotor.position.value.asRadians * wheelRadius.asMeters / 5.8
        distLeft = distanceLeft
        distRight = distanceRight
        pose = poseEstimator.update(heading.asRotation2d, distanceLeft, distanceRight)
    }

    fun aimToGoal(): Command {
        return runCommand(Drive) {
            val relativePose = FieldManager.goalPose - pose.translation

            val angleError = relativePose.angle.measure.unWrap(heading).asDegrees - heading.asDegrees

            val power = if (abs(angleError) > 1.0) aimPIDControler.calculate(heading.asDegrees, relativePose.angle.measure.unWrap(heading).asDegrees) else 0.0

            joystickDrive(power)
        }.withName("Aim To Goal")
    }
}