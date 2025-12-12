package frc.team2471.bunnyBots2025_Minibot

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.GenericHID
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.loggedTalonFX.LoggedTalonFX

object Intake: SubsystemBase("Intake") {
   val table = NetworkTableInstance.getDefault().getTable("Intake")

   private val upperIntakingPercentEntry = table.getEntry("Upper Intaking Percentage")
   private val sideIntakingPercentEntry = table.getEntry("Side Intaking Percentage")
   private val indexingPercentEntry = table.getEntry("Indexing Percentage")
   private val indexerMotorShootingPercentEntry = table.getEntry("Index Motor Shooting Percentage")

   val upperIntakeMotor = TalonSRX(Talons.INTAKE_TOP)
   val leftIntakeMotor = LoggedTalonFX(Falcons.INTAKE_LEFT)
   val rightIntakeMotor = LoggedTalonFX(Falcons.INTAKE_RIGHT)
   val indexerMotor = LoggedTalonFX(Talons.UPTAKE)

   val lowerBeambreakSensor = DigitalInput(DigitalSensors.LOWER_BEAMBREAK)
   val upperBeambreakSensor = DigitalInput(DigitalSensors.UPPER_BEAMBREAK)

   val lowerBeambreakDebouncer = Debouncer(0.05, Debouncer.DebounceType.kFalling)

   @get:AutoLogOutput(key = "Intake/Lower Beambreak")
   val lowerBeambreak: Boolean get() = lowerBeambreakDebouncer.calculate(!lowerBeambreakSensor.get())
   @get:AutoLogOutput(key = "Intake/Upper Beambreak")
   val upperBeambreak: Boolean get() = !upperBeambreakSensor.get()

   val upperIntakingPercentage: Double get() = upperIntakingPercentEntry.getDouble(0.7) // 70%
   val sideIntakingPercentage: Double get() = sideIntakingPercentEntry.getDouble(0.7) // 70%
   val indexingPercentage: Double get() = indexingPercentEntry.getDouble(0.5)
   val indexerMotorShootingPercentage: Double get() = indexerMotorShootingPercentEntry.getDouble(0.7)

   @get:AutoLogOutput(key = "Intake/Current State")
   var currentState = State.HOLDING
       set(value) {
         if (value == State.HOLDING) {
            OI.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
         }
         field = value
      }

   @get:AutoLogOutput(key = "Intake/Is Full")
   val isFull get() = lowerBeambreak && upperBeambreak

   val alternateFrames: Int = 6
   var alternateCenteringMotors = 0

   init {
      if (!upperIntakingPercentEntry.exists()) {
         upperIntakingPercentEntry.setDouble(upperIntakingPercentage)
      }
      if (!sideIntakingPercentEntry.exists()) {
         sideIntakingPercentEntry.setDouble(sideIntakingPercentage)
      }
      if (!indexingPercentEntry.exists()) {
         indexingPercentEntry.setDouble(indexingPercentage)
      }
      if (!indexerMotorShootingPercentEntry.exists()) {
         indexerMotorShootingPercentEntry.setDouble(indexerMotorShootingPercentage)
      }



      upperIntakeMotor.configContinuousCurrentLimit(20)
      upperIntakeMotor.configPeakCurrentLimit(30)
      upperIntakeMotor.configPeakCurrentDuration(1000)

      leftIntakeMotor.applyConfiguration {
         currentLimits(30.0, 40.0, 1.0)
         inverted(true)
         coastMode()
      }

      rightIntakeMotor.applyConfiguration {
         currentLimits(20.0, 25.0, 1.0)
         inverted(false)
         coastMode()
      }

      indexerMotor.applyConfiguration {
         currentLimits(20.0, 25.0, 1.0)
         inverted(true)
         coastMode()
      }

   }

   override fun periodic() {
      when (currentState) {
         State.INTAKING -> {

            OI.driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0.3)

            if (upperBeambreak && lowerBeambreak) currentState = State.HOLDING

            if (upperBeambreak) {
               indexerMotor.setControl(VoltageOut(0.0))
            } else {
               indexerMotor.setControl(VoltageOut(indexingPercentage * 12.0))
            }

            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, upperIntakingPercentage)
            alternateLogic()
         }

         State.HOLDING -> {
            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)
            leftIntakeMotor.setControl(VoltageOut(0.0))
            rightIntakeMotor.setControl(VoltageOut(0.0))
            indexerMotor.setControl(VoltageOut(0.0))
         }

         State.SHOOTING -> {
            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, upperIntakingPercentage)
            alternateLogic()
            indexerMotor.setControl(VoltageOut(indexerMotorShootingPercentage * 12.0))
         }

         State.REVERSING -> {
            upperIntakeMotor.set(TalonSRXControlMode.PercentOutput, -upperIntakingPercentage)
            leftIntakeMotor.setControl(VoltageOut(-sideIntakingPercentage * 12.0))
            rightIntakeMotor.setControl(VoltageOut(-sideIntakingPercentage * 12.0))
            indexerMotor.setControl(VoltageOut(-indexingPercentage * 12.0))
         }

         State.INDEXERREVERSING -> {
            indexerMotor.setControl(VoltageOut(-indexingPercentage * 12.0))
         }
      }
   }

   fun alternateLogic() {
      if (alternateCenteringMotors % alternateFrames < (alternateFrames / 2)) {
         leftIntakeMotor.setControl(DutyCycleOut(sideIntakingPercentage))
         rightIntakeMotor.setControl(DutyCycleOut(0.0))
      } else {
         leftIntakeMotor.setControl(DutyCycleOut(0.0))
         rightIntakeMotor.setControl(DutyCycleOut(sideIntakingPercentage))
      }
      alternateCenteringMotors++
   }

   enum class State {
      INTAKING,
      HOLDING,
      SHOOTING,
      REVERSING,
      // Like reversing but only runs indexer
      INDEXERREVERSING
   }
}