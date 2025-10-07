package frc.team2471.bunnyBots2025_Minibot.util.ctre.loggedTalonFX

import frc.team2471.bunnyBots2025_Minibot.util.isSim

object MasterMotor {
    private val motors = mutableListOf<LoggedMotor>()

    fun simPeriodic() {
        if (isSim) {
            motors.forEach {
                it.simPeriodic()
            }
        }
    }

    fun addMotor(motor: LoggedMotor) = motors.add(motor)
}