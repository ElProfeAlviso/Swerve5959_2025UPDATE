package com.team5959; // Declaración del paquete al que pertenece esta clase.

// Importa la clase RobotBase de la biblioteca WPILib, que es la base para inicializar y ejecutar el robot.
import edu.wpi.first.wpilibj.RobotBase;

public final class Main { // Declaración de la clase principal, marcada como final para evitar herencia.

  // Constructor privado para evitar que se creen instancias de esta clase.
  private Main() {
  }

  /**
   * Función principal de inicialización. No realices ninguna inicialización aquí.
   *
   * <p>
   * Si cambias la clase principal del robot, cambia el tipo de parámetro.
   */
  public static void main(String... args) { // Método principal que se ejecuta al iniciar el programa.

    // Llama al método startRobot de RobotBase, pasando una referencia al
    // constructor de la clase Robot.
    // Esto inicializa y ejecuta la clase principal del robot.
    RobotBase.startRobot(Robot::new);
  }
}
