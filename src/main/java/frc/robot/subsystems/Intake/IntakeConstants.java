package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int PIVOT_MOTOR_ID = 30;
    public static final int ROLLER_MOTOR_ID = 31;

    // Pivot Pozisyonları (Mekanizma turu cinsinden. Örn: 0.25 = Çeyrek tur / 90 derece)
    public static final double DEPLOY_POSITION = 0.25; 
    public static final double RETRACT_POSITION = 0.0;

    public static final double INTAKE_SPEED = 1.0;
    public static final double EJECT_SPEED = -1.0;

    public static final double GEAR_RATIO = 25.0;
    
    public static final double kP = 24.0; //++++
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    
    // Yerçekimi Çarpanı (Kol yataylaştıkça yerçekimini yenmek için gereken güç)
    public static final double kG = 0.2; 
    
    // Hareket Profili Sınırları (Sarsıntıyı önler)
    public static final double CRUISE_VELOCITY = 1.5; // Maksimum RPS (Saniyedeki Tur)
    public static final double ACCELERATION = 3.0;    // RPS/s
    public static final double JERK = 30.0;           // İvmelenme yumuşaklığı
}