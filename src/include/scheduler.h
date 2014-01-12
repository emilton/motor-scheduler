typedef struct MotorMovement MotorMovement;

enum MotorStatus {
    Idle = 0,
    Accelerating = 1,
    Deaccelerating = 2,
    ConstantSpeed = 3
};

struct MotorMovement {
    enum MotorStatus motorStatus;
    uint32_t steps;
    uint32_t stepsTaken;
    uint32_t deaccelerationStart;
    int32_t fractionalStep;
    int32_t maxSpeed;
    int32_t speed;
    int32_t acceleration;
};

int schedulerInit( void );
int updateMotors( void );

