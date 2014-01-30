typedef struct MotorMovement MotorMovement;

struct MotorMovement {
    int32_t steps;
    int32_t fractionalStep;
    int32_t speed;
    int32_t acceleration;
};

int schedulerInit( void );
int updateMotors( void );

