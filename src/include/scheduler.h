typedef struct MotorMovement MotorMovement;

struct MotorMovement {
    int32_t steps;
    int32_t fractionalStep;
    int32_t speed;
    int32_t acceleration;
};

/*TODO: Make these externs*/
int schedulerInit( void );
void getNewCommand( void );
int applyCommand( Command_t *command );
int updateMotors( void );
int moveMotor( int motorNumber );
int setDirection( int motorNumber, int forwardDirection );
