//enum Priority {
//    NONE,
//    NORMAL,
//    IMPORTANT
//};
//
//enum Direction {
//    NONE,
//    FORWARD,
//    LEFT,
//    RIGHT,
//    BACKWARDS
//};
//
//enum Actor {
//    NONE,
//    BOUNDARY_OUTSIDE,
//    MOTOR_LIMIT,
//    COLLISION
//};
//
//typedef struct Action {
//    Direction direction;
//    Priority priority;
//    Actor actor;
//    int run_for;
//    int started_at;
//    float speed;
//} Action;
//
//
//typedef struct Signal {
//    float magnitude;
//    float bwf_boundary;
//    float bwf_guide;
//} Signal;
//
//typedef struct Sensors {
//    Signal sig1;
//    Signal sig2;
//    Signal sig3;
//    float m1current;
//    float m2current;
//    float m3current;
//    bool bumper;
//} Sensors;
//
//typedef struct State {
//    Action action;
//    Sensors sensors;
//} State;
//
//long GetTick() {
//    return 10;
//}
//
//void check_stuff(State* state) {
//
//}
//
//void decide(State* state) {
//    if (state->sensors.sig1 > 0) {
//
//    }
//}
//
//void loop(void) {
//
//    State state;
//    state.action.direction = NONE;
//
//    while (true) {
//        checkStuff(&state);
//
//    }
//}
//
//Action right_for(int ms) {
//    Action action;
//    action.speed = 1;
//    action.direction = RIGHT;
//    action.run_for = ms;
//    return action;
//}
