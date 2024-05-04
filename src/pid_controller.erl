-module(pid_controller).

-export([pid/5]).

pid({Kp, Ki, Kd}, {Ierr0, Derr0}, Set_Point, Measure, Dt) ->

    Error = Set_Point - Measure,
    Ierr1 = Ierr0 + Error * Dt,
    Derr1 = (Error-Derr0)/Dt,

    Command = Kp * Error + Ki * Ierr1 + Kd * Derr1,
    {Command, {Ierr1, Err}}.



