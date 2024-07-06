-module(controller).

% -export([pid/5]).
-export([init/1]).
-export([controller_kalman/1,controller_complem/1]).

-export([modif_coef/1]).
% -export([set_speed/2]).
-export([pause_ctrl/1]).

-define(Speed_Limit, 50.0).
-define(Dist_Coef, 0.0).

-define(Coef_Filter, 0.667).
% -define(K, 0.97).


%Une fois qu'on a de bons coeffs on peut tous les def avec un "-define()" pour plus d'efficacité


init({Cal}) ->
    ets:new(variables, [set, public, named_table]),

    ets:insert(variables, {"Angle_Rate", 0.0}),
    ets:insert(variables, {"Angle", 0.0}),
    ets:insert(variables, {"DC_Bias", Cal}),
    ets:insert(variables, {"AngleInt", 0.0}),
    ets:insert(variables, {"Angle_kalman", 0.0}),
    
    ets:insert(variables, {"Kp1", -0.12}),
    ets:insert(variables, {"Ki1", -0.05}),
    ets:insert(variables, {"Kp2", 23.0}),
    ets:insert(variables, {"Kd2", 4.0}),
    ets:insert(variables, {"K", 0.99}),
    
    ets:insert(variables, {"PID_error_int", 0.0}),
    % ets:insert(variables, {"Input_speed", 0.0}),
    % ets:insert(variables, {"Speed_time", 0.0}),

    ets:insert(variables, {"Reset", 1.0}),
    ok.



controller_complem(Measures) ->

    {Ax,Az,Gy,Speed,Dt,C} = Measures,

    compute_angle({Ax,Az,Gy,Speed,Dt}),

    [{_,Angle}] = ets:lookup(variables, "Angle"),   %For graphs
    [{_,Reset}] = ets:lookup(variables, "Reset"),

    [_Stop,_,_,Get_up,_Forward,_Backward,_Left,_Right] = hera_com:get_bits(C),

    Acc = balance_controller_complem(Dt,Speed,C),
    if   
        Reset == 1.0 ->
            if  
                Get_Up ->
                    {sign(Angle)*30.0,1.0, Angle};
                abs(Angle) > 30.0 ->
                    % io:format("Too big~n"),
                    {Acc, 0.0, Angle};
                true ->
                    % io:format("Perfect~n"),
                    {Acc, 1.0, Angle}  
            end;
        true ->
            % io:format("Pause~n"),
            {Acc, 0.0, Angle}    
    end.


pause_ctrl(R) ->
    ets:insert(variables, {"Reset", R}),
    ets:insert(variables, {"PID_error_int", 0.0}),
    ok.

% set_speed(Speed, Time) ->
%     ets:insert(variables, {"Input_speed", Speed}),
%     ets:insert(variables, {"Speed_time", erlang:system_time()/1.0e6+Time}),
%     ok.

modif_coef({Kp1,Ki1,Kp2,Kd2,K}) ->
    ets:insert(variables, {"Kp1", Kp1}),
    ets:insert(variables, {"Ki1", Ki1}),
    ets:insert(variables, {"Kp2", Kp2}),
    ets:insert(variables, {"Kd2", Kd2}),
    ets:insert(variables, {"K", K}),

    ets:insert(variables, {"PID_error_int", 0.0}),
    ok.

balance_controller_complem(Dt,Speed,C) ->

    [{_,Angle_complem}] = ets:lookup(variables, "Angle"),
    [{_,Kp1}] = ets:lookup(variables, "Kp1"),
    [{_,Ki1}] = ets:lookup(variables, "Ki1"),
    [{_,Kp2}] = ets:lookup(variables, "Kp2"),
    [{_,Kd2}] = ets:lookup(variables, "Kd2"),

    [_Stop,_,_,Get_up,Forward,Backward,_Left,_Right] = hera_com:get_bits(C),

    % io:format("~p, ~p, ~p, ~p~n", [C, Forward, Backward, hera_com:get_bits(C)]),

    % T = erlang:system_time()/1.0e6,
    if   
        Forward ->
            % io:format("Going forwards ~n"),
            Speed_setpoint = 15.0;
        Backward ->
            % io:format("Going backwards ~n"),
            Speed_setpoint = -15.0;
        true ->
            % io:format("Stability ~n"),
            Speed_setpoint = 0
    end,

    Target_angle = speed_PI(Dt,Speed,Speed_setpoint,Kp1,Ki1),

    Acc = stability_PD(Dt,Angle_complem,Target_angle,Kp2,Kd2),

    % io:format("~.3f, ~.3f, ~.3f~n",[Speed,Target_angle,Angle_complem]),

    Acc.

speed_PI(Dt,Speed,SetPoint,Kp,Ki) ->

    [{_,PID_error_int}] = ets:lookup(variables, "PID_error_int"),

    Error = SetPoint - Speed,
    PID_error_int_new = saturation(PID_error_int + Error * Dt, 60),
    ets:insert(variables, {"PID_error_int", PID_error_int_new}),
    PI_output = Kp * Error + Ki * PID_error_int_new,
    PI_output.

stability_PD(Dt,Angle,Setpoint,Kp,Kd) ->
    [{_,Angle_Rate}] = ets:lookup(variables, "Angle_Rate"),

    ErrorP = Setpoint - Angle,
    ErrorD = -Angle_Rate,
    PD_output = Kp*ErrorP + Kd*ErrorD,
    PD_output.

saturation(Input, Bound) ->
  if   
    Input > Bound ->
        Bound;
    Input < -Bound -> 
        -Bound; 
    true ->
        Input
    end.


compute_angle({Ax,Az,Gy,Speed,Dt}) ->
    [{_,DC_Bias}] = ets:lookup(variables, "DC_Bias"),
    [{_,Angle_Rate}] = ets:lookup(variables, "Angle_Rate"),
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    [{_,K}] = ets:lookup(variables, "K"),

    %low pass filter on derivative
    New_Angle_Rate = (Gy - DC_Bias) * ?Coef_Filter + Angle_Rate * (1 - ?Coef_Filter), %D
    ets:insert(variables, {"Angle_Rate", New_Angle_Rate}),

    %Delta angle computed from gyro
    Delta_Gyr = New_Angle_Rate * Dt,
    %Absolute angle computed form accelerometer
    Angle_Acc = math:atan(Ax / Az) * 180 / math:pi(),

    %complementary filter
    New_Angle = (Angle + Delta_Gyr) * K + (1 - K) * Angle_Acc, 
    ets:insert(variables, {"Angle", New_Angle}),

    %New_Angle_Int = AngleInt + New_Angle*Dt, %I
    %ets:insert(variables, {"AngleInt", New_Angle_Int}),

    ok.

sign(Value) ->
    if
        Value < 0 ->
            -1;
        true ->
            1
    end.








































controller_kalman(Measures) ->

    {Ax,Az,Gy,Speed,Dt,Th} = Measures,
    ets:insert(variables, {"Angle_kalman", Th}),

    compute_angle({Ax,Az,Gy,Speed,Dt}),             %For graphs
    [{_,Angle}] = ets:lookup(variables, "Angle"),   %For graphs

    [{_,Reset}] = ets:lookup(variables, "Reset"),
    Acc = balance_controller_kalman(Dt,Speed),

    {Acc, Reset, Angle}.

balance_controller_kalman(Dt,Speed) ->

    [{_,Angle_kalman}] = ets:lookup(variables, "Angle_kalman"),
    [{_,Kp1}] = ets:lookup(variables, "Kp1"),
    [{_,Ki1}] = ets:lookup(variables, "Ki1"),
    [{_,Kp2}] = ets:lookup(variables, "Kp2"),
    [{_,Kd2}] = ets:lookup(variables, "Kd2"),

    Target_angle = speed_PI(Dt,Speed,0,Kp1,Ki1),
    Acc = stability_PD(Dt,Angle_kalman,Target_angle,Kp2,Kd2),

    io:format("~.3f, ~.3f, ~.3f~n",[Speed,Target_angle,Angle_kalman]),

    Acc.