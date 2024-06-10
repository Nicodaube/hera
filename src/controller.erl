-module(controller).

% -export([pid/5]).
-export([init/1]).
-export([controller/1]).

-export([modif_coef/1]).
-export([print_coef/0, pause_ctrl/1]).

-define(Speed_Limit, 50.0).
-define(Dist_Coef, 0.0).
-define(Speed_Coef, 2.0).
% -define(Angle_Coef_P, -0.7).
% -define(Angle_Coef_I, -0.7).
% -define(Angle_Coef_D, -4.9).
-define(Coef_Filter, 0.667).
-define(K, 0.97).
-define(Angle_Int_Coef, 1.0).


%Une fois qu'on a de bons coeffs on peut tous les def avec un "-define()" pour plus d'efficacité


init({Cal}) ->
    I2Cbus = grisp_i2c:open(i2c1),

    ets:new(variables, [set, public, named_table]),
    ets:insert(variables, {"SpeedCommand", 0}),
    ets:insert(variables, {"Angle_Rate", 0}),
    ets:insert(variables, {"Angle", 0}),
    ets:insert(variables, {"DC_Bias", Cal}),
    ets:insert(variables, {"Offset", 0}),
    ets:insert(variables, {"Bus", I2Cbus}),
    ets:insert(variables, {"AngleInt", 0}),
    ets:insert(variables, {"Angle_Coef_P", 0}),
    ets:insert(variables, {"Angle_Coef_I", 0}),
    ets:insert(variables, {"Angle_Coef_D", 0}),
    ets:insert(variables, {"Offset_Coef", 0.9}),
    ets:insert(variables, {"Reset", 1.0}),
    ok.


controller(Measures) ->

    {Ax,Az,Gy,Dt} = Measures, %Pas vraiment utile sauf pour print
    Balance_enable = true,

    compute_angle(Measures),
    compute_angle_offset(),

    [{_,Reset}] = ets:lookup(variables, "Reset"),

    Acc = balance_controller(),
    % io:format("Acc command: ~p~n",[Acc]),
    {Acc, Reset}.


pause_ctrl(R) ->
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    ets:insert(variables, {"Reset", R}),
    ets:insert(variables, {"Offset", Angle}),
    ok.


modif_coef({P,I,D,F}) ->
    ets:insert(variables, {"Angle_Coef_P", P}),
    ets:insert(variables, {"Angle_Coef_I", I}),
    ets:insert(variables, {"Angle_Coef_D", D}),
    ets:insert(variables, {"Offset_Coef", F}),
    ok.

print_coef() ->
    [{_,P}] = ets:lookup(variables, "Angle_Coef_P"),
    [{_,I}] = ets:lookup(variables, "Angle_Coef_I"),
    [{_,D}] = ets:lookup(variables, "Angle_Coef_D"),
    io:format("Coefs: ~p, ~p, ~p~n",[P,I,D]),
    ok.


balance_controller() ->

    [{_,Offset}] = ets:lookup(variables, "Offset"),
    [{_,Angle_Rate}] = ets:lookup(variables, "Angle_Rate"),
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    [{_,SpeedCommand}] = ets:lookup(variables, "SpeedCommand"),
    [{_,AngleInt}] = ets:lookup(variables, "AngleInt"),

    Distance_saturated = 0.0, %saturation((distances[0] + distances[2]) / 2, 20),
    % AngleRateError = (Angle + Offset) * ?Angle_Coef_P + (AngleInt) * ?Angle_Coef_I + (Angle_Rate) * ?Angle_Coef_D,

    [{_,P}] = ets:lookup(variables, "Angle_Coef_P"),
    [{_,I}] = ets:lookup(variables, "Angle_Coef_I"),
    [{_,D}] = ets:lookup(variables, "Angle_Coef_D"),
    AngleRateError = (Angle + Offset) * P + (AngleInt) * I + (Angle_Rate) * D,

    io:format("~.3f, ~.3f~n",[Angle, Offset]),

    Acc_comm = ( AngleRateError
                         + ?Dist_Coef * Distance_saturated
                         + ?Speed_Coef * SpeedCommand),
    Acc_comm.


saturation(Input, Bound) ->
  if   
    Input > Bound ->
        Bound;
    Input < -Bound -> 
        -Bound; 
    true ->
        Input
    end.


compute_angle({Ax,Az,Gy,Dt}) ->

    % io:format("~p, ~p~n",[ets:lookup(variables, "DC_Bias"),ets:lookup(variables, "Angle_Rate")]),
    % io:format("~p~n",[Gy - ets:lookup(variables, "DC_Bias")]),
    % io:format("~p~n",[1 - ?Coef_Filter]),

    [{_,DC_Bias}] = ets:lookup(variables, "DC_Bias"),
    [{_,Angle_Rate}] = ets:lookup(variables, "Angle_Rate"),
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    [{_,AngleInt}] = ets:lookup(variables, "AngleInt"),

    %low pass filter on derivative
    New_Angle_Rate = (Gy - DC_Bias) * ?Coef_Filter + Angle_Rate * (1 - ?Coef_Filter), %D
    ets:insert(variables, {"Angle_Rate", New_Angle_Rate}),

    %complementary filter
    Delta_Gyr = New_Angle_Rate * Dt,
    Angle_Acc = math:atan(Ax / Az) * 180 / math:pi(),
    New_Angle = (Angle + Delta_Gyr) * ?K + (1 - ?K) * Angle_Acc, %P
    ets:insert(variables, {"Angle", New_Angle}),

    New_Angle_Int = AngleInt + New_Angle*Dt, %I
    ets:insert(variables, {"AngleInt", New_Angle_Int}),


    ok.


compute_angle_offset() ->

    %Retreive values from table
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    [{_,Offset}] = ets:lookup(variables, "Offset"),
    [{_,Offset_Coef}] = ets:lookup(variables, "Offset_Coef"),

    New_Offset = (( Angle + Offset) * Offset_Coef) - Angle,
    ets:insert(variables, {"Offset", New_Offset}),
    ok.






% pid({Kp, Ki, Kd}, {Ierr0, Derr0}, Set_Point, Measure, Dt) ->

%     Error = Set_Point - Measure,
%     Ierr1 = Ierr0 + Error * Dt,
%     Derr1 = (Error-Derr0)/Dt,

%     Command = Kp * Error + Ki * Ierr1 + Kd * Derr1,
%     {Command, {Ierr1, Error}}.



