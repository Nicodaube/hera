-module(controller).

% -export([pid/5]).
-export([init/1]).
-export([controller/1]).

-define(Speed_Limit, 50).
-define(Angle_Coef, -0.7).
-define(Dist_Coef, 0).
-define(Speed_Coef, 2).
-define(Angle_Rate_Coef, 7).
-define(Coef_Filter, 0.667).
-define(K, 0.97).

%Une fois qu'on a de bons coeffs on peut tous les def avec un "-define()" pour plus d'efficacité


init({Cal}) ->
    I2Cbus = grisp_i2c:open(i2c1),

    ets:new(variables, [set, named_table]),
    ets:insert(variables, {"SpeedCommand", 0}),
    ets:insert(variables, {"Angle_Rate", 0}),
    ets:insert(variables, {"Angle", 0}),
    ets:insert(variables, {"DC_Bias", Cal}),
    ets:insert(variables, {"Offset", 0}),
    ets:insert(variables, {"Bus", I2Cbus}),
    ok.

    
    


controller(Measures) ->

    {Ax,Az,Gy,Dt} = Measures, %Pas vraiment utile sauf pour print
    Balance_enable = true,

    compute_angle(Measures),
    compute_angle_offset(),

    Acc = balance_controller(),
    io:format("Acc command: ~p~n",[Acc]),
    Acc.




balance_controller() ->

    [{_,Offset}] = ets:lookup(variables, "Offset"),
    [{_,Angle_Rate}] = ets:lookup(variables, "Angle_Rate"),
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    [{_,SpeedCommand}] = ets:lookup(variables, "SpeedCommand"),

    Distance_saturated = 0, %saturation((distances[0] + distances[2]) / 2, 20),
    AngleRateError = (Angle + Offset + ?Angle_Rate_Coef * Angle_Rate),

    Acc_comm = (?Angle_Coef * AngleRateError
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

    %low pass filter on derivative
    AR = (Gy - DC_Bias) * ?Coef_Filter + Angle_Rate * (1 - ?Coef_Filter),
    ets:insert(variables, {"Angle_Rate", AR}),

    %complementary filter
    Delta_Gyr = AR * Dt,
    Angle_Acc = math:atan(Ax / Az) * 180 / math:pi(),
    New_Angle = (Angle + Delta_Gyr) * ?K + (1 - ?K) * Angle_Acc,
    ets:insert(variables, {"Angle", New_Angle}),
    ok.


compute_angle_offset() ->

    %Retreive values from table
    [{_,Angle}] = ets:lookup(variables, "Angle"),
    [{_,Offset}] = ets:lookup(variables, "Offset"),

    OS = (( Angle + Offset) * 999.9 / 1000) - Angle,
    ets:insert(variables, {"Offset", OS}),
    ok.






% pid({Kp, Ki, Kd}, {Ierr0, Derr0}, Set_Point, Measure, Dt) ->

%     Error = Set_Point - Measure,
%     Ierr1 = Ierr0 + Error * Dt,
%     Derr1 = (Error-Derr0)/Dt,

%     Command = Kp * Error + Ki * Ierr1 + Kd * Derr1,
%     {Command, {Ierr1, Error}}.



