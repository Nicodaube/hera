-module(controller).

% -export([pid/5]).
-export([init/1]).

-define(Speed_Limit, 50).
-define(Angle_Coef, 50).
-define(Dist_Coef, 50).
-define(Speed_Coef, 50).
-define(Angle_Rate_Coef, 50).
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
    Distance_saturated = 0, %saturation((distances[0] + distances[2]) / 2, 20),
    AngleRateError = (ets:lookup(variables, "Angle") + ets:lookup(variables, "Offset") + ?Angle_Rate_Coef * ets:lookup(variables, "Angle_Rate")),

    Acc_comm = (?Angle_Coef * AngleRateError
                         + ?Dist_Coef * Distance_saturated
                         + ?Speed_Coef * ets:lookup(variables, "SpeedCommand")),
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

    %low pass filter on derivative
    AR = (Gy - ets:lookup(variables, "DC_bias")) * ?Coef_Filter + ets:lookup(variables, "Angle_Rate") * (1 - ?Coef_Filter),
    ets:insert(variables, {"Angle_Rate", AR}),

    %complementary filter
    Delta_Gyr = AR * Dt,
    Angle_Acc = math:atan(Ax / Az) * 180 / math:pi(),
    New_Angle = (ets:lookup(variables, "Angle") + Delta_Gyr) * ?K + (1 - ?K) * Angle_Acc,
    ets:insert(variables, {"Angle", New_Angle}),
    ok.


compute_angle_offset() ->
    A = ets:lookup(variables, "Angle"),
    OS = (( A + ets:lookup(variables, "Offset")) * 999.9 / 1000) - A,
    ets:insert(variables, {"Offset", OS}),
    ok.






% pid({Kp, Ki, Kd}, {Ierr0, Derr0}, Set_Point, Measure, Dt) ->

%     Error = Set_Point - Measure,
%     Ierr1 = Ierr0 + Error * Dt,
%     Derr1 = (Error-Derr0)/Dt,

%     Command = Kp * Error + Ki * Ierr1 + Kd * Derr1,
%     {Command, {Ierr1, Error}}.



