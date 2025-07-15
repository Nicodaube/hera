-module(hera_kalman).

-export([predict/3, update/4, extended_predict/3, extended_update/4]).
-export([filter/6, extended_filter/6, extended_control/7]).

%% see https://en.wikipedia.org/wiki/Kalman_filter


%% A kalman filter without control input
filter({X0, P0}, F, H, Q, R, Z) ->  
    {Xp, Pp} = predict({X0, P0}, F, Q),
    update({Xp, Pp}, H, R, Z).


predict({X0, P0}, F, Q) ->
    Xp = mat:'*'(F, X0), 
    Pp = mat:eval([F, '*', P0, '*´', F, '+', Q]),
    {Xp, Pp}.


update({Xp, Pp}, H, R, Z) ->
    S = mat:eval([H, '*', Pp, '*´', H, '+', R]), 
    Sinv = mat:inv(S), 
    K = mat:eval([Pp, '*´', H, '*', Sinv]),
    Y = mat:'-'(Z, mat:'*'(H, Xp)),
    X1 = mat:eval([K, '*', Y, '+', Xp]),  
    P1 = mat:'-'(Pp, mat:eval([K, '*', H, '*', Pp])), 
    {X1, P1}.


%% An extended kalman filter without control input, [X0, P0, Q, R, Z] must be mat matrices, [F, Jf, H, Jh] must be functions
extended_filter({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z) ->
    % Prediction
    {Xp, Pp} = extended_predict({X0, P0}, {F, Jf}, Q),

    % Update
    extended_update({Xp, Pp}, {H, Jh}, R, Z).

extended_predict({X0, P0}, {F, Jf}, Q) ->
    Xp = F(X0),
    Jfx = Jf(X0),
    Pp = mat:eval([Jfx, '*', P0, '*´', Jfx, '+', Q]),
    {Xp, Pp}.

extended_update({Xp, Pp}, {H, Jh}, R, Z) ->
    Jhx = Jh(Xp),
    S = mat:eval([Jhx, '*', Pp, '*´', Jhx, '+', R]),
    Sinv = mat:inv(S),
    K = mat:eval([Pp, '*´', Jhx, '*', Sinv]),
    Y = mat:'-'(Z, H(Xp)),
    X1 = mat:eval([K, '*', Y, '+', Xp]),
    P1 = mat:'-'(Pp, mat:eval([K, '*', Jhx, '*', Pp])),
    {X1, P1}.

%% Same function as ekf/ with command input
extended_control({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z, U) -> 
    % Prediction
    Xp = F(X0,U),
    Jfx = Jf(X0),
    Pp = mat:eval([Jfx, '*', P0, '*´', Jfx, '+', Q]),

    % Update
    Jhx = Jh(Xp),
    S = mat:eval([Jhx, '*', Pp, '*´', Jhx, '+', R]),
    Sinv = mat:inv(S),
    K = mat:eval([Pp, '*´', Jhx, '*', Sinv]),
    Y = mat:'-'(Z, H(Xp)),
    X1 = mat:eval([K, '*', Y, '+', Xp]),
    P1 = mat:'-'(Pp, mat:eval([K, '*', Jhx, '*', Pp])),
    {X1, P1}.