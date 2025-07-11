-module(hera_measure_sender).
-export([init/0]).

init() ->
    loop().

loop() ->
    receive
        {Name, Seq, From, Values}->
            hera_com:send(Name, Seq, From, Values);
        {Name, Seq, Values}->
            hera_com:send(Name, Seq, Values);
        Msg ->
            hera:logg("[HERA_MEASURE_SENDER] received strange message ~p~n", [Msg])
    end,
    loop().