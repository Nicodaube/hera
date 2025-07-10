-module(hera).

-behaviour(application).

-export([start_measure/2, timestamp/0, logg/2]).
-export([start/2, stop/1]).

-type timestamp() :: integer() | undefined.

-export_type([timestamp/0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% starts and supervise the measure defined in the callback module
%% using Module:init(Args)
-spec start_measure(Module, Args) -> {ok, pid()} | {error, term()} when
    Module :: module(),
    Args :: term().

start_measure(Module, Args) ->
    hera_measure_sup:start_child(Module, Args).


-spec timestamp() -> timestamp().

timestamp() ->
  erlang:monotonic_time(millisecond).

logg(Message, Args) ->
    DebugMode = persistent_term:get(debugMode),
    if
        DebugMode ->
            io:format(Message, Args);
        true ->
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_StartType, _StartArgs) ->
    io:format("[HERA] Startup~n"),
    Args = application:get_env(hera, startup_args, [false]),
    io:format("~p", [Args]),
    case Args of 
        [true] ->
            persistent_term:put(debugMode, true),
            hera:logg("[HERA] DebugMode on~n", []);
        _ ->
            persistent_term:put(debugMode, false),
            ok
    end,    
    hera_sup:start_link().

stop(_State) ->
    ok.