-module(hera_network).
-behaviour(gen_server).

-export([start_link/0, next_network/0]).
-export([init/1, handle_call/3, handle_cast/2, handle_info/2,
         terminate/2, code_change/3]).

-define(FILE_PATH, "./wpa_alternatives.txt").
-define(SUPPLICANT_PATH, "./wpa_supplicant.conf").

start_link() ->
    gen_server:start_link({local, ?MODULE}, ?MODULE, [], []).

next_network() ->
    gen_server:call(?MODULE, next).


init([]) ->
    {ok, 1}.

handle_call(next, _From, Idx) ->
    case file:read_file(?FILE_PATH) of
        {ok, Binary} ->
            Lines = string:tokens(binary_to_list(Binary), "\n"),
            Total = length(Lines),
            case Total of
                0 ->
                    io:format("[HERA_NETWORK] wpa_alternatives.txt is empty~n"),
                    {reply, {error, empty_file}, Idx};
                _ ->
                    RealIdx = ((Idx - 1) rem Total) + 1,
                    Line = lists:nth(RealIdx, Lines),
                    {Ssid, Pskey} = parse_info(Line),
                    io:format("[HERA_NETWORK] Switching to network:~n    SSID : ~p~n    PSKEY: ~p~n", [Ssid, Pskey]),
                    modify_supplicant(Ssid, Pskey),
                    if 
                        RealIdx == Total -> 
                            NextIdx = 1;
                        true -> 
                            NextIdx = RealIdx + 1 
                    end,
                    {reply, ok, NextIdx}
            end;
        {error, Reason} ->
            io:format("[HERA_NETWORK] Error reading ~s: ~p~n", [?FILE_PATH, Reason]),
            {reply, {error, Reason}, Idx}
    end;

handle_call(_Request, _From, State) ->
    {reply, {error, unsupported}, State}.


parse_info(Line) when is_list(Line) ->
    case string:tokens(Line, ",") of
        [Ssid, Pskey] ->
            {Ssid, string:trim(Pskey)};
        Tokens ->
            io:format("[HERA_NETWORK] Error: wrong format in line ~p~n", [Tokens]),
            {"", ""}
    end.

modify_supplicant(Ssid, Pskey) ->
    NewContent = "network={\n" ++ "\s\s\s\sssid=\"" ++ Ssid ++ "\"\n" ++ "\s\s\s\skey_mgmt=WPA-PSK\n" ++ "\s\s\s\spsk=\"" ++ Pskey ++ "\"\n" ++"}\n",
    case file:write_file(?SUPPLICANT_PATH, NewContent) of
        ok ->
            case hera_network_nif:restart_wifi() of
                ok ->
                    ok;
                {error, Reason} ->
                    io:format("[HERA_NETWORK] Error while restarting wifi : ~p~n", [Reason])
            end;
        {error, Reason} ->
            io:format("[HERA_NETWORK] Error writing wpa_supplicant.conf: ~p~n", [Reason]),
            error
    end.

handle_cast(_Msg, State) ->
    {noreply, State}.

handle_info(_Info, State) ->
    {noreply, State}.

terminate(_Reason, _State) ->
    ok.

code_change(_OldVsn, State, _Extra) ->
    {ok, State}.
