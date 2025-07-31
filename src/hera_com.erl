-module(hera_com).

-export([start_link/0]).
-export([send/3, send/4, send_unicast/3, add_device/3, reset_devices/0]).
-export([encode_half_float/1,decode_half_float/1]).
-export([get_bits/1]).

-define(MULTICAST_ADDR, {239,255,0,1}).
-define(PORT, 9000).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_link() ->
    persistent_term:put(devices, []),
    Pid = spawn_link(fun init/0),
    register(?MODULE, Pid),
    {ok, Pid}.


-spec send(Name, Seq, Values) -> ok when
    Name :: atom(),
    Seq :: pos_integer(),
    Values :: [number(), ...].

send(Name, Seq, Values) ->
    Message = {hera_data, Name, node(), Seq, Values},
    try ?MODULE ! {send_packet, term_to_binary(Message)}
    catch
        error:_ -> ok
    end,
    ok.

send(Name, Seq, From, Values) ->
    % To use when wanting to use personalized naming
    Message = {hera_data, Name, From, Seq, Values},
    try ?MODULE ! {send_packet, term_to_binary(Message)}
    catch
        error:_ -> ok
    end,
    ok.

send_unicast(Name, Message, Type) ->
    hera:logg("[HERA_COM] Sending ~p to ~p~n", [Message, Name]),
    NewMessage = case Type of
        "UTF8" -> Message;
        "Binary" -> term_to_binary(Message);
        _ -> Message
    end,
    ?MODULE ! {send_packet_unicast, Name, NewMessage},
    ok.
    

%
%Returns a list of each bit in the byte given by Byte
%e.g. Byte = 163 gives [true, false, true, false, false, false, true, true]
%
get_bits(Byte) ->
    if
        Byte =/= 255 ->
            _ = [ (Byte band round(math:pow(2,X))) =/= 0 || X <- [7,6,5,4,3,2,1,0]];
        true ->
            [false, false, false, false, false, false, false, false]
    end.

%
%Encodes a list of values from double (8 bytes) to half-float (2 bytes)
%Values = [Double1,Double2,...]
%
encode_half_float(Values) -> 
	lists:map(fun(X) -> enc_hf(X) end, Values).

%
%Decodes a list of values from half-float (2 bytes) to double (8 bytes)
%Values = [<<Hf1A, Hf1B>>, <<Hf2A, Hf2B>>, ...]
%e.g. Values = [<<16#43,16#0A>>, <<16#4B,16#0C>>] gives [3.52, 14.1]
%
decode_half_float(Values) when is_list(Values) -> decode_half_float(Values, []). 
decode_half_float([<<A:8, B:8>> | Rest], Acc) -> decode_half_float(Rest, Acc ++ [dec_hf(<<A, B>>)]); 
decode_half_float([], Acc) -> Acc.
 

add_device(Name, Ip, Port) ->
    Devices = persistent_term:get(devices),
    case lists:member({Name, Ip, Port}, Devices) of
        false ->
            hera:logg("[HERA_COM] Discovered new device : ~p~n", [Name]),
            NewDevices = [{Name, Ip, Port} | Devices],
            persistent_term:put(devices, NewDevices);
        _ ->
            ok
    end.
    
reset_devices() ->
    persistent_term:put(devices, []).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init() ->
    Socket = open_socket(1, 1),
    Propag = persistent_term:get(gossip_propagation),
    loop(Socket, Propag).

open_socket(Delay, Attempts) ->
    try open_socket()
    catch
        error:Reason ->
            hera:logg("[HERA_COM] Could not open socket: ~p~n", [Reason]),
            hera:logg("[HERA_COM] Retrying in ~p [s], attempt number ~p~n", [Delay, Attempts]),
            timer:sleep(Delay*1000),
            open_socket(min(2*Delay, 8), Attempts+1)
    end.


open_socket() ->
    {ok, Addrs} = inet:getifaddrs(),
    Ipaddr = hd([
        Addr || {_, Opts} <- Addrs, {addr, Addr} <- Opts,
        size(Addr) == 4, Addr =/= {127,0,0,1}
    ]),
    {ok, Socket} = gen_udp:open(?PORT, [binary, {active, true}, {reuseaddr, true}]),

    case Ipaddr of
        {192, _, _, _} ->
            hera:logg("[HERA_COM] Connected to private network with IP: ~p~n", [Ipaddr]),       
            persistent_term:put(multicast, false);
        {10, _, _, _} ->
            hera:logg("[HERA_COM] Connected to private network with IP: ~p~n", [Ipaddr]),
            persistent_term:put(multicast, false);
        {172, _, _, _} ->
            hera:logg("[HERA_COM] Connected to mobile hotspot (unicast only) IP: ~p~n", [Ipaddr]),
            persistent_term:put(multicast, false);
        _ ->
            hera:logg("[HERA_COM] Unknown network IP: ~p~n", [Ipaddr]),
            persistent_term:put(multicast, false)
    end,
    hera_subscribe:notify("connected"),
    Socket.

loop(Socket, Propag) ->
    receive
        {udp, _Sock, _IP, _InPortNo, Packet} ->
            case catch binary_to_term(Packet) of
                {'EXIT', _} ->
                    handle_string_packet(binary_to_list(Packet));
                {hera_data, Name, From, Seq, Values} -> 
                    case Propag of
                        true ->
                            case hera_data:is_new_data(Name, From, Seq) of
                                true ->
                                    hera_data:store(Name, From, Seq, Values),
                                    gossip_loop({hera_data, Name, From, Seq, Values}); 
                                false ->
                                    ok
                            end;
                        _ ->
                            hera_data:store(Name, From, Seq, Values)
                    end
            end,
            loop(Socket, Propag);
        {send_packet, Packet} ->
            Multicast_enabled = persistent_term:get(multicast),
            if 
                Multicast_enabled ->
                    gen_udp:send(Socket, ?MULTICAST_ADDR, ?PORT, Packet);
                true ->
                    [gen_udp:send(Socket, IP, Port, Packet) || {_, IP, Port} <- persistent_term:get(devices)]
            end,
            loop(Socket, Propag);
        {send_packet_unicast, Name, Packet} ->
            Devices = persistent_term:get(devices),
            SelectedDevice = lists:keyfind(Name, 1, Devices),
            case SelectedDevice of
                false -> hera:logg("[HERA_COM] Unregistered Device : ~p~n", [Name]);
                {_, IP, Port} -> gen_udp:send(Socket, IP, Port, Packet)
            end,
            loop(Socket, Propag);             
        _ ->
            loop(Socket, Propag)
    after 5000 ->
        case test_connection() of
            ok ->
                loop(Socket, Propag);
            error ->
                New_Socket = restart(Socket),
                loop(New_Socket, Propag)
        end
    end.
    
test_connection() ->
    {ok, Addrs} = inet:getifaddrs(),
    case lists:keyfind("wlan0", 1, Addrs) of
        {"wlan0", Fields} ->
            case proplists:get_value(broadaddr, Fields, none) of
                none ->
                    error;
                _BroadAddr ->
                    ok             
            end;
        _ ->
            error
    end.
                    
    
restart(Socket) ->
    hera:logg("[HERA_COM] Lost connection~n", []),
    gen_udp:close(Socket),
    hera_subscribe:notify("disconnected"),
    open_socket(1, 1).  

handle_string_packet(String) ->
    Tokens = string:tokens(String, ": ,"),
    hera_subscribe:notify(Tokens).

%Encodes one value from a double to a half-float
enc_hf(Double) ->
	<<_,_,A,B,C,_,_,_,_,_>>=term_to_binary(Double),
	A2 = (A band 192) bor ((B bsr 2) band 63),
	B2 = ((B bsl 6) band 192) bor ((C bsr 2) band 63),
	<<A2,B2>>.

%Decodes one value from a half-float to a double
dec_hf(<<0,0>>) -> 0.0;
dec_hf(Half_Float) ->
	<<X,Y>> = Half_Float,	
	if
		(X band 64) == 0 ->
			A = (X band 192) bor 63;
		true ->
			A = (X band 192)
	end,
	B = ((X bsl 2) band 252) bor ((Y bsr 6) band 3),
	C = ((Y bsl 2) band 252),
	binary_to_term(<<131,70,A,B,C,0,0,0,0,0>>).

gossip_loop({hera_data, Name, Seq, From, Values}) ->
    send(Name, Seq, From, Values).