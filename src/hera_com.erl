-module(hera_com).

-export([start_link/0]).
-export([send/3]).
-export([encode_half_float/1,decode_half_float/1]).

-define(MULTICAST_ADDR, {224,0,2,15}).
-define(MULTICAST_PORT, 62476).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_link() ->
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

%
%Returns a list of each bit in the byte given by Byte
%e.g. Byte = 163 gives [1,0,1,0,0,0,1,1]
%
get_bits(Byte) ->
	L = [ (Byte band round(math:pow(2,X))) =/=0 || X <- [7,6,5,4,3,2,1,0]].

%
%Encodes a list of values from double (8 bytes) to half-float (2 bytes)
%Values = [Double1,Double2,...]
%
encode_half_float(Values) -> 
	lists:map(fun(X) -> enc_hf(X) end, Values).

%
%Decodes a list of values from half-float (2 bytes) to double (8 bytes)
%Values = <<Hf1A,Hf1B,Hf2A,Hf2B,...>>
%e.g. Values = <<16#43,16#0A,16#4B,16#0C>> gives [3.52,14.1] (with some approximation due to the conversion)
%
decode_half_float(Values) when is_binary(Values) -> decode_half_float(Values, []). 
decode_half_float(<<A:8, B:8, Rest/binary>>, Acc) -> decode_half_float(Rest, Acc ++ [dec_hf(<<A, B>>)]); 
decode_half_float(<<>>, Acc) -> Acc.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init() ->
    Socket = open_socket(1),
    io:format("Connection established!~n"),
    loop(Socket).


open_socket(Delay) ->
    try open_socket()
    catch
        error:Reason ->
            io:format("Could not open socket:~p~n", [Reason]),
            io:format("Retrying in ~p [s]~n", [Delay]),
            timer:sleep(Delay*1000),
            open_socket(min(2*Delay, 4))
    end.


open_socket() ->
    {ok, Addrs} = inet:getifaddrs(),
    OwnAddr = hd([
        Addr || {_, Opts} <- Addrs, {addr, Addr} <- Opts,
        size(Addr) == 4, Addr =/= {127,0,0,1}
    ]),
    {ok, Socket} = gen_udp:open(?MULTICAST_PORT, [
        binary,
        inet,
        {active, true},
        {multicast_if, OwnAddr},
        {multicast_loop, true},
        {reuseaddr, true},
        {add_membership, {?MULTICAST_ADDR, OwnAddr}}
    ]),
    Socket.


loop(Socket) ->
    receive
        {udp, _Sock, _IP, _InPortNo, Packet} ->
            Message = binary_to_term(Packet),
            case Message of
                {hera_data, Name, From, Seq, Values} ->
                    hera_data:store(Name, From, Seq, Values);
                _ ->
                    ok
            end;
        {send_packet, Packet} ->
            gen_udp:send(Socket, ?MULTICAST_ADDR, ?MULTICAST_PORT, Packet);
        _ ->
            ok
    end,
    loop(Socket).


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





