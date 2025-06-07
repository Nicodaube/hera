-module(hera_network_nif).
-on_load(init/0).
-export([restart_wifi/0]).

init() ->
    Priv = code:priv_dir(hera),
    So   = filename:join(Priv, "hera_wifi.so"),
    ok = erlang:load_nif(So, 0).

restart_wifi() ->
    erlang:nif_error("NIF hera_network_nif:restart_wifi/0 not loaded").
