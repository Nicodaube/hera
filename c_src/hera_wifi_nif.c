#include <erl_nif.h>
#include <grisp/init.h>
#include <wlan.h>

static ERL_NIF_TERM
restart_wifi_nif(ErlNifEnv* env, int argc, const ERL_NIF_TERM argv[])
{
    extern const char *wpa_supplicant_conf;
    if (!wpa_supplicant_conf) {
        return enif_make_tuple2(env,
            enif_make_atom(env,"error"),
            enif_make_string(env,"no_conf_loaded",ERL_NIF_LATIN1));
    }

    /* call into the BSP init routines */
    grisp_init_wpa_supplicant(wpa_supplicant_conf, PRIO_WPA, create_wlandev);
    grisp_init_dhcpcd(PRIO_DHCP);

    return enif_make_atom(env, "ok");
}

static ErlNifFunc nif_funcs[] = {
    {"restart_wifi", 0, restart_wifi_nif}
};

ERL_NIF_INIT(sensor_nif, nif_funcs, NULL, NULL, NULL, NULL);
