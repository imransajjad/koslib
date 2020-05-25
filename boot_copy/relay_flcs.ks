
function has_connection_to_base {
    if addons:available("RT") {
        return addons:RT:AVAILABLE AND addons:RT:HASKSCCONNECTION(SHIP).
    } else {
        return true.
    }
    return false.
}

function spin_if_not_us {
    until (SHIP_NAME_ACRO_IN_PARAMS = string_acro(ship:name) ) {
        wait 1.0.
    }
}

WAIT UNTIL SHIP:LOADED.
wait 0.25. // so that connection is established if possible.

global DEV_FLAG is false.


if (DEV_FLAG or not exists("param.json")) and has_connection_to_base() {
    COPYPATH("0:/koslib/util/common.ks","util_common").
    run once "util_common".
    COPYPATH("0:/param/"+string_acro(ship:name)+".json","param.json").
    
    COPYPATH("0:/koslib/util/fldr.ks","util_fldr").
    COPYPATH("0:/koslib/util/shbus.ks","util_shbus").
    COPYPATH("0:/koslib/util/shsys.ks","util_shsys").

    COPYPATH("0:/koslib/ap/nav.ks","ap_nav").
    COPYPATH("0:/koslib/ap/mode.ks","ap_mode").
    print "loaded resources from base".
}
run once "util_common".
global SHIP_NAME_ACRO_IN_PARAMS is
        readJson("1:/param.json")["ship_name_acro"].
spin_if_not_us().

LOCK vel TO (choose SHIP:AIRSPEED if ship:altitude < 36000 else SHIP:VELOCITY:ORBIT:mag).

LOCK DELTA_PRO_UP TO R(90,0,0)*(-SHIP:UP)*
    (choose SHIP:SRFPROGRADE if ship:altitude < 36000 else SHIP:PROGRADE).
LOCK vel_pitch TO (mod(DELTA_PRO_UP:pitch+90,180)-90).
LOCK vel_bear TO (360-DELTA_PRO_UP:yaw).

run once "util_common".
run once "util_shbus".
run once "util_shsys".

run once "ap_mode".
run once "ap_nav".

GLOBAL BOOT_RCOM_ENABLED IS true.

until false {
    spin_if_not_us().
    util_shbus_rx_msg().
    util_shsys_check().

    ap_mode_update().
    if not AP_MODE_NAV and not has_connection_to_base(){
        ap_mode_set("NAV").
    }

    if AP_MODE_PILOT {
        unlock THROTTLE.
        SET SHIP:CONTROL:NEUTRALIZE to true.
    } else if AP_MODE_NAV {
        ap_nav_do().
    } else {
        unlock THROTTLE.
        SET SHIP:CONTROL:NEUTRALIZE to true.
    }
    wait 0.02.
}
