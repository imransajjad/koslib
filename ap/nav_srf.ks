
GLOBAL AP_NAV_SRF_ENABLED IS TRUE.
local PARAM is readJson("1:/param.json")["AP_NAV"].

// glimits
local ROT_GNOM_VERT is get_param(PARAM,"ROT_GNOM_VERT",1.5).
local ROT_GNOM_LAT is get_param(PARAM,"ROT_GNOM_LAT",0.1).
local ROT_GNOM_LONG is get_param(PARAM,"ROT_GNOM_LONG",1.0).
local MIN_SRF_RAD is get_param(PARAM,"MIN_SRF_RAD",250).

local TAIL_STRIKE is get_param(PARAM,"TAIL_STRIKE").
local VSET_MAX is get_param(PARAM,"VSET_MAX").
local GEAR_HEIGHT is get_param(PARAM,"GEAR_HEIGHT").

local GOT_USE_UTIL_WP is false.
local USE_UTIL_WP is false.

// local lock AG to AG3.

local VSET_MAN is FALSE.
local MIN_NAV_SRF_VEL is 0.01.


// handles a surface type waypoint
function ap_nav_srf_wp_guide {
    parameter wp.

    local final_radius is 100.
    if wp:haskey("nomg") {
        set final_radius to max(MIN_SRF_RAD, (wp["vel"])^2/(wp["nomg"]*g0)).
    } else {
        set final_radius to max(MIN_SRF_RAD, (wp["vel"])^2/(ROT_GNOM_VERT*g0)).
    }

    local align_data is list().
    if wp:haskey("lat") {
        local wp_vec is
            latlng(wp["lat"],wp["lng"]):altitudeposition(wp["alt"]+
            (choose GEAR_HEIGHT if GEAR else 0)).
        set AP_NAV_TIME_TO_WP to
            (ship:body:radius+ship:altitude)*DEG2RAD*
            haversine(ship:geoposition:lat,ship:geoposition:lng, wp["lat"],wp["lng"])[1]
            /max(1,ship:airspeed).
        
        if wp:haskey("elev") and (wp_vec:mag < 9*final_radius) { 
            // do final alignment
            ap_nav_check_done(wp_vec, heading(wp["head"],wp["elev"]), ship:velocity:surface, final_radius).
            set align_data to ap_nav_align(wp_vec, heading(wp["head"],wp["elev"]), ship:velocity:surface, final_radius).
        } else {
            // do q_follow for height and wp heading
            ap_nav_check_done(wp_vec, ship:facing, ship:velocity:surface, final_radius).
            set align_data to ap_nav_q_target(wp["alt"],wp["vel"],latlng(wp["lat"],wp["lng"]):heading).
        }
    } else {
        // do q_follow for height and current heading
        set align_data to ap_nav_q_target(wp["alt"],wp["vel"],vel_bear).
        set AP_NAV_TIME_TO_WP to 0.
    }
    return list(max(MIN_NAV_SRF_VEL,wp["vel"])*align_data[0], align_data[1], ship:facing).
}

function ap_nav_srf_stick {
    parameter u0. // throttle input
    parameter u1. // pitch input
    parameter u2. // yaw input
    parameter u3. // roll input

    local increment is 0.0.

    if AP_MODE_PILOT {
        set AP_NAV_VEL to ship:velocity:surface.
        set VSET_MAN to false.
    } else if AP_MODE_VEL {
        // set AP_NAV_VEL to ship:velocity:surface.
        set VSET_MAN to true.
    } else if AP_MODE_NAV {
        set increment to 2.0*deadzone(u1,0.25).
        if increment <> 0 {
            set AP_NAV_VEL to ANGLEAXIS(-increment,ship:facing:starvector)*AP_NAV_VEL.
        }
        set increment to 2.0*deadzone(u3,0.25).
        if increment <> 0 {
            set AP_NAV_VEL to ANGLEAXIS(increment,ship:facing:topvector)*AP_NAV_VEL.
            // if increment > 0 and wrap_angle(AP_NAV_H_SET - vel_bear) > 175 {
            // } else if increment < 0 and wrap_angle(AP_NAV_H_SET - vel_bear) < -175 {
            // } else {
            //     set AP_NAV_H_SET To wrap_angle(AP_NAV_H_SET + increment).
            // }
        }
        set VSET_MAN to true.
    }
    if VSET_MAN AND is_active_vessel() {
        set increment to 2.7*deadzone(2*u0-1,0.1).
        if increment <> 0 {
            set AP_NAV_VEL To MIN(MAX(AP_NAV_VEL:mag+increment,MIN_NAV_SRF_VEL),VSET_MAX)*AP_NAV_VEL:normalized.
        }
        set V_SET_DELTA to increment.
    }
}

local V_SET_DELTA is 0.
function ap_nav_srf_status_string {
    local dstr is "".

    local py_temp is pitch_yaw_from_dir(AP_NAV_VEL:direction).
    local AP_NAV_E_SET is py_temp[0].
    local AP_NAV_H_SET is py_temp[1].
    local AP_NAV_V_SET is AP_NAV_VEL:mag.

    if not GOT_USE_UTIL_WP {
        set USE_UTIL_WP to (defined UTIL_WP_ENABLED).
        set GOT_USE_UTIL_WP to true.
    }

    if AP_MODE_NAV or AP_MODE_VEL or (USE_UTIL_WP and (util_wp_queue_length() > 0)) {
        set dstr to "/"+round_dec(AP_NAV_V_SET,0).
        if (V_SET_DELTA > 0){
            set dstr to dstr + "+".
        } else if (V_SET_DELTA < 0){
            set dstr to dstr + "-".
        }
        set V_SET_DELTA to 0.
        set dstr to dstr+char(10)+"("+round_dec(AP_NAV_E_SET,2)+","+round(AP_NAV_H_SET)+")".
    }
    return dstr.
}
