
global UTIL_PHYS_ENABLED is true.

local PARAM is get_param(readJson("param.json"),"UTIL_PHYS", lexicon()).

// Load saved settings
if exists("phys-settings.json") {
    local PREV_SETTINGS is readJson("phys-settings.json").
    for key in PREV_SETTINGS:keys {
        set PARAM[key] to PREV_SETTINGS[key].
        print "setting " + key +" to "+ PREV_SETTINGS[key].
    }
}

local C_SCHED_TYPE is get_param(PARAM, "C_SCHED_TYPE", "FS3T").

local A_ffactor is get_param(PARAM, "FORGET_FACTOR_AREA", 0.9).
local MOI_ffactor is get_param(PARAM, "FORGET_FACTOR_MOI", 0.9999).

local A_wing is get_param(PARAM, "WING_AREA", 20).
local A_fues is get_param(PARAM, "FUES_AREA", 2).
local MOI is V(
    get_param(PARAM, "MOI_XX", 1),
    get_param(PARAM, "MOI_YY", 1),
    get_param(PARAM, "MOI_ZZ", 1)
). // V(xx, yy, zz)

local AERO_ADAPT is get_param(PARAM, "AERO_ADAPT", true).
local MOI_ADAPT is get_param(PARAM, "MOI_ADAPT", true).
local DISPLAY_ON is get_param(PARAM, "DISPLAY_ON", true).


local function settings_save {
    set PARAM["WING_AREA"] to A_wing.
    set PARAM["FUES_AREA"] to A_fues.
    set PARAM["MOI_XX"] to MOI:x.
    set PARAM["MOI_YY"] to MOI:y.
    set PARAM["MOI_ZZ"] to MOI:z.
    set PARAM["AERO_ADAPT"] to AERO_ADAPT.
    set PARAM["MOI_ADAPT"] to MOI_ADAPT.
    set PARAM["DISPLAY_ON"] to DISPLAY_ON.
    writeJson(PARAM, "phys-settings.json").
}

// global lock GRAV_ACC to -(ship:body:mu/((ship:altitude + ship:body:radius)^2))*ship:up:forevector.
global lock GRAV_ACC to ship:body:mu/(ship:body:position:mag^2)*(ship:body:position:normalized).

// initialize/assign schedule functions
if C_SCHED_TYPE = "FS3T" {
    set cl_sched_assigned to cl_sched_fs3t@.
    set cd_sched_assigned to cd_sched_fs3t@.
} else if C_SCHED_TYPE = "ASM" {
    set cl_sched_assigned to cl_sched_fs3t@.
    set cd_sched_assigned to cd_sched_fs3t@.
}

local function cl_sched_fs3t {
    parameter vel.

    if ( vel < 100) {
        return -(5/100)*vel + 8.5.
    } else if (vel < 300) {
        return -(2.5/200)*vel + 4.75.
    } else if (vel < 2100) {
        return -(0.2/700)*vel + 1.09.
    } else {
        return 0.49.
    }
}

local function cd_sched_fs3t {
    parameter vel.

    if (vel < 50) {
        return 1.0.
    } else if ( vel < 100) {
        return -(0.5/50)*vel + 1.5.
    } else if (vel < 300) {
        return 0.5.
    } else if (vel < 400) {
        return (1.0/100)*vel - 2.5.
    } else if (vel < 500) {
        return -(0.3/100)*vel + 2.7.
    } else {
        return 1.2.
    }
}


function cl_sched {
    parameter vel.
    return cl_sched_assigned:call(vel).
}

function cd_sched {
    parameter vel.
    return cd_sched_assigned:call(vel).
}


local Tlast is 0.
local Vlast is V(0,0,0).
local Olast is V(0,0,0).
local acc_now is V(0,0,0).
local ang_acc_now is V(0,0,0).
local acclast is V(0,0,0).
local jerk_now is V(0,0,0).
function get_acc {
    if time:seconds-Tlast > 0.0 {
        set acc_now to 0.5*(ship:velocity:orbit-Vlast)/(time:seconds-Tlast) + 0.5*acc_now.
        set ang_acc_now to 0.5*(ship:angularvel-Olast)/(time:seconds-Tlast) + 0.5*ang_acc_now.
        set jerk_now to 0.5*(acc_now-acclast)/(time:seconds-Tlast) + 0.5*jerk_now.
        set Vlast to ship:velocity:orbit.
        set Olast to ship:angularvel.
        set acclast to acc_now.
        set Tlast to time:seconds.
    }
    // set get_acc_vec1 to VECDRAW(V(0,0,0), (ship:angularvel), RGB(0,0,1),
    //     "", 1.0, true, 0.25, true ).
    return acc_now.
}

function get_jerk {
    get_acc().
    return jerk_now.
}

function get_applied_acc {
    return get_acc() - GRAV_ACC.
}

function get_aero_acc {
    if defined AP_AERO_ENGINES_ENABLED {
        return get_acc() - GRAV_ACC - ap_aero_engines_get_current_thrust()/ship:mass.
    } else if defined AP_ME_ENABLED {
        return get_acc() - GRAV_ACC - ship:facing*ap_me_get_thrust()/ship:mass.
    } else {
        return get_acc() - GRAV_ACC.
    }
}

function get_frame_accel_orbit {
    // returns a force that if subtracted from the ship
    // will result in a constant height in SOI
    return GRAV_ACC + ship:up:vector*
    ((VECTOREXCLUDE(ship:up:vector,ship:velocity:orbit):mag^2
        /(ship:altitude+ship:body:radius))).
}

function get_frame_accel {
    // if the negative of this value is applied to ship
    // it will always move in a straight line in sidereal frame

    return ship:up:vector*(-1.0*g0).
}

function get_max_applied_acc {
    return abs_max(get_applied_acc()*ship:facing:topvector, get_applied_acc()*ship:facing:forevector).
}

function get_angular_acc {
    get_acc().
    return ang_acc_now.
}

// RLS matrix elements
local A_11 is 1.
local A_21_12 is 0.
local A_22 is 1.
local function aero_rls_update {

    // Do a Recursive Least Square filter to estimate two values
    // Fueselage Area and Wing Area that minimize the cumulative vector error
    // e = ( A_fues*e_fues + A_wing*e_wing ) - aero_forces
    // e_fues is a a vector pointing in the direction of fueselage aero force
    // e_wing is a a vector pointing in the direction of wing aero force
    local e_fues is ship:q/ship:mass*V(0, -cl_sched(ship:airspeed)*sin(alpha)*cos(alpha), -cd_sched(ship:airspeed)*cos(alpha)^2).
    local e_wing is ship:q/ship:mass*V(0, cl_sched(ship:airspeed)*sin(alpha)*cos(alpha), -cd_sched(ship:airspeed)*sin(alpha)^2).

    local e_11 is e_fues*e_fues.
    local e_21_12 is e_fues*e_wing.
    local e_22 is e_wing*e_wing.

    local vel_acc is (-ship_vel_dir)*get_aero_acc().
    local e_fues_y is e_fues*vel_acc.
    local e_wing_y is e_wing*vel_acc.

    set A_11 to A_ffactor*A_11 + e_11.
    set A_21_12 to A_ffactor*A_21_12 + e_21_12.
    set A_22 to A_ffactor*A_22 + e_22.

    local disc is (A_11*A_22 - A_21_12^2).
    local diff1 is e_fues_y - e_11*A_fues - e_21_12*A_wing.
    local diff2 is e_wing_y - e_21_12*A_fues - e_22*A_wing.

    set A_fues to A_fues + (A_22*diff1 - A_21_12*diff2)/disc.
    set A_wing to A_wing + (-A_21_12*diff1 + A_11*diff2)/disc.

    if false {
        local error is (-ship_vel_dir)*get_aero_acc() - ( A_fues*e_fues + A_wing*e_wing).
        util_hud_push_left("get_pre_aero_acc", "eA " + round_dec(error:x,2) + "," + round_dec(error:y,2) + "," + round_dec(error:z,2) +
            char(10) + "A f/w " + round_dec(A_fues,0) + "/" + round_dec(A_wing,0)).
        set phys_debug_vec0 to VECDRAW((ship_vel_dir*( A_fues*e_fues + A_wing*e_wing)), (ship_vel_dir*error), RGB(1,0,0),
                    "", 1.0, true, 0.125, true ).
        set phys_debug_vec1 to VECDRAW(V(0,0,0), (ship_vel_dir*( A_fues*e_fues + A_wing*e_wing)), RGB(0,1,1),
                    "", 1.0, true, 0.125, true ).
        set phys_debug_vec2 to VECDRAW(V(0,0,0), (ship_vel_dir*(A_fues*e_fues)), RGB(0,0.5,1),
                    "", 1.0, true, 0.125, true ).
        set phys_debug_vec3 to VECDRAW( (ship_vel_dir*(A_fues*e_fues)), (ship_vel_dir*(A_wing*e_wing)), RGB(0.5,1.0,0),
                    "", 1.0, true, 0.125, true ).
    }
}

function get_wing_area {
    return A_wing.
}

function get_fues_area {
    return A_fues.
}

function get_pre_aero_acc {
    parameter vel is ship:airspeed.
    parameter a is alpha.
    parameter dpres is ship:q.
    parameter m is ship:mass.

    local e_fues is dpres/m*V(0, -cl_sched(vel)*sin(a)*cos(a), -cd_sched(vel)*cos(a)^2).
    local e_wing is dpres/m*V(0, cl_sched(vel)*sin(a)*cos(a), -cd_sched(vel)*sin(a)^2).

    return ship_vel_dir*( A_fues*e_fues + A_wing*e_wing).
}

function get_sus_turn_rate {
    parameter vel is ship:airspeed.
    parameter a is alpha.
    parameter dpres is ship:q.
    parameter m is ship:mass.

    local Tmax is (choose ap_aero_engines_get_max_thrust() if defined AP_AERO_ENGINES_ENABLED else 0 ).
    local qcl is dpres*cl_sched(vel).
    local Eta_turn is (Tmax/(dpres*cd_sched(vel)) - A_wing)/(A_fues-A_wing).
    print Eta_turn.

    if Eta_turn > 0 and Eta_turn < 1 {
        local phys_deg_per_sec is dpres*cl_sched(vel)*(A_wing-A_fues)*sqrt(Eta_turn*(1-Eta_turn))/(m*vel)*RAD2DEG.
        util_hud_push_left("get_sus_turn_rate", "phpmax " + round_dec(phys_deg_per_sec,1) ).
        return phys_deg_per_sec.
    } else {
        return 2*g0/vel*RAD2DEG.
    }
}

local moi_stage is stage:number.
local moi_cross is V(0,0,0). // V(xy, yz, xz)
local moi_mass is ship:mass.
local moi_update_acc_2 is V(0,0,0).
local function init_moi {
    // some MOI_spec calculations from scratch
    
    set MOI to V(0,0,0).
    set moi_cross to V(0,0,0).
    set moi_mass to ship:mass.

    for pt in ship:parts {

        local offset is -ship:facing*pt:position.
        local vdiag is V(offset:y^2 + offset:z^2, offset:x^2 + offset:z^2, offset:x^2 + offset:y^2).
        local vcross is V(offset:y*offset:z, offset:x*offset:z, offset:x*offset:y).

        set MOI to MOI + pt:mass*vdiag + ship:mass*V(0.05,0.05,0.05).
        set moi_cross to moi_cross + pt:mass*vcross.
    }

    print "  MOI " + round_vec(get_moment_of_inertia(),2).
}

local function moi_update {

    if SAS {
        return.
    }

    local ang_acc is (-ship:facing)*get_angular_acc(). // angular acc in ship frame
    local B is ap_get_control_bu().

    if MOI:x <= 0 or MOI:y <= 0 or MOI:z <= 0 {
        init_moi().
    }

    if moi_stage <> stage:number {
        set moi_stage to stage:number.
        set MOI to MOI*ship:mass/moi_mass.
    }
    set moi_mass to ship:mass.

    local dI is V(0,0,0).
    if abs(1.03*ang_acc:x) > ang_acc:mag and ang_acc:mag > 0.05*B:x/MOI:x {
        set moi_update_acc_2:x to MOI_ffactor*moi_update_acc_2:x + ang_acc:x^2.
        set dI:x to (-ang_acc:x*B:x*ship:control:pitch - MOI:x*ang_acc:x^2)/moi_update_acc_2:x.
    }
    if abs(1.03*ang_acc:y) > ang_acc:mag and ang_acc:mag > 0.05*B:y/MOI:y {
        set moi_update_acc_2:y to MOI_ffactor*moi_update_acc_2:y + ang_acc:y^2.
        set dI:y to (ang_acc:y*B:y*ship:control:yaw - MOI:y*ang_acc:y^2)/moi_update_acc_2:y.
    }
    if abs(1.03*ang_acc:z) > ang_acc:mag and ang_acc:mag > 0.05*B:z/MOI:z {
        set moi_update_acc_2:z to MOI_ffactor*moi_update_acc_2:z + ang_acc:z^2.
        set dI:z to (-ang_acc:z*B:z*ship:control:roll - MOI:z*ang_acc:z^2)/moi_update_acc_2:z.
    }
    set MOI to MOI + dI.

}

function get_moment_of_inertia {
    return V(MOI:x,MOI:y,MOI:z).
    // return V(0.412,0.412,0.291).
}

function util_phys_update {

    local do_update is false.

    if AERO_ADAPT and ship:q > 0.0003 and not BRAKES and alpha > 0 and alpha < 45 and get_jerk():mag < 1.5 {
        aero_rls_update().
        set do_update to true.
    }
    if MOI_ADAPT and defined AP_ORB_ENABLED {
        moi_update().
        set do_update to true.
    }

    if do_update {
        display_udpate().
    }
}

local function display_udpate {
    if not (defined UTIL_HUD_ENABLED) {
        return.
    }
    if DISPLAY_ON {
        util_hud_push_left("util_phys",
            char(10) + "A_wing " + round_fig(A_wing,3) +
            char(10) + "A_fues " + round_fig(A_fues,3) +
            char(10) + "MOI:x " + round_fig(MOI:x,3) +
            char(10) + "MOI:y " + round_fig(MOI:y,3) +
            char(10) + "MOI:z " + round_fig(MOI:z,3)).
    } else {
        util_hud_pop_left("util_phys").
    }
}

// shbus_tx compatible send messages
// TX_SECTION

function util_phys_get_help_str {
    return list(
        "UTIL_PHYS running on "+core:tag,
        "phys load",
        "phys commit",
        "phys reset",
        "phys aero set (A_wing,A_fues)",
        "phys moi set (Ix,Iy,Iz)",
        "phys aero get",
        "phys moi get",
        "phys [setting] [on/off]",
        "     aero",
        "     moi",
        "     display",
        "phys help           print help"
        ).
}

function util_phys_parse_command {
    parameter commtext.
    parameter args is list().

    if commtext:startswith("phys ") {
        set commtext to commtext:remove(0,5).
    } else {
        return false.
    }
    local word_0 is "".
    local word_1 is "".
    local words is commtext:split(" ").
    if words:length = 2 {
        set word_0 to words[0]. // reset or moi/aero/display
        set word_1 to words[1]. // on/off/get/set
    }

    if commtext = "load" {
        util_shbus_tx_msg("PHYS_LOAD").
    } else if commtext = "reset" {
        util_shbus_tx_msg("PHYS_RESET",list()).
    } else if word_1 = "on" or word_1 = "off" {
        if words:length > 2 {
            print "usage: phys [setting] [on/off]".
        } else {
            util_shbus_tx_msg("PHYS_SETTING_SET", list(word_0, (word_1 = "on") ) ).
        }
    } else if word_1 = "get" {
        if word_0 = "aero" {
            util_shbus_tx_msg("PHYS_AERO_GET").
        } else if word_0 = "moi" {
            util_shbus_tx_msg("PHYS_MOI_GET").
        } else {
            print "usage: phys aero get".
            print "       phys moi get".
        }
    } else if word_1 = "set" and all_scalar(args) {
        if word_0 = "aero" and args:length = 2 {
            util_shbus_tx_msg("PHYS_AERO_SET", args).
        } else if word_0 = "moi" and args:length = 3 {
            util_shbus_tx_msg("PHYS_MOI_SET", args ).
        } else {
            print "usage: phys aero set (A_wing,A_fues)".
            print "       phys moi set (Ix,Iy,Iz)".
        }
    } else if commtext = "commit" {
        util_shbus_tx_msg("PHYS_COMMIT").
    } else if commtext = "help" {
        util_term_parse_command("help PHYS").
    } else {
        return false.
    }
    return true.
}

// TX_SECTION END


// RX SECTION

// shbus_rx compatible receive message
function util_phys_decode_rx_msg {
    parameter sender.
    parameter recipient.
    parameter opcode.
    parameter data.

    if not opcode:startswith("PHYS") {
        return false.
    }

    if opcode = "PHYS_RESET" {
        deletepath("phys-settings.json").
    } else if opcode = "PHYS_LOAD" {
        set A_wing to get_param(PARAM, "WING_AREA", 20).
        set A_fues to get_param(PARAM, "FUES_AREA", 2).
        set MOI to V(
            get_param(PARAM, "MOI_XX", 1),
            get_param(PARAM, "MOI_YY", 1),
            get_param(PARAM, "MOI_ZZ", 1)
        ). // V(xx, yy, zz)
        set AERO_ADAPT to get_param(PARAM, "AERO_ADAPT", true).
        set MOI_ADAPT to get_param(PARAM, "MOI_ADAPT", true).
        set DISPLAY_ON to get_param(PARAM, "DISPLAY_ON", true).

    } else if opcode = "PHYS_SETTING_SET" {
        if data[0] = "aero" {
            set AERO_ADAPT to data[1].
        } else if data[0] = "moi" {
            set MOI_ADAPT to data[1].
        } else if data[0] = "display" {
            set DISPLAY_ON to data[1].
        }
    } else if opcode = "PHYS_AERO_GET" {
        util_shbus_ack("A_wing " + round_fig(A_wing,3) + ",A_fues " + round_fig(A_fues,3), sender).
    } else if opcode = "PHYS_MOI_GET" {
        util_shbus_ack("moi " + round_vec(MOI,3), sender).
    } else if opcode = "PHYS_AERO_SET" {
        set A_wing to data[0].
        set A_fues to data[1].
        util_shbus_ack("A_wing " + round_fig(A_wing,3) + ",A_fues " + round_fig(A_fues,3), sender).
    } else if opcode = "PHYS_MOI_SET" {
        set MOI:x to data[0].
        set MOI:y to data[1].
        set MOI:z to data[2].
        util_shbus_ack("moi " + round_vec(MOI,3), sender).
    } else if opcode = "PHYS_COMMIT" {
        settings_save().
    } else {
        util_shbus_ack("could not decode phys rx msg", sender).
        print "could not decode phys rx msg".
        return false.
    }
    display_udpate().
    return true.
}

// RX SECTION END
