
// Required Dependencies
// UTIL_SHBUS_TX_ENABLED (for tx commands)
// UTIL_SHBUS_RX_ENABLED (for rx commands)

global UTIL_FLDR_ENABLED is true.

local lock AG to AG5.
local PREV_AG is AG5.


local Ts is 0.04.
local Tdur is 0.0.
local Tdel is 0.

local logtag to "".
local filename is "".
local logdesc is "".


local PARAM is readJson("1:/param.json").
local MAIN_ENGINE_NAME is (choose PARAM["AP_ENGINES"]["MAIN_ENGINE_NAME"]
        if PARAM:haskey("AP_ENGINES") and
        PARAM["AP_ENGINES"]:haskey("MAIN_ENGINE_NAME") else "").

local MAIN_ENGINES is get_engines(MAIN_ENGINE_NAME).

// TX SECTION

local function get_last_log_filename {
    return (choose "0" if has_connection_to_base() else "1")+":/logs/lastlog".
}

local function list_logs {
    if exists("logs"){
        cd("logs").
        LIST files.
        cd("..").
    }
}

local function send_logs {
    if NOT has_connection_to_base() {
        print "send_logs: no connection to KSC".
        return.
    }
    if exists("logs"){
        cd("logs").
        LIST files IN FLIST.
        FOR F in FLIST {
            print "send_logs: "+F.
            COPYPATH(F,"0:/logs/"+F).
            DELETEPATH(F).
        }
        cd("..").
    }
}

local function stash_last_log {
    if exists(get_last_log_filename()+".csv") {
        local i is 0.
        until not exists(get_last_log_filename()+i+".csv") {
            set i to i+1.
        }
        copypath(get_last_log_filename()+".csv",get_last_log_filename()+i+".csv").
        print "log stashed to " + get_last_log_filename()+i+".csv".
    } else {
        print "logs/lastlog.csv does not exist".
    }
}

local function print_pos_info {
    print "time " + round_dec(TIME:SECONDS,3).
    print "dur " + round_dec(Tdur,3).
    print "dt  " + round_dec(Ts,3).
    print "lat  " + ship:GEOPOSITION:LAT.
    print "lng  " + ship:GEOPOSITION:LNG.
    print "h    " + ship:ALTITUDE.
    print "vs   " + ship:AIRSPEED.
    print "engine " + MAIN_ENGINE_NAME.
    print "logtag " + logtag.
}

local function start_logging {
    local lock h to ship:ALTITUDE.
    local lock m to ship:MASS.

    local lock u0 to ship:control:PILOTMAINTHROTTLE.
    local lock u1 to ship:control:pitch.
    local lock u2 to ship:control:yaw.
    local lock u3 to ship:control:roll.

    local lock vel to ship:AIRSPEED.
    local lock pitch_rate to (-ship:ANGULARVEL*ship:FACING:STARVECTOR).
    local lock yaw_rate to (ship:ANGULARVEL*ship:FACING:TOPVECTOR).
    local lock roll_rate to (-ship:ANGULARVEL*ship:FACING:FOREVECTOR).

    local lock DELTA_FACE_UP to R(90,0,0)*(-ship:UP)*(ship:FACING).
    local lock pitch to DEG2RAD*(mod(DELTA_FACE_UP:pitch+90,180)-90).
    local lock yaw to DEG2RAD*(360-DELTA_FACE_UP:yaw).
    local lock roll to DEG2RAD*(180-DELTA_FACE_UP:roll).

    local lock DELTA_SRFPRO_UP to R(90,0,0)*(-ship:UP)*(ship:SRFPROGRADE).
    local lock vel_pitch to DEG2RAD*(mod(DELTA_SRFPRO_UP:pitch+90,180)-90).
    local lock vel_bear to DEG2RAD*(360-DELTA_SRFPRO_UP:yaw).
    
    local get_thrust is {
        local total_thrust is 0.
        for e in MAIN_ENGINES {
            set total_thrust to total_thrust+e:MAXTHRUST.
        }
        return total_thrust.
    }.
    local lock thrust to get_thrust().
    local lock thrust_vector to (get_thrust()/ship:MASS)*ship:FACING:FOREVECTOR.

    local lock dynamic_pres to ship:DYNAMICPRESSURE.


    local lock DELTA_ALPHA to R(0,0,RAD2DEG*roll)*(-ship:SRFPROGRADE)*(ship:FACING).
    local lock alpha to -DEG2RAD*(mod(DELTA_ALPHA:pitch+180,360)-180).
    local lock beta to  DEG2RAD*(mod(DELTA_ALPHA:yaw+180,360)-180).

    local lock VEL_FROM_FACE to R(0,0,RAD2DEG*roll)*(-ship:SRFPROGRADE).

    lock Aup to ship:MASS*(VEL_FROM_FACE*(ship:SENSORS:ACC-ship:SENSORS:GRAV-thrust_vector)):Y.
    lock Afore to ship:MASS*(VEL_FROM_FACE*(ship:SENSORS:ACC-ship:SENSORS:GRAV-thrust_vector)):Z.
    lock Alat to ship:MASS*(VEL_FROM_FACE*(ship:SENSORS:ACC-ship:SENSORS:GRAV-thrust_vector)):X.

    local lock lat to ship:GEOPOSITION:LAT.
    local lock lng to ship:GEOPOSITION:LNG.

    set filename to get_last_log_filename() + ".csv".

    if exists(filename) {
        deletepath(filename).
    }

    set logdesc to "log_"+string_acro(ship:NAME)+"_"+TIME:CALENDAR:replace(":","")+
            "_"+TIME:CLOCK:replace(":","")+"_"+logtag.

    set logdesc to logdesc:replace(" ", "_").
    set logdesc to logdesc:replace(",", "").
    print "log writing to "+ filename.

    log logdesc to filename.
    log "t,u0,u1,u2,u3,y0,y1,y2,y3,ft,p,y,r,vp,vh,afore,aup,alat,alpha,beta,h,m,q,lat,lng" to filename.
    log "" to filename.

    set starttime to TIME:SECONDS.
    until (TIME:SECONDS-starttime > Tdur) and (Tdur > 0) {
        log TIME:SECONDS+","+u0+","+u1+","+u2+","+u3+
            ","+vel+","+pitch_rate+","+yaw_rate+","+roll_rate+
            ","+thrust+","+pitch+","+yaw+","+roll+
            ","+vel_pitch+","+vel_bear+
            ","+Afore+","+Aup+","+Alat+
            ","+alpha+","+beta+
            ","+h+","+m+","+dynamic_pres+
            ","+lat+","+lng
             to filename.
        wait Ts.
        if check_for_stop_logging() {
            break.
        }
    }

    print "log written to "+ filename.
}

local function check_for_stop_logging {
     if AG <> PREV_AG {
        set PREV_AG to AG.
        return true.
    }
    return false.
}

function util_fldr_get_help_str {
    return list(
        " ",
        "UTIL_FLDR running on "+core:tag,
        "logtime(T)   total log time=T",
        "logdt(dt)    log interval =dt",
        "logtag TAG   set log tag",
        "log          start logging",
        "testlog      start test, log",
        "listloginfo  list logs",
        "sendlogs     send logs",
        "stashlog     save last log",
        "logstp(SEQ)  set pulse_time(SEQ)",
        "logsu0(SEQ)  set throttle_seq(SEQ)",
        "logsu1(SEQ)  set pitch_seq(SEQ)",
        "logsu2(SEQ)  set yaw_seq(SEQ)",
        "logsu3(SEQ)  set roll_seq(SEQ)",
        "logsupr.  print test sequences",
        "  SEQ = sequence of num"
        ).
}

// This function returns true if the command was parsed and Sent
// Otherwise it returns false.
function util_fldr_parse_command {
    parameter commtext.

    // don't even try if it's not a log command
    if commtext:contains("log") {
        set args to util_shbus_tx_raw_input_to_args(commtext).
        if not (args = -1) and args:length = 0 {
            print "fldr args expected but empty".
            return true.
        }
    } else {
        return false.
    }


    if commtext:startswith("logtime(") {
        set Tdur to args[0].
    } else if commtext:startswith("logdt(") {
        set Ts to args[0].
    } else if commtext:startswith("logtag") {
        if commtext:length > 7 {
            set logtag to args.
        } else {
            set logtag to "".
        }
    } else if commtext = "log" {
        start_logging().
    } else if commtext = "testlog" {
        util_shbus_tx_msg("FLDR_RUN_TEST").
        start_logging().
    } else if commtext:startswith("listloginfo") {
        print_pos_info().
        list_logs().
    } else if commtext:startswith("sendlogs") {
        send_logs().
    } else if commtext:startswith("stashlog") {
        stash_last_log().
    } else if commtext:startswith("logstp(") {
        util_shbus_tx_msg("FLDR_SET_SEQ_TP", args).
        set Tdur to listsum(args)+Tdel.

    } else if commtext:startswith("logsu0(") {
        util_shbus_tx_msg("FLDR_SET_SEQ_U0", args).
        print "Sent FLDR_SET_SEQ_U0 "+ args:join(" ").

    } else if commtext:startswith("logsu1(") {
        util_shbus_tx_msg("FLDR_SET_SEQ_U1", args).
        print "Sent FLDR_SET_SEQ_U1 "+ args:join(" ").

    } else if commtext:startswith("logsu2(") {
        util_shbus_tx_msg("FLDR_SET_SEQ_U2", args).
        print "Sent FLDR_SET_SEQ_U2 "+ args:join(" ").

    } else if commtext:startswith("logsu3(") {
        util_shbus_tx_msg("FLDR_SET_SEQ_U3", args).
        print "Sent SET_SEQ_U3 "+ args:join(" ").
    } else if commtext:startswith("logsupr") {
        util_shbus_tx_msg("FLDR_PRINT_TEST").
    } else {
        return false.
    }
    return true.
}

// TX SECTION END

// RX SECTION

set U_seq to LIST(LIST(0,0,0,0,0),LIST(0,0,0,0,0),LIST(0,0,0,0,0),LIST(0,0,0,0,0)).
set PULSE_TIMES to LIST(0,0,0,0,0).

function print_sequences {
    local pstr is "times" + char(10).
    for t in PULSE_TIMES {
        set pstr to pstr + round_dec(t,2) + " ".
    }
    set pstr to pstr + char(10) + "set sequences" + char(10).
    for U in U_seq {
        for ux in U {
            set pstr to pstr + round_dec(ux,2) + " ".
        }
        set pstr to pstr + char(10).
    }
    print pstr.
    return pstr.
}

// Run a test sequence
FUNCTION run_test_control {
    local u0_trim is ship:control:mainthrottle.
    local u1_trim is ship:control:pitch.
    local u2_trim is ship:control:yaw.
    local u3_trim is ship:control:roll.

    if NOT (U_seq[0]:length = U_seq[1]:length and 
        U_seq[0]:length = U_seq[2]:length and
        U_seq[0]:length = U_seq[3]:length and
        U_seq[0]:length = PULSE_TIMES:length) {
        print "lengths of control sequences not equal, aborting".
        return.
    }
    local N is U_seq[0]:length.

    //UNLOCK ship:control:mainthrottle.

    set INICES to LIST().
    FOR i IN RANGE(0, N, 1) {
        set ship:control:mainthrottle to u0_trim + U_seq[0][i].
        set ship:control:pitch to u1_trim + U_seq[1][i].
        set ship:control:yaw to u2_trim + U_seq[2][i].
        set ship:control:roll to u3_trim + U_seq[3][i].
        wait PULSE_TIMES[i].
        if AG <> PREV_AG {
            break.
        }
    }
    print "TEST COMPLETE, RETURNING control.".
}


// Returns true if message was decoded successfully
// Otherwise false
function util_fldr_decode_rx_msg {
    parameter received.

    set opcode to received:content[0].
    if not opcode:startswith("FLDR") {
        return.
    } else if received:content:length > 1 {
        set data to received:content[1].
    }

    if opcode:startswith("FLDR_SET_SEQ_U") {
        set Uindex to opcode[14]:tonumber(-1).
        if Uindex >= 0 and Uindex < 4 {
            set U_seq[Uindex] to data.
        }

    } else if opcode = "FLDR_SET_SEQ_TP" {
        set PULSE_TIMES to data.

    } else if opcode = "FLDR_RUN_TEST" {
        run_test_control().
    } else if opcode = "FLDR_PRINT_TEST" {
        util_shbus_rx_send_back_ack(print_sequences()). 
    } else {
        util_shbus_rx_send_back_ack("could not decode fldr rx msg").
        print "could not decode fldr rx msg".
        return false.
    }
    return true.
}

// RX SECTION END

// log on action group
function util_fldr_log_on_ag {
    if AG <> PREV_AG {
        set PREV_AG to AG.
        start_logging().
    }
    wait Ts.
}