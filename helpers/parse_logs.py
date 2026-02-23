import numpy as np
from numpy import rad2deg, deg2rad
import ksp_physics as kspp
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets


app = QtWidgets.QApplication([])

pg.setConfigOptions(antialias=True)

win = pg.GraphicsLayoutWidget()
win.show()
win.resize(1000,600)

def pole_zero_plot(plot_handle, zeros, poles, name="a"):
    Szeros = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='g'), symbol='o', size=4)
    Spoles = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='+', size=4, name=name)
    
    pos = [{'pos': [np.real(z), np.imag(z)]} for z in zeros]
    Szeros.setData(pos)
    pos = [{'pos': [np.real(p), np.imag(p)]} for p in poles]
    Spoles.setData(pos)

    plot_handle.addItem(Szeros)
    plot_handle.addItem(Spoles)

def get_plot(x_key, y_keys):
    plot_key = x_key + "_" + "_".join(y_keys)

    if plot_key in get_plot.Plex.keys():
        return get_plot.Plex[plot_key]
    else:
        P = win.addPlot()
        get_plot.Plex[plot_key] = P
        P.showGrid(x=True, y=True)
        P.addLegend()
        get_plot.i += 1
        if (get_plot.i == get_plot.max_cols):
            get_plot.i = 0
            win.nextRow()
        return P
get_plot.Plex = {}
get_plot.i = 0
get_plot.max_cols = 3

def plot_from_keys(D,x_key,y_keys,x_map=lambda x: x, y_maps=[lambda y: y], markers=False, pen_i=0, events_ykeys=[]):
    """
    plot D[y_keys] against D[x_key] on p
    x_map, y_maps can be provided if needed
    """
    y_maps.extend( [y_maps[0]]*(len(y_keys)-len(y_maps)))

    if not all(key in D for key in y_keys):
        # print("keys not preset in D, plot_from_keys returning")
        return
    
    p = get_plot(x_key, y_keys)

    p.setLabel("bottom",text=x_key)
    for i,(key,y_map) in enumerate(zip(y_keys,y_maps)):
        p.plot(x_map(D[x_key]), y_map(D[key]), pen=(i+pen_i,8), name=key+plot_from_keys.suffix)
        if markers:
            p.addItem(pg.ScatterPlotItem(x_map(D[x_key])[::10], y_map(D[key])[::10], pen=None,brush=(0,255,0), symbol='x')) 
    for e in events_ykeys:
        events_ymap = y_maps[e == y_keys]
        plot_events(p,D, x_key, e, x_map, events_ymap)
plot_from_keys.suffix = ""


def plot_events(p,D,x_key, y_key, x_map=lambda x: x, y_map=lambda y: y):
    x_pts = []
    y_pts = []
    last_time = -1
    anchor_y = 0
    for evt in D["evts"]:
        idx = np.argmin( np.abs(D["t"]-evt[0]) )
        x_pts.append(x_map(D[x_key][idx]))
        y_pts.append(y_map(D[y_key][idx]))
        if last_time != D["t"][idx]:
            last_time = D["t"][idx]
            anchor_y = 0
        ti = pg.TextItem(evt[1], anchor=(0.0,anchor_y))
        ti.setPos(x_pts[-1],y_pts[-1])
        p.addItem(ti)
        anchor_y += 0.5


    p.addItem(pg.ScatterPlotItem(x_pts, y_pts, pen=None,brush=(0,255,0), symbol='x')) 

def plot_log_math(D,**kwargs):
    # print(D)
    # win.setWindowTitle(D["logname"])
    tag = D["logname"].replace("\n","").split("-")[-1]
    plot_from_keys.suffix = "-" + tag if tag else ""
   

    plot_from_keys(D, "lng", ["lat"])
    
    plot_from_keys(D,"t",["lng"])
    plot_from_keys(D,"lng",["h"], events_ykeys=["h"])

    plot_from_keys(D,"y0",["h","q"])

    
    plot_from_keys(D,"t",["q","q_simp","E_srf_s"])

    plot_from_keys(D,"t",["h","y0"], events_ykeys=["h","y0"])
    plot_from_keys(D,"t",["u0","u1","u2","u3"])
    # plot_from_keys(D,"t",["u0","u4","u5","u6"])
    
    plot_from_keys(D,"t",["y1","y2","y3"], y_maps=[lambda y: rad2deg(y)], events_ykeys=["y1"])
    plot_from_keys(D,"t",["alpha","beta", "gamma"], y_maps=[lambda y: rad2deg(y)], events_ykeys=["alpha"])
    plot_from_keys(D,"t",["u2","beta","y2"], events_ykeys=["beta"])
    
    

    # plot_from_keys(D,"t",["gx_att","gy_att","gz_att"])

    # plot_from_keys(D,"t",["gx_att","gy_att","gz_att","fx_att","fy_att","fz_att"])
    # plot_from_keys(D,"t",["fx_att","fy_att","fz_att"])
    # plot_from_keys(D,"t",["faerox_att","faeroy_att","faeroz_att"])
    
    # plot_from_keys(D,"t",["fx_vel","fy_vel","fz_vel"])    
    plot_from_keys(D,"t",["faerox_vel","faeroy_vel","faeroz_vel"], events_ykeys=["faeroy_vel"])
    plot_from_keys(D,"q",["faerox_vel","faeroy_vel","faeroz_vel"], events_ykeys=["faeroy_vel"])
    plot_from_keys(D,"y0",["faerox_vel","faeroy_vel","faeroz_vel","p_faerox_vel","p_faeroy_vel","p_faeroz_vel"])
    # plot_from_keys(D,"t",["pe_faerox_vel","pe_faeroy_vel","pe_faeroz_vel"])

    # plot_from_keys(D,"t",["Area_fues","Area_wing"])


    # plot_from_keys(D,"t",["acc","g","acc_g_angle","acc_diff"])
    
    # plot_from_keys(D,"h",["acc","g","acc_ratio"])
    
    # plot_from_keys(D,"h",["acc_g_angle"], y_maps=[lambda y: rad2deg(y)])

    # plot_from_keys(D,"h",["acc_diff"])
    # plot_from_keys(D,"t",["m"], events_ykey="m")
    # plot_from_keys(D,"t",["cd_est","cl_est","cl_fit","cd_fit"])
    # plot_from_keys(D,"t",["tem","pres"])

    # D["rho_pre"] = 1.0*np.exp(-D["h"]/5000)
    # D["rho"] = 1.0*D["pres"] +0.00000000000001
    # plot_from_keys(D,"h",["rho","rho_pre"], y_maps=[lambda x : np.log(x)])


def plot_q_grid(max_v, max_h):
    P = get_plot("y0",["h","q"])
    vel = np.arange(10,max_v,10)
    for q in np.logspace(-5,0.1,10):
        P.plot(vel, 5000*np.log(0.00000840159/2/q*vel*vel))

    for E in np.logspace(6.5,6.8,20):
        P.plot(vel, (3.5316e12)/(0.5*vel**2 - (-E)) - 600e3, pen=(1,3))
    P.plot(vel, (-600e3 + (3.5316e12)/(vel**2)), pen=(2,3))
    P.plot(vel, (-600e3 + (3.5316e12)/(2*vel**2)), pen=(2,3))
    P.setXRange(0, max_v)
    P.setYRange(0, max_h)

def plot_rwy():
    P = get_plot("lng",["lat"])
    P.plot([-74.73766837,-74.625860287-0.01], \
        [-0.0485911247,-0.049359350], name="rwy", pen=(1,2))

def main():
    fnames = list(sys.argv[1:])
    if not fnames:
        fnames = list(["logs/lastlog.csv"])

    for fname in fnames:
        D = kspp.parse_log_to_dict(fname)
        kspp.do_ship_math(D)
        kspp.do_vel_math(D)
        plot_log_math(D)
        
        # kspp.do_area_estimate(D)
        
        # kspp.do_glide_prediction(D, N=10, tstart=0)
        # for G in D["glide_paths"]:
            # plot_log_math(G)
        
    plot_q_grid(2500,80000)
    plot_rwy()




if __name__ == '__main__':
    main()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()
