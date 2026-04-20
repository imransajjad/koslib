import numpy as np
from numpy import rad2deg, deg2rad
import ksp_physics as kspp
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
import argparse
import padasip as pa


args = None

app = QtWidgets.QApplication([])

pg.setConfigOptions(antialias=True)

win = pg.GraphicsLayoutWidget()
win.show()
win.resize(1000,600)

class PlotStore:
    def __init__(self, win, max_cols):
        self.Plex = {}
        self.i = 0
        self.max_cols = max_cols
        self.win = win
    
    def get_plot(self, x_key, y_keys):
        plot_key = x_key + "_" + "_".join(y_keys)

        if plot_key in self.Plex.keys():
            return self.Plex[plot_key]
        else:
            P = self.win.addPlot()
            self.Plex[plot_key] = P
            P.showGrid(x=True, y=True)
            P.addLegend()
            self.i += 1
            if (self.i == self.max_cols):
                self.i = 0
                self.win.nextRow()
            return P

def pole_zero_plot(plot_handle, zeros, poles, name="a"):
    Szeros = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='g'), symbol='o', size=4)
    Spoles = pg.ScatterPlotItem(pen=pg.mkPen(width=5, color='r'), symbol='+', size=4, name=name)
    
    pos = [{'pos': [np.real(z), np.imag(z)]} for z in zeros]
    Szeros.setData(pos)
    pos = [{'pos': [np.real(p), np.imag(p)]} for p in poles]
    Spoles.setData(pos)

    plot_handle.addItem(Szeros)
    plot_handle.addItem(Spoles)

class PlotStore:
    def __init__(self, win, max_cols):
        self.Plex = {}
        self.i = 0
        self.max_cols = max_cols
        self.win = win
        self.suffix = ""
    
    def get_plot(self, x_key, y_keys):
        plot_key = x_key + "_" + "_".join(y_keys)

        if plot_key in self.Plex.keys():
            return self.Plex[plot_key]
        else:
            P = self.win.addPlot()
            self.Plex[plot_key] = P
            P.showGrid(x=True, y=True)
            P.addLegend()
            self.i += 1
            if (self.i == self.max_cols):
                self.i = 0
                self.win.nextRow()
            return P

    def plot_from_keys(self,D,x_key,y_keys,x_map=lambda x: x, y_maps=[lambda y: y], markers=False, pen_i=0, events_ykeys=[]):
        """
        plot D[y_keys] against D[x_key] on p
        x_map, y_maps can be provided if needed
        """
        y_maps.extend( [y_maps[0]]*(len(y_keys)-len(y_maps)))

        if not all(key in D for key in y_keys):
            print(f"keys {y_keys} not preset in D, plot_from_keys returning")
            return
        
        p = self.get_plot(x_key, y_keys)

        p.setLabel("bottom",text=x_key)
        for i,(key,y_map) in enumerate(zip(y_keys,y_maps)):
            p.plot(x_map(D[x_key]), y_map(D[key]), pen=(i+pen_i,8), name=key+self.suffix)
            if markers:
                p.addItem(pg.ScatterPlotItem(x_map(D[x_key])[::10], y_map(D[key])[::10], pen=None,brush=(0,255,0), symbol='x')) 
        for e in events_ykeys:
            events_ymap = y_maps[e == y_keys]
            self.plot_events(p,D, x_key, e, x_map, events_ymap)


    def plot_events(self,p,D,x_key, y_key, x_map=lambda x: x, y_map=lambda y: y):
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

    def plot_ground_pos(self, D):
        tag = D["logname"].replace("\n","").split("-")[-1]
        self.suffix = "-" + tag if tag else ""

        self.plot_from_keys(D, "lng", ["lat"])
        self.plot_from_keys(D,"t",["lng"])
        # self.plot_from_keys(D,"t",["lat"])
        self.plot_from_keys(D,"lng",["h"], events_ykeys=["h"])
    
    def plot_q_e(self, D):
        self.plot_from_keys(D,"y0",["h"])
        self.plot_from_keys(D,"t",["q","q_simp","E_srf_s"])
        self.plot_from_keys(D,"t",["h","y0"], events_ykeys=["h","y0"])
    
    def plot_speed_controls(self, D):
        self.plot_from_keys(D,"t",["u0", "y0"])

    def plot_w_controls(self, D):
        self.plot_from_keys(D,"t",["u1","u2","u3"])
        self.plot_from_keys(D,"t",["y1","y2","y3"], y_maps=[lambda y: rad2deg(y)], events_ykeys=["y1"])
    
    def plot_trans_controls(self, D):
        self.plot_from_keys(D,"t",["u4","u5","u6"])

    def plot_aero_angles(self, D):
        self.plot_from_keys(D,"t",["alpha","beta", "gamma"], y_maps=[lambda y: rad2deg(y)], events_ykeys=["alpha"])
        self.plot_from_keys(D,"t",["u1","alpha","y1"], events_ykeys=["alpha"])
        self.plot_from_keys(D,"t",["u2","beta","y2"], events_ykeys=["beta"])
        
        
    def plot_aero_forces(self, D):
        # self.plot_from_keys(D,"t",["gx_att","gy_att","gz_att"])

        # self.plot_from_keys(D,"t",["gx_att","gy_att","gz_att","fx_att","fy_att","fz_att"])
        # self.plot_from_keys(D,"t",["fx_att","fy_att","fz_att"])
        # self.plot_from_keys(D,"t",["faerox_att","faeroy_att","faeroz_att"])
        
        # self.plot_from_keys(D,"t",["fx_vel","fy_vel","fz_vel"])    
        self.plot_from_keys(D,"t",["faerox_vel","faeroy_vel","faeroz_vel"], events_ykeys=["faeroy_vel"])
        self.plot_from_keys(D,"q",["faerox_vel","faeroy_vel","faeroz_vel"], events_ykeys=["faeroy_vel"])
        self.plot_from_keys(D,"y0",["faerox_vel","faeroy_vel","faeroz_vel","p_faerox_vel","p_faeroy_vel","p_faeroz_vel"])

    def plot_other(self, D):

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

        return


    def plot_q_grid(self, max_v, max_h):
        P = self.get_plot("y0",["h","q"])
        vel = np.arange(10,max_v,10)
        for q in np.logspace(-5,0.1,10):
            P.plot(vel, 5000*np.log(0.00000840159/2/q*vel*vel))

        for E in np.logspace(6.5,6.8,20):
            P.plot(vel, (3.5316e12)/(0.5*vel**2 - (-E)) - 600e3, pen=(1,3))
        P.plot(vel, (-600e3 + (3.5316e12)/(vel**2)), pen=(2,3))
        P.plot(vel, (-600e3 + (3.5316e12)/(2*vel**2)), pen=(2,3))
        P.setXRange(0, max_v)
        P.setYRange(0, max_h)

    def plot_rwy(self):
        P = self.get_plot("lng",["lat"])
        P.plot([-74.73766837,-74.625860287-0.01], \
            [-0.0485911247,-0.049359350], name="rwy", pen=(1,2))

class RLSfir:
    """
    An implementation of a recursive least squares filter with
    y : output of dimension 1
    x : input of dimension 1

    w : the weights of dimension n s.t e[t]^2 is minimized with
        e[t] = y[t] - x[t]*w0 + x[t-1]*w1 + ... + x[t-n+1]*wn-1
    """
    def __init__(self, dim, ffactor, noise_floor=0.05, filter=None):
        """
        initialize the object

        dim     : the number of previous values of x to incorporate
        ffactor : the forgetting factor
        filter  : a pre filter to apply to the input and output
        """
        self.dim = dim
        self.ffactor = ffactor
        self.noise_floor = noise_floor
        self.reset()
    
    def reset(self):
        self.Q = np.identity(self.dim)
        self.w = np.zeros( (self.dim,1) )
        self.exes = np.zeros( (1,self.dim) )
        self.prev_y = np.array([[0.0]])

    def update(self, y, x):
        """
        supply a sample of input and output and get the filter weights and output

        y : the output variable
        x : the input variable

        outputs
        w : an 1 x dim array of the weights (already transposed)
        y : the predicted value of y 
        """
        # self.exes = np.concatenate( (self.exes[:,1:], np.array([[x]])), axis=1 )
        self.exes = np.concatenate( (np.array([[x]]),self.exes[:,:-1]), axis=1 )

        self.Q = self.ffactor*self.Q
        y_predicted = self.exes @ self.w
        if np.abs(x) > self.noise_floor:
            self.Q += np.transpose(self.exes)*self.exes
            self.w = self.w + np.linalg.solve(self.Q, np.transpose(self.exes) @ ( np.array([[y]]) - y_predicted ))

        self.prev_y = y
        

        return np.transpose(self.w), y_predicted

    def solve(self, y, x):
        """
        call update on an entire dataseries
        """
        self.reset()
        w = np.zeros( (len(x), self.dim) )
        y_pre = np.zeros( (len(x), 1) )
        for i, _ in enumerate(x):
            w[i,:], y_pre[i] = self.update(y[i],x[i])
        
        return w, y_pre

def least_squares_fir(dim, y, x):
    X = np.zeros( (len(x)-dim+1,0))
    for i in range(0,dim):
        start_index = i
        end_index = len(x)-dim+i+1
        X = np.column_stack((x[start_index:end_index],X))

    y = y[dim-1:].T

    return np.linalg.solve(X.transpose()@X, X.transpose()@y)

from scipy import signal

def butter_lowpass_filter(data, cutoff, fs, order=5):
    nyq = 0.5 * fs  # Nyquist Frequency
    normal_cutoff = cutoff / nyq
    b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
    y = signal.filtfilt(b, a, data)
    return y

def main():

    parser = argparse.ArgumentParser("parse fldr logs")
    parser.add_argument("-p", "--preset", default="rls_orbit", type=str, help="a preset for plotting")
    parser.add_argument("filenames", default=["logs/lastlog.csv"], nargs='*', type=str, help="list of filenames to parse")

    args = parser.parse_args()

    P = PlotStore(win, 3)

    for fname in args.filenames:
        D = kspp.parse_log_to_dict(fname)
        
        # kspp.do_area_estimate(D)
        
        # kspp.do_glide_prediction(D, N=10, tstart=0)
        # for G in D["glide_paths"]:
            # plot_log_math(G)
        if args.preset == "reentry":
            kspp.do_ship_math(D)
            kspp.do_vel_math(D)

            P.plot_ground_pos(D)
            P.plot_q_e(D)
            P.plot_w_controls(D)

            P.plot_q_grid(2500,80000)
            P.plot_rwy()

        if args.preset == "orbit":
            kspp.do_ship_math(D)
            P.plot_w_controls(D)

        if args.preset == "rls_orbit":
            kspp.do_ship_math(D)
            N = 2

            w_keys = [f"rls_w{i}" for i in range(0,N)]

            D["y1diff"] = kspp.derivative2(D["t"], D["y1"])

            w, y_pre = RLSfir(N, 0.999, noise_floor=0.05).solve(D["y1diff"], D["u1"])

            for i, w_key in enumerate(w_keys):
                D[w_key] = w[:,i]
            
            D["y1diff_pre"] = y_pre[:,0]

            print("w_rls", w[-1,:])
            print("w_ls", least_squares_fir(N, D["y1diff"], D["u1"]))

            # P.plot_from_keys(D, "t", ["u1", "y1"])
            P.plot_from_keys(D, "t", w_keys)
            # P.plot_from_keys(D, "t", ["u1", "y1"] + w_keys + ["y1diff_pre"])
            P.plot_from_keys(D, "t", ["u1", "y1", "y1diff", "y1diff_pre"])

if __name__ == '__main__':
    main()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtWidgets.QApplication.instance().exec_()
