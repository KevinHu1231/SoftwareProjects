from scipy.io import loadmat
import pickle

data_mat = loadmat('figure8.mat')

data = {
    't': data_mat['t'].flatten() ,
    'x_true': data_mat['x_true'].flatten(),
    'y_true': data_mat['y_true'].flatten(),
    'th_true': data_mat['th_true'].flatten(),
    'v': data_mat['v'].flatten(),
    'v_var': data_mat['v_var'].flatten()[0],
    'om': data_mat['om'].flatten(),
    'om_var': data_mat['om_var'].flatten()[0],
    'l': data_mat['l'],
    'd': data_mat['d'].flatten(),
    'b': data_mat['b'],
    'b_var': data_mat['b_var'].flatten()[0],
    'r': data_mat['r'],
    'r_var': data_mat['r_var'].flatten()[0]
}

with open('data.pickle', 'wb') as f:
    # Pickle the 'data' dictionary using the highest protocol available.
    pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

with open('data.pickle', 'rb') as f:
    # The protocol version used is detected automatically, so we do not
    # have to specify it.
    data_l = pickle.load(f)

print(data_l)

