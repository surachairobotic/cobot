import sys
from numpy import *
from math import sqrt
import matplotlib.pyplot as plt

# http://nghiaho.com/?page_id=671


# average of norms from camera
# got from find_norms()
norms_avg = array([0.0999894, 0.08125654, 0.12574291, 0.10634581, 0.07529343, 0.11320507])
CAD_bar_xyz = [0.06036,-0.018,0.094]


'''
Base Marker

        2 (Bar)
3

1         0
'''




'''
R : [[ 0.85907309 -0.02061672  0.51143756]
 [ 0.51161294  0.00399138 -0.85920677]
 [ 0.01567268  0.99977949  0.01397667]]
t : [[-0.26211133]
 [-0.18565007]
 [-0.01812064]]
'''

names = ['RigidBody-Base:Marker1'
  , 'RigidBody-Base:Marker2'
  , 'RigidBody-Base:Marker3'
  , 'RigidBody-Base:Marker4']
#  , 'RigidBody-Base']
types = ['Position', 'Rotation']# , 'Marker Quality', 'Error Per Marker']
xyzs = ['X','Y','Z','W']




def rigid_transform_3D(A, B):
  assert len(A) == len(B)
  N = A.shape[0]; # total points
  centroid_A = mean(A, axis=0)
  centroid_B = mean(B, axis=0)

  # centre the points
  AA = A - tile(centroid_A, (N, 1))
  BB = B - tile(centroid_B, (N, 1))

  # dot is matrix multiplication for array
  H = transpose(AA) * BB
  U, S, Vt = linalg.svd(H)
  R = Vt.T * U.T

  # special reflection case
  if linalg.det(R) < 0:
     print "Reflection detected"
     Vt[2,:] *= -1
     R = Vt.T * U.T

  t = -R*centroid_A.T + centroid_B.T
  return R, t

def get_col(names, l_marker_type, l_name, l_value_type, l_xyz):
  global types

  col = {}
  arr_marker_type = l_marker_type.strip().replace('"','').split(',')
  arr_name = l_name.strip().replace('"','').split(',')
  arr_value_type = l_value_type.strip().replace('"','').split(',')
  arr_xyz = l_xyz.strip().replace('"','').split(',')
  for n in names:
    col[n] = {}
    for t in types:
      if t=='Position':
        col[n][t] = {'X':None,'Y':None,'Z':None}
      elif t=='Rotation':
        col[n][t] = {'X':None,'Y':None,'Z':None,'W':None}
      else:
        col[n][t] = None
  for i in range(len(arr_name)):
    m = arr_marker_type[i]
    n = arr_name[i]
    t = arr_value_type[i]
    x = arr_xyz[i]
    if m=='Marker' and n in names and t in types:
#      print(n+', '+t+', '+unicode(type(col[n][t])))
      if type(col[n][t]) is dict:
        col[n][t][x] = i
      else:
        col[n][t] = i
  #print(col)
  return col

def find_bar_xyz():
  # [ 0-1, 0-2, 0-3, 1-2, 1-3, 2-3]
  global norms_avg, CAD_bar_xyz
  pnt = array([[0.0, 0.1, 0.01735]
    , [0.0, 0.0, 0.01735]
    , [-0.075, 0.0, 0.01735]])
  print('** find_bar_xyz **')

  # [ 0-2, 1-2, 2-3]
  norm2 = array([norms_avg[1], norms_avg[3], norms_avg[5]])

  # M*[x,y,z].T = H
  M = zeros([3,3])
  H = zeros([3,1])
  for i in range(3):
    k = (i+1)%3
    for j in range(3):
      M[i,j] = 2*(-pnt[i][j]+pnt[k][j])
      H[i]+= -pnt[i][j]**2 + pnt[k][j]**2
    H[i]+= norm2[i]**2 - norm2[k]**2

#  xyz = array([0.0]*3)
  xyz = zeros((3,3))
  for i in range(3):
    j = (i+1)%3
    m = array([[M[i,0], M[i,1]], [M[j,0], M[j,1]]] )
    h = array([[H[i,0], H[j,0]]]).T
    r = linalg.inv(m).dot(h)
    x = r[0,0]
    y = r[1,0]
    z = sqrt(norm2[i]**2 - (x-pnt[i,0])**2 - (y-pnt[i,1])**2) + pnt[i,2]
    xyz[i,0] = x
    xyz[i,1] = y
    xyz[i,2] = z

  for i in range(3):
    for j in range(1,3):
      if linalg.norm(xyz[i]-xyz[j])>0.001:
        raise Exception('find_bar_xyz() : too large xyz err : '+str(xyz))
  xyz = xyz.mean(0)

  # find real pos
  pnt2 = array([pnt[0], pnt[1], xyz, pnt[2]])
  real_xyz = array(CAD_bar_xyz)
  off = real_xyz - xyz
  real_pnt = []
  for p in pnt2:
    real_pnt.append(p + off)
  real_pnt = array(real_pnt)
  print('xyz (cal from norm) : '+str(real_pnt))

  # check error
  err_norm = []
  for i in range(3):
    err_norm.append(linalg.norm(xyz-pnt[i]) - norm2[i])
  print('err norm to bar : '+unicode(err_norm))

  n2 = []
  for i in range(3):
    for j in range(i+1,4):
      n2.append(linalg.norm( pnt2[i] - pnt2[j] ))
  print('err norm all : '+unicode(array(n2) - norms_avg))

  return real_pnt


def find_norms(pnt):
  global norms_avg
  norms = []
  for p in pnt:
    norm = []
    for i in range(3):
      for j in range(i+1,4):
        norm.append(linalg.norm( p[i*3:(i+1)*3] - p[j*3:(j+1)*3] ))
    norms.append(norm)
  norms = array(norms)
  norms_avg = norms.mean(0)
  print('** find_norms **')
  print('norm : '+unicode(norms_avg))
  print('norm err : '+unicode(norms.max(0) - norms.min(0)))




def get_data(fname, names, pnt_num):
  pnt = []
  err_marker = []
  index_err = []
  time = []
  with open(fname) as f:
    n_line = 0
    l_name = None
    l_type = None
    l_xyz = None
    for line in f:
      n_line+=1
      if n_line==3:
        l_marker_type = line
      elif n_line==4:
        l_name = line
      elif n_line==6:
        l_type = line
      elif n_line==7:
        l_xyz = line
        col = get_col(names, l_marker_type, l_name, l_type, l_xyz)
        index_xyz = []
        for n in names:
          c = col[n]['Position']
          for i in range(3):
            index_xyz.append(c[xyzs[i]])
          '''
          if col[n]['Marker Quality'] is None:
            index_err.append(col[n]['Error Per Marker'])
          else:
            index_err.append(col[n]['Marker Quality'])
          '''
      elif n_line>7:
        arr = line.strip().replace('"','').split(',')
        val = []
        for index in index_xyz:
          if arr[index]:
            val.append(float(arr[index]))
          else:
            break
        if len(val)==len(index_xyz):
          '''
          e = []
          for i in range(len(index_err)):
            print('e : '+str(arr[index_err[i]]))
            print(n_line)
            print(val)
            e.append(float(arr[index_err[i]]))
          err_marker.append(e)
          '''
          pnt.append(val)
          time.append(float(arr[1]))
        if len(pnt)>pnt_num:
          break
  return array(pnt), array(time) - time[0]


# compare norm from camera and CAD
def compare_norm():
  global norms_avg, CAD_bar_xyz
  
  # postion of pnt 1 from CAD
  px = 0.0899 # 0.10-0.01
  py = -0.0901 # -0.10+0.01
  pz = 0.01736 # 0.005+0.007+0.00536
  real_xyz = array([[px,py+0.1,pz]
    ,[px,py,pz]
    ,CAD_bar_xyz #,[0.06036,-0.018,0.09405]
    ,[px-0.075,py,pz] ])
  norm = []
  for i in range(len(real_xyz)):
    for j in range(i+1, len(real_xyz)):
      norm.append(linalg.norm( real_xyz[i] - real_xyz[j] ))
  print('** compare_norm **')
  print('err norm (CAD - camera) : '+str(array(norm)-norms_avg))
  print('real xyz (CAD) : '+str(real_xyz))

if __name__ == "__main__":

  pnt, time = get_data('Take 2018-10-19 Test 01.csv', names, 100000)
  find_norms(pnt)
#  exit()
  compare_norm()
  real_pnt = find_bar_xyz()


  # find R,t
  A = pnt[:,0:3]
  B = tile(real_pnt[0,:], (len(pnt), 1))
  for i in range(1,4):
    A = concatenate( (A, pnt[:,(i*3):((i+1)*3)]) )
    B = concatenate( (B, tile(real_pnt[i,:], (len(pnt), 1))) )

  R,t = rigid_transform_3D(mat(A),mat(B))

  # Find the error
  B2 = (R*A.T) + tile(t, (1, len(B)))
  B2 = B2.T
  err = B2 - B

  print('R : '+str(R))
  print('t : '+str(t))

  '''
  err_marker = array(err_marker)
  err_marker2 = err_marker
  for i in range(1,4):
    err_marker2 = concatenate( (err_marker2, err_marker) )
  '''
  f, axarr = plt.subplots(2, sharex=True)
  for i in range(len(axarr)):
    axarr[i].hold(True)
    axarr[i].grid(linestyle='-', linewidth='0.5')
  for i in range(3):
    axarr[0].plot(err[:,i])
  '''
  for i in range(4):
    axarr[1].plot(err_marker2[:,i])
  '''
  axarr[0].set_ylabel('error [m]')
  axarr[1].set_ylabel('Marker quality')
  plt.show()
