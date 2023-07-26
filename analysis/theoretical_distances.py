#!/usr/bin/env python3
import numpy as np
from scipy.integrate import odeint
from scipy.spatial.transform import Rotation
import pickle


DEG_TO_RAD = np.pi/180

def theoretical_distance_traveled( shifts ) :

	lx = 0.200 # m
	ly = 0.200 # m
	r  = 0.112 # mm

	beta_max = 40 # °
	dbeta_dt = 15 # °/s
	nb_pts = 10000

	beta_list = []
	for i in range( ( shifts + 1 )//2 ) :
		beta_list += np.linspace( -beta_max, beta_max, nb_pts ).tolist()
		if shifts - 2*i - 1 :
			beta_list += np.linspace( beta_max, -beta_max, nb_pts ).tolist()


	dt = beta_max*2/nb_pts/dbeta_dt

	dbdt = []
	w = [ [] for _ in range( 4 ) ]
	V = []
	beta_prev = beta_list[0]

	Rbeta = Rotation.from_rotvec( beta_max/2*np.array([ 0, 0, 1 ]), degrees=True ).as_matrix()

	Xw = []
	Xw.append( Rbeta@np.array([  lx,  ly, beta_max/2*DEG_TO_RAD ]) )
	Xw.append( Rbeta@np.array([  lx, -ly, beta_max/2*DEG_TO_RAD ]) )
	Xw.append( Rbeta.T@np.array([ -lx,  ly, -beta_max/2*DEG_TO_RAD ]) )
	Xw.append( Rbeta.T@np.array([ -lx, -ly, -beta_max/2*DEG_TO_RAD ]) )

	for i, beta in enumerate( np.array( beta_list )*DEG_TO_RAD ) :

		if beta >= beta_prev :
			direction = 1
		else :
			direction = -1
		dbdt.append( direction*dbeta_dt )
		vs = np.array( [ ( -1 if wheel//2 else 1 )*( -lx*np.tan( beta/2 ) + ( -1 if wheel%2 else 1 )*ly )*direction*dbeta_dt*DEG_TO_RAD/2 for wheel in range( 4 ) ] )
		vd = np.array( [ ( -1 if wheel%2 else 1 )*ly/lx*np.tan( beta/2 ) for wheel in range( 4 ) ] )
		V.append( max( -vs/( 1 + vd ) ) )
		for wheel in range( 4 ) :
			w[wheel].append( ( V[-1]*( 1 + vd[wheel] ) + vs[wheel] )/r )
		beta_prev = beta

		
		for wheel in range( 4 ) :

			def dX( X, t ) :
				x = X[0]
				y = X[1]
				theta = X[2]
				Vw = r*w[wheel][i]
				Vr = V[i]

				dx = np.cos( theta )*Vw
				dy = np.sin( theta )*Vw
				dtheta = Vr/lx*np.tan( -beta/2 ) + ( -1 if wheel < 2 else 1 )*dbdt[i]*DEG_TO_RAD/2

				return np.array([ dx, dy, dtheta ])

			Xw[wheel] = odeint( dX, Xw[wheel], [ 0, dt ] )[-1]

	#return np.linalg.norm( sum( x[:2] for x in Xw )/4 )*1e3
	return sum( x[0] for x in Xw )/4*1e3



theoretical_distances = {}

for n_shifts in range( 1, 31 ) :
	print( f'n_shifts={n_shifts} ', end='' )
	theoretical_distances[str(n_shifts)] = theoretical_distance_traveled( n_shifts )
	print( f'-> {theoretical_distances[str(n_shifts)]}' )


with open( 'theoretical_distances.pkl', 'wb' ) as f:
    pickle.dump( theoretical_distances, f )

print( 'Theoretical distances saved in theoretical_distances.pkl' )
