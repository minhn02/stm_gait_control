import sys


class TrapezTraj() :

	def __init__( self, pmin, pmax, vmax, t_trans=None, ntrips=1, t0=0 ) :
		self._p_orig = pmin if vmax > 0 or pmin > pmax else pmax
		self._vmax = vmax if pmin < pmax else -abs( vmax )
		self._Tt = t_trans if t_trans is not None else abs( ( pmax - pmin )/self._vmax )
		self._t0 = t0
		self._ntrips = ntrips
		self._completed = False
		if self._ntrips is not None :
			self._p_end = self._p_orig if self._ntrips%2 == 0 else pmax if self._p_orig == pmin else pmin

		# Compute the half-cycle period:
		self._Tc = abs( ( pmax - pmin )/self._vmax ) + self._Tt
		# Reduce the maximum velocity and extend the half-cycle period if the transient duration is too long:
		if 2*self._Tt > self._Tc :
			self._Tc = 2*self._Tt
			self._vmax = ( 1 if vmax > 0 else -1 )*( pmax - pmin )/self._Tt
			print( 'Transient duration too long! vmax is reduced to %.1f mm/s' % self._vmax, file=sys.stderr )

		# Pre-compute recurring division to optimize computation time:
		self._vmax_Tt = self._vmax/self._Tt


	@property
	def trip_duration( self ) :
		return self._Tc


	@property
	def total_duration( self ) :
		return self._Tc*self._ntrips


	@property
	def completed( self ) :
		return self._completed


	def set_t0( self, t0 ) :
		self._t0 = t0


	def get_trip_count( self, t ) :
		return ( t - self._t0 )//self._Tc


	def _check_if_completed( self, t ) :
		#if self._ntrips is not None and not self._completed :
		if self._ntrips is not None :
			self._completed = t - self._t0 >= self._Tc*self._ntrips

		return self._completed


	def get_vel( self, t ) :
		if self._check_if_completed( t ) :
			return 0

		t -= self._t0

		sign = 1
		if t%(2*self._Tc) > self._Tc :
			sign = -1
		t = t%self._Tc
			
		if t < self._Tt :
			return sign*self._vmax_Tt*t
		elif t < self._Tc - self._Tt :
			return sign*self._vmax
		else :
			return sign*self._vmax_Tt*( self._Tc - t )


	def get_pos( self, t ) :
		if self._check_if_completed( t ) :
			return self._p_end

		t -= self._t0

		t = t%(2*self._Tc)
		if t > self._Tc :
			t = 2*self._Tc - t

		if t < self._Tt :
			return 0.5*self._vmax_Tt*t**2 + self._p_orig
		elif t < self._Tc - self._Tt :
			return self._vmax*( t - 0.5*self._Tt ) + self._p_orig
		else :
			return self._vmax*( self._Tc - self._Tt ) - 0.5*self._vmax_Tt*( self._Tc - t )**2 + self._p_orig
	

	def __call__( self, t ) :
		return self.get_pos( t ), self.get_vel( t )
